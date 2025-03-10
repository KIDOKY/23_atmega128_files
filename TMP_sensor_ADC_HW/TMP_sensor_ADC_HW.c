//< TMP_sensor_ADC_HW.c >  : 온도센서 (LM35)를 이용하여 온도를 검출하는 ATmega128 코드 

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 
#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함. 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // 부호없는 정수형 변수를 10진수 형태로 
                                                                     // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void Display_TMP_LCD( unsigned short tp );    // 온도를 10진수 형태로 소숫점 위 2자리, 소숫점 아래 1자리까지 
                                              // LCD 에 디스플레이 하는 함수

void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연

unsigned  short ADC_Read( unsigned char ch_no );	//AD 변환값 읽기 함수

static volatile unsigned short   TMP_value = 0, TMP_adc_value = 0,  TMP_adc_value_avg = 0 ;

 
int main() 
{   
    unsigned short  Temperature = 0  ;

    LcdInit();                   // LCd 초기화 함수 

    LcdMove(0,0);                // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("TMP Sensor");       // LCD에 문자열 디스플레이 
    LcdMove(1,0);                // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)
    LcdPuts("TMP = ");           // LCD에 문자열 디스플레이 


    ////////  온도센서 (LM35) 구동을 위한 HW 관련 레지스터 설정  ////////////
    //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRB |= 0x20;          // LED ON/OFF 신호( PB5 : 출력포트 설정  )
                           // DDRB = 0b**1* ****, DDRB = DDRB | 0b0010 0000( 0x20 ) 

    DDRF &= ~0x08;         // 온도센서출력신호 Vo 연결핀 ( ADC3(PF3) : 입력포트 설정 )  
                           // 입력포트 설정.  DDRF = 0b**** 0***, DDRF = DDRF & ~0b0000 1000(~0x08)
  
    ////////////  Timer 2 설정  ( 10 msec 주기의 타이머2 오버플로 인터럽트 설정 )  ///////////////
    // 교재 P133-137(레지스터 그림과 표8.1-표8.6) 참조    

    TCCR2 &= ~0x4A;           // Normal mode(타이머모드), 타이머 2 시작(1024분주 사용)
                              // TCCR2 = 0b*0**0101 
                              // TCCR2 =TCCR2 & ~0b01001010(~0x4A )  
    TCCR2 |= 0x05;            // TCCR2 =TCCR2 | 0b00000101( 0x05 )  

    TCNT2 = 256 - 156;        // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 2msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    TIMSK |= 0x40;            // 타이머2 오버플로 인터럽트 허용 
                              // TIMSK = 0b*1** ****, TIMSK = TIMSK | 0b0100 0000( 0x40 )

    sei();                    // 전역 인터럽트 허용



   //////////   ADC( AD변환기 ) 설정  /////////////

   //////  (교재 pp321-322 레제스터 ADMUX 그림, 표15.1, 표15.2 참조)
    ADMUX &= ~0xE0;          // 기준전압선택( AREF ), ADC결과 오른쪽 정렬 
                             // ADMUX = 0b000* ****  
                             // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 

   //////  (교재 pp323-324 레제스터 ADCSRA 그림, 표15.3 참조)
    ADCSRA |= 0x87;          // ADC 가능(enable), 프리스케일러(Prescaler) 선택: 128 분주
                             // ADCSRA = 0b1*** *111
                             // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 
    while (1) 
    { 

        cli();                                // 전역인터럽트 금지

 	    Temperature = TMP_value;              // 검출된 온도값을 변수 Temperature에 저장 

        sei();                                // 전역인터럽트 허용

	    LcdMove(1, 6);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 8열) 
        Display_TMP_LCD( Temperature  );      // 온도를 소숫점 아래 첫째자리까지 디스플레이  


   }

}      // int main() 함수의 끝 


////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )      // 10msec 주기의 타이머2 오버플로 인터럽트 서비스프로그램
{                           // 10msec(= 1 x 10msec) 마다 온도센서(ADC3)값 검출 

    static unsigned short  index_time  = 0, tmp_cnt = 0;
    static unsigned long   TMP_adc_value_sum = 0;


    TCNT2 = 256 - 156;        // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156

    index_time++; 

    if( index_time  == 1 )    //  10msec(= 1 x 10msec ) 마다 온도 검출 
    { 

        index_time = 0;    // 변수 초기화

       /////////////   온도센서신호(ADC3) 검출(AD변환)  ////////////////

       //////  (교재 P321 레제스터 ADMUX 그림, 표15.1, p322 표15.2 참조)
       ADMUX &= ~0x1F;      // ADC 채널 리셋 (채널 0, ADC0)
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F ) 

       ADMUX |=  0x00;      // ADC 채널 3 ( ADC3 ) 선택 
                            // ADMUX = 0b**** **11  
                            // ADMUX = ADMUX | 0b0000 0011 ( 0x03 ) 

       ADCSRA |= 0x40;         // ADC 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
                               // ADCSRA = 0b*1** ****  
                               // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

       while( !( ADCSRA & 0x10) );  // ADC 변환이 완료될 때까지 기다림. 
                                    // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) = 1 이면 변환완료.
 
       ADCSRA |= 0x10;           // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 ) 

       TMP_adc_value = ADC_Read( 3 );  // ADC 변환 완료된 디지털 값( 레지스터 ADC )을 전역변수에 저장  


      ////// 노이즈 제거를 위해 ADC 값의 평균값을 구함.   ////////////////////
 
       TMP_adc_value_sum +=  TMP_adc_value; 
	   tmp_cnt++;

	   if( tmp_cnt == 128 )       // 128개 평균(1.28초 = 128 * 10msec  마다 평균값 구함.)
	   {
          TMP_adc_value_avg = TMP_adc_value_sum / tmp_cnt  ;

          tmp_cnt = 0;
          TMP_adc_value_sum = 0; 
	   }


       /////  ADC 데이터의 평균값으로부터 실제온도(섭씨) 계산  ////// 

       TMP_value = (unsigned long)  TMP_adc_value_avg * 1250 / 256 ;  
 
       if( TMP_value < 270 )       // 검출된 온도가 27.0C 미만이면 LED(PB5) OFF 
       {    
          PORTB |=  0x20;         //  LED  OFF ( PB5 = 1)
                                  // PORTB = 0b**1* ****,   PORTB = PORTB | 0b0010 0000( 0x20 )
       }

       else if( TMP_value >= 270 )  // 검출된 온도가 27.0C 이상이면 LED(PB5) ON
       {
           PORTB &= ~0x20;         //  LED ON ( PB5 = 0 )
                                   // PORTB = 0b**0* ****,   PORTB = PORTB & ~0b0010 0000( ~0x20 )
       }

   }    // End of  if( index_time == 10 ) 
 
}       // End of  ISR( TIMER2_OVF_vect ) 


/////////////////////////////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch_no )		// AD 변환값 읽기 함수
{
	if( ch_no >= 8 )	ch_no = 7;

	/////////////   반사된 적외선 센서신호 검출(AD변환)  ////////////////
    //////  (교재 P321 레제스터 ADMUX 그림, 표15.1, p322 표15.2 참조) 

    ADMUX &= ~0x1F;         // ADC 채널 리셋
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F )

	ADMUX |= ch_no;			// 채널 넘버 설정

    ADCSRA |= 0x40;         // ADC 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
                            // ADCSRA = 0b*1** ****  
                            // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

    while( !( ADCSRA & 0x10) );  // ADC 변환이 완료될 때까지 기다림. 
                                 // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) = 1 이면 변환완료.
 
    ADCSRA |= 0x10;              // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 )

	return ADC;


}
/////////////////////////////////////////////////////////////////
 
void Display_Number_LCD( unsigned short num, unsigned char digit ) //부호없는 정수형변수(num)을 10진수 형태로 
{   
                                                    // 정해진 자릿수(digit) 만큼 LCD 에 디스플레이 하는 함수 
      HexToDec( num, 10); //10진수로 변환

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000자리 디스필레이 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스필레이 
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스필레이 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스필레이
      if( digit >= 1)  LcdPutchar(NumToAsc(cnumber[0]));    // 1자리 디스플레이

}


void Display_TMP_LCD( unsigned short tp  )   // 온도를 10진수 형태로 소숫점 위 2자리, 소숫점 아래 1자리까지 
{                                            // LCD 에 디스플레이 하는 함수

    HexToDec( tp, 10); //10진수로 변환 

    LcdPutchar(NumToAsc(cnumber[2]) );   // 10자리 디스플레이
    LcdPutchar(NumToAsc(cnumber[1]));    // 1자리 디스플레이 
    LcdPuts( ".");                       // 소숫점(.) 디스플레이 
    LcdPutchar(NumToAsc(cnumber[0]));    // 0.1 자리 디스플레이 

    LcdPutchar( 0xDF );                  // LCD에 문자 도 디스플레이
    LcdPutchar('C');                     // LCD에 문자 C 디스플레이

}


void HexToDec( unsigned short num, unsigned short radix)   // num으로 넘어온 16진수 형태의 데이터를 10진수로 
{                                                          //  변환하여 각각의 자릿수를 전역변수 배열 cnumber[0](1자리) - cnumber[4](10000자리)에 저장하는 함수. 

	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;
	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 
	} while(num);

} 


char NumToAsc( unsigned char Num )       // Num으로 넘어온 16진수 1자리 숫자를 문자데이터(ASCII 코드)로 
{                                        // 변환하여 리턴하는 함수
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}

/////////////////////////////////////

void msec_delay(int n)      // n msec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		    // 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}

void usec_delay(int n)      // n usec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		    // 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}

////////////////////////////////////

