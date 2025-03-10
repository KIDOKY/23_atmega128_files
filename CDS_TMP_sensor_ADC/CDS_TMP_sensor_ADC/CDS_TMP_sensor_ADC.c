
// < CDS_TMP_sensor_ADC.c >  : 조도센서와 온도센서(LM35)를 이용하여 광량과 온도를 검출하는 ATmega128 코드 

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 
#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함. 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // 부호없는 정수형 변수를 10진수 형태로 
                                                                     // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void Display_TMP_LCD( unsigned short tp );   // 온도를 10진수 형태로 소숫점 위 2자리, 소숫점 아래 1자리까지 
                                             // LCD 에 디스플레이 하는 함수

void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연


static volatile unsigned short   CDS_value = 0, CDS_adc_value = 0,  CDS_adc_value_avg = 0 ;
static volatile unsigned short   TMP_value = 0, TMP_adc_value = 0,  TMP_adc_value_avg = 0 ;


int main() 
{   

    unsigned short  Temperature = 0, CDS = 0  ;

    LcdInit();                      // LCd 초기화 함수 

    LcdMove(0,0);                   // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("CDS, TEMP Sensor");    // LCD에 문자열 디스플레이 
    LcdMove(1,0);                   // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Test Program!!");      // LCD에 문자열 디스플레이

	msec_delay(2000);

	LcdCommand( 0x01 );             // LCD Clear

    LcdMove(0,0);                   // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("CDS = ");              // LCD에 문자열 디스플레이 
    LcdMove(1,0);                   // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("TMP = ");              // LCD에 문자열 디스플레이


   ////////  조도센서 (CDS센서) 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA |= 0x04;          // LED1 ON/OFF 신호( PA2 : 출력포트 설정  )
                           // DDRA = 0b**** *1**, DDRA = DDRA | 0b0000 0100( 0x04 )
    DDRF &= ~0x04;         // 조도센서출력신호 Vo 연결핀 ( ADC2(PF2) : 입력포트 설정 )  
                           // 입력포트 설정.  DDRF = 0b**** *0**, DDRF = DDRF & ~0b0000 0100(~0x04)

    ////////  온도센서 (LM35) 구동을 위한 HW 관련 레지스터 설정  ////////////
    //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA |= 0x08;          // LED2 ON/OFF 신호( PA3 : 출력포트 설정  )
                           // DDRA = 0b**** 1***, DDRA = DDRA | 0b0000 1000( 0x08 )
    DDRF &= ~0x08;         // 온도센서출력신호 Vo 연결핀 ( ADC3(PF3) : 입력포트 설정 )  
                           // 입력포트 설정.  DDRF = 0b**** 0***, DDRF = DDRF & ~0b0000 1000(~0x08)
  
    ////////////  Timer 2 설정  ( 10 msec 주기의 타이머2 오버플로 인터럽트 설정 )  ///////////////
    // 교재 P133-137(레지스터 그림과 표8.1-표8.6) 참조    

    TCCR2 &= ~0x4A;        // Normal mode(타이머모드), 타이머 2 시작(1024분주 사용)
                           // TCCR2 = 0b*0**0101 
                           // TCCR2 =TCCR2 & ~0b01001010(~0x4A )  
    TCCR2 |= 0x05;         // TCCR2 =TCCR2 | 0b00000101( 0x05 )  

    TCNT2 = 256 - 156;     // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                           // 오버플로인터럽트 주기 = 2msec
                           // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    TIMSK |= 0x40;         // 타이머2 오버플로 인터럽트 허용 
                           // TIMSK = 0b*1** ****, TIMSK = TIMSK | 0b0100 0000( 0x40 ) 

    sei();                 // 전역 인터럽트 허용

    //////////   ADC( AD변환기 ) 설정  /////////////
    //////  (교재 pp321-322 레제스터 ADMUX 그림, 표15.1, 표15.2 참조)

    ADMUX &= ~0xE0;         // 기준전압선택( AREF ), ADC결과 오른쪽 정렬 
                            // ADMUX = 0b000* ****  
                            // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 

    //////  (교재 pp323-324 레제스터 ADCSRA 그림, 표15.3 참조)
	ADCSRA |= 0x87;        // ADC 가능(enable), 프리스케일러(Prescaler) 선택: 128 분주
                           // ADCSRA = 0b1*** *111
                           // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 

    while (1) 
    { 

       cli();                               // 전역인터럽트 금지

 	   CDS = CDS_adc_value;                 // 검출된 광량의 디지털 값을 변수 CDS에 저장 
 	   Temperature = TMP_value;             // 검출된 온도값을 변수 Temperature에 저장 

       sei();                               // 전역인터럽트 허용

	   LcdMove(0, 6);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 8열) 
       Display_Number_LCD( CDS, 3 );         // 광량의 디지털 값을 3자리 십진수로 디스플레이  

	   LcdMove(1, 6);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 8열) 
       Display_TMP_LCD( Temperature  );      // 온도를 소숫점 아래 첫째자리까지 디스플레이  

//	   msec_delay(100);

   }

}      // int main() 함수의 끝 


////////////////////////////////////////////////////////////////////


ISR( TIMER2_OVF_vect )    // 10msec 주기의 타이머2 오버플로 인터럽트 서비스프로그램
{                         // 샘플링주기 10msec(= 1 x 10msec) 마다 조도센서(ADC2)값, 온도센서(ADC3)값 검출 

    static unsigned short  index_time  = 0,  tmp_cnt = 0, cds_cnt = 0;
    static unsigned long   TMP_adc_value_sum = 0,  CDS_adc_value_sum = 0;


    TCNT2 = 256 - 156;        // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156

    index_time++; 

    if( index_time  == 1 )    //  샘플링주기 10msec(= 1 x 10msec ) 마다 광량, 온도 검출 
    { 

        index_time = 0;    // 변수 초기화

       /////////////   조도센서(CDS센서)신호(ADC2) 검출(AD변환)  ////////////////
       //////  (교재 P321 레제스터 ADMUX 그림, 표15.1, p322 표15.2 참조)

       ADMUX &= ~0x1F;      // ADC 채널 리셋 
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F ) 
       ADMUX |=  0x02;      // ADC 채널 2 ( ADC2 ) 선택 
                            // ADMUX = 0b**** **10   (이미 앞서 ADC 채널을 리셋했으므로 1로 설정하는 것만 
                            // 해주면 됨) 
                            // ADMUX = ADMUX | 0b0000 0010 ( 0x02 ) 

       ADCSRA |= 0x40;         // AD 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
                               // ADCSRA = 0b*1** ****  
                               // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

       while( !( ADCSRA & 0x10) ); // AD 변환이 완료될 때까지 기다림. 
                                   // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) = 1 이면 변환완료.
 
       ADCSRA |= 0x10;           // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 ) 

       CDS_adc_value = ADC;      // AD 변환 완료된 디지털 값( 레지스터 ADC )을 전역변수에 저장  



       /////////////   온도센서신호(ADC3) 검출(AD변환)  ////////////////
       //////  (교재 P321 레제스터 ADMUX 그림, 표15.1, p322 표15.2 참조) 

       ADMUX &= ~0x1F;      // ADC 채널 리셋 
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F ) 

       ADMUX |=  0x03;      // ADC 채널 3 ( ADC3 ) 선택 
                            // ADMUX = 0b**** **11  
                            // ADMUX = ADMUX | 0b0000 0011 ( 0x03 ) 

       ADCSRA |= 0x40;         // AD 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
                               // ADCSRA = 0b*1** ****  
                               // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

       while( !( ADCSRA & 0x10) ); // AD 변환이 완료될 때까지 기다림. 
                                   // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) = 1 이면 변환완료.
 
       ADCSRA |= 0x10;           // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 ) 

       TMP_adc_value = ADC;  // AD 변환 완료된 디지털 값( 레지스터 ADC )을 전역변수에 저장  

 
      ////// 노이즈를 제거하기 위해 CDS 센서 ADC값의 평균값을 구함.   ////////////////////

       CDS_adc_value_sum +=  CDS_adc_value; 
	   cds_cnt++;

	   if( cds_cnt == 16 )       // 16개 평균( 0.16초 = 16 * 10msec  마다 평균값 구함.)
	   {
          CDS_adc_value_avg = CDS_adc_value_sum / cds_cnt  ;

          cds_cnt = 0;
          CDS_adc_value_sum = 0; 
	   }

       /////  ADC 데이터의 평균값을 CDS 센서 전역변수에 저장   ////// 

       CDS_value = (unsigned short) CDS_adc_value_avg;  


      ////// 노이즈를 제거하기 위해 온도센서 ADC 값 평균값 구함.   ////////////////////

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
	   
	    
      ///////////////////////////////////////////////////////////////////////////////////////////////////

      if( CDS_value < 250 )           // 검출된 디지털 광량 값이 500 미만이면 LED1 (PA2) ON 
      {    
          PORTA &=  ~0x04;            // LED1 ON ( PA2 = 0 )
                                      // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
      }
      else if( CDS_value >= 250 )     // 검출된 디지털 광량 값이 500 이상이면 LED1 (PA2) OFF
      {
           PORTA |=  0x04;            // LED1 OFF ( PA2 = 1 )
                                      // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
      }

      if( TMP_value < 300 )           // 검출된 온도가 27.0C 미만이면 LED2 (PA3) OFF 
      {    
           PORTA |=  0x08;            // LED2 OFF ( PA3 = 1 )
                                      // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
      }
      else if( TMP_value >= 300 )    // 검출된 온도가 27.0C 이상이면 LED2 (PA3) ON
      {
          PORTA &=  ~0x08;           // LED2 ON ( PA3 = 0 )
                                     // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
      }

   }    // End of  if( index_time == 10 ) 
 
}       // End of  ISR( TIMER2_OVF_vect ) 



void Display_Number_LCD( unsigned short num, unsigned char digit )       // 부호없는 정수형 변수를 10진수 형태로 LCD 에 디스플레이 
{

	HexToDec( num, 10); //10진수로 변환 

	if( digit == 0 )     digit = 1 ;
	if( digit > 5 )      digit = 5 ;
 
    if( digit >= 5 )     LcdPutchar( NumToAsc(cnumber[4]) );  // 10000자리 디스플레이
	
	if( digit >= 4 )     LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스플레이 

	if( digit >= 3 )     LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스플레이 

	if( digit >= 2 )     LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스플레이

	if( digit >= 1 )     LcdPutchar(NumToAsc(cnumber[0]));    //  1자리 디스플레이

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



void HexToDec( unsigned short num, unsigned short radix) 
{
	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;

	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 

	} while(num);

} 

char NumToAsc( unsigned char Num )
{
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}



void msec_delay(int n)
{	
	for(; n>0; n--)		    // 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}

void usec_delay(int n)
{	
	for(; n>0; n--)		    // 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}



