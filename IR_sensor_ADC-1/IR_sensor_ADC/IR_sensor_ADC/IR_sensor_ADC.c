
// < IR_sensor_ADC.c >  : 적외선센서 모듈(TCRT5000)를 이용하여 전방 장애물을 검출하는 ATmega128 코드
 
#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함. 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit ) ;    // 부호없는 정수형 변수를 10진수 형태로 
                                                                      // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);    // msec 단위 시간지연
void usec_delay(int n);    // usec 단위 시간지연

static volatile unsigned short    IR_adc0_value = 0  ;


int main() 
{   

    unsigned short  IR_data0 = 0  ;

    LcdInit();                     // LCd 초기화 함수 

    LcdMove(0,0);                  // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("IR Sensor");          // LCD에 문자열 디스플레이 
    LcdMove(1,0);                  // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("IR_ADC =     ");      // LCD에 문자열 디스플레이


   ////////  적외선센서모듈(TCRT5000) 구동을 위한 HW 관련 레지스터 설정  ////////////

   //  입력/출력 포트 설정 	(교재 pp75-76 레지스터그림, 표6.1 참조)
    DDRA |= 0x01;          // 적외선LED ON/OFF 신호( PA0 : 출력포트 설정  )
                           // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 ) 

    PORTA &= ~0x01;        // 적외선LED(EL-7L) OFF ( PA0 = 0 )
                           // PORTA = 0b**** ***0,   PORTA = PORTA & ~0b0000 0001( ~0x01 )

    DDRF &= ~0x01;         // 적외선센서출력신호V?(포토TR Emitter 신호) 연결핀 ( ADC0(PF0) : 입력포트 설정 )  
                           // 입력포트 설정.  DDRF = 0b**** ***0, DDRF = DDRF & ~0b0000 0001(~0x01)
  
    ////////////  Timer 0 설정  ( 2 msec 주기의 타이머0 오버플로 인터럽트 설정 )  ///////////////
    // 교재 P133-137(레지스터 그림과 표8.1-표8.5) 참조    

    TCCR0 &= ~0x48;           // Normal mode(타이머모드), 타이머 0 시작(1024분주 사용)
                              // TCCR0 = 0b*0**0111 
                              // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;            // TCCR0 =TCCR0 | 0b00000111( 0x07 )  

    TCNT0 = 256 - 31;         // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 2msec
                              // 31 = 2msec/ 64usec,  TCNT0 = 256 - 31 

    TIMSK |= 0x01;            // 타이머0 오버플로 인터럽트 허용 
                              // TIMSK = 0b**** ***1, TIMSK = TIMSK | 0b0000 0001( 0x01 ) 

    sei();                    // 전역 인터럽트 허용


    //////////   ADC( AD변환기 ) 설정  /////////////
   //////  (교재 pp321-322 레제스터 ADMUX 그림, 표15.1, 표15.2 참조)

    ADMUX &= ~0xE0;       // 기준전압선택( AREF ), ADC결과 오른쪽 정렬 
                          // ADMUX = 0b000* ****  
                          // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 )  

    //////  (교재 pp323-324 레제스터 ADCSRA 그림, 표15.3 참조)

	ADCSRA |= 0x87;       // ADC 가능(enable), 프리스케일러(Prescaler) 선택: 128 분주
                          // ADCSRA = 0b1*** *111
                          // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 
    while (1) 
    { 

       cli();                               // 전역인터럽트 금지

 	   IR_data0 = IR_adc0_value;            // ADC 값을 변수 IR_data0 에 저장 

       sei();                              // 전역인터럽트 허용

	   LcdMove(1, 9);                     // LCD에 쓸 데이터 초기 위치 설정 (1행, 9열)
       Display_Number_LCD(IR_data0, 3);  // 반사된 적외선양(ADC값) 변수 IR_data0 을 100자리까지 디스플레이 

   }

}      // int main() 함수의 끝 



ISR( TIMER0_OVF_vect )     //  2 msec 주기의 타이머0 오버플로 인터럽트 서비스프로그램
{                          //  2msec 마다 적외선 LED 쏘고 반사되는 적외선 양 검출 

    TCNT0 = 256 - 31;         // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 2msec
                              // 31 = 2msec/ 64usec,  TCNT0 = 256 - 31

    /////  적외선LED(EL-7L) 켜기(ON)  //////

    PORTA |=  0x01;        // 적외선LED(EL-7L) ON ( PA0 = 1)
                           // PORTA = 0b**** ***1,   PORTA = PORTA | 0b0000 0001( 0x01 )
    usec_delay(50);        // 50usec동안 계속 켜기(적외선이 반사되어 돌아올때(안정화될 때)까지 계속 켠다.) 

    /////////////   반사된 적외선 센서신호 검출(AD변환)  ////////////////
    //////  (교재 P321 레제스터 ADMUX 그림, 표15.1, p322 표15.2 참조) 

    ADMUX &= ~0x0F;         // ADC 채널 0 ( ADC0 ) 선택 
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F ) 

    ADCSRA |= 0x40;         // ADC 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
                            // ADCSRA = 0b*1** ****  
                            // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

    while( !( ADCSRA & 0x10) );  // ADC 변환이 완료될 때까지 기다림. 
                                 // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) = 1 이면 변환완료.
 
    ADCSRA |= 0x10;              // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 ) 

    IR_adc0_value = ADC;         // ADC 변환 완료된 디지털 값( 레지스터 ADC )을 전역변수에 저장  

    /////  적외선LED(EL-7L) 끄기(OFF)  //////
    PORTA &= ~0x01;        // 적외선LED(EL-7L) OFF ( PA0 = 0 )
                           // PORTA = 0b**** ***0,   PORTA = PORTA & ~0b0000 0001( ~0x01 )

}

/////////////////////////////////////////////////////////////////

 
void Display_Number_LCD( unsigned short num, unsigned char digit ) //부호없는 정수형변수(num)을 10진수 형태로 
{                                                       // 정해진 자릿수(digit) 만큼 LCD 에 디스플레이 하는 함수 

      HexToDec( num, 10); //10진수로 변환

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000자리 디스필레이 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스필레이 
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스필레이 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스필레이
      if( digit >= 1)  LcdPutchar(NumToAsc(cnumber[0]));    // 1자리 디스플레이

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
