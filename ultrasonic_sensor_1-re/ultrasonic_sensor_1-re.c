
// < ultrasonic_sensor_1.c > :초음파센서모듈(HC-SR04)을 이용하여 장애물까지의 거리를 측정하는 ATmega128 코드 

#include <avr/io.h>         // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 
#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함. 


void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // 부호없는 정수형 변수를 10진수 형태로 
                                                                      // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);    // msec 단위 시간지연
void usec_delay(int n);    // usec 단위 시간지연

static volatile unsigned short    distance_1 = 0,  distance_1_prev = 0;
static volatile unsigned char     flag = 0;


int main() 
{   

    unsigned short  dist_1 = 0;


    LcdInit();                     // LCd 초기화 함수 

    LcdMove(0,0);                  // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("UltrasonicSensor");   // LCD에 문자열 디스플레이 
    LcdMove(1,0);                  // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Dist_1 =    cm");     // LCD에 문자열 디스플레이

    //// 초음파센서모듈(HC-SR04) 구동을 위한 HW 관련 레지스터 설정  ////////////
    //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRB |= 0x08;        // 초음파센서 Trigger 신호( PB3 : 출력포트 설정  )
    PORTB &= ~0x08;      // PB3  : Low  ( Trigger 신호 OFF )  
    DDRE &= ~0x40;       // Echo 신호( 외부인터럽트6, INT6(PE6) : 입력포트 설정 )

    ////////////  Timer 0 설정  ( 10 msec 주기의 타이머0 오버플로 인터럽트 설정 )  ///////////////
    // 교재 P133-137(레지스터 그림과 표8.1-표8.5) 참조    

    TCCR0 &= ~0x48;         // Normal mode(타이머모드), 타이머 0 시작(1024분주 사용)
                            // TCCR0 = 0b*0**0111 
                            // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;          // TCCR0 =TCCR0 | 0b00000111( 0x07 ) 
	 
    TCNT0 = 256 - 156;      // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                            // 오버플로인터럽트 주기 = 10msec
                            // 156 = 10msec/ 64usec,  TCNT0 = 256 - 156 

    TIMSK |= 0x01;          // 타이머0 오버플로 인터럽트 허용 

    ////////////////////////////////////////////////////////////////////////////////////////////    
    // Echo 신호 펄스폭 시간 측정을 위한 Timer 3 설정
    // 교재 PP238-244(레지스터 그림과 표12.4 - 표12.5 참조) 

    TCCR3A &= ~0x03;         // Normal mode(타이머모드), 타이머 3 시작(분주비 8) 
                             // 내부클럭주기 = 8/ (16x10^6) = 0.5 usec (0.5usec 단위로 측정) 
                             // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                             // TCCR3A = TCCR3A & ~0b00000011(~0x03)   
    TCCR3B &= ~0x1D;         // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;         // TCCR3B = TCCR3B | 0b00000010( 0x02 )  
	 
   //////////////////////////////////////////////////////////////////////////////////////////
   // 외부인터럽트 6( pin: INT6/PE6 ) :  초음파센서모듈 Echo 신호 입력 
   // 교재 pp108-109 (레지스터 그림, 표 7.4-표7.5 참조 )

    EICRB &= ~0x20;        // INT6 : 하강에지(falling edge) 상승에지(rising edge) 모두에서 인터럽트 요구
                           // EICRB = 0b**01**** 
                           // EICRB = EICRB & ~0b00100010(~0x20 ) 
    EICRB |=   0x10;       // EICRB = EICRB | 0b00010000( 0x10 ) 
 
    EIMSK |= 0x40;         // INT6 Enable(허용) 
    sei();                 // 전역 인터럽트 허용

///////////////////////////////////////////////////////////////////////////////////
	 
    while (1) 
    { 

       cli();                           // 전역인터럽트 금지 

 	   dist_1 = distance_1 ;            // 거리측정값을 변수 dist_1에 저장 

       sei();                           // 전역인터럽트 허용 

	   LcdMove(1, 9);                   // LCD에 쓸 데이터 초기 위치 설정 (1행, 9열)
       Display_Number_LCD(dist_1, 3);   // 거리측정값(변수 야st_1)을 100자리까지 디스플레이 

   }

}      // int main() 함수의 끝 

///////////////////////////////////////////////////////////////

ISR( TIMER0_OVF_vect )     //  10 msec 주기의 타이머0 오버플로 인터럽트 서비스프로그램
{                          //  50msec 마다 초음파신호 발사 요청신호(Trigger 신호) 출력

    static unsigned short  time_index = 0; 


    TCNT0 = 256 - 156;     //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                           //  오버플로인터럽트 주기 = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156

    time_index++ ; 

    if( time_index == 5 )   // 50 msec (=10msec * 5) 마다 
    {

       time_index = 0;    // 초기화

       //  초음파센서 트리거 신호 출력(초음파 발사) 
	   PORTB |= 0x08;      // PA0 : High
	   usec_delay(20);     // 20usec 동안 High 유지 
	   PORTB &= ~0x08;     // PA0 : Low    

       flag = 0;   
   }

}

//////////////////////////////////////////////////////////

ISR( INT6_vect )     // 외부인터럽트 6(INT6) 서비스 프로그램
{                    // Echo 신호 펄스폭의 시간 측정 및 전방 장애물까지의 거리 계산 

      static unsigned short  count1 = 0, count2 = 0, del_T = 0; 


	  if( flag == 0 )        // Echo신호의 상승에지에서 인터럽트 걸릴 때 
	  {

		  count1 = TCNT3;    // 상승에지에서의 카운터레지스터값(TCNT3) 저장 
		  flag = 1;          // flag 변수 1로 설정

	  } 
	  else if( flag == 1 )    // Echo신호의 하강에지에서 인터럽트 걸릴 때
	  { 

		  count2 = TCNT3;                     // 하강에지에서의 카운터레지스터값(TCNT3) 저장 
		  del_T = ( count2 - count1 ) / 2;    // Echo 신호 펄스폭의 시간 측정(usec 단위)
    	  distance_1 = del_T / 58;            // 전방 장애물까지의 거리계산(cm 단위)

          if( distance_1 > 380 )              // 반사되는 초음파가 검출되지 않을때 
		  {
		      distance_1 = distance_1_prev ;  // 직전 거리측정값 사용 
		  } 

          distance_1_prev = distance_1;       // 직전 거리측정값 저장 변수 업데이트  
		  flag = 0;                           // flag 변수 0으로 설정 
	  } 

} 

////////////////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned int num, unsigned char digit )   //부호없는 정수형변수(num)을 10진수 형태로 
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



