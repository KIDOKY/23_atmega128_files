
// < ultrasonic_sensor_2.c >  :   2개의 초음파센서모듈(HC-SR04)을 동시에 구동하는 ATmega128 소스 코드 

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 
#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.

#define NO_KEY    0


#define KEY1      1  
#define KEY2      2
#define KEY3      3            
 

// 함수 정의

unsigned char key_scan(void);

unsigned char KeyInput(void);


void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // 부호없는 정수형 변수를 10진수 형태로 
                                                                      // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);    // msec 단위 시간지연
void usec_delay(int n);    // usec 단위 시간지연

unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // 시간지연 체크함수(폴링방식)

static volatile unsigned short    distance_1 = 0, distance_2 = 0, distance_min = 300; 
static volatile unsigned short    distance_1_prev = 0, distance_2_prev = 0; 
static volatile unsigned char     sensor_count = 0, active_sensor_flag = 0;
static volatile  unsigned char    Warning_Flag = 0, flag = 0 ;
static volatile  unsigned short   Delay_Time = 0, curr_delay = 0;


int main() 
{   

    unsigned short  dist_1 = 0,  dist_2 = 0, pressed_key = 0, cnt = 0;


    LcdInit();                     // LCd 초기화 함수 호출

    LcdMove(0,0);                  // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("Dist_1 =    cm");     // LCD에 문자열 디스플레이 

    LcdMove(1,0);                  // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)  
    LcdPuts("Dist_2 =    cm");     // LCD에 문자열 디스플레이 

   //// 2개의 초음파센서모듈(HC-SR04) 구동을 위한 HW 관련 레지스터 설정  ////////////

  //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA |= 0x03;        // 2개의 초음파센서 Trigger 신호( Trigger신호1 : PA0, Trigger신호2 : PA1 )출력포트 설정. 
                         // DDRA = 0b******11, DDRA = DDRA | 0b00000011(0x03)   
    PORTA &= ~0x03;      // PA0 : Low,  PA1 : Low  ( Trigger 신호 OFF )  
                         // PORTA = 0b******00, PORTA = PORTA & ~0b00000011(~0x03)  

    DDRE &= ~0x30;       // 2개의 초음파센서 Echo 신호( 외부인터럽트4(INT4/PE4), 외부인터럽트5(INT5/PE5) 
                         // 입력포트 설정.  DDRE = 0b**00****, DDRE = DDRE & ~0b00110000(~0x30) 
						   
    DDRA |= 0x08;        // 버저(Buzzer) ( PA3 : 출력포트 설정 )
                         // DDRA = 0b****1***, DDRA = DDRA | 0b00001000(0x08)   
    PORTA &= ~0x08;      // PA3  : Low  ( 버저 OFF )  
                         // PORTA = 0b****0***, PORTA = PORTA | ~0b00001000(~0x08) 
						  
    DDRA |= 0x10;        // LED ( PA4 : 출력포트 설정 )
                         // DDRA = 0b***1****, DDRA = DDRA | 0b00010000(0x10)   
    PORTA |= 0x10;       // PA4  : High ( LED OFF)    
                         // PORTA = 0b***1****, PORTA = PORTA | 0b00010000(0x10)  
	
	DDRB &= ~0x01;        // Push SW ( PB0 : 입력포트 설정 )
                          // DDRB = 0b*******0, DDRA = DDRA & 0b00000001(~0x01)   
    PORTB |= 0x01;        // PORTB : 0b*******1	:	High 내부풀압저항 사용모드로 설정    
                          // PORTB = PORTB | 0b00010000(0x10)

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

    ////////////////////////////////////////////////////////////////////////////     
    // Echo 신호 펄스폭 시간 측정을 위한 Timer 3 설정
   // 교재 PP238-244(레지스터 그림과 표12.4 - 표12.5 참조) 

    TCCR3A &= ~0x03;      // Normal mode(타이머모드), 타이머 3 시작(분주비 8) 
                          // 내부클럭주기 = 8/ (16x10^6) = 0.5 usec (0.5usec 단위로 측정) 
                          // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                          // TCCR3A = TCCR3A & ~0b00000011(~0x03)  
						   
    TCCR3B &= ~0x1D;      // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;      // TCCR3B = TCCR3B | 0b00000010( 0x02 )  

   ///////////////////////////////////////////////////////////////////////////// 

   // 외부인터럽트 4( pin: INT4/PE4 ) 설정 :  초음파센서모듈 Echo 신호1가 입력됨.    
   // 외부인터럽트 5( pin: INT5/PE5 ) 설정 :  초음파센서모듈 Echo 신호2가 입력됨.  
   // 교재 pp108-109 (레지스터 그림, 표 7.4-표7.5 참조 )

    EICRB &= ~0x0A;       // INT4, INT5 : 하강에지(falling edge) 상승에지(rising edge) 모두에서 인터럽트 요구
                          // EICRB = 0b****0101 
                          // EICRB = EICRB & ~0b00001010(~0x0A ) 
    EICRB |=   0x05;      // EICRB = EICRB | 0b00000101( 0x05 ) 

    EIMSK |= 0x30;        // INT4, INT5 Enable(허용) 
                          // EIMSK = 0b**11****,  EIMSK = EIMSK | 0b00110000( 0x30 ) 
    sei();                // 전역 인터럽트 허용
 
	 
    while (1) 
    {
	   pressed_key = KeyInput();
	   
	   if(	pressed_key == KEY1 )
	   {
	   		cnt++;
			if( cnt == 2 ) cnt = 0;

			if( cnt == 1 )		//Key1 이  홀수번 눌러지면
			{
				//TIMSK |= 0x01;		    // 타이머0 오버플로 인터럽트 허용
				sei();
				
				TCNT0 = 256 - 156;      // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,
										// 오버플로인터럽트 주기 = 10msec
                              
				EIMSK |= 0x30;        // INT4, INT5 Enable(허용) 
                          			  // EIMSK = 0b**11****,  EIMSK = EIMSK | 0b00110000( 0x30 ) 
				flag = 0;
				sensor_count = 0;
				active_sensor_flag = 0;
				Warning_Flag = 0;
				Delay_Time = 0;
				curr_delay = 0;
				PORTA &= ~0x01;

			}
			else if( cnt == 0 )	//Key1 이 짝수번 눌러지면
			{
			    cli();

				//TIMSK &= ~0x01; // 타이머0 오버플로 인터럽트금지(초음파신호를 쏘지 않음)

				//EIMSK &= ~0x30;        // INT4, INT5 Enable(허용) 
                          			   // EIMSK = 0b**00****,  EIMSK = EIMSK & 0b00110000( 0x30 ) 

				PORTA &= ~0x08;		   // PA3(버저) OFF : 버저 OFF
				PORTA |= 0x10;		   // PA4(LED) OFF : LED OFF
			}
	   
	   }
	   else if(	pressed_key == KEY2 )
	   {
	   		cnt++;
			if( cnt1 == 3 ) cnt1 = 0;

			if( cnt1 == 0 )		//Key2가 1번 눌러지면
			{
				
			}

			else if( cnt1 == 1 ) // Key2가 2번 눌러지면
			{

			}
			else if( cnt1 == 2 )  //Key2가 3번 눌러지면 
			{

			}
	   }

       cli();                            // 전역인터럽트 금지

 	   dist_1 = distance_1 ;            // 초음파센서모듈1이 측정한 거리측정값을 변수 dist_1에 저장 
 	   dist_2 = distance_2 ;            // 초음파센서모듈2가 측정한 거리측정값을 변수 dist_2에 저장 

       sei();                           // 전역인터럽트 허용

	   LcdMove(0, 9);                   // LCD에 쓸 데이터 초기 위치 설정 (0행, 9열)
       Display_Number_LCD(dist_1, 3);   // 초음파센서모듈1이 측정한 거리측정값을 100자리까지 디스플레이 
       LcdMove(1, 9);                   // LCD에 쓸 데이터 초기 위치 설정 (1행, 9열)
       Display_Number_LCD(dist_2, 3);   // 초음파센서모듈2가 측정한 거리측정값을 100자리까지 디스플레이 

    } 

}      // int main() 함수의 끝 




ISR( TIMER0_OVF_vect )     //  10 msec 주기의 타이머0 오버플로 인터럽트 서비스프로그램
{                          //  50msec 마다 초음파신호 발사 요청신호(Trigger 신호1, Trigger 신호2) 출력
                           //  전방 장애물의 거리가 가까우면 경고 버저음(단속음) 발생 , LED 깜박임.

    static unsigned short  time_index = 0 ; 


    TCNT0 = 256 - 156;     //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                           //  오버플로인터럽트 주기 = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156

    time_index++ ; 

    if( time_index == 5 )   // 50 msec (=10msec * 5) 마다 
    {

       time_index = 0;      // 초기화

       sensor_count++;          // 초음파 센서 카운터 값 증가        
	   if( sensor_count == 3 )  sensor_count = 1; 

       if ( sensor_count == 1 )        //  초음파센서 트리거 신호1 발생(초음파 1 발사) 
	   {

	      PORTA |= 0x01;       // PA0 : High
	      usec_delay(20);      // 20usec 동안 High 유지 
	      PORTA &= ~0x01;      // PA0 : Low   
     
	      active_sensor_flag = 1; 
		  flag = 0;

	   }
       else if ( sensor_count == 2 )   //  초음파센서 트리거 신호2 발생(초음파 2 발사)
	   {

	      PORTA |= 0x02;       // PA1 : High
	      usec_delay(20) ;     // 20usec 동안 High 유지 
	      PORTA &= ~0x02;      // PA1 : Low 

	      active_sensor_flag = 2;
		  flag = 0;

	   }

       ////////  경고음 발생   ///////////// 

       // 초음파센서모듈 1과 2가 측정한 거리 중 최솟값 저장 
       if( distance_1 <= distance_2  && distance_1 != 0 )       distance_min = distance_1;
       else if( distance_2 < distance_1  && distance_2 != 0 )   distance_min = distance_2;

       if( distance_min <=  40 ) Warning_Flag = 1 ;  // 측정된 거리가 40 cm 이하이면 경고음 발생 플래그 set
       else                      Warning_Flag = 0 ;    
		
       Delay_Time = distance_min /10 + 1;  //거리에 비례하는 주기(=Delay_Time*50 msec )를 갖는 경고음 발생

	   if( Delay_Time <= 1)   Delay_Time = 1 ;   // 경고음주기 하한 : 0.1초
	   if( Delay_Time >= 4)   Delay_Time = 4 ;   // 경고음주기 상한 : 0.4초 
 
       if( Warning_Flag == 1 )      // 전방 장애물의 거리가 가까우면 경고 버저음(단속음) 발생 , LED 깜박임. 
	   {
           if( Time_Delay_Polling( Delay_Time ) == 1 )     // 50msec * Delay_Time 경과 후 
	       {
                 PORTA ^= 0x08;    // PA3(버저) toggle :  버저 단속음 
		         PORTA ^= 0x10;    // PA4(LED) toggle :  LED ON, OFF 반복 
	       }
	   }
       else if( Warning_Flag == 0 )  // 전방 장애물의 거리가 가깝지 않으면 버저 OFF , LED OFF. 
	   {
            PORTA &= ~0x08;     // PA3(버저) OFF : 버저 OFF 
		    PORTA |= 0x10;      // PA4(LED) OFF :  LED  OFF 
	   }
      
   }

}


//////////////////////////////////////////////////////////////////



ISR( INT4_vect )     // 외부인터럽트 4(INT4) 서비스 프로그램
{                    // Echo 신호1 펄스폭의 시간 측정 및 전방 장애물까지의 거리 계산 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0 ;

    if ( active_sensor_flag == 1 )
    {

	  if( flag == 0 )          // Echo 신호1의 상승에지에서 인터럽트 걸릴 때 
	  {
		  count1 = TCNT3;      // 상승에지에서의 카운터레지스터값(TCNT3) 저장 
		  flag = 1;            // flag 변수 1로 설정
	  } 
	  else if( flag == 1 )     // Echo 신호1의 하강에지에서 인터럽트 걸릴 때
	  { 
		  count2 = TCNT3;                    // 하강에지에서의 카운터레지스터값(TCNT3) 저장 
		  del_T = ( count2 - count1 ) / 2 ;  // Echo 신호1 펄스폭의 시간 측정(usec 단위)
    	  distance_1 = del_T / 58;           // 초음파센서모듈1이 측정한 전방 장애물까지의 거리계산(cm 단위)

          if( distance_1 > 380 )  distance_1 = distance_1_prev ;   // 반사되는 초음파가 검출되지 않을때 
		                                                           // 직전 거리측정값 사용 
          distance_1_prev = distance_1;       // 직전 거리측정값 저장 변수 업데이트  
		  flag = 0;                           // flag 변수 0으로 설정 
          active_sensor_flag = 0;             // active_sensor_flag 변수 리셋
	   } 

    }
} 


ISR( INT5_vect )     // 외부인터럽트 5(INT5) 서비스 프로그램
{                    // Echo 신호2 펄스폭의 시간 측정 및 전방 장애물까지의 거리 계산 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0 ;

    if ( active_sensor_flag == 2 )
    {

	  if( flag == 0 )          // Echo신호2의 상승에지에서 인터럽트 걸릴 때 
	  {
		  count1 = TCNT3;      // 상승에지에서의 카운터레지스터값(TCNT3) 저장 
		  flag = 1;            // flag 변수 1로 설정
	  } 
	  else if( flag == 1 )     // Echo신호2의 하강에지에서 인터럽트 걸릴 때
	  { 
		  count2 = TCNT3;                    // 하강에지에서의 카운터레지스터값(TCNT3) 저장 
		  del_T = ( count2 - count1 ) / 2 ;  // Echo 신호2 펄스폭의 시간 측정(usec 단위)
    	  distance_2 = del_T / 58;           // 초음파센서모듈2가 측정한 전방 장애물까지의 거리계산(cm 단위)

          if( distance_2 > 380 )  distance_2 = distance_2_prev ;   // 반사되는 초음파가 검출되지 않을때 
		                                                           // 직전 거리측정값 사용 
          distance_2_prev = distance_2;       // 직전 거리측정값 저장 변수 업데이트  
		  flag = 0;                           // flag 변수 0으로 설정 
          active_sensor_flag = 0;             // active_sensor_flag 변수 리셋
	  } 

    }
}
 
///////////////////////////////////////////////////////////////////

//==============================================
// 키패드의 스캔 코드값을 반환한다.
//
// 리턴 값 : 0이 아닐 때 : 스캔 코드값
//           0일 때      : 입력값이 없음
//==============================================
unsigned char KeyInput(void)
{
    unsigned char in, in1;

    static unsigned char pin = NO_KEY;

    
    in = key_scan();            // key 입력 읽기 
    
    while(1)               // key 입력이 안정화될때까지 기다림
    {
        //디바운싱 시간 지연
        msec_delay(10); msec_delay(10);  msec_delay(10);
        
        in1 = key_scan();         // key 입력 읽기    
        
        if(in==in1) break;
        in = in1;
    }
    
    if(!(in & 0xFF))    // key를 누르지 않고 있으면  0 ( NO_KEY) 리턴
    {
        pin= 0;			
        return 0;
    }
    
    if(pin == in)     // 동일한 key를 계속 누르고 있으면  0 ( NO_KEY) 리턴
    {
        return 0;  
    }
    
    pin = in;
    
    return in;
}

 
//----------------------------------------
//스캔 코드값을 리턴함
//----------------------------------------

unsigned char key_scan(void)
{
  
    unsigned char   in = 0 ;
    
    if (       ! ( PINB  & 0x01  )  )        in = KEY1 ; 	  //   PB0 key scan 	
    else if (  ! ( PINB  & 0x02  )  )        in = KEY2 ; 	  //   PB1 key scan  		
    else if (  ! ( PINB  & 0x04  )  )        in = KEY3 ; 	  //   PB2 key scan  	

    else                                     in = NO_KEY ; 	
    
    return in ;
    
}

///////////////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned int num, unsigned char digit )   //부호없는 정수형변수(num)을 10진수 형태로 
{                                                                  // 정해진 자릿수(digit) 만큼 LCD 에 디스플레이 하는 함수 
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

char NumToAsc( unsigned char Num )   // Num으로 넘어온 16진수 1자리 숫자를 문자데이터(ASCII 코드)로 
{                                        // 변환하여 리턴하는 함수
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 
	return Num ;
}

unsigned char Time_Delay_Polling( unsigned short d_time )
{
    static unsigned short  curr_delay = 0; 
    unsigned char  ret_val = 0;

    curr_delay++ ;  
    if( curr_delay >= d_time )   // 50msec * d_time 경과 후 
    {
       ret_val = 1; 
       curr_delay = 0 ;
    } 
    return  ret_val ;
}

////////////////////////////////////////////////////

void msec_delay(int n)               // n msec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)  _delay_ms(1);	 // 1msec 시간 지연을 n회 반복
}

void usec_delay(int n)               // n usec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)   _delay_us(1);	 // 1usec 시간 지연을 n회 반복
}

