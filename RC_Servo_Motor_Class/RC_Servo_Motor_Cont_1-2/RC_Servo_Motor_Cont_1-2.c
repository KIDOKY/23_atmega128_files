
/////  < RC_Servo_Motor_Cont_1.c >  : Key(5개)를 이용하여 서보모터(2개)의 위치를 제어하는 ATmega128 코드  ///// 

#include <avr/io.h>         // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함.

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.

// Key 관련 심볼 정의
#define  NO_KEY  0
#define  KEY1     1
#define  KEY2     2

#define  KEY3     3

// 서보모터 회전방향 관련 심볼 정의
#define  FORWARD    0
#define  BACKWARD   1 

// 서보모터 동작모드 관련 심볼 정의
#define  MANUAL_MODE     0
#define  SEQUENCE_MODE   1 

unsigned char Key_Input(void); // 함수 호출시 새롭게 눌러진 key 값을 리턴함(채터링 방지 기능포함.)
                               // 눌러진 key가 없을 시 0(NO_KEY) 이 리턴됨. 한번 호출시 마다 50 msec 이상 시간이 소요. 
unsigned char key_scan(void);  // Key_Input( ) 함수에서 호출하는 내부 함수. 실제로 어느 HW 입력포트에 연결된 
                               // push 스위치가 눌러졌는지 체크(스캔)하는 함수 임. 
                               // key와 관련된 HW 변경 시 (key를 다른 입력포트에 연결할 때) 반드시 수정해야함.

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // 부호없는 정수형 변수를 10진수 형태로 
                                                                     // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연

unsigned char Time_Delay_Polling( unsigned short d_time );   // 시간경과 체크함수(폴링방식)

void Servo1_Move( short sv_pos_cmd );    // 서보모터1을 주어진 각도(sv_pos_cmd) 만큼 회전. 0도 - 180도만 가능



static volatile short   PWM_Period = 0, Servo1_Position = 0 ;   
static volatile unsigned short  curr_delay = 0;    // 시간경과 체크함수에서 사용되는 카운터 변수 
static volatile unsigned char  Step = 0, Servo1_Direction = 0 ; 
static volatile unsigned char  Operating_Mode = MANUAL_MODE,  Key_cnt = 0;


int main() 
{   

    unsigned char   Key_Data = 0 ;

    LcdInit();                      // LCd 초기화 함수 
    LcdMove(0,0);                   // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("Servo Control");       // LCD에 서보모터제어 메시지 디스플레이 
    LcdMove(1,0);                   // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Test Program");        // LCD에 서보모터제어 메시지 디스플레이
    msec_delay(2000);               // 2초(2000msec) 시간지연

   ////////  Key 5개 사용을 위한 HW 관련 레지스터 설정  ////////////
   //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA &= ~0x30;       // Key1(PA4), Key2(PA5): 입력포트로 설정 
                         // 입력포트 설정.  DDRA = 0b0000 ****, DDRA = DDRA & ~0b1111 0000(~0xF0)
    PORTA |= 0x30;       // Key1(PA4), Key2(PA5): 내부풀업사용모드로 설정  
                         // PORTA = 0b1111 ****, PORTA = PORTA | 0b1111 0000( 0xF0 )

    DDRB &= ~0x01;       // Key3(PB0) : 입력포트로 설정   
                         // 입력포트 설정.  DDRB = 0b**** ***0, DDRB = DDRB & ~0b0000 0001(~0x01)
    PORTB |= 0x01;       // Key3(PB0)  : 내부풀업사용모드로 설정  
                         // PORTB = 0b**** ***1, PORTB = PORTB | 0b0000 0001( 0x01 )

   // (주의):  Key 입력포트 설정을 변경하면 key_scan() 함수도 변경해야함. key_scan() 함수 변경 내용을 확인할 것. 

   ////////  서보모터 2개 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)    

    DDRE |= 0x08;      // 서보모터1 위치제어 PWM포트(OC3A/PE3)
                       // 출력포트로 설정   
                       // DDRE = 0b***1 1***, DDRE = DDRE | 0b0001 1000( 0x18 )
    DDRA |= 0x04;      // LED1(서보모터1 동작상태표시)(PA2) : 출력포트로설정 // DDRA = 0b**** 11**, DDRA = DDRA | 0b0000 1100( 0x0C )

   /////////////  Timer3 설정( 서보모터1 PWM신호(OC3A/PE3), 서보모터2 PWM신호(OC3B/PE4) 발생 )   //////////// 
   ////////////   PWM신호 주파수 = 50Hz (주기 = 20msec )  //////////////////////
   /////// 교재 P238-245(레지스터 그림과 표12.2, 표12.4, 표12.5) 참조    

    TCCR3A &= ~0x41;     // Fast PWM: 비교일치시 OC3A/PE3, OC3B/PE4 핀을 0으로 하고 TOP에서 1을 출력
                         // (표12.2 참조),   Fast PWM ( mode 14 ) 설정 (표12.4 참조)
                         // TCCR3A = 0b1010 **10 
                         // TCCR3A = TCCR3A & ~0b0101 0001(~0x51 )  
    TCCR3A |= 0x82;      // TCCR3A = TCCR3A |   0b1010 0010( 0xA2 )  

    TCCR3B &= ~0x03;     // 256 분주 타이머3 시작(내부클럭 주기=256/(16*10^6Hz)=16usec ), 
                         // Fast PWM(mode 14)설정  (표12.4 - 표12.5 참조)
                         // TCCR3B = 0b***1 1100   
                         // TCCR3B = TCCR3B & ~0b0000 0011(~0x03 )  
    TCCR3B |= 0x1C;      // TCCR3B = TCCR3B |   0b0001 1100( 0x1C )  

    ICR3 = 1250;         // PWM 주파수=50Hz(PWM주기 = 1/50Hz = 20msec)
                         // PWM 주기(주파수) 설정( 1250 = 20msec(PWM주기)/16usec(256분주된 내부클럭주기),
						  
    PWM_Period = ICR3;   // PWM 신호 주기를 전역변수에 저장 

    Servo1_Position = 90;  
    Servo1_Move( Servo1_Position );       // 서보모터1 가운데(90도) 위치로 회전 
   

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


    while (1) 
    { 

       Key_Data = Key_Input( );           // 함수 Key_Input( )호출. 어떤 Key를 누르는지 체크  

       if( Key_Data == KEY1 && Operating_Mode == MANUAL_MODE )  // 매뉴얼모드에서 Key 1을 누르면
       {  
 	       Servo1_Position += 10;                               // 서보모터1 위치명령(Servo1_Position)을 10도씩 증가
           if( Servo1_Position > 180 )  Servo1_Position = 180;  // 최대 범위를 벗어나면 180도로 설정 

           Servo1_Move( Servo1_Position );                      // 서보모터1을 Servo1_Position 으로 회전

           Servo1_Direction  = FORWARD;                         // 서보모터1 정방향 회전상태
       }

       else if( Key_Data == KEY2 && Operating_Mode == MANUAL_MODE )  // 매뉴얼모드에서 Key 2를 누르면
       {  
 	       Servo1_Position -= 10;                           // 서보모터1 위치명령(Servo1_Position)을 10도씩 감소  
           if( Servo1_Position < 0 )  Servo1_Position = 0;  // 최소 범위를 벗어나면 0도로 설정 

           Servo1_Move( Servo1_Position );                  // 서보모터1을 Servo1_Position 으로 회전 

           Servo1_Direction  = BACKWARD;                    // 서보모터1 역방향 회전상태
       }

      
       else if( Key_Data == KEY3 )     // Key 5를 누르면 서보모터1, 2의 동작모드를 변경(시퀀스모드, 매뉴얼모드)
       {  
           Key_cnt++;   
           if( Key_cnt >= 2 ) Key_cnt = 0; 

           if( Key_cnt == 0 ) Operating_Mode = MANUAL_MODE;     // Key5가 짝수번 눌러지면 매뉴얼모드 
           if( Key_cnt == 1 ) Operating_Mode = SEQUENCE_MODE;   // Key5가 홀수번 눌러지면 시퀀스모드 

           if( Operating_Mode == SEQUENCE_MODE ) //서보모터1,2가 시퀀스모드 이면 관련변수초기화 
           {         
               Step = 0 ;           // 서보모터1, 서보모터2 동작시퀀스 단계 초기화 
               curr_delay = 0;      // 시간경과 체크함수에서 사용되는 카운터 변수 초기화 
           }
           else if( Operating_Mode == MANUAL_MODE )  //서보모터1,2가 매뉴얼모드 이면  
           {         
              Servo1_Position = 90;                // 서보모터1 위치명령(Servo1_Position)을 90도로 설정
              Servo1_Move( Servo1_Position );      // 서보모터 1을 가운데 위치(90도 위치)로 회전
              PORTA |=  0x04;                      // LED1 OFF ( PA2 = 1 ) : 서보모터1 정지상태로 표시
                                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 ) 

              
           }
       }


       ///////  LCD에 서보모터1, 서보모터2 동작상태 디스플레이  ///////////
       LcdCommand( ALLCLR );                          // LCD 화면 지우기

	   LcdMove(0, 0);                                 // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 

       LcdPuts( "Servo Position" );                        // 서보모터1 위치 메시지 디스플레이 
       Display_Number_LCD( Servo1_Position, 3 );      // 서보모터1 위치 3자리(000-180) 디스플레이 

       if( Servo1_Direction == FORWARD )        LcdPuts( " Fwd" );   // 서보모터1이 정방향 회전상태 이면 
                                                                     // 정방향 회전 메시지 디스플레이 
       else if( Servo1_Direction == BACKWARD )  LcdPuts( " Rev" );   // 서보모터1이 역방향 회전상태 이면 
                                                                     // 역방향 회전 메시지 디스플레이 

	  

    }   // 무한루프 while (1) 의 끝 

}      // int main() 함수의 끝 


///////////////////////////////////////////////////


ISR( TIMER2_OVF_vect )   // 10msec 주기의 타이머2 오버플로 인터럽트 서비스프로그램
{                        // 동작모드가 시퀀스모드이면 서보모터1, 서보모터2의 순차적이고 반복적인 동작시퀀스 실행 

  TCNT2 = 256 - 156;          // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

  ///////  서보모터1, 서보모터2 동작 시퀀스 ////////////

  if( Operating_Mode == SEQUENCE_MODE )    // 동작모드가 시퀀스모드이면 
  {
      if( Step == 0 )                      // 초기위치로 복귀단계 (Step 0는 초기에 1번만 실행 됨)
      {

        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2초가 경과되지 않았으면 (2초 동안 회전)
        { 
           Servo1_Direction = BACKWARD;      // 서보모터1 역방향 회전상태 
           Servo1_Position = 0;              // 서보모터1 위치명령(Servo1_Position)을 0도로 설정
           Servo1_Move( Servo1_Position );   // 서보모터1 0도(초기시작위치)로 복귀 

           PORTA &=  ~0x04;                  // LED1 ON ( PA2 = 0 ) : 서보모터1 회전상태 표시
                                             // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

          

           
         }
         else                            // 2초 경과 후 
         {
           Step++;                      // Step1 로 넘어감. 
         }
      }

      if( Step == 1 )      
	  {

        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2초가 경과되지 않았으면 (2초 동안 회전)
        { 
           Servo1_Direction = FORWARD;      // 서보모터1 정방향 회전상태 
           Servo1_Position = 180;           // 서보모터1 위치명령(Servo1_Position)을 180도로 설정
           Servo1_Move( Servo1_Position );  // 서보모터1 0도 --> 180도 로 회전 

           PORTA &=  ~0x04;                 // LED1 ON ( PA2 = 0 ) : 서보모터1 회전상태 표시
                                            // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
 
           PORTA |=  0x08;                  // LED2 OFF ( PA3 = 1 ) : 서보모터2 정지상태 표시
                                            // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                                // 2초 경과 후 
        {
           Step=0;                          // Step2 로 넘어감. 
        }
      }

     

     

      
      

    }   // End of  if( Operating_Mode = SEQUENCE_MODE )

}       // End of  ISR( TIMER2_OVF_vect ) 

////////////////////////////////////////////////////////

void Servo1_Move( short sv_pos_cmd )    // 서보모터1을 주어진 각도(sv_pos_cmd) 만큼 회전. 0도 - 180도만 가능
{
      if( sv_pos_cmd > 180 )   sv_pos_cmd = 180;   // 허용범위(0도 - 180도)를 벗어나지 않도록 설정 
      if( sv_pos_cmd < 0 )     sv_pos_cmd = 0; 

      OCR3A = ( 5 * sv_pos_cmd )/9  + 44 ;       // OC3A(PE3)핀으로 출력되는 PWM 신호의 펄스폭(duty) 결정 
                                                 // 레지스터값 설정 

      //  펄스폭 = 0.7msec = 16usec * 44,   왼쪽 끝(0 도)  (펄스폭 = 0.7msec )
      //  펄스폭 = 1.5msec = 16usec * 94 ,  가운데(90 도) (펄스폭 = 1.5msec )
      //  펄스폭 = 2.3msec = 16usec * 144 , 오른쪽 끝(180 도) (펄스폭 = 2.3msec ) 
}

void Servo2_Move( short sv_pos_cmd )   // 서보모터2를 주어진 각도(sv_pos_cmd) 만큼 회전. 0도 - 180도만 가능
{
      if( sv_pos_cmd > 180 )   sv_pos_cmd = 180;   // 허용범위(0도 - 180도)를 벗어나지 않도록 설정 
      if( sv_pos_cmd < 0 )     sv_pos_cmd = 0; 

      OCR3B = ( 5 * sv_pos_cmd )/9  + 44 ;       // OC3B(PE4)핀으로 출력되는 PWM 신호의 펄스폭(duty) 결정 
                                                 // 레지스터값 설정 

      //  펄스폭 = 0.7msec = 16usec * 44,   왼쪽 끝(0 도)  (펄스폭 = 0.7msec )
      //  펄스폭 = 1.5msec = 16usec * 94 ,  가운데(90 도) (펄스폭 = 1.5msec )
      //  펄스폭 = 2.3msec = 16usec * 144 , 오른쪽 끝(180 도) (펄스폭 = 2.3msec ) 

}

/////////////////////////////////////////////////////

unsigned char Time_Delay_Polling( unsigned short d_time )  // 시간경과 체크함수(폴링방식)
{
    unsigned char  ret_val = 0;

    curr_delay++ ;  
    if( curr_delay >= d_time )   // 50msec * d_time 경과 후 
    {
       ret_val = 1; 
       curr_delay = 0 ;
    } 
    return  ret_val ;
}


/////////////////////////////////////                                                              
unsigned char Key_Input(void)   // 함수 호출시 새롭게 눌러진 key 값을 리턴함(채터링 방지 기능포함.)
{                               // 눌러진 key가 없을 시 0(NO_KEY) 이 리턴됨. 한번 호출시 마다 50 msec 이상 시간이 소요. 

    static unsigned char pin = NO_KEY; 
    unsigned char in, in1;

    in = key_scan();             // key 입력 읽기 
    while(1)                     // key 입력이 안정화될때까지 기다림
    {
        msec_delay(50);          // 디바운싱 시간 지연     
        in1 = key_scan();        // key 입력 읽기     
        if( in == in1 ) break;
        in = in1;
    }
    if( !( in & 0xFF ) )         // key가 눌려지지 않고 있으면  0 ( NO_KEY) 리턴
    {	
        pin= NO_KEY;				
        return NO_KEY;
    }
    if( pin == in )              // 동일한 key가 계속 눌려지고 있으면  0 ( NO_KEY) 리턴
    {
        return NO_KEY;  
    }
    pin = in;

    return  in;
}


//////////////////////////////////////////
unsigned char key_scan(void)   // Key_Input( ) 함수에서 호출하는 내부 함수. 실제로 어느 HW 입력포트에 연결된 
{                              // push 스위치가 눌러졌는지 체크(스캔)하는 함수 임. 
                               // key와 관련된 HW 변경 시 (key를 다른 입력포트에 연결할 때) 반드시 수정해야함.

    unsigned char   in = 0 ;
    
    if (       ( ~PINA  & 0x10  ) == 0x10  )        in = KEY1 ; 	  //   Key1 (PA4) 이 눌러지면    	
    else if (  ( ~PINA  & 0x20  ) == 0x20  )        in = KEY2 ; 	  //   Key2 (PA5) 가 눌러지면   		 
    else if (  ( ~PINA  & 0x40  ) == 0x40  )        in = KEY3 ; 	  //   Key3 (PA6) 이 눌러지면 
        	 
    else                                             in = NO_KEY;   //	Key가 눌러지지 않으면 
    
    return   in ;
    
}


void Display_Number_LCD( unsigned short num, unsigned char digit ) //부호없는 정수형변수(num)을 10진수 형태로 
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
{       //  변환하여 각각의 자릿수를 전역변수 배열 cnumber[0](1자리) - cnumber[4](10000자리)에 저장하는 함수. 
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

void msec_delay(int n)           // n msec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		         // 1msec 시간 지연을 n회 반복
		_delay_ms(1);		     // 1msec 시간 지연
}

void usec_delay(int n)            // n usec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		         // 1usec 시간 지연을 n회 반복
		_delay_us(1);		     // 1usec 시간 지연
}

