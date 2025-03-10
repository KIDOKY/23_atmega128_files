
/////  < DC_Motor_Cont_2.c >  : Key(5개)를 이용하여 DC모터(2개)의 속도를 조절하는 ATmega128 코드  ///// 

#include <avr/io.h>         // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.

// Key 관련 심볼 정의
#define  NO_KEY  0
#define  KEY1     1
#define  KEY2     2
#define  KEY3     3
#define  KEY4     4
#define  KEY5     5

// 모터 동작상태(회전방향, 정지) 관련 심볼 정의
#define  FORWARD   0
#define  REVERSE    1 
#define  STOP        2

// 모터 동작시퀀스 관련 심볼 정의
#define  STOP_MODE  0
#define  RUN_MODE   1 

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

void DC_Motor1_Run_Fwd( short duty );     // DC 모터1 정회전(PWM구동) 함수 
void DC_Motor1_Run_Rev( short duty );     // DC 모터1 역회전(PWM구동) 함수  
void DC_Motor1_Stop( void );              // DC 모터1 정지 함수  
 
void DC_Motor2_Run_Fwd( short duty );     // DC 모터2 정회전(PWM구동) 함수 
void DC_Motor2_Run_Rev( short duty );     // DC 모터2 역회전(PWM구동) 함수  
void DC_Motor2_Stop( void );              // DC 모터2 정지 함수  

void DC_Motor1_PWM( short duty );         // DC 모터1 PWM 구동 함수  
                                          // 정회전(duty>0), 역회전(duty<0), 정지(duty=0) 모두 포함 
void DC_Motor2_PWM( short duty );         // DC 모터2 PWM 구동 함수  
                                          // 정회전(duty>0), 역회전(duty<0), 정지(duty=0) 모두 포함 

static volatile short  Duty1_Max = 100, Duty2_Max = 100, PWM_Period = 0;  //모터1, 모터2의 최대PWM duty = 100[%]
static volatile short  Duty1_Min = 0, Duty2_Min = 0;   //모터1, 모터2의 최소PWM duty = 0[%]
static volatile short  Motor1_Duty = 0, Motor2_Duty = 0, Step = 0 ;    //모터1, 모터2의 PWM duty [%]

static volatile unsigned short  curr_delay = 0;    // 시간경과 체크함수에서 사용되는 카운터 변수 

static volatile unsigned char  DC_Motor1_Direction=0, DC_Motor2_Direction=0, Operating_Sequence=0, Key_cnt=0;


int main() 
{   
    unsigned char   Key_Data = 0 ;

    LcdInit();                     // LCd 초기화 함수 
    LcdMove(0,0);                 // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("DC Motor Control");  // LCD에 DC모터제어 메시지 디스플레이 
    LcdMove(1,0);                 // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Test Program");      // LCD에 DC모터제어 메시지 디스플레이
    msec_delay(2000);            // 2초(2000msec) 시간지연

   ////////  Key 5개 사용을 위한 HW 관련 레지스터 설정  ////////////
   //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA &= ~0xF0;      // Key1(PA4), Key2(PA5), Key3(PA6), Key4(PA7): 입력포트로 설정 
                           // 입력포트 설정.  DDRA = 0b0000 ****, DDRA = DDRA & ~0b1111 0000(~0xF0)
    PORTA |= 0xF0;       // Key1(PA4), Key2(PA5), Key3(PA6), Key4(PA7)  : 내부풀업사용모드로 설정  
                           // PORTA = 0b1111 ****, PORTA = PORTA | 0b1111 0000( 0xF0 )

    DDRB &= ~0x01;      // Key5(PB0) : 입력포트로 설정   
                           // 입력포트 설정.  DDRB = 0b**** ***0, DDRB = DDRB & ~0b0000 0001(~0x01)
    PORTB |= 0x01;       // Key5(PB0)  : 내부풀업사용모드로 설정  
                           // PORTB = 0b**** ***1, PORTB = PORTB | 0b0000 0001( 0x01 )
   // (주의):  Key 입력포트 설정을 변경하면 key_scan() 함수도 변경해야함. key_scan() 함수 변경 내용을 확인할 것. 

    ////////  DC모터 2개 구동을 위한 HW 관련 레지스터 설정  ////////////
    //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA |= 0x03;       // DC모터1 방향제어포트(PA0), DC모터2 방향제어포트(PA1) : 출력포트로 설정   
                        // DDRA = 0b**** **11, DDRA = DDRA | 0b0000 0011( 0x03 )

    DDRB |= 0x60;       // DC모터1 속도제어 PWM포트(OC1A/PB5), DC모터2 속도제어 PWM포트(OC1B/PB6): 
                        // 출력포트로 설정   
                        // DDRB = 0b*11* ****, DDRB = DDRB | 0b0110 0000( 0x60 )

    DDRA |= 0x0C;       // LED1(DC모터1 동작상태표시)(PA2), LED2(DC모터2 동작상태표시)(PA3) : 출력포트로 설정                          // DDRA = 0b**** 11**, DDRA = DDRA | 0b0000 1100( 0x0C )

    ////////  Timer1 설정( DC모터1 PWM신호(OC1A/PB5), DC모터2 PWM신호(OC1B/PB6) 발생 )   //////////// 
    ///////   PWM신호 주파수 = 5kHz (주기 = 200usec )  //////////////////////
    ///////   교재 P238-245(레지스터 그림과 표12.2, 표12.4, 표12.5) 참조    

    TCCR1A &= ~0x51;     // Fast PWM: 비교일치시 OC1A(PB5)), OC1B(PB6)핀을 0으로하고 TOP에서 1을 출력
                         // (표12.2 참조),   Fast PWM ( mode 14 ) 설정 (표12.4 참조)
                         // TCCR1A = 0b1010 **10 
                         // TCCR1A = TCCR1A & ~0b0101 0001(~0x51 )  
    TCCR1A |= 0xA2;      // TCCR1A = TCCR1A |   0b1010 0010( 0xA2 )  

    TCCR1B &= ~0x04;     // 64분주 타이머1 시작(내부클럭 주기=64/(16*10^6Hz)=4usec ), Fast PWM(mode 14)설정
                         // (표12.4 - 표12.5 참조)
                         // TCCR1B = 0b***1 1011   
                         // TCCR1B = TCCR1B & ~0b0000 0100(~0x04 )  
    TCCR1B |= 0x1B;      // TCCR1B = TCCR1B |   0b0001 1011( 0x1B )  

    ICR1 = 50;           // PWM 주기(주파수) 설정( 주기= 50*4usec = 200usec, 주파수 = 1/(200usec) = 5kHz )
    PWM_Period = ICR1;   // PWM 신호 주기를 전역변수에 저장 

    Motor1_Duty = 0;         // DC모터1 속도 0 설정( 최대 = 100[%],  최소 = 0[%] )
    DC_Motor1_Stop( );       // DC모터1 정지
    Motor2_Duty = 0;         // DC모터2 속도 0 설정( 최대 = 100[%],  최소 = 0[%] )
    DC_Motor2_Stop( );       // DC모터2 정지

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

       if( Key_Data == KEY1 )             // Key 1을 누르면
       {  
 	       Motor1_Duty += 5;                                          // DC모터1 duty 값을 5씩 증가 
           if( Motor1_Duty > Duty1_Max )  Motor1_Duty = Duty1_Max;    // 최대 범위를 벗어나면 Duty1_Max로 설정 
       }

       else if( Key_Data == KEY2 )        // Key 2를 누르면
       {  
            Motor1_Duty -= 5;                                        // DC모터1 duty 값을 5씩 감소
           if( Motor1_Duty < Duty1_Min )  Motor1_Duty = Duty1_Min;   // 최소 범위를 벗어나면 Duty1_Min으로 설정 
       }
       else if( Key_Data == KEY3 )        // Key 3을 누르면
       {  
  	       Motor2_Duty += 5;                                         // DC모터2 duty 값을 5씩 증가 
           if( Motor2_Duty > Duty2_Max )  Motor2_Duty = Duty2_Max;  // 최대 범위를 벗어나면 Duty2_Max로 설정  
       }
       else if( Key_Data == KEY4 )        // Key 4를 누르면
       {  
           Motor2_Duty -= 5;                                        // DC모터2 duty 값을 5씩 감소
           if( Motor2_Duty < Duty2_Min )  Motor2_Duty = Duty2_Min;  // 최소 범위를 벗어나면 Duty2_Min으로 설정  
       } 

       else if( Key_Data == KEY5 )        // Key 5를 누르면
       {  
           Key_cnt++;   
           if( Key_cnt >= 2 ) Key_cnt = 0; 

           if( Key_cnt == 0 ) Operating_Sequence = STOP_MODE;  // Key5가 짝수번 눌러지면 모터 정지모드 
           if( Key_cnt == 1 ) Operating_Sequence = RUN_MODE;   // Key5가 홀수번 눌러지면 모터 회전모드 

           if( Operating_Sequence == RUN_MODE )   //DC모터1,2가 회전모드(동작시퀀스 실행)시작시 관련변수초기화 
           {         
               Step = 0 ;         // DC모터1, DC모터2 동작시퀀스 단계 초기화 
               curr_delay = 0;    // 시간경과 체크함수에서 사용되는 카운터 변수 초기화 
           }
           else if( Operating_Sequence == STOP_MODE )  // 모터가 정지모드 시  
           {         
              DC_Motor1_Direction = STOP;   // DC모터1 정지상태 
              DC_Motor1_Stop( );            // DC 모터1 정지(PWM구동) 함수 호출 
              PORTA |=  0x04;               // LED1 OFF ( PA2 = 1 ) : DC모터1 정지상태 표시
                                            // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

              DC_Motor2_Direction = STOP;   // DC모터2 정지상태 
              DC_Motor2_Stop( );            // DC 모터2 정지(PWM구동) 함수 호출
              PORTA |=  0x08;               // LED2 OFF ( PA3 = 1 ) : DC모터2 정지상태 표시
                                            // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
           }
       }
       ///////  LCD에 모터1, 모터2 동작상태 디스플레이  ///////////

       LcdCommand( ALLCLR );              // LCD 화면 지우기

       if( Operating_Sequence == STOP_MODE )   // 모터 정지 모드 이면 
       {
	      LcdMove(0, 0);                  // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "Stop Mode!!" );       // 정지모드 메시지 디스플레이 
	      LcdMove(1, 0);                  // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "M1:Stop, M2:Stop" );  // DC모터1, DC모터2 정지상태 메시지 디스플레이  
       } 

       else if( Operating_Sequence == RUN_MODE )      // 모터 회전 모드(동작시퀀스 실행) 이면 
       { 
	      LcdMove(0, 0);                              // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 

          if( DC_Motor1_Direction == FORWARD )        // 모터1이 정방향 회전상태 이면 
          {
              LcdPuts( "M1: Run Fwd " );              // 모터1 정방향 회전 메시지 디스플레이 
              Display_Number_LCD( Motor1_Duty, 3 );   // 모터1 속도(Duty) 3자리 디스플레이
              LcdPutchar( '%' );                      // 문자 % 디스플레이 
          }
          else if( DC_Motor1_Direction == REVERSE )   // 모터1이 역방향 회전상태 이면 
          {
              LcdPuts( "M1: Run Rev " );              // 모터1 역방향 회전 메시지 디스플레이 
              Display_Number_LCD( Motor1_Duty, 3 );   // 모터1 속도(Duty) 3자리 디스플레이
              LcdPutchar( '%' );                      // 문자 % 디스플레이 
          }
          else if( DC_Motor1_Direction == STOP )      // 모터1이 정지상태 이면 
          {
              LcdPuts( "M1: Stop " );                 // 모터1 정지 메시지 디스플레이 
          }

	      LcdMove(1, 0);                              // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 

          if( DC_Motor2_Direction == FORWARD )        // 모터2가 정방향 회전상태 이면 
          {
              LcdPuts( "M2: Run, Fwd " );             // 모터2 정방향 회전 메시지 디스플레이 
              Display_Number_LCD( Motor2_Duty, 3 );   // 모터2 속도(Duty) 3자리 디스플레이
              LcdPutchar( '%' );                      // 문자 % 디스플레이 
          }
          else if( DC_Motor2_Direction == REVERSE )   // 모터2가 역방향 회전상태 이면 
          {
              LcdPuts( "M2: Run, Rev " );             // 모터2 역방향 회전 메시지 디스플레이 
              Display_Number_LCD( Motor2_Duty, 3 );   // 모터2 속도(Duty) 3자리 디스플레이
              LcdPutchar( '%' );                      // 문자 % 디스플레이 
          }
          else if( DC_Motor2_Direction == STOP )      // 모터2가 정지상태 이면 
          {
              LcdPuts( "M2: Stop " );                 // 모터2 정지 메시지 디스플레이 
          }

       }  // else if( Operating_Sequence == RUM_MODE ) 의 끝

    }   // 무한루프 while (1) 의 끝 

}      // int main() 함수의 끝 



////////////////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )   // 10msec 주기의 타이머2 오버플로 인터럽트 서비스프로그램
{                        // DC모터1, DC모터2 동작시퀀스 실행 

    TCNT2 = 256 - 156;        // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

   ///////  DC모터1, DC모터2 동작 시퀀스 ////////////

  if( Operating_Sequence == RUN_MODE )
  {

      if( Step == 0 )      
	  { 

         if( Time_Delay_Polling( 200 ) != 1 )    //10msec * 200=2000msec=2초가 경과되지 않았으면 (2초 동안 회전)
         { 
             DC_Motor1_Direction = FORWARD;      // DC모터1 정방향 회전상태 
             DC_Motor1_Run_Fwd( Motor1_Duty );   // DC 모터1 정회전(PWM구동) 함수(2초동안 정회전) 
             PORTA &=  ~0x04;                    // LED1 ON ( PA2 = 0 ) : DC모터1 회전상태 표시
                                                 // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

             DC_Motor2_Direction = REVERSE;      // DC모터2 역방향 회전상태 
             DC_Motor2_Run_Rev( Motor2_Duty );   // DC 모터2 역회전(PWM구동) 함수(2초동안 역회전)  
             PORTA &=  ~0x08;                    // LED2 ON ( PA3 = 0 ) : DC모터2 회전상태 표시
                                                 // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
          }
          else                             // 2초 경과 후 
          {
             Step++;                      // Step1 로 넘어감. 
          }
      }

      if( Step == 1 )      
	  {   

         if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1초가 경과되지 않았으면(1초 동안 정지)
         { 
             DC_Motor1_Direction = STOP;      // DC모터1 정지상태 
             DC_Motor1_Stop( );               // DC 모터1 정지(PWM구동) 함수 호출 (1초동안 정지) 
             PORTA |=  0x04;                  // LED1 OFF ( PA2 = 1 ) : DC모터1 정지상태 표시
                                              // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

             DC_Motor2_Direction = STOP;      // DC모터2 정지상태 
             DC_Motor2_Stop( );               // DC 모터2 정지(PWM구동) 함수 호출 (1초동안 정지)
             PORTA |=  0x08;                  // LED2 OFF ( PA3 = 1 ) : DC모터2 정지상태 표시
                                              // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
         }
         else                             // 1초 경과 후 
         {
             Step++;                      // Step2 로 넘어감. 
         }
      }

      if( Step == 2 )      
	  {   

         if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec= 2초가 경과되지 않았으면(2초 동안 회전) 
         { 
             DC_Motor1_Direction = REVERSE;     // DC모터1 역방향 회전상태 
             DC_Motor1_Run_Rev( Motor1_Duty );  // DC 모터1 역회전(PWM구동) 함수 호출(2초동안 역회전) 
             PORTA &=  ~0x04;                   // LED1 ON ( PA2 = 0 ) : DC모터1 회전상태 표시
                                                // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 ) 

             DC_Motor2_Direction = FORWARD;     // DC모터2 정방향 회전상태 
             DC_Motor2_Run_Fwd( Motor2_Duty );  // DC 모터2 정회전(PWM구동) 함수 호출(2초동안 정회전) 
             PORTA &=  ~0x08;                   // LED2 ON ( PA3 = 0 ) : DC모터2 회전상태 표시
                                                // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
         }
         else                             // 2초 경과 후 
         {
             Step++;                      // Step3 으로 넘어감. 
         }
      }

      if( Step == 3 )      
	  {   

         if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1초가 경과되지 않았으면(1초 동안 정지)
         { 
             DC_Motor1_Direction = STOP;   // DC모터1 정지상태 
             DC_Motor1_Stop( );            // DC 모터1 정지(PWM구동) 함수 호출  
             PORTA |=  0x04;               // LED1 OFF ( PA2 = 1 ) : DC모터1 정지상태 표시
                                           // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

             DC_Motor2_Direction = STOP;   // DC모터2 정지상태 
             DC_Motor2_Stop( );            // DC 모터2 정지(PWM구동) 함수 호출
             PORTA |=  0x08;               // LED2 OFF ( PA3 = 1 ) : DC모터2 정지상태 표시
                                           // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
         }
         else                       // 1초 경과 후 
         {
             Step = 0;              // 동작시퀀스 종료. 초기단계(Step = 0)로 돌아가서 다시 반복. 
         }

      } 

    }   // End of  if( Operating_Sequence = RUN_MODE )


}       // End of  ISR( TIMER2_OVF_vect ) 

////////////////////////////////////////////////////////


void DC_Motor1_Run_Fwd( short duty )   // DC 모터1 정회전 함수 
{
    if( duty > Duty1_Max )     duty = Duty1_Max; // duty 범위를 벗어나면 Duty1_Max로 설정 
    duty = ( duty*PWM_Period )/Duty1_Max;        // %단위의 duty 값을 실제 레지스터(OCR1A) 설정 값으로 변환

    PORTA &= ~0x01;      // 회전방향설정(정회전) :   DC모터1 B단자(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = duty;        // 회전속도(PWM Duty)설정 : DC모터1 A단자(+) = PWM신호(OC1A(PB5))  
}

void DC_Motor1_Run_Rev( short duty )   // DC 모터1 역회전 함수 
{
    if( duty > Duty1_Max )     duty = Duty1_Max;  // duty 범위를 벗어나면 Duty1_Max로 설정 
    duty = ( duty*PWM_Period )/Duty1_Max;         // %단위의 duty 값을 실제 레지스터(OCR1A) 설정 값으로 변환

    PORTA |= 0x01;               // 회전방향설정(역회전) :   DC모터1 B단자(-) = 1(High) ( PA0 = 1 )    
    OCR1A = PWM_Period - duty;   // 회전속도(PWM Duty)설정 : DC모터1 A단자(+) = PWM신호(OC1A(PB5))  
}

void DC_Motor1_Stop( void )   // DC 모터1 정지 함수 
{
    PORTA &= ~0x01;      // 회전방향설정(정회전) :          DC모터1 B단자(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = 0;           // 회전속도 0(PWM Duty = 0) 설정 : DC모터1 A단자(+) = PWM신호(OC1A(PB5))  
}

void DC_Motor2_Run_Fwd( short duty )   // DC 모터2 정회전 함수 
{
    if( duty > Duty2_Max )     duty = Duty2_Max;  // duty 범위를 벗어나면 Duty2_Max로 설정 
    duty = ( duty*PWM_Period )/Duty2_Max;         // %단위의 duty 값을 실제 레지스터(OCR1B) 설정 값으로 변환


    PORTA &= ~0x02;      // 회전방향설정(정회전) :   DC모터2 B단자(-) = 0(Low) ( PA1 = 0 )    
    OCR1B = duty;        // 회전속도(PWM Duty)설정 : DC모터2 A단자(+) = PWM신호(OC1B(PB6))  
}

void DC_Motor2_Run_Rev( short duty )   // DC 모터2 역회전 함수 
{
    if( duty > Duty2_Max )     duty = Duty2_Max;  // duty 범위를 벗어나면 Duty2_Max로 설정 
    duty = ( duty*PWM_Period )/Duty2_Max;         // %단위의 duty 값을 실제 레지스터(OCR1B) 설정 값으로 변환

    PORTA |= 0x02;               // 회전방향설정(역회전) :   DC모터2 B단자(-) = 1(High) ( PA1 = 1 )    
    OCR1B = PWM_Period - duty;   // 회전속도(PWM Duty)설정 : DC모터2 A단자(+) = PWM신호(OC1B(PB6))  
}

void DC_Motor2_Stop( void )   // DC 모터2 정지 함수 
{
    PORTA &= ~0x02;      // 회전방향설정(정회전) :          DC모터2 B단자(-) = 0(Low) ( PA1 = 0 )    
    OCR1B = 0;           // 회전속도 0(PWM Duty = 0) 설정 : DC모터2 A단자(+) = PWM신호(OC1B(PB6))  
}

void DC_Motor1_PWM( short duty )   // DC 모터1 PWM 구동 함수  
{
   if ( duty > Duty1_Max )       duty = Duty1_Max;    // duty가 양의 최대법위 보다 크면 Duty1_Max로 설정 
   else if( duty < -Duty1_Max )  duty = -Duty1_Max;   // duty가 음의 최소범위 보다 작으면 -Duty1_Max로 설정 

   if( duty > 0 )          DC_Motor1_Run_Fwd( duty );
   else if( duty == 0 )    DC_Motor1_Stop();
   else if( duty < 0 )     DC_Motor1_Run_Rev( -duty );
}

void DC_Motor2_PWM( short duty )   // DC 모터2 PWM 구동 함수  
{
   if ( duty > Duty2_Max )       duty = Duty2_Max;   // duty가 양의 최대범위 보다 크면 Duty2_Max로 설정 
   else if( duty < -Duty2_Max )  duty = -Duty2_Max;   // duty가 음의 최소범위 보다 작으면 -Duty2_Max로 설정 

   if( duty > 0 )         DC_Motor2_Run_Fwd( duty );
   else if( duty == 0 )   DC_Motor2_Stop();
   else if( duty < 0 )    DC_Motor2_Run_Rev( -duty );
}
 

/////////////////////////////////////   
                                                           
unsigned char Key_Input(void)   // 함수 호출시 새롭게 눌러진 key 값을 리턴함(채터링 방지 기능포함.)
{                      // 눌러진 key가 없을 시 0(NO_KEY) 이 리턴됨. 한번 호출시 마다 50 msec 이상 시간이 소요. 
    static unsigned char pin = NO_KEY; 
    unsigned char in, in1;

    in = key_scan();             // key 입력 읽기 
    while(1)                     // key 입력이 안정화될때까지 기다림
    {
        msec_delay(50);         // 디바운싱 시간 지연     
        in1 = key_scan();        // key 입력 읽기     
        if( in == in1 ) break;
        in = in1;
    }
    if( !( in & 0xFF ) )    // key가 눌려지지 않고 있으면  0 ( NO_KEY) 리턴
    {	
        pin= NO_KEY;				
        return NO_KEY;
    }
    if( pin == in )         // 동일한 key가 계속 눌려지고 있으면  0 ( NO_KEY) 리턴
    {
        return NO_KEY;  
    }
    pin = in;

    return  in;
}

//////////////////////////////////////////

unsigned char key_scan(void)  // Key_Input( ) 함수에서 호출하는 내부 함수. 실제로 어느 HW 입력포트에 연결된 
{                              // push 스위치가 눌러졌는지 체크(스캔)하는 함수 임. 
                               // key와 관련된 HW 변경 시 (key를 다른 입력포트에 연결할 때) 반드시 수정해야함.
    unsigned char   in = 0 ;
    
    if (       ( ~PINA  & 0x10  ) == 0x10  )        in = KEY1 ; 	  //   Key1 (PA4) 이 눌러지면    	
    else if (  ( ~PINA  & 0x20  ) == 0x20  )        in = KEY2 ; 	  //   Key2 (PA5) 가 눌러지면   		 
    else if (  ( ~PINA  & 0x40  ) == 0x40  )        in = KEY3 ; 	  //   Key3 (PA6) 이 눌러지면 
    else if (  ( ~PINA  & 0x80  ) == 0x80  )        in = KEY4 ; 	  //   Key4 (PA7) 가 눌러지면   		 
    else if (  ( ~PINB  & 0x01  ) == 0x01  )        in = KEY5 ; 	  //   Key5 (PB0) 가 눌러지면     	 
    else                                             in = NO_KEY;   //	Key가 눌러지지 않으면 
    
    return   in ;
    
}
/////////////////////////////////////////


unsigned char Time_Delay_Polling( unsigned short d_time )  // 시간경과 체크함수(폴링방식)
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

//////////////////////////////////////////

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


char NumToAsc( unsigned char Num )   // Num으로 넘어온 16진수 1자리 숫자를 문자데이터(ASCII 코드)로 
{                                        // 변환하여 리턴하는 함수
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}


void msec_delay(int n)           // n msec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		// 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}
void usec_delay(int n)            // n usec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		// 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}

