
/////////   < DC_Motor_Cont_1.c >  : DC 모터 2개를 구동하는 ATmega128 코드   //////////

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.


void msec_delay(int n);    // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연

unsigned char Time_Delay_Polling( unsigned short d_time );   // 시간경과 체크함수(폴링방식)

void DC_Motor1_Run_Fwd( short duty );    // DC 모터1 정회전(PWM구동) 함수 
void DC_Motor1_Run_Rev( short duty );    // DC 모터1 역회전(PWM구동) 함수  
void DC_Motor1_Stop( void );             // DC 모터1 정지 함수  
 
void DC_Motor2_Run_Fwd( short duty );    // DC 모터2 정회전(PWM구동) 함수 
void DC_Motor2_Run_Rev( short duty );    // DC 모터2 역회전(PWM구동) 함수  
void DC_Motor2_Stop( void );             // DC 모터2 정지 함수  

void DC_Motor1_PWM( short duty );        // DC 모터1 PWM 구동 함수  
                                         // 정회전(duty>0), 역회전(duty<0), 정지(duty=0) 모두 포함 
void DC_Motor2_PWM( short duty );        // DC 모터2 PWM 구동 함수  
                                         // 정회전(duty>0), 역회전(duty<0), 정지(duty=0) 모두 포함 

static volatile short  Duty1_Max= 100, Duty2_Max=100, PWM_Period=0;    //모터1, 모터2의 최대PWM duty = 100[%]
static volatile short  Motor1_Duty = 0, Motor2_Duty = 0, Step = 0 ;    //모터1, 모터2의 PWM duty [%]


int main() 
{   
    short  duty_1 = 0, duty_2 = 0 ; 

    LcdInit();                     // LCd 초기화 함수 

    LcdMove(0,0);                 // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("DC Motor Control");  // LCD에 DC모터제어 메시지 디스플레이 
    LcdMove(1,0);                 // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Test Program");      // LCD에 DC모터제어 메시지 디스플레이
    msec_delay(2000);            // 2초(2000msec) 시간지연

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

       cli();                             // 전역인터럽트 금지

 	   duty_1 = Motor1_Duty;              // DC모터1 duty 값을 변수 duty_1에 저장 
 	   duty_2 = Motor2_Duty;              // DC모터2 duty 값을 변수 duty_2에 저장 

       sei();                             // 전역인터럽트 허용

       if( duty_1 == 0 && duty_2 == 0 )      // DC모터1, DC모터2 모두 정지상태이면 
       {  
          LcdCommand( ALLCLR );              // LCD 화면 지우기
	      LcdMove(0, 0);                     // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "Motor1 : Stop" );        // DC모터1 정지상태 메시지 디스플레이  
	      LcdMove(1, 0);                     // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "Motor2 : Stop" );        // DC모터2 정지상태 메시지 디스플레이 
       }

       else if( duty_1 == 0 && duty_2 != 0 )    // DC모터1 정지상태, DC모터2 회전상태이면 
       {  
          LcdCommand( ALLCLR );                 // LCD 화면 지우기
	      LcdMove(0, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "Motor1 : Stop" );           // DC모터1 정지상태 메시지 디스플레이  
	      LcdMove(1, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "Motor2 : Run" );            // DC모터2 회전상태 메시지 디스플레이 
       }

       else if( duty_1 != 0 && duty_2 == 0 )    // DC모터1 회전상태, DC모터2 정지상태이면 
       {  
          LcdCommand( ALLCLR );                 // LCD 화면 지우기
	      LcdMove(0, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "Motor1 : Run" );            // DC모터1 회전상태 메시지 디스플레이  
	      LcdMove(1, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "Motor2 : Stop" );           // DC모터2 정지상태 메시지 디스플레이 
       }

       else if( duty_1 != 0 && duty_2 != 0 )    // DC모터1, DC모터2 모두 회전상태이면 
       {  
          LcdCommand( ALLCLR );                 // LCD 화면 지우기
	      LcdMove(0, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "Motor1 : Run" );            // DC모터1 회전상태 메시지 디스플레이  
	      LcdMove(1, 0);                        // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "Motor2 : Run" );            // DC모터2 회전상태 메시지 디스플레이 
       }
      
       msec_delay(50);      // 50msec 시간지연 (안정적인 LCD 디스플레이를 위함.)

   }   // 무한루프 while (1) 의 끝 

}      // int main() 함수의 끝 


////////////////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )   // 10msec 주기의 타이머2 오버플로 인터럽트 서비스프로그램
{                        // DC모터1, DC모터2 동작시퀀스 실행 

    TCNT2 = 256 - 156;        // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                              // 오버플로인터럽트 주기 = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    ///////  DC모터1, DC모터2 동작 시퀀스 ////////////

    if( Step == 0 )    
	{
        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2초가 경과되지 않았으면 (2초 동안 회전)
        { 
           Motor1_Duty = 80;                    // DC모터1 duty(속도) = 80[%] 설정 
           DC_Motor1_Run_Fwd( Motor1_Duty );    // DC 모터1 정회전(PWM구동) 함수(2초동안 정회전) 
           PORTA &=  ~0x04;                     // LED1 ON ( PA2 = 0 ) : DC모터1 회전상태 표시
                                                // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

           Motor2_Duty = 80;                    // DC모터2 duty(속도) = 80[%] 설정
           DC_Motor2_Run_Rev( Motor2_Duty );    // DC 모터2 역회전(PWM구동) 함수(2초동안 역회전)  
           PORTA &=  ~0x08;                     // LED2 ON ( PA3 = 0 ) : DC모터2 회전상태 표시
                                                // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
        }
        else                                    // 2초 경과 후 
        {
           Step++;                              // Step1 로 넘어감. 
        }
    }

    if( Step == 1 )    
	{   
        if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1초가 경과되지 않았으면(1초 동안 정지)
        { 
           Motor1_Duty = 0;        // DC모터1 duty = 0[%] 설정(정지) 
           DC_Motor1_Stop( );      // DC 모터1 정지(PWM구동) 함수 호출 (1초동안 정지) 

           PORTA |=  0x04;         // LED1 OFF ( PA2 = 1 ) : DC모터1 정지상태 표시
                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
           Motor2_Duty = 0;        // DC모터2 duty = 0[%] 설정(정지)  
           DC_Motor2_Stop( );      // DC 모터2 정지(PWM구동) 함수 호출 (1초동안 정지)

           PORTA |=  0x08;         // LED2 OFF ( PA3 = 1 ) : DC모터2 정지상태 표시
                                   // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                       // 1초 경과 후 
        {
           Step++;                 // Step2 로 넘어감. 
        }
    }

    if( Step == 2 )    
	{   
        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec= 2초가 경과되지 않았으면(2초 동안 회전) 
        { 
           Motor1_Duty = 100;                  // DC모터1 duty(속도) = 100[%] 설정 (최대속도) 
           DC_Motor1_Run_Rev( Motor1_Duty );   // DC 모터1 역회전(PWM구동) 함수 호출(2초동안 역회전) 

           PORTA &=  ~0x04;                    // LED1 ON ( PA2 = 0 ) : DC모터1 회전상태 표시
                                               // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 ) 

           Motor2_Duty = 100;                  // DC모터2 duty(속도) = 100[%] 설정 (최대속도) 
           DC_Motor2_Run_Fwd( Motor2_Duty );   // DC 모터2 정회전(PWM구동) 함수 호출(2초동안 정회전) 

           PORTA &=  ~0x08;                    // LED2 ON ( PA3 = 0 ) : DC모터2 회전상태 표시
                                               // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
        }
        else                                   // 2초 경과 후 
        {
           Step++;                             // Step3 으로 넘어감. 
        }
    }

    if( Step == 3 )    
	{   
        if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1초가 경과되지 않았으면(1초 동안 정지)
        { 
           Motor1_Duty = 0;        // DC모터1 duty = 0[%] 설정(정지) (1초동안 정지)
           DC_Motor1_Stop( );      // DC 모터1 정지(PWM구동) 함수 호출  

           PORTA |=  0x04;         // LED1 OFF ( PA2 = 1 ) : DC모터1 정지상태 표시
                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

           Motor2_Duty = 0;        // DC모터2 duty = 0[%] 설정(정지) (1초동안 정지)
           DC_Motor2_Stop( );      // DC 모터2 정지(PWM구동) 함수 호출

           PORTA |=  0x08;         // LED2 OFF ( PA3 = 1 ) : DC모터2 정지상태 표시
                                   // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                       // 1초 경과 후 
        {
           Step = 0;              // 동작시퀀스 종료. 초기단계(Step0)로 돌아가서 다시 반복. 
        }

    } 

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

