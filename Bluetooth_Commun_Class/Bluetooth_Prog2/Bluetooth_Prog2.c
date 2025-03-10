
/// < Bluetooth_Prog2.c > : 스마트폰 블루투스통신으로 조도센서, 초음파센서, DC모터 제어하는 ATmega128 코드  //

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.


#define  CPU_CLOCK_KHZ 16000UL    // CPU 클록 주파수(kHz 단위) = 16MHz


void init_serial_USART0( unsigned long brate );  // USART0 포트 (시리얼통신) 관련 레지스터 설정 함수
void SerialPutChar_USART0( char ch );       // USART0 통신(Serial 통신)포트로 1바이트 데이터를 송신하는 함수
void SerialPutString_USART0( char str[] );  // USART0 통신(Serial 통신)포트로 문자열 데이터를 송신하는 함수

void ADC_enable(void);                           // ADC( AD변환기 ) 관련 레지스터 설정 함수 
unsigned short ADC_Read( unsigned char ch ) ;    // AD변환할 채널값을 파라미터로 받아서 해당 채널의 ADC 값을 
                                                 // 읽은 후 그 값을 리턴하는 함수 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // 부호없는 정수형 변수를 10진수 형태로 
                                                                     // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연

void DC_Motor1_Run_Fwd( short duty );     // DC 모터1 정회전(PWM구동) 함수 
void DC_Motor1_Run_Rev( short duty );     // DC 모터1 역회전(PWM구동) 함수  
void DC_Motor1_Stop( void );              // DC 모터1 정지 함수  

static volatile short  Duty1_Max = 100, Duty1_Min = 0 ; //모터1의 최대 PWM duty=100[%], 최소 PWM duty=0[%]
static volatile short  PWM_Period=0, Motor1_Duty = 0 ;  //모터1의 PWM duty [%]

static volatile unsigned short    distance = 0,  distance_prev = 0; // 초음파센서 거리측정값 저장 변수 
static volatile unsigned short    CDS_adc_value  = 0;               // 조도센서값 저장 변수 

////  스마트폰 블루투스 명령(문자열) 6개  ////
static volatile char Cmd_Message_1[] = { "led on" } ;   
static volatile char Cmd_Message_2[] = { "led off" } ;  
static volatile char Cmd_Message_3[] = { "dc motor run" } ; 
static volatile char Cmd_Message_4[] = { "dc motor stop" } ;  
static volatile char Cmd_Message_5[] = { "read ultrasonic sensor" } ;  
static volatile char Cmd_Message_6[] = { "read cds sensor" } ;  

static volatile  char  rdata = 0,  recv_cnt = 0, new_recv_flag = 0  ;                
static volatile  char  recv_data[25] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  

int main() 
{   

    char   eq_count1 = 0, eq_count2 = 0, eq_count3 = 0, eq_count4 = 0, eq_count5 = 0, eq_count6 = 0; 
    char   cmd_data = 0xFF  ;  
    unsigned  char   i = 0 ;   	
    unsigned short   Distance_UltraSonic =0, CDS_Value =0;


    LcdInit();                      // LCD 초기화 함수 호출

    LcdMove(0,0);                   // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("Bluetooth Prog");      // LCD에 블루투스 테스트 메시지 디스플레이 
    LcdMove(1,0);                   // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Send Command.");       // LCD에 블루투스 테스트 메시지 디스플레이

    msec_delay(1000);               // 1초(1000msec) 시간지연

  //  LED1( 블루투스통신 테스트 용)(PA2) : 출력포트로 설정 
  //  출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)   
   
    DDRA |= 0x04;      // LED1( 블루투스통신 테스트 용)(PA2) : 출력포트로 설정                         
                       // DDRA = 0b**** *1**, DDRA = DDRA | 0b0000 0100( 0x04 )  : 1설정 
    PORTA |= 0x04;     // 초기에 LED1 OFF :  PA2 = 1 출력                        
                       // PORTA = 0b**** *1**, PORTA = PORTA | 0b0000 0100( 0x04 ) 

   //// 초음파센서모듈(HC-SR04) 1개 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)

    DDRA |= 0x02;      // 1개의 초음파센서 트리거 신호( 트리거신호1 : PA1 ) : 출력포트 설정. 
                       // DDRA = 0b**** **1*, DDRA = DDRA | 0b0000 0010( 0x02 ) : 1설정  
    PORTA &= ~0x02;    // PA1 : Low  ( Trigger 신호 OFF )  
                       // PORTA = 0b**** **0*, PORTA = PORTA & ~0b0000 0010( ~0x02 ) : 0설정

   ////////  CDS센서(조도센서, ADC2(PF2)) 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  입력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조) 

    DDRF &= ~0x04;      // 조도센서출력신호 v? 연결핀 ( ADC2(PF2) : 입력포트 설정 )  
                        // 입력포트 설정.  DDRF = 0b**** *0**, DDRF = DDRF & ~0b0000 0100(~0x04) : 0설정 

   ////////  DC모터1 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조) 

    DDRA |= 0x01;       // DC모터1 방향제어포트(PA0) : 출력포트로 설정   
                        // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 )
    DDRB |= 0x20;       // DC모터1 속도제어 PWM포트(OC1A/PB5) : 출력포트로 설정   
                        // DDRB = 0b**1* ****, DDRB = DDRB | 0b0010 0000( 0x20 )

   /////////////  Timer1 설정( DC모터 PWM신호(OC1A/PB5) 발생 )   //////////// 
   ////////////   PWM신호 주파수 = 5kHz (주기 = 200usec )  //////////////////////
   /////// 교재 P238-245(레지스터 그림과 표12.2, 표12.4, 표12.5) 참조    

    TCCR1A &= ~0x41;  // Fast PWM: 비교일치시 OC1A(PB5)) 핀에 0을 출력하고 TOP에서 1을 출력
                         // (표12.2 참조),   Fast PWM ( mode 14 ) 설정 (표12.4 참조)
                         // TCCR1A = 0b10** **10 
                         // TCCR1A = TCCR1A & ~0b0100 0001(~0x41 )  : 0출력
    TCCR1A |= 0x82;      // TCCR1A = TCCR1A |   0b1000 0010( 0x82 )  : 1출력

    TCCR1B &= ~0x04;     // 64분주 타이머1 시작(내부클럭 주기=64/(16*10^6Hz)=4usec ), Fast PWM(mode 14)설정
                         // (표12.4 - 표12.5 참조)
                         // TCCR1B = 0b***1 1011   
                         // TCCR1B = TCCR1B & ~0b0000 0100(~0x04 )  
    TCCR1B |= 0x1B;      // TCCR1B = TCCR1B |   0b0001 1011( 0x1B )  

    ICR1 = 50;           // PWM 주기(주파수) 설정( 주기= 50*4usec = 200usec, 주파수 = 1/(200usec) = 5kHz )
    PWM_Period = ICR1;   // PWM 신호 주기를 전역변수에 저장 
    DC_Motor1_Stop( );   // DC모터1 정지

   ////////////  Timer 0 설정  ( 10 msec 주기의 타이머0 오버플로 인터럽트 설정 )  ///////////////
   // 교재 P133-137(레지스터 그림과 표8.1-표8.5) 참조    

    TCCR0 &= ~0x48;       // Normal mode(타이머모드), 타이머 0 시작(1024분주 사용)
                          // TCCR0 = 0b*0**0111 
                          // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;        // TCCR0 =TCCR0 | 0b00000111( 0x07 )  
    TCNT0 = 256 - 156;    // 내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                          // 오버플로인터럽트 주기 = 10msec
                          // 156 = 10msec/ 64usec,  TCNT0 = 256 - 156
    TIMSK |= 0x01;        // 타이머0 오버플로 인터럽트 허용

   ////////////////////////////////////////////////////////////////////////////////////////////    
   // Echo 신호 펄스폭 시간 측정을 위한 Timer 3 설정
   // 교재 PP238-244(레지스터 그림과 표12.4 - 표12.5 참조)

    TCCR3A &= ~0x03;      // Normal mode(타이머모드), 타이머 3 시작(분주비 8) 
                          // 내부클럭주기 = 8/ (16x10^6) = 0.5 usec (0.5usec 단위로 측정) 
                          // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                          // TCCR3A = TCCR3A & ~0b00000011(~0x03)   
    TCCR3B &= ~0x1D;      // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;      // TCCR3B = TCCR3B | 0b00000010( 0x02 )  

   //////////////////////////////////////////////////////////////////////////////////////////
   // 외부인터럽트 4( pin: INT4/PE4 ) 설정 :  초음파센서모듈 Echo 신호가 입력됨.    
   // 교재 pp108-109 (레지스터 그림, 표 7.4-표7.5 참조 )

   EICRB &= ~0x02;        // INT4 : 하강에지(falling edge) 상승에지(rising edge) 모두에서 인터럽트 요구
                          // EICRB = 0b**** **01 
                          // EICRB = EICRB & ~0b0000 0010(~0x02 ) : 0 설정
   EICRB |=  0x01;        // EICRB = EICRB | 0b0000 0001( 0x01 ) :   1 설정
   EIMSK |= 0x10;         // INT4 Enable(허용) 
                          // EIMSK = 0b***1 ****,  EIMSK = EIMSK | 0b0001 0000( 0x10 ) 
   //////////////////////////////////////////////////

   ADC_enable( );           // ADC( AD변환기 ) 관련 레지스터 설정 함수 호출 

   init_serial_USART0( 9600 );   // USART0 포트 시리얼통신 모드 설정 함수 호출, 
                                 // 관련 레지스터(송수신 허용, 보레이트 = 9600bps 설정 등) 설정 

   UCSR0B |= 0x80;          // UART0 수신(RX) 완료 인터럽트 허용
                            // UCSR0B = 0b1*** ****, UCSR0B = UCSR0B | 0b1000 0000( 0x80 )  
           
   sei();                   // 전역인터럽트허용 

   while (1) 
   { 

        cli();                           // 전역인터럽트 금지

 	    CDS_Value = CDS_adc_value;       // 검출된 조도센서의 디지털 값을 변수 CDS_Value에 저장 
 	    Distance_UltraSonic = distance;  // 검출된 거리 측정값을 변수 Distance_UltraSonic에 저장 

        sei();                           // 전역인터럽트 허용

        if( new_recv_flag == 1 )      // 스마트폰으로부터 새로운 명령(문자열) 수신완료 시 
	    { 

            //////////  스마트폰으로부터 수신된 명령(문자열) 판독  ///////////////

            for( i=0; i < recv_cnt ; i++)     // 저장되어있는 명령(문자열)과 일치하는 명령을 찾아냄
		    {
			    if( recv_data[i] == Cmd_Message_1[i] ) eq_count1++ ;
			    if( recv_data[i] == Cmd_Message_2[i] ) eq_count2++ ; 
			    if( recv_data[i] == Cmd_Message_3[i] ) eq_count3++ ;
			    if( recv_data[i] == Cmd_Message_4[i] ) eq_count4++ ;  
			    if( recv_data[i] == Cmd_Message_5[i] ) eq_count5++ ;  
			    if( recv_data[i] == Cmd_Message_6[i] ) eq_count6++ ;  
            }

            if     ( eq_count1 == 6  && eq_count1 == recv_cnt )  cmd_data = 1 ;     // 명령 1
            else if( eq_count2 == 7  && eq_count2 == recv_cnt )  cmd_data = 2 ;     // 명령 2   
            else if( eq_count3 == 12 && eq_count3 == recv_cnt )  cmd_data = 3 ;     // 명령 3
            else if( eq_count4 == 13 && eq_count4 == recv_cnt )  cmd_data = 4 ;     // 명령 4 
            else if( eq_count5 == 22 && eq_count5 == recv_cnt )  cmd_data = 5 ;     // 명령 5
            else if( eq_count6 == 15 && eq_count6 == recv_cnt )  cmd_data = 6 ;     // 명령 6
		    else                                                 cmd_data = 0xFE ;  // 명령 오류

            eq_count1 = 0; eq_count2 = 0; eq_count3 = 0; eq_count4 = 0; eq_count5 = 0; eq_count6 = 0; 

            new_recv_flag = 0;                      // 새로운 명령(문자열) 수신 플래그 리셋


            /////////  판독된 새로운 명령(Command)에 대한 실행    //////

	        if( cmd_data ==  1 )          // 명령 1("led on") 이면
	        {
                  PORTA &= ~0x04;        // LED1(PA2 포트) ON
	        }
	        else if( cmd_data == 2 )      // 명령 2("led off") 이면
	        {
                  PORTA |= 0x04;          // LED1(PA2 포트) OFF
	        }
	        else if( cmd_data == 3 )      // 명령 3("dc motor run") 이면
	        { 
                  Motor1_Duty = 100;                    // DC 모터1 회전속도 최대(100[%])로 설정 
                  DC_Motor1_Run_Fwd( Motor1_Duty );   // DC 모터1 정회전(PWM구동)  
	       }
	       else if( cmd_data == 4 )      // 명령 4("dc motor stop") 이면
	       {
                  DC_Motor1_Stop( );       // DC모터1 정지
	       }
	       else if( cmd_data == 5 )      // 명령 5("read ultrasonic sensor")  이면
	       {
                  SerialPutString_USART0( "measured distance = " ); // 스마트폰으로 메시지 전송

		     // 초음파센서 1에 의해 측정된 거리 Distance_UltraSonic를 십진수로 변환 후 
                  // 각 자리수(3자리)를 문자데이터(ASCII)로 변환후 스마트폰으로 전송 

                  HexToDec( Distance_UltraSonic, 10 );             
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // Distance_UltraSonic 값의 100자리 전송
                  SerialPutChar_USART0( NumToAsc(cnumber[1]) );  // Distance_UltraSonic 값의 10자리 전송 
                  SerialPutChar_USART0( NumToAsc(cnumber[0]) );  // Distance_UltraSonic 값의 1자리 전송
                  SerialPutString_USART0( "cm" );                    // 스마트폰으로 메시지(거리 단위 cm) 전송
                  SerialPutChar_USART0('\n');       // 스마트폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함 
	       }
	       else if( cmd_data == 6 )      // 명령 6("read cds sensor")  이면
	       {
                  SerialPutString_USART0( "CDS value = " ); // 스마트폰으로 메시지 전송

		          // CDS센서(조도센서)에 의해 측정된 조도값(변수 CDS_Value)을 십진수로 변환 후 
                  // 각 자리수(4자리)를 문자데이터(ASCII)로 변환후 스마트폰으로 전송 

                  HexToDec( CDS_Value, 10 );      
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // 조도값(CDS_Value)의 1000 자리 전송      
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // 조도값(CDS_Value)의 100자리 전송
                  SerialPutChar_USART0( NumToAsc(cnumber[1]) );  // 조도값(CDS_Value)의 10자리 전송 
                  SerialPutChar_USART0( NumToAsc(cnumber[0]) );  // 조도값(CDS_Value)의 1자리 전송

                  SerialPutChar_USART0('\n');      // 스마트폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함 
	        }

            else if( cmd_data == 0xFE )      //  수신된 명령이 오류이면 
	        {
                   SerialPutString_USART0( "Command Error!!  Try again.\n" ); //  스마트폰으로 명령 오류 메시지 전송
	        }

           /////////////////////  LCD에 동작 모드 또는 통신 오류메시지 디스플레이  ////////////// 

            LcdCommand( ALLCLR );         // LCD Clear

            if( cmd_data != 0xFE  )          // 스마트폰으로부터 수신된 명령(문자열)에 오류가 없으면  
	        {  
	            LcdMove(0,0);    
	            LcdPuts("Operating Mode:"); 

	            LcdMove(1,0); 
                if(      cmd_data == 1 || cmd_data == 2)       LcdPuts("LED ON/OFF"); 
                else if( cmd_data == 3 || cmd_data == 4)       LcdPuts("DC Motor Control"); 
                else if( cmd_data == 5 )                       LcdPuts("UltrasonicSensor"); 
                else if( cmd_data == 6 )                       LcdPuts("CDS Sensor"); 
             }
             else if( cmd_data == 0xFE  )      // 스마트폰으로부터 수신된 명령(문자열)에 오류가 있으면  
	         {  
		          LcdMove(0, 0 );                // LCD에 오류메시지 디스플레이
		          LcdPuts("Cmd Error!!"); 
	              LcdMove(1, 0 );
		          LcdPuts("Try Again."); 
             }


	   }    // End of if( new_recv_flag == 1 )      // 새로운 명령(문자열) 수신완료 시 


   }     // 무한루프 while (1) 의 끝 

}        // int main() 함수의 끝 


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


ISR( TIMER0_OVF_vect )      //  10 msec 주기의 타이머0 오버플로 인터럽트 서비스프로그램
{                           //  샘플링주기 100msec 마다 CDS센서(조도센서, ADC2채널)값 검출 
                            //  50msec 마다 초음파신호 발사 신호(트리거 신호) 출력

    static unsigned short  time_index_1 = 0, time_index_2 = 0 ; 

    TCNT0 = 256 - 156;     //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                           //  오버플로인터럽트 주기 = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156
    time_index_1++; 

    if( time_index_1 == 10 )    // 샘플링주기 100msec(= 10 x 10msec ) 마다 광량(조도) 검출 
    { 
        time_index_1 = 0;       // 변수 초기화 

         /////////////   조도센서(CDS센서)신호(ADC2) 검출(AD변환)  //////////////// 

         CDS_adc_value = ADC_Read( 2 );  // ADC 채널 2의 AD 변환된 디지털 값을 읽은 후 전역변수에 저장  
    }

    time_index_2++ ;  

    if( time_index_2 == 5 )   // 50 msec (=10msec * 5) 마다 초음파 트리거신호 발생(초음파를 전방으로 발사)
    {
         time_index_2 = 0;    // 초기화

         // 트리거신호(PA1 포트) 발생
	     PORTA |= 0x02;      // PA1 : High
	     usec_delay(20) ;      // 20usec 동안 High 유지 
	     PORTA &= ~0x02;    // PA1 : Low   
    }

}

///////////////////////////////////////////////////

ISR( INT4_vect )     // 외부인터럽트 4(INT4) 서비스 프로그램
{                    // Echo 신호 펄스폭 측정 및 전방 장애물까지의 거리 계산 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0, flag = 0 ;

    if( flag == 0 )            // Echo 신호1의 상승에지에서 인터럽트 걸릴 때 
    {
	    count1 = TCNT3;        // 상승에지에서의 카운터레지스터값(TCNT3) 저장 
	    flag = 1;              // flag 변수 1로 설정
    } 

    else if( flag == 1 )    // Echo 신호1의 하강에지에서 인터럽트 걸릴 때
    { 
	     count2 = TCNT3;                     // 하강에지에서의 카운터레지스터값(TCNT3) 저장 
	     del_T = ( count2 - count1 ) / 2;    // Echo 신호1 펄스폭의 시간 측정(usec 단위)
    	 distance = del_T / 58;              // 초음파센서모듈1이 측정한 전방 장애물까지의 거리계산(cm 단위)

         if( distance > 380 )  distance = distance_prev;  //반사되는 초음파가 검출되지 않을때 직전 거리측정값 사용 
         distance_prev = distance;                        // 직전 거리측정값 저장 변수 업데이트  
	     flag = 0;                                        // flag 변수 0으로 설정 
    } 

} 


//////  USART0 통신(시리얼통신) 수신완료 인터럽트 서비스 프로그램   ///////

ISR( USART0_RX_vect )      // 107쪽( 또는 108쪽) 표7.2 인터럽트벡터의 매크로 참조
{
    static unsigned char r_cnt = 0 ; 


    rdata = UDR0;           // USART0 수신단자(RXD0 핀)로부터 수신된 1바이트 데이터를 전역변수 rdata에 저장
 
    if( rdata != '.' )                   // 수신된 데이터가 마지막 문자를 나타내는 데이터(마침표)가 아니면
    {
        SerialPutChar_USART0( rdata );   // Echo : 수신된 문자를 바로 송신하여 수신된 문자가 정확한지 확인 
   	    recv_data[r_cnt] = rdata;        // 수신된 문자를 전역변수(배열)에 저장 
	    r_cnt++;                         // 수신된 문자 바이트수 증가  
    }

    else if(  rdata == '.' )             // 수신된 문자가 명령(문자열)의 마지막 문자(마침표) 이면
    {
        SerialPutChar_USART0('\n');      // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함 
        recv_cnt = r_cnt ;               // 수신된 명령(문자열)의 바이트수를 전역변수에 저장
        r_cnt = 0;                       // 수신된 문자 바이트수를 저장하는 로컬 변수 초기화
        
	    new_recv_flag = 1;      // 스마트폰으로부터 새로운 명령(문자열)이 수신되었음을 나타내는 플래그 변수를 Set
    }

}

////////////////////////////////////////////////////////////

void init_serial_USART0( unsigned long brate )    ///  USART0 포트(시리얼통신) 관련 레지스터 설정 함수
{

    unsigned short ubrr = 0;


    ////  USART0 통신 포트 설정 (344쪽 - 351쪽 관련 레지스터와 표 참조 )  //////// 
 
    UCSR0A &= ~0x01;      // 다중 프로세서통신모드 사용 안함. --> bit0=0  
                          // 전송속도 배가 모드 사용함. --> bit1=1
                          // UCSR0A = 0b**** **10 --> 
						  // UCSR0A = UCSR0A & ~0b0000 0001 ( ~0x01 ) : 0 설정 
    UCSR0A |=  0x02;	  // UCSR0A = UCSR0A & ~0b0000 0010 ( ~0x02 ) : 1 설정 

    UCSR0B &= ~0x04;      // 문자크기(데이터 전송비트 수 ) = 8비트 --> bit2(UCSZn2)=0 
                          // 송수신 허용(enable) --> bit4=1, bit3=1                           
                          // UCSR0B = 0b***1 10** --> 
                          // UCSR0B = UCSR0B & ~0b0000 0100 ( ~0x04 ) : 0설정
    UCSR0B |= 0x18;       // UCSR0B = UCSR0B |   0b0001 1000 (  0x18 ) : 1설정

    UCSR0C &= ~0x78;      // 문자크기(데이터 전송비트 수 ) = 8비트 --> bit2(UCSZn1)=1,  bit1(UCSZn0)=1  
                          // 정지비트수 = 1비트 사용 --> bit3 = 0   
                          // 비동기 통신모드 선택 --> bit6=0,  패리티체크모드 사용 안함. --> bit5=0, bit4=0 
                          // UCSR0C = 0b*000 011* --> 
                          // UCSR0C = UCSR0C & ~0b0111 1000 ( ~0x78 ) : 0설정
    UCSR0C |= 0x06;       // UCSR0C = UCSR0C |   0b0000 0110 (  0x06 ) : 1설정

	// 보레이트 설정 ( 360쪽, 표 17.7 참조 )
    ubrr = (unsigned short) ( CPU_CLOCK_KHZ * 1000UL / ( 8*brate ) - 1 ); 
    UBRR0H = ( unsigned char ) ( ubrr >> 8 );
    UBRR0L = ( unsigned char )   ubrr ; 

/***
    // USART0 포트의 보레이트 설정 레지스터(UBRR0)는 2바이트(16비트) 레지스터 인데 여기에 값을 쓸때는 
    // 2바이트 변수로 쓰기를 할 수 없고 상위 1바이트(UBRR0H) 쓰기와 하위 1바이트(UBRR0L)쓰기를 따로 해야 함. 
    // 표17.8을 참조하면 시스템클럭 주파수가 16MHz일 때 보레이트 9600 bps로 설정하려면 
    // UBRR0(16비트) 레지스터에 207을 써주어야 함. 207은 8비트 이하 숫자이므로 UBRR0 레지스터의  
    // 하위 1바이트(UBRR0L)에 207을 써주고 상위 1바이트(UBRR0H)에는 0을 써주면 됨.

    UBRR0H = 0x00;      
    UBRR0L = 207;          // 보레이트(Baud Rate) = 9600 bps로 설정 ( 361쪽, 표 17.8 참조 ) 
****/

}



////////////////////////////////////////////////////////////
// 한 문자를 송신한다.
///////////////////////////////////////////////////////////

void SerialPutChar_USART0(char ch)
{
	while( !( UCSR0A & 0x20 ) );	 	// 버퍼가 빌 때(새로운 데이터를 송신 가능할때)를 기다림
                                        // UCSR0A 레지스터의 데이터레지스터빔( bit5(UDRE) )비트가 
                                        // 1 이 될 때까지 기다림.

  	UDR0 = ch;					        // 버퍼에 문자를 쓴다
}

//////////////////////////////////////////////////////////////////
// 문자열을 송신한다.
// 입력   : str - 송신한 문자열을 저장할 버퍼의 주소
/////////////////////////////////////////////////////////////////

void SerialPutString_USART0(char *str)
{
    while(*str != '\0')         // 수신된 문자가 Null 문자( 0x00 )가 아니면 
    {
        SerialPutChar_USART0(*str++);
    }
}

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

    PORTA |= 0x01;               // 회전방향설정(역회전) :     DC모터1 B단자(-) = 1(High) ( PA0 = 1 )    
    OCR1A = PWM_Period - duty;   // 회전속도(PWM Duty)설정 : DC모터1 A단자(+) = PWM신호(OC1A(PB5))  
}

void DC_Motor1_Stop( void )   // DC 모터1 정지 함수 
{
    PORTA &= ~0x01;     // 회전방향설정(정회전) :            DC모터1 B단자(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = 0;           // 회전속도 0(PWM Duty = 0) 설정 : DC모터1 A단자(+) = PWM신호(OC1A(PB5))  
}

void   ADC_enable(void)         // ADC( AD변환기 ) 관련 레지스터 설정 함수 
{
    //////  (교재 pp321-322 레제스터 ADMUX 그림, 표15.1, 표15.2 참조)
    ADMUX &= ~0xE0;          // 기준전압선택( AREF ), ADC결과 오른쪽 정렬 
                             // ADMUX = 0b000* ****  
                             // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 
    //////  (교재 pp323-324 레제스터 ADCSRA 그림, 표15.3 참조)
    ADCSRA |= 0x87;     // ADC 가능(enable), 프리스케일러(Prescaler) 선택: 128 분주
                        // ADCSRA = 0b1*** *111
                        // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
}

////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch )      //  AD변환할 채널값을 파라미터(ch)로 받아서 해당 채널의 ADC 
{                                                //  값을 읽은 후 그 값을 리턴하는 함수 
       unsigned short ad_result  = 0 ;

       if( ch > 7 )  ch = 7;         // ADC 채널은 0 - 7 이므로 범위를 벗어나는 것은 채널 7로 설정

       ADMUX &= ~0x1F;              // ADC 채널 리셋 
       ADMUX |=  ch;                // ADC 채널(ch) 선택
       ADCSRA |= 0x40;              // AD 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
       while( !( ADCSRA & 0x10) );  // AD 변환이 완료될 때까지 기다림. 
       ADCSRA |= 0x10;              // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
       ad_result = ADC;             // AD 변환 완료된 디지털 값( 레지스터 ADC )을 변수에 저장  

       return  ad_result;
}

/////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned short num, unsigned char digit ) //부호없는 정수형변수(num)을 10진수 형태로 
{                                                       // 정해진 자릿수(digit) 만큼 LCD 에 디스플레이 하는 함수 
      HexToDec( num, 10); //10진수로 변환

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000자리 디스플레이 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스플레이 
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스플레이 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스플레이
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

void msec_delay(int n)           // n msec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		         // 1msec 시간 지연을 n회 반복
		_delay_ms(1);		     // 1msec 시간 지연
}

void usec_delay(int n)            // n usec 만큼의 시간지연 발생 함수 
{	
	for(; n>0; n--)		          // 1usec 시간 지연을 n회 반복
		_delay_us(1);		      // 1usec 시간 지연
}

