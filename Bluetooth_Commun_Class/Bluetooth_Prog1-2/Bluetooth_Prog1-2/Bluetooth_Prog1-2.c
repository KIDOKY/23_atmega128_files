
/////  < Bluetooth_Prog1-2.c >  : 블루투스모듈(HC-06)을 이용하여 스마트폰과 통신하는 ATmega128 코드  ///// 


#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함.
 
#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함.


void Servo1_Move( short sv_pos_cmd );    // 서보모터1을 주어진 각도(sv_pos_cmd) 만큼 회전. 0도 - 180도만 가능 

void SerialPutChar_USART1( char ch );       // USART1 통신(Serial 통신)포트로 1바이트 데이터를 송신하는 함수
void SerialPutString_USART1( char str[] );  // USART1 통신(Serial 통신)포트로 문자열 데이터를 송신하는 함수

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // 부호없는 정수형 변수를 10진수 형태로 
              
unsigned char AscToNum( char asc );     // Num으로 넘어온 16진수 1자리 숫자를 문자데이터(ASCII 코드)로 
                                        // 변환하여 리턴하는 함수
			  
			  
			                                                         // LCD 에 디스플레이, digit: 디스플레이할 자릿수 
void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연

static volatile short   PWM_Period = 0, Servo1_Position = 0 ;

static volatile char Send_Message_1[] = { "Received Data Count = " } ;     
static volatile  char  recv_cnt = 0, rdata=0, new_recv_flag = 0, recv_cmd = 0, data_flag = 0  ;               
static volatile unsigned char   Command_Error_Flag = 0 ; 


int main() 
{   

    LcdInit();                     // LCd 초기화 함수 호출

    LcdMove(0,0);                  // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("Bluetooth Prog");     // LCD에 블루투스 테스트 메시지 디스플레이 
    LcdMove(1,0);                  // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("Send Command.");      // LCD에 블루투스 테스트 메시지 디스플레이

    msec_delay(1000);             // 1초(1000msec) 시간지연 

//  출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)   
 
    DDRA |= 0x01;      // LED1( 블루투스통신 테스트 용)(PA0) : 출력포트로 설정                         
                       // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 ) 

    PORTA |= 0x01;     // 초기에 LED1 OFF :  PA0 = 1 출력                        
                       // PORTA = 0b**** ***1, PORTA = PORTA | 0b0000 0001( 0x01 ) 

  ////////  서보모터 1개 구동을 위한 HW 관련 레지스터 설정  ////////////
   //  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)    

    DDRE |= 0x08;      // 서보모터1 위치제어 PWM포트(OC3A/PE3): 출력포트로 설정          

   /////////////  Timer3 설정( 서보모터1 PWM신호(OC3A/PE3) 발생 )   //////////// 
   ////////////   PWM신호 주파수 = 50Hz (주기 = 20msec )  //////////////////////
   /////// 교재 P238-245(레지스터 그림과 표12.2, 표12.4, 표12.5) 참조    

    TCCR3A &= ~0x41;     // Fast PWM: 비교일치시 OC3A/PE3 핀을 0으로 하고 TOP에서 1을 출력
                         // (표12.2 참조),   Fast PWM ( mode 14 ) 설정 (표12.4 참조)
    TCCR3A |= 0x82;      

    TCCR3B &= ~0x03;     // 256 분주 타이머3 시작(내부클럭 주기=256/(16*10^6Hz)=16usec ), 
                         // Fast PWM(mode 14)설정  (표12.4 - 표12.5 참조) 
    TCCR3B |= 0x1C;       

    ICR3 = 1250;         // PWM 주파수=50Hz(PWM주기 = 1/50Hz = 20msec)
                         // PWM 주기(주파수) 설정( 1250 = 20msec(PWM주기)/16usec(256분주된 내부클럭주기),
						  
    PWM_Period = ICR3;   // PWM 신호 주기를 전역변수에 저장 

    Servo1_Position = 0;  
    Servo1_Move( Servo1_Position );       // 서보모터1 가운데(0도) 위치로 회전 


/////////  USART1 통신 포트 설정 (344쪽 - 351쪽 관련 레지스터와 표 참조 )  //////////////////   
 
    UCSR1A &= ~0x01;      // 다중 프로세서통신모드 사용 안함. --> bit0=0  
                          // 전송속도 배가 모드 사용함. --> bit1=1
                          // UCSR1A = 0b**** **10 --> 
			  // UCSR1A = UCSR1A & ~0b0000 0001 ( ~0x01 ) : 0 설정 
    UCSR1A |=  0x02;	  // UCSR1A = UCSR1A & ~0b0000 0010 ( ~0x02 ) : 1 설정 


    UCSR1B &= ~0x04;      // 문자크기(데이터 전송비트 수 ) = 8비트 --> bit2(UCSZn2)=0 
                          // 송수신 허용(enable) --> bit4=1, bit3=1                           
                          // UCSR1B = 0b***1 10** --> 
                          // UCSR1B = UCSR1B & ~0b0000 0100 ( ~0x04 ) : 0설정
    UCSR1B |= 0x18;       // UCSR1B = UCSR1B |   0b0001 1000 (  0x18 ) : 1설정

    UCSR1C &= ~0x78;     // 문자크기(데이터 전송비트 수 ) = 8비트 --> bit2(UCSZn1)=1,  bit1(UCSZn0)=1  
                         // 정지비트수 = 1비트 사용 --> bit3 = 0   
                         // 비동기 통신모드 선택 --> bit6=0,  패리티체크모드 사용 안함. --> bit5=0, bit4=0 
                         // UCSR1C = 0b*000 011* --> 
                         // UCSR1C = UCSR1C & ~0b0111 1000 ( ~0x78 ) : 0설정
    UCSR1C |= 0x06;      // UCSR1C = UCSR1C |   0b0000 0110 (  0x06 ) : 1설정

    // USART1 포트의 보레이트 설정 레지스터(UBRR1)는 2바이트(16비트) 레지스터 인데 여기에 값을 쓸때는 
    // 2바이트 변수로 쓰기를 할 수 없고 상위 1바이트(UBRR1H) 쓰기와 하위 1바이트(UBRR1L)쓰기를 따로 해야 함. 
    // 표17.8을 참조하면 시스템클럭 주파수가 16MHz일 때 보레이트 9600 bps로 설정하려면 
    // UBRR1(16비트) 레지스터에 207을 써주어야 함. 207은 8비트 이하 숫자이므로 UBRR1 레지스터의  
    // 하위 1바이트(UBRR1L)에 207을 써주고 상위 1바이트(UBRR1H)에는 0을 써주면 됨.
 
    UBRR1H = 0x00;      
    UBRR1L = 207;          // 보레이트(Baud Rate) = 9600 bps로 설정 ( 361쪽, 표 17.8 참조 )

    UCSR1B |= 0x80;        // UART1 송신(RX) 완료 인터럽트 허용
                           // UCSR1B = 0b1*** ****, UCSR1B = UCSR1B | 0b1000 0000( 0x80 ) 
						                
    sei();                 // 전역인터럽트허용 


    while (1) 
    { 
       if( new_recv_flag == 1 )             // 새로운 한 문자데이터(1바이트)가 수신되었으면  
	   { 

	       /////////////  수신된 데이터(명령어) 처리   //////////////

	       if( recv_cmd == 'a' )          // 수신된 문자 데이터가 ‘a’ 이면 
	       {
               PORTA |= 0x01;          // LED1 OFF 
	       }

	       else if( recv_cmd == 'b' )     // 수신된 문자 데이터가 'b' 이면
	       {
               PORTA &= ~0x01;         // LED1 ON
	       }

	       else if( recv_cmd == 'c' )     // 수신된 문자 데이터가 'c' 이면
	       {
                    PORTA ^= 0x01;         // LED1 Toggle
	       }

	       else if( recv_cmd == 'd')      // 수신된 문자데이터가 ‘d’ 이면
	       {
               SerialPutString_USART1( "Received Data Count = " );     //  메시지를 휴대폰으로 전송
 
		        HexToDec(recv_cnt,10);            // 수신된 바이트수를 나타내는 변수인 recv_cnt를 십진수로 변환

                // 십진수로 변환된 각각의 자리수를 문자데이터(ASCII 코드)로 변환하여 스마트폰으로 전송

                SerialPutChar_USART1( NumToAsc(cnumber[2]) );  // 변수 recv_cnt 값을 전송 상위(100자리)부터 전송 
                SerialPutChar_USART1( NumToAsc(cnumber[1]) );  // 변수 recv_cnt 값의 10자리 전송
                SerialPutChar_USART1( NumToAsc(cnumber[0]) );  // 변수 recv_cnt 값의 1자리 전송
                SerialPutChar_USART1('\n');    // 휴대폰으로 데이터 전송 시 Line Feed('\n')를 항상 끝에 전송해야함
	       } 

	       else if( recv_cmd == 's')      // 수신된 문자데이터가 ‘s’ 이면
	       {        
		              
               Servo1_Move( Servo1_Position );       // 서보모터1을 Servo1_Position 위치로 회전 

               SerialPutString_USART1( "Servo1 position = " );     //  메시지를 휴대폰으로 전송
 
		        HexToDec(Servo1_Position,10);            // 수신된 바이트수를 나타내는 변수인 recv_cnt를 십진수로 변환

                // 십진수로 변환된 각각의 자리수를 문자데이터(ASCII 코드)로 변환하여 스마트폰으로 전송

                SerialPutChar_USART1( NumToAsc(cnumber[2]) );  // 변수 recv_cnt 값을 전송 상위(100자리)부터 전송 
                SerialPutChar_USART1( NumToAsc(cnumber[1]) );  // 변수 recv_cnt 값의 10자리 전송
                SerialPutChar_USART1( NumToAsc(cnumber[0]) );  // 변수 recv_cnt 값의 1자리 전송
                SerialPutChar_USART1('\n');    // 휴대폰으로 데이터 전송 시 Line Feed('\n')를 항상 끝에 전송해야함

           }

	       else                                //  수신된 문자데이터가 정의되지 않은 명령이면(즉, 명령 오류 이면)
	       {
              SerialPutString_USART1( "Command Error!!  Try again.\n" ); //  명령 오류 메시지를 휴대폰으로 전송

		      Command_Error_Flag = 1;          // 명령 오류 플래그 셋
	       }

        ////////////////  LCD 디스플레이 //////////////////////////////////

           if( Command_Error_Flag == 0  )          // 명령에 오류가 없으면  
	       {  
                /////  수신된 바이트수(변수 값) LCD 디스플레이  /////////////////
                LcdCommand( ALLCLR ) ;    // LCD Clear

	            LcdMove(0,0);    
	            LcdPuts("Recv cnt = "); 
                Display_Number_LCD( recv_cnt, 3 ); //수신된 바이트수 recv_cnt를 십진수로 변환하여 LCD에 디스플레이
	            LcdMove(1,0);    
	            LcdPuts("Recv data = "); 
	            LcdPutchar( recv_cmd );       // 수신된 명령 recv_cmd 를 LCD에 디스플레이

           }

	       else if( Command_Error_Flag == 1 )    // 명령에 오류가 있으면
	       {  

		      LcdCommand( 0x01) ;         // LCD Claear
		      LcdMove(0, 0 );             // LCD에 오류메시지 디스플레이
		      LcdPuts("Cmd Error!!"); 
		      LcdMove(1, 0 );
		      LcdPuts("Try Again."); 

		      Command_Error_Flag = 0 ;     // Command_Error_Flag 리셋 
           }


           new_recv_flag = 0;                      // 새 문자(명령) 수신 플래그 리셋

  
       }    //  if( new_recv_flag == 1 )  의 끝      // 새로운 한 문자데이터(1바이트)가 수신완료 시 


   }     // 무한루프 while (1) 의 끝 

}        // int main() 함수의 끝 

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

//////  USART1 통신(시리얼통신) 수신완료 인터럽트 서비스 프로그램   ///////


ISR( USART1_RX_vect )      // 107쪽( 또는 108쪽) 표7.2 인터럽트벡터의 매크로 참조
{

    static  unsigned short  data_buf = 0;

    rdata = UDR1;           // USART1 수신단자(RXD1 핀)로부터 수신된 1바이트 데이터를 전역변수 rdata에 저장
 
    SerialPutChar_USART1(rdata);  //Echo: 수신된 데이터를 바로 휴대폰으로 송신하여 수신된데이터가 정확한지 확인 

    recv_cnt++ ;            // 수신된 데이터 바이트수 증가 및 저장

      
    if( data_flag == 0 )
	{
       if( rdata == 's' )
       {
           recv_cmd = rdata; 
           data_flag = 1;
       }
       else if( rdata != 's' )
       {
           SerialPutChar_USART1('\n');   //휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함 

           recv_cmd = rdata; 
           new_recv_flag = 1;      // 새로운 문자(명령)가 휴대폰으로부터 수신되었음을 나타내는 수신 플래그 변수를 Set
       }
    }
    else if( data_flag == 1 )
    {
       if( rdata >='0'  &&  rdata <='9' )
       {
          data_buf *= 10;
          data_buf += AscToNum( rdata );
       }
       else if( rdata =='.')
       {
           SerialPutChar_USART1('\n');   //휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

           Servo1_Position = data_buf; 
           if( Servo1_Position > 180 )  Servo1_Position = 180;

           data_flag = 0 ;
           data_buf = 0;
           new_recv_flag = 1;  // 새로운 문자(명령)가 휴대폰으로부터 수신되었음을 나타내는 수신 플래그 변수를 Set
       }
    }
}


////////////////////////////////////////////////////////////
// 한 문자를 송신한다.
///////////////////////////////////////////////////////////

void SerialPutChar_USART1(char ch)
{
	while( !( UCSR1A & 0x20 ) );	// 버퍼가 빌 때(새로운 데이터를 송신 가능할때)를 기다림
                                        // UCSR1A 레지스터의 데이터레지스터빔( bit5(UDRE) )비트가 
                                        // 1 이 될 때까지 기다림.

  	UDR1 = ch;		        // 버퍼에 문자를 쓴다
}


//////////////////////////////////////////////////////////////////
// 문자열을 송신한다.
// 입력   : str - 송신한 문자열을 저장할 버퍼의 주소
/////////////////////////////////////////////////////////////////

void SerialPutString_USART1(char *str)
{

    while(*str != '\0')         // 수신된 문자가 Null 문자( 0x00 )가 아니면 
    {

        SerialPutChar_USART1(*str++);
    }
}

//////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////

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

unsigned char AscToNum( char asc )       // Num으로 넘어온 16진수 1자리 숫자를 문자데이터(ASCII 코드)로 
{                                        // 변환하여 리턴하는 함수
	if( asc >= '0'  &&  asc <= '9')  asc -= 0x30; 
	else                             asc -= 0x37; 

	return  (unsigned char) asc ;
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

