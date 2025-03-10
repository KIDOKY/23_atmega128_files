
// < CDS_TMP_sensor_ADC_1.c >  : 조도센서, 온도센서(LM35), Key 3개를 이용하여 광량과 온도를 검출하는 ATmega128 코드 

#include <avr/io.h>          // ATmega128 이용시 반드시 포함해야함.(각종 HW관련 레지스터가 선언되어 있음)
#include <avr/interrupt.h>   // 인터럽트 이용시 반드시 포함해야함.  
#include <util/delay.h>      // 시간지연함수 이용시 반드시 포함해야함. 

#include "lcd.h"             // LCD에 문자 디스플레이할 때 반드시 포함해야함. 

// Key 관련 심볼 정의
#define  NO_KEY  0
#define  KEY1    1
#define  KEY2    2
#define  KEY3    3

unsigned short ADC_Read( unsigned char ch ) ;    // AD변환할 채널값을 파라미터로 받아서 해당 채널의 ADC 값을 
                                                 // 읽은 후 그 값을 리턴하는 함수 

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
void Display_TMP_LCD( unsigned short tp );   // 온도를 10진수 형태로 소숫점 위 2자리, 소숫점 아래 1자리까지 
                                             // LCD 에 디스플레이 하는 함수

void msec_delay(int n);     // msec 단위 시간지연
void usec_delay(int n);     // usec 단위 시간지연


static volatile unsigned short   CDS_value = 0, CDS_adc_value = 0,  CDS_adc_value_avg = 0 ;
static volatile unsigned short   TMP_value = 0, TMP_adc_value = 0,  TMP_adc_value_avg = 0 ;


int main() 
{   

    unsigned short  Temperature = 0, CDS = 0  ;
    unsigned char   Key_Data = 0 ;

    LcdInit();                     // LCd 초기화 함수 

    LcdMove(0,0);                // LCD에 쓸 데이터 초기 위치 설정( 0행 0열)
    LcdPuts("ILLUM Sensor");     // LCD에 조도센서 메시지 디스플레이 
    LcdMove(1,0);                // LCD에 쓸 데이터 초기 위치 설정( 1행 0열)   
    LcdPuts("TEMP Sensor");      // LCD에 온도센서 메시지 디스플레이

////////  Key 3개 사용을 위한 HW 관련 레지스터 설정  ////////////
//  입력/출력 포트 설정 	   (교재 pp75-76 레지스터그림, 표6.1 참조)
    DDRA &= ~0x70;      // Key1(PA4), Key2(PA5), Key3(PA6) : 입력포트로 설정   
                           // 입력포트 설정.  DDRA = 0b*000 ****, DDRA = DDRA & ~0b0111 0000(~0x70)
    PORTA |= 0x70;       // Key1(PA4), Key2(PA5), Key3(PA6) : 내부풀업사용모드로 설정  
                           // PORTA = 0b*111 ****, PORTA = PORTA | 0b0111 0000( 0x70 )

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

       Key_Data = Key_Input( );           // 함수 Key_Input( )호출. 어떤 Key가 눌러졌는지 체크  

       if( Key_Data == KEY1 )                  // Key1 이 눌러졌으면 조도측정값 디스플레이
       {  
          LcdCommand( ALLCLR );               // LCD 화면 지우기
	      LcdMove(0, 0);                      // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "ILLUMINANCE Mode" );      // 조도측정모드 메시지 디스플레이  
	      LcdMove(1, 0);                      // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "CDS = " );                // 조도 메시지 디스플레이
          Display_Number_LCD( CDS, 3 );       // 광량의 디지털 값을 3자리 십진수로 디스플레이 
       }

       else if( Key_Data == KEY2 )             // Key2 가 눌러졌으면 온도측정값 디스플레이
       {  
          LcdCommand( ALLCLR );                // LCD 화면 지우기
	      LcdMove(0, 0);                       // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "TEMPERATURE Mode" );       // 온도측정모드 메시지 디스플레이   
	      LcdMove(1, 0);                       // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "TMP = " );                 // 온도 메시지 디스플레이
          Display_TMP_LCD( Temperature  );     // 온도를 소숫점 아래 첫째자리까지 디스플레이  
       }

       else if( Key_Data == KEY3 )             // Key3 이 눌러졌으면 조도, 온도측정값 모두 디스플레이
       {  
          LcdCommand( ALLCLR );                // LCD 화면 지우기
	      LcdMove(0, 0);                       // LCD에 쓸 데이터 초기 위치 설정 (0행, 0열) 
          LcdPuts( "CDS = " );                 // 조도 메시지 디스플레이
          Display_Number_LCD( CDS, 3 );        // 광량의 디지털 값을 3자리 십진수로 디스플레이  
	      LcdMove(1, 0);                       // LCD에 쓸 데이터 초기 위치 설정 (1행, 0열) 
          LcdPuts( "TMP = " );                 // 온도 메시지 디스플레이
          Display_TMP_LCD( Temperature  );     // 온도를 소숫점 아래 첫째자리까지 디스플레이  
       }
 

   }  // 무한루프 while (1) 의 끝 


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

       /////////////   조도센서(CDS센서)신호(ADC2) 검출(AD변환)  ////////////////
       CDS_adc_value = ADC_Read( 2 ) ;  // ADC 채널 2의 AD 변환된 디지털 값을 읽은 후 전역변수에 저장  

       /////////////   온도센서신호(ADC3) 검출(AD변환)  ////////////////
       TMP_adc_value  = ADC_Read( 3 ) ;  // ADC 채널 3의 AD 변환된 디지털 값을 읽은 후 전역변수에 저장  

 
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

      if( CDS_value < 250 )           // 검출된 디지털 광량 값이 250 미만이면 LED1 (PA2) ON 
      {    
          PORTA &=  ~0x04;            // LED1 ON ( PA2 = 0 )
                                      // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
      }
      else if( CDS_value >= 250 )     // 검출된 디지털 광량 값이 250 이상이면 LED1 (PA2) OFF
      {
           PORTA |=  0x04;            // LED1 OFF ( PA2 = 1 )
                                      // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
      }

      if( TMP_value < 270 )           // 검출된 온도가 27.0C 미만이면 LED2 (PA3) OFF 
      {    
           PORTA |=  0x08;            // LED2 OFF ( PA3 = 1 )
                                      // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
      }
      else if( TMP_value >= 270 )    // 검출된 온도가 27.0C 이상이면 LED2 (PA3) ON
      {
          PORTA &=  ~0x08;           // LED2 ON ( PA3 = 0 )
                                     // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
      }

   }    // End of  if( index_time == 10 ) 
 
}       // End of  ISR( TIMER2_OVF_vect ) 

//////////////////////////////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch )   //  AD변환할 채널값을 파라미터(ch)로 받아서 해당 채널의 ADC 
{                                                //  값을 읽은 후 그 값을 리턴하는 함수 
    unsigned short ad_result  = 0 ;

    if( ch > 7 )  ch = 7;         // ADC 채널은 0 - 7 이므로 범위를 벗어나는 것은 채널 7로 설정

    ADMUX &= ~0x1F;           // ADC 채널 리셋 
    ADMUX |=  ch;              // ADC 채널(ch) 선택
    ADCSRA |= 0x40;            // AD 변환 시작 ( ADCSRA 레지스터 bit6 = 1 )
    while( !( ADCSRA & 0x10) );  // AD 변환이 완료될 때까지 기다림. 
    ADCSRA |= 0x10;            // ADCSRA 레지스터의 ADC 인터럽트 플래그비트(ADIF, bit4) 지움.
    ad_result = ADC;             // AD 변환 완료된 디지털 값( 레지스터 ADC )을 변수에 저장  

    return  ad_result;
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
    else                                             in = NO_KEY;   //	Key가 눌러지지 않으면 
    
    return   in ;
    
}


/////////////////////////////////////////////////////////////

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



