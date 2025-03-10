
/// < Bluetooth_Prog2.c > : ����Ʈ�� �������������� ��������, �����ļ���, DC���� �����ϴ� ATmega128 �ڵ�  //

#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.


#define  CPU_CLOCK_KHZ 16000UL    // CPU Ŭ�� ���ļ�(kHz ����) = 16MHz


void init_serial_USART0( unsigned long brate );  // USART0 ��Ʈ (�ø������) ���� �������� ���� �Լ�
void SerialPutChar_USART0( char ch );       // USART0 ���(Serial ���)��Ʈ�� 1����Ʈ �����͸� �۽��ϴ� �Լ�
void SerialPutString_USART0( char str[] );  // USART0 ���(Serial ���)��Ʈ�� ���ڿ� �����͸� �۽��ϴ� �Լ�

void ADC_enable(void);                           // ADC( AD��ȯ�� ) ���� �������� ���� �Լ� 
unsigned short ADC_Read( unsigned char ch ) ;    // AD��ȯ�� ä�ΰ��� �Ķ���ͷ� �޾Ƽ� �ش� ä���� ADC ���� 
                                                 // ���� �� �� ���� �����ϴ� �Լ� 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // ��ȣ���� ������ ������ 10���� ���·� 
                                                                     // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);     // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����

void DC_Motor1_Run_Fwd( short duty );     // DC ����1 ��ȸ��(PWM����) �Լ� 
void DC_Motor1_Run_Rev( short duty );     // DC ����1 ��ȸ��(PWM����) �Լ�  
void DC_Motor1_Stop( void );              // DC ����1 ���� �Լ�  

static volatile short  Duty1_Max = 100, Duty1_Min = 0 ; //����1�� �ִ� PWM duty=100[%], �ּ� PWM duty=0[%]
static volatile short  PWM_Period=0, Motor1_Duty = 0 ;  //����1�� PWM duty [%]

static volatile unsigned short    distance = 0,  distance_prev = 0; // �����ļ��� �Ÿ������� ���� ���� 
static volatile unsigned short    CDS_adc_value  = 0;               // ���������� ���� ���� 

////  ����Ʈ�� ������� ���(���ڿ�) 6��  ////
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


    LcdInit();                      // LCD �ʱ�ȭ �Լ� ȣ��

    LcdMove(0,0);                   // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("Bluetooth Prog");      // LCD�� ������� �׽�Ʈ �޽��� ���÷��� 
    LcdMove(1,0);                   // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Send Command.");       // LCD�� ������� �׽�Ʈ �޽��� ���÷���

    msec_delay(1000);               // 1��(1000msec) �ð�����

  //  LED1( ���������� �׽�Ʈ ��)(PA2) : �����Ʈ�� ���� 
  //  ��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)   
   
    DDRA |= 0x04;      // LED1( ���������� �׽�Ʈ ��)(PA2) : �����Ʈ�� ����                         
                       // DDRA = 0b**** *1**, DDRA = DDRA | 0b0000 0100( 0x04 )  : 1���� 
    PORTA |= 0x04;     // �ʱ⿡ LED1 OFF :  PA2 = 1 ���                        
                       // PORTA = 0b**** *1**, PORTA = PORTA | 0b0000 0100( 0x04 ) 

   //// �����ļ������(HC-SR04) 1�� ������ ���� HW ���� �������� ����  ////////////
   //  ��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA |= 0x02;      // 1���� �����ļ��� Ʈ���� ��ȣ( Ʈ���Ž�ȣ1 : PA1 ) : �����Ʈ ����. 
                       // DDRA = 0b**** **1*, DDRA = DDRA | 0b0000 0010( 0x02 ) : 1����  
    PORTA &= ~0x02;    // PA1 : Low  ( Trigger ��ȣ OFF )  
                       // PORTA = 0b**** **0*, PORTA = PORTA & ~0b0000 0010( ~0x02 ) : 0����

   ////////  CDS����(��������, ADC2(PF2)) ������ ���� HW ���� �������� ����  ////////////
   //  �Է� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����) 

    DDRF &= ~0x04;      // ����������½�ȣ v? ������ ( ADC2(PF2) : �Է���Ʈ ���� )  
                        // �Է���Ʈ ����.  DDRF = 0b**** *0**, DDRF = DDRF & ~0b0000 0100(~0x04) : 0���� 

   ////////  DC����1 ������ ���� HW ���� �������� ����  ////////////
   //  ��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����) 

    DDRA |= 0x01;       // DC����1 ����������Ʈ(PA0) : �����Ʈ�� ����   
                        // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 )
    DDRB |= 0x20;       // DC����1 �ӵ����� PWM��Ʈ(OC1A/PB5) : �����Ʈ�� ����   
                        // DDRB = 0b**1* ****, DDRB = DDRB | 0b0010 0000( 0x20 )

   /////////////  Timer1 ����( DC���� PWM��ȣ(OC1A/PB5) �߻� )   //////////// 
   ////////////   PWM��ȣ ���ļ� = 5kHz (�ֱ� = 200usec )  //////////////////////
   /////// ���� P238-245(�������� �׸��� ǥ12.2, ǥ12.4, ǥ12.5) ����    

    TCCR1A &= ~0x41;  // Fast PWM: ����ġ�� OC1A(PB5)) �ɿ� 0�� ����ϰ� TOP���� 1�� ���
                         // (ǥ12.2 ����),   Fast PWM ( mode 14 ) ���� (ǥ12.4 ����)
                         // TCCR1A = 0b10** **10 
                         // TCCR1A = TCCR1A & ~0b0100 0001(~0x41 )  : 0���
    TCCR1A |= 0x82;      // TCCR1A = TCCR1A |   0b1000 0010( 0x82 )  : 1���

    TCCR1B &= ~0x04;     // 64���� Ÿ�̸�1 ����(����Ŭ�� �ֱ�=64/(16*10^6Hz)=4usec ), Fast PWM(mode 14)����
                         // (ǥ12.4 - ǥ12.5 ����)
                         // TCCR1B = 0b***1 1011   
                         // TCCR1B = TCCR1B & ~0b0000 0100(~0x04 )  
    TCCR1B |= 0x1B;      // TCCR1B = TCCR1B |   0b0001 1011( 0x1B )  

    ICR1 = 50;           // PWM �ֱ�(���ļ�) ����( �ֱ�= 50*4usec = 200usec, ���ļ� = 1/(200usec) = 5kHz )
    PWM_Period = ICR1;   // PWM ��ȣ �ֱ⸦ ���������� ���� 
    DC_Motor1_Stop( );   // DC����1 ����

   ////////////  Timer 0 ����  ( 10 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���� )  ///////////////
   // ���� P133-137(�������� �׸��� ǥ8.1-ǥ8.5) ����    

    TCCR0 &= ~0x48;       // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 0 ����(1024���� ���)
                          // TCCR0 = 0b*0**0111 
                          // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;        // TCCR0 =TCCR0 | 0b00000111( 0x07 )  
    TCNT0 = 256 - 156;    // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                          // �����÷����ͷ�Ʈ �ֱ� = 10msec
                          // 156 = 10msec/ 64usec,  TCNT0 = 256 - 156
    TIMSK |= 0x01;        // Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���

   ////////////////////////////////////////////////////////////////////////////////////////////    
   // Echo ��ȣ �޽��� �ð� ������ ���� Timer 3 ����
   // ���� PP238-244(�������� �׸��� ǥ12.4 - ǥ12.5 ����)

    TCCR3A &= ~0x03;      // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 3 ����(���ֺ� 8) 
                          // ����Ŭ���ֱ� = 8/ (16x10^6) = 0.5 usec (0.5usec ������ ����) 
                          // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                          // TCCR3A = TCCR3A & ~0b00000011(~0x03)   
    TCCR3B &= ~0x1D;      // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;      // TCCR3B = TCCR3B | 0b00000010( 0x02 )  

   //////////////////////////////////////////////////////////////////////////////////////////
   // �ܺ����ͷ�Ʈ 4( pin: INT4/PE4 ) ���� :  �����ļ������ Echo ��ȣ�� �Էµ�.    
   // ���� pp108-109 (�������� �׸�, ǥ 7.4-ǥ7.5 ���� )

   EICRB &= ~0x02;        // INT4 : �ϰ�����(falling edge) ��¿���(rising edge) ��ο��� ���ͷ�Ʈ �䱸
                          // EICRB = 0b**** **01 
                          // EICRB = EICRB & ~0b0000 0010(~0x02 ) : 0 ����
   EICRB |=  0x01;        // EICRB = EICRB | 0b0000 0001( 0x01 ) :   1 ����
   EIMSK |= 0x10;         // INT4 Enable(���) 
                          // EIMSK = 0b***1 ****,  EIMSK = EIMSK | 0b0001 0000( 0x10 ) 
   //////////////////////////////////////////////////

   ADC_enable( );           // ADC( AD��ȯ�� ) ���� �������� ���� �Լ� ȣ�� 

   init_serial_USART0( 9600 );   // USART0 ��Ʈ �ø������ ��� ���� �Լ� ȣ��, 
                                 // ���� ��������(�ۼ��� ���, ������Ʈ = 9600bps ���� ��) ���� 

   UCSR0B |= 0x80;          // UART0 ����(RX) �Ϸ� ���ͷ�Ʈ ���
                            // UCSR0B = 0b1*** ****, UCSR0B = UCSR0B | 0b1000 0000( 0x80 )  
           
   sei();                   // �������ͷ�Ʈ��� 

   while (1) 
   { 

        cli();                           // �������ͷ�Ʈ ����

 	    CDS_Value = CDS_adc_value;       // ����� ���������� ������ ���� ���� CDS_Value�� ���� 
 	    Distance_UltraSonic = distance;  // ����� �Ÿ� �������� ���� Distance_UltraSonic�� ���� 

        sei();                           // �������ͷ�Ʈ ���

        if( new_recv_flag == 1 )      // ����Ʈ�����κ��� ���ο� ���(���ڿ�) ���ſϷ� �� 
	    { 

            //////////  ����Ʈ�����κ��� ���ŵ� ���(���ڿ�) �ǵ�  ///////////////

            for( i=0; i < recv_cnt ; i++)     // ����Ǿ��ִ� ���(���ڿ�)�� ��ġ�ϴ� ����� ã�Ƴ�
		    {
			    if( recv_data[i] == Cmd_Message_1[i] ) eq_count1++ ;
			    if( recv_data[i] == Cmd_Message_2[i] ) eq_count2++ ; 
			    if( recv_data[i] == Cmd_Message_3[i] ) eq_count3++ ;
			    if( recv_data[i] == Cmd_Message_4[i] ) eq_count4++ ;  
			    if( recv_data[i] == Cmd_Message_5[i] ) eq_count5++ ;  
			    if( recv_data[i] == Cmd_Message_6[i] ) eq_count6++ ;  
            }

            if     ( eq_count1 == 6  && eq_count1 == recv_cnt )  cmd_data = 1 ;     // ��� 1
            else if( eq_count2 == 7  && eq_count2 == recv_cnt )  cmd_data = 2 ;     // ��� 2   
            else if( eq_count3 == 12 && eq_count3 == recv_cnt )  cmd_data = 3 ;     // ��� 3
            else if( eq_count4 == 13 && eq_count4 == recv_cnt )  cmd_data = 4 ;     // ��� 4 
            else if( eq_count5 == 22 && eq_count5 == recv_cnt )  cmd_data = 5 ;     // ��� 5
            else if( eq_count6 == 15 && eq_count6 == recv_cnt )  cmd_data = 6 ;     // ��� 6
		    else                                                 cmd_data = 0xFE ;  // ��� ����

            eq_count1 = 0; eq_count2 = 0; eq_count3 = 0; eq_count4 = 0; eq_count5 = 0; eq_count6 = 0; 

            new_recv_flag = 0;                      // ���ο� ���(���ڿ�) ���� �÷��� ����


            /////////  �ǵ��� ���ο� ���(Command)�� ���� ����    //////

	        if( cmd_data ==  1 )          // ��� 1("led on") �̸�
	        {
                  PORTA &= ~0x04;        // LED1(PA2 ��Ʈ) ON
	        }
	        else if( cmd_data == 2 )      // ��� 2("led off") �̸�
	        {
                  PORTA |= 0x04;          // LED1(PA2 ��Ʈ) OFF
	        }
	        else if( cmd_data == 3 )      // ��� 3("dc motor run") �̸�
	        { 
                  Motor1_Duty = 100;                    // DC ����1 ȸ���ӵ� �ִ�(100[%])�� ���� 
                  DC_Motor1_Run_Fwd( Motor1_Duty );   // DC ����1 ��ȸ��(PWM����)  
	       }
	       else if( cmd_data == 4 )      // ��� 4("dc motor stop") �̸�
	       {
                  DC_Motor1_Stop( );       // DC����1 ����
	       }
	       else if( cmd_data == 5 )      // ��� 5("read ultrasonic sensor")  �̸�
	       {
                  SerialPutString_USART0( "measured distance = " ); // ����Ʈ������ �޽��� ����

		     // �����ļ��� 1�� ���� ������ �Ÿ� Distance_UltraSonic�� �������� ��ȯ �� 
                  // �� �ڸ���(3�ڸ�)�� ���ڵ�����(ASCII)�� ��ȯ�� ����Ʈ������ ���� 

                  HexToDec( Distance_UltraSonic, 10 );             
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // Distance_UltraSonic ���� 100�ڸ� ����
                  SerialPutChar_USART0( NumToAsc(cnumber[1]) );  // Distance_UltraSonic ���� 10�ڸ� ���� 
                  SerialPutChar_USART0( NumToAsc(cnumber[0]) );  // Distance_UltraSonic ���� 1�ڸ� ����
                  SerialPutString_USART0( "cm" );                    // ����Ʈ������ �޽���(�Ÿ� ���� cm) ����
                  SerialPutChar_USART0('\n');       // ����Ʈ������ ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ��� 
	       }
	       else if( cmd_data == 6 )      // ��� 6("read cds sensor")  �̸�
	       {
                  SerialPutString_USART0( "CDS value = " ); // ����Ʈ������ �޽��� ����

		          // CDS����(��������)�� ���� ������ ������(���� CDS_Value)�� �������� ��ȯ �� 
                  // �� �ڸ���(4�ڸ�)�� ���ڵ�����(ASCII)�� ��ȯ�� ����Ʈ������ ���� 

                  HexToDec( CDS_Value, 10 );      
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // ������(CDS_Value)�� 1000 �ڸ� ����      
                  SerialPutChar_USART0( NumToAsc(cnumber[2]) );  // ������(CDS_Value)�� 100�ڸ� ����
                  SerialPutChar_USART0( NumToAsc(cnumber[1]) );  // ������(CDS_Value)�� 10�ڸ� ���� 
                  SerialPutChar_USART0( NumToAsc(cnumber[0]) );  // ������(CDS_Value)�� 1�ڸ� ����

                  SerialPutChar_USART0('\n');      // ����Ʈ������ ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ��� 
	        }

            else if( cmd_data == 0xFE )      //  ���ŵ� ����� �����̸� 
	        {
                   SerialPutString_USART0( "Command Error!!  Try again.\n" ); //  ����Ʈ������ ��� ���� �޽��� ����
	        }

           /////////////////////  LCD�� ���� ��� �Ǵ� ��� �����޽��� ���÷���  ////////////// 

            LcdCommand( ALLCLR );         // LCD Clear

            if( cmd_data != 0xFE  )          // ����Ʈ�����κ��� ���ŵ� ���(���ڿ�)�� ������ ������  
	        {  
	            LcdMove(0,0);    
	            LcdPuts("Operating Mode:"); 

	            LcdMove(1,0); 
                if(      cmd_data == 1 || cmd_data == 2)       LcdPuts("LED ON/OFF"); 
                else if( cmd_data == 3 || cmd_data == 4)       LcdPuts("DC Motor Control"); 
                else if( cmd_data == 5 )                       LcdPuts("UltrasonicSensor"); 
                else if( cmd_data == 6 )                       LcdPuts("CDS Sensor"); 
             }
             else if( cmd_data == 0xFE  )      // ����Ʈ�����κ��� ���ŵ� ���(���ڿ�)�� ������ ������  
	         {  
		          LcdMove(0, 0 );                // LCD�� �����޽��� ���÷���
		          LcdPuts("Cmd Error!!"); 
	              LcdMove(1, 0 );
		          LcdPuts("Try Again."); 
             }


	   }    // End of if( new_recv_flag == 1 )      // ���ο� ���(���ڿ�) ���ſϷ� �� 


   }     // ���ѷ��� while (1) �� �� 

}        // int main() �Լ��� �� 


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////


ISR( TIMER0_OVF_vect )      //  10 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ �������α׷�
{                           //  ���ø��ֱ� 100msec ���� CDS����(��������, ADC2ä��)�� ���� 
                            //  50msec ���� �����Ľ�ȣ �߻� ��ȣ(Ʈ���� ��ȣ) ���

    static unsigned short  time_index_1 = 0, time_index_2 = 0 ; 

    TCNT0 = 256 - 156;     //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                           //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156
    time_index_1++; 

    if( time_index_1 == 10 )    // ���ø��ֱ� 100msec(= 10 x 10msec ) ���� ����(����) ���� 
    { 
        time_index_1 = 0;       // ���� �ʱ�ȭ 

         /////////////   ��������(CDS����)��ȣ(ADC2) ����(AD��ȯ)  //////////////// 

         CDS_adc_value = ADC_Read( 2 );  // ADC ä�� 2�� AD ��ȯ�� ������ ���� ���� �� ���������� ����  
    }

    time_index_2++ ;  

    if( time_index_2 == 5 )   // 50 msec (=10msec * 5) ���� ������ Ʈ���Ž�ȣ �߻�(�����ĸ� �������� �߻�)
    {
         time_index_2 = 0;    // �ʱ�ȭ

         // Ʈ���Ž�ȣ(PA1 ��Ʈ) �߻�
	     PORTA |= 0x02;      // PA1 : High
	     usec_delay(20) ;      // 20usec ���� High ���� 
	     PORTA &= ~0x02;    // PA1 : Low   
    }

}

///////////////////////////////////////////////////

ISR( INT4_vect )     // �ܺ����ͷ�Ʈ 4(INT4) ���� ���α׷�
{                    // Echo ��ȣ �޽��� ���� �� ���� ��ֹ������� �Ÿ� ��� 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0, flag = 0 ;

    if( flag == 0 )            // Echo ��ȣ1�� ��¿������� ���ͷ�Ʈ �ɸ� �� 
    {
	    count1 = TCNT3;        // ��¿��������� ī���ͷ������Ͱ�(TCNT3) ���� 
	    flag = 1;              // flag ���� 1�� ����
    } 

    else if( flag == 1 )    // Echo ��ȣ1�� �ϰ��������� ���ͷ�Ʈ �ɸ� ��
    { 
	     count2 = TCNT3;                     // �ϰ����������� ī���ͷ������Ͱ�(TCNT3) ���� 
	     del_T = ( count2 - count1 ) / 2;    // Echo ��ȣ1 �޽����� �ð� ����(usec ����)
    	 distance = del_T / 58;              // �����ļ������1�� ������ ���� ��ֹ������� �Ÿ����(cm ����)

         if( distance > 380 )  distance = distance_prev;  //�ݻ�Ǵ� �����İ� ������� ������ ���� �Ÿ������� ��� 
         distance_prev = distance;                        // ���� �Ÿ������� ���� ���� ������Ʈ  
	     flag = 0;                                        // flag ���� 0���� ���� 
    } 

} 


//////  USART0 ���(�ø������) ���ſϷ� ���ͷ�Ʈ ���� ���α׷�   ///////

ISR( USART0_RX_vect )      // 107��( �Ǵ� 108��) ǥ7.2 ���ͷ�Ʈ������ ��ũ�� ����
{
    static unsigned char r_cnt = 0 ; 


    rdata = UDR0;           // USART0 ���Ŵ���(RXD0 ��)�κ��� ���ŵ� 1����Ʈ �����͸� �������� rdata�� ����
 
    if( rdata != '.' )                   // ���ŵ� �����Ͱ� ������ ���ڸ� ��Ÿ���� ������(��ħǥ)�� �ƴϸ�
    {
        SerialPutChar_USART0( rdata );   // Echo : ���ŵ� ���ڸ� �ٷ� �۽��Ͽ� ���ŵ� ���ڰ� ��Ȯ���� Ȯ�� 
   	    recv_data[r_cnt] = rdata;        // ���ŵ� ���ڸ� ��������(�迭)�� ���� 
	    r_cnt++;                         // ���ŵ� ���� ����Ʈ�� ����  
    }

    else if(  rdata == '.' )             // ���ŵ� ���ڰ� ���(���ڿ�)�� ������ ����(��ħǥ) �̸�
    {
        SerialPutChar_USART0('\n');      // �޴������� ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ��� 
        recv_cnt = r_cnt ;               // ���ŵ� ���(���ڿ�)�� ����Ʈ���� ���������� ����
        r_cnt = 0;                       // ���ŵ� ���� ����Ʈ���� �����ϴ� ���� ���� �ʱ�ȭ
        
	    new_recv_flag = 1;      // ����Ʈ�����κ��� ���ο� ���(���ڿ�)�� ���ŵǾ����� ��Ÿ���� �÷��� ������ Set
    }

}

////////////////////////////////////////////////////////////

void init_serial_USART0( unsigned long brate )    ///  USART0 ��Ʈ(�ø������) ���� �������� ���� �Լ�
{

    unsigned short ubrr = 0;


    ////  USART0 ��� ��Ʈ ���� (344�� - 351�� ���� �������Ϳ� ǥ ���� )  //////// 
 
    UCSR0A &= ~0x01;      // ���� ���μ�����Ÿ�� ��� ����. --> bit0=0  
                          // ���ۼӵ� �谡 ��� �����. --> bit1=1
                          // UCSR0A = 0b**** **10 --> 
						  // UCSR0A = UCSR0A & ~0b0000 0001 ( ~0x01 ) : 0 ���� 
    UCSR0A |=  0x02;	  // UCSR0A = UCSR0A & ~0b0000 0010 ( ~0x02 ) : 1 ���� 

    UCSR0B &= ~0x04;      // ����ũ��(������ ���ۺ�Ʈ �� ) = 8��Ʈ --> bit2(UCSZn2)=0 
                          // �ۼ��� ���(enable) --> bit4=1, bit3=1                           
                          // UCSR0B = 0b***1 10** --> 
                          // UCSR0B = UCSR0B & ~0b0000 0100 ( ~0x04 ) : 0����
    UCSR0B |= 0x18;       // UCSR0B = UCSR0B |   0b0001 1000 (  0x18 ) : 1����

    UCSR0C &= ~0x78;      // ����ũ��(������ ���ۺ�Ʈ �� ) = 8��Ʈ --> bit2(UCSZn1)=1,  bit1(UCSZn0)=1  
                          // ������Ʈ�� = 1��Ʈ ��� --> bit3 = 0   
                          // �񵿱� ��Ÿ�� ���� --> bit6=0,  �и�Ƽüũ��� ��� ����. --> bit5=0, bit4=0 
                          // UCSR0C = 0b*000 011* --> 
                          // UCSR0C = UCSR0C & ~0b0111 1000 ( ~0x78 ) : 0����
    UCSR0C |= 0x06;       // UCSR0C = UCSR0C |   0b0000 0110 (  0x06 ) : 1����

	// ������Ʈ ���� ( 360��, ǥ 17.7 ���� )
    ubrr = (unsigned short) ( CPU_CLOCK_KHZ * 1000UL / ( 8*brate ) - 1 ); 
    UBRR0H = ( unsigned char ) ( ubrr >> 8 );
    UBRR0L = ( unsigned char )   ubrr ; 

/***
    // USART0 ��Ʈ�� ������Ʈ ���� ��������(UBRR0)�� 2����Ʈ(16��Ʈ) �������� �ε� ���⿡ ���� ������ 
    // 2����Ʈ ������ ���⸦ �� �� ���� ���� 1����Ʈ(UBRR0H) ����� ���� 1����Ʈ(UBRR0L)���⸦ ���� �ؾ� ��. 
    // ǥ17.8�� �����ϸ� �ý���Ŭ�� ���ļ��� 16MHz�� �� ������Ʈ 9600 bps�� �����Ϸ��� 
    // UBRR0(16��Ʈ) �������Ϳ� 207�� ���־�� ��. 207�� 8��Ʈ ���� �����̹Ƿ� UBRR0 ����������  
    // ���� 1����Ʈ(UBRR0L)�� 207�� ���ְ� ���� 1����Ʈ(UBRR0H)���� 0�� ���ָ� ��.

    UBRR0H = 0x00;      
    UBRR0L = 207;          // ������Ʈ(Baud Rate) = 9600 bps�� ���� ( 361��, ǥ 17.8 ���� ) 
****/

}



////////////////////////////////////////////////////////////
// �� ���ڸ� �۽��Ѵ�.
///////////////////////////////////////////////////////////

void SerialPutChar_USART0(char ch)
{
	while( !( UCSR0A & 0x20 ) );	 	// ���۰� �� ��(���ο� �����͸� �۽� �����Ҷ�)�� ��ٸ�
                                        // UCSR0A ���������� �����ͷ������ͺ�( bit5(UDRE) )��Ʈ�� 
                                        // 1 �� �� ������ ��ٸ�.

  	UDR0 = ch;					        // ���ۿ� ���ڸ� ����
}

//////////////////////////////////////////////////////////////////
// ���ڿ��� �۽��Ѵ�.
// �Է�   : str - �۽��� ���ڿ��� ������ ������ �ּ�
/////////////////////////////////////////////////////////////////

void SerialPutString_USART0(char *str)
{
    while(*str != '\0')         // ���ŵ� ���ڰ� Null ����( 0x00 )�� �ƴϸ� 
    {
        SerialPutChar_USART0(*str++);
    }
}

void DC_Motor1_Run_Fwd( short duty )   // DC ����1 ��ȸ�� �Լ� 
{
    if( duty > Duty1_Max )     duty = Duty1_Max; // duty ������ ����� Duty1_Max�� ���� 
    duty = ( duty*PWM_Period )/Duty1_Max;        // %������ duty ���� ���� ��������(OCR1A) ���� ������ ��ȯ

    PORTA &= ~0x01;      // ȸ�����⼳��(��ȸ��) :   DC����1 B����(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = duty;        // ȸ���ӵ�(PWM Duty)���� : DC����1 A����(+) = PWM��ȣ(OC1A(PB5))  
}

void DC_Motor1_Run_Rev( short duty )   // DC ����1 ��ȸ�� �Լ� 
{
    if( duty > Duty1_Max )     duty = Duty1_Max;  // duty ������ ����� Duty1_Max�� ���� 
    duty = ( duty*PWM_Period )/Duty1_Max;         // %������ duty ���� ���� ��������(OCR1A) ���� ������ ��ȯ

    PORTA |= 0x01;               // ȸ�����⼳��(��ȸ��) :     DC����1 B����(-) = 1(High) ( PA0 = 1 )    
    OCR1A = PWM_Period - duty;   // ȸ���ӵ�(PWM Duty)���� : DC����1 A����(+) = PWM��ȣ(OC1A(PB5))  
}

void DC_Motor1_Stop( void )   // DC ����1 ���� �Լ� 
{
    PORTA &= ~0x01;     // ȸ�����⼳��(��ȸ��) :            DC����1 B����(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = 0;           // ȸ���ӵ� 0(PWM Duty = 0) ���� : DC����1 A����(+) = PWM��ȣ(OC1A(PB5))  
}

void   ADC_enable(void)         // ADC( AD��ȯ�� ) ���� �������� ���� �Լ� 
{
    //////  (���� pp321-322 �������� ADMUX �׸�, ǥ15.1, ǥ15.2 ����)
    ADMUX &= ~0xE0;          // �������м���( AREF ), ADC��� ������ ���� 
                             // ADMUX = 0b000* ****  
                             // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 
    //////  (���� pp323-324 �������� ADCSRA �׸�, ǥ15.3 ����)
    ADCSRA |= 0x87;     // ADC ����(enable), ���������Ϸ�(Prescaler) ����: 128 ����
                        // ADCSRA = 0b1*** *111
                        // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
}

////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch )      //  AD��ȯ�� ä�ΰ��� �Ķ����(ch)�� �޾Ƽ� �ش� ä���� ADC 
{                                                //  ���� ���� �� �� ���� �����ϴ� �Լ� 
       unsigned short ad_result  = 0 ;

       if( ch > 7 )  ch = 7;         // ADC ä���� 0 - 7 �̹Ƿ� ������ ����� ���� ä�� 7�� ����

       ADMUX &= ~0x1F;              // ADC ä�� ���� 
       ADMUX |=  ch;                // ADC ä��(ch) ����
       ADCSRA |= 0x40;              // AD ��ȯ ���� ( ADCSRA �������� bit6 = 1 )
       while( !( ADCSRA & 0x10) );  // AD ��ȯ�� �Ϸ�� ������ ��ٸ�. 
       ADCSRA |= 0x10;              // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) ����.
       ad_result = ADC;             // AD ��ȯ �Ϸ�� ������ ��( �������� ADC )�� ������ ����  

       return  ad_result;
}

/////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned short num, unsigned char digit ) //��ȣ���� ����������(num)�� 10���� ���·� 
{                                                       // ������ �ڸ���(digit) ��ŭ LCD �� ���÷��� �ϴ� �Լ� 
      HexToDec( num, 10); //10������ ��ȯ

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000�ڸ� ���÷��� 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000�ڸ� ���÷��� 
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100�ڸ� ���÷��� 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10�ڸ� ���÷���
      if( digit >= 1)  LcdPutchar(NumToAsc(cnumber[0]));    // 1�ڸ� ���÷���
}
void HexToDec( unsigned short num, unsigned short radix)   // num���� �Ѿ�� 16���� ������ �����͸� 10������ 
{                                                          //  ��ȯ�Ͽ� ������ �ڸ����� �������� �迭 cnumber[0](1�ڸ�) - cnumber[4](10000�ڸ�)�� �����ϴ� �Լ�. 

	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;
	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 
	} while(num);
} 

char NumToAsc( unsigned char Num )       // Num���� �Ѿ�� 16���� 1�ڸ� ���ڸ� ���ڵ�����(ASCII �ڵ�)�� 
{                                        // ��ȯ�Ͽ� �����ϴ� �Լ�
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}

void msec_delay(int n)           // n msec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		         // 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		     // 1msec �ð� ����
}

void usec_delay(int n)            // n usec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		          // 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		      // 1usec �ð� ����
}

