
/////  < RC_Servo_Motor_Cont_1.c >  : Key(5��)�� �̿��Ͽ� ��������(2��)�� ��ġ�� �����ϴ� ATmega128 �ڵ�  ///// 

#include <avr/io.h>         // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���.

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.

// Key ���� �ɺ� ����
#define  NO_KEY  0
#define  KEY1     1
#define  KEY2     2

#define  KEY3     3

// �������� ȸ������ ���� �ɺ� ����
#define  FORWARD    0
#define  BACKWARD   1 

// �������� ���۸�� ���� �ɺ� ����
#define  MANUAL_MODE     0
#define  SEQUENCE_MODE   1 

unsigned char Key_Input(void); // �Լ� ȣ��� ���Ӱ� ������ key ���� ������(ä�͸� ���� �������.)
                               // ������ key�� ���� �� 0(NO_KEY) �� ���ϵ�. �ѹ� ȣ��� ���� 50 msec �̻� �ð��� �ҿ�. 
unsigned char key_scan(void);  // Key_Input( ) �Լ����� ȣ���ϴ� ���� �Լ�. ������ ��� HW �Է���Ʈ�� ����� 
                               // push ����ġ�� ���������� üũ(��ĵ)�ϴ� �Լ� ��. 
                               // key�� ���õ� HW ���� �� (key�� �ٸ� �Է���Ʈ�� ������ ��) �ݵ�� �����ؾ���.

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // ��ȣ���� ������ ������ 10���� ���·� 
                                                                     // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);     // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����

unsigned char Time_Delay_Polling( unsigned short d_time );   // �ð���� üũ�Լ�(�������)

void Servo1_Move( short sv_pos_cmd );    // ��������1�� �־��� ����(sv_pos_cmd) ��ŭ ȸ��. 0�� - 180���� ����



static volatile short   PWM_Period = 0, Servo1_Position = 0 ;   
static volatile unsigned short  curr_delay = 0;    // �ð���� üũ�Լ����� ���Ǵ� ī���� ���� 
static volatile unsigned char  Step = 0, Servo1_Direction = 0 ; 
static volatile unsigned char  Operating_Mode = MANUAL_MODE,  Key_cnt = 0;


int main() 
{   

    unsigned char   Key_Data = 0 ;

    LcdInit();                      // LCd �ʱ�ȭ �Լ� 
    LcdMove(0,0);                   // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("Servo Control");       // LCD�� ������������ �޽��� ���÷��� 
    LcdMove(1,0);                   // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Test Program");        // LCD�� ������������ �޽��� ���÷���
    msec_delay(2000);               // 2��(2000msec) �ð�����

   ////////  Key 5�� ����� ���� HW ���� �������� ����  ////////////
   //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA &= ~0x30;       // Key1(PA4), Key2(PA5): �Է���Ʈ�� ���� 
                         // �Է���Ʈ ����.  DDRA = 0b0000 ****, DDRA = DDRA & ~0b1111 0000(~0xF0)
    PORTA |= 0x30;       // Key1(PA4), Key2(PA5): ����Ǯ�������� ����  
                         // PORTA = 0b1111 ****, PORTA = PORTA | 0b1111 0000( 0xF0 )

    DDRB &= ~0x01;       // Key3(PB0) : �Է���Ʈ�� ����   
                         // �Է���Ʈ ����.  DDRB = 0b**** ***0, DDRB = DDRB & ~0b0000 0001(~0x01)
    PORTB |= 0x01;       // Key3(PB0)  : ����Ǯ�������� ����  
                         // PORTB = 0b**** ***1, PORTB = PORTB | 0b0000 0001( 0x01 )

   // (����):  Key �Է���Ʈ ������ �����ϸ� key_scan() �Լ��� �����ؾ���. key_scan() �Լ� ���� ������ Ȯ���� ��. 

   ////////  �������� 2�� ������ ���� HW ���� �������� ����  ////////////
   //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)    

    DDRE |= 0x08;      // ��������1 ��ġ���� PWM��Ʈ(OC3A/PE3)
                       // �����Ʈ�� ����   
                       // DDRE = 0b***1 1***, DDRE = DDRE | 0b0001 1000( 0x18 )
    DDRA |= 0x04;      // LED1(��������1 ���ۻ���ǥ��)(PA2) : �����Ʈ�μ��� // DDRA = 0b**** 11**, DDRA = DDRA | 0b0000 1100( 0x0C )

   /////////////  Timer3 ����( ��������1 PWM��ȣ(OC3A/PE3), ��������2 PWM��ȣ(OC3B/PE4) �߻� )   //////////// 
   ////////////   PWM��ȣ ���ļ� = 50Hz (�ֱ� = 20msec )  //////////////////////
   /////// ���� P238-245(�������� �׸��� ǥ12.2, ǥ12.4, ǥ12.5) ����    

    TCCR3A &= ~0x41;     // Fast PWM: ����ġ�� OC3A/PE3, OC3B/PE4 ���� 0���� �ϰ� TOP���� 1�� ���
                         // (ǥ12.2 ����),   Fast PWM ( mode 14 ) ���� (ǥ12.4 ����)
                         // TCCR3A = 0b1010 **10 
                         // TCCR3A = TCCR3A & ~0b0101 0001(~0x51 )  
    TCCR3A |= 0x82;      // TCCR3A = TCCR3A |   0b1010 0010( 0xA2 )  

    TCCR3B &= ~0x03;     // 256 ���� Ÿ�̸�3 ����(����Ŭ�� �ֱ�=256/(16*10^6Hz)=16usec ), 
                         // Fast PWM(mode 14)����  (ǥ12.4 - ǥ12.5 ����)
                         // TCCR3B = 0b***1 1100   
                         // TCCR3B = TCCR3B & ~0b0000 0011(~0x03 )  
    TCCR3B |= 0x1C;      // TCCR3B = TCCR3B |   0b0001 1100( 0x1C )  

    ICR3 = 1250;         // PWM ���ļ�=50Hz(PWM�ֱ� = 1/50Hz = 20msec)
                         // PWM �ֱ�(���ļ�) ����( 1250 = 20msec(PWM�ֱ�)/16usec(256���ֵ� ����Ŭ���ֱ�),
						  
    PWM_Period = ICR3;   // PWM ��ȣ �ֱ⸦ ���������� ���� 

    Servo1_Position = 90;  
    Servo1_Move( Servo1_Position );       // ��������1 ���(90��) ��ġ�� ȸ�� 
   

   ////////////  Timer 2 ����  ( 10 msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ ���� )  ///////////////
   // ���� P133-137(�������� �׸��� ǥ8.1-ǥ8.6) ����  
     
    TCCR2 &= ~0x4A;           // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 2 ����(1024���� ���)
                              // TCCR2 = 0b*0**0101 
                              // TCCR2 =TCCR2 & ~0b01001010(~0x4A )  
    TCCR2 |= 0x05;            // TCCR2 =TCCR2 | 0b00000101( 0x05 ) 
	 
    TCNT2 = 256 - 156;        // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 2msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    TIMSK |= 0x40;            // Ÿ�̸�2 �����÷� ���ͷ�Ʈ ��� 
                              // TIMSK = 0b*1** ****, TIMSK = TIMSK | 0b0100 0000( 0x40 )
    sei();                    // ���� ���ͷ�Ʈ ���


    while (1) 
    { 

       Key_Data = Key_Input( );           // �Լ� Key_Input( )ȣ��. � Key�� �������� üũ  

       if( Key_Data == KEY1 && Operating_Mode == MANUAL_MODE )  // �Ŵ����忡�� Key 1�� ������
       {  
 	       Servo1_Position += 10;                               // ��������1 ��ġ���(Servo1_Position)�� 10���� ����
           if( Servo1_Position > 180 )  Servo1_Position = 180;  // �ִ� ������ ����� 180���� ���� 

           Servo1_Move( Servo1_Position );                      // ��������1�� Servo1_Position ���� ȸ��

           Servo1_Direction  = FORWARD;                         // ��������1 ������ ȸ������
       }

       else if( Key_Data == KEY2 && Operating_Mode == MANUAL_MODE )  // �Ŵ����忡�� Key 2�� ������
       {  
 	       Servo1_Position -= 10;                           // ��������1 ��ġ���(Servo1_Position)�� 10���� ����  
           if( Servo1_Position < 0 )  Servo1_Position = 0;  // �ּ� ������ ����� 0���� ���� 

           Servo1_Move( Servo1_Position );                  // ��������1�� Servo1_Position ���� ȸ�� 

           Servo1_Direction  = BACKWARD;                    // ��������1 ������ ȸ������
       }

      
       else if( Key_Data == KEY3 )     // Key 5�� ������ ��������1, 2�� ���۸�带 ����(���������, �Ŵ�����)
       {  
           Key_cnt++;   
           if( Key_cnt >= 2 ) Key_cnt = 0; 

           if( Key_cnt == 0 ) Operating_Mode = MANUAL_MODE;     // Key5�� ¦���� �������� �Ŵ����� 
           if( Key_cnt == 1 ) Operating_Mode = SEQUENCE_MODE;   // Key5�� Ȧ���� �������� ��������� 

           if( Operating_Mode == SEQUENCE_MODE ) //��������1,2�� ��������� �̸� ���ú����ʱ�ȭ 
           {         
               Step = 0 ;           // ��������1, ��������2 ���۽����� �ܰ� �ʱ�ȭ 
               curr_delay = 0;      // �ð���� üũ�Լ����� ���Ǵ� ī���� ���� �ʱ�ȭ 
           }
           else if( Operating_Mode == MANUAL_MODE )  //��������1,2�� �Ŵ����� �̸�  
           {         
              Servo1_Position = 90;                // ��������1 ��ġ���(Servo1_Position)�� 90���� ����
              Servo1_Move( Servo1_Position );      // �������� 1�� ��� ��ġ(90�� ��ġ)�� ȸ��
              PORTA |=  0x04;                      // LED1 OFF ( PA2 = 1 ) : ��������1 �������·� ǥ��
                                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 ) 

              
           }
       }


       ///////  LCD�� ��������1, ��������2 ���ۻ��� ���÷���  ///////////
       LcdCommand( ALLCLR );                          // LCD ȭ�� �����

	   LcdMove(0, 0);                                 // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 

       LcdPuts( "Servo Position" );                        // ��������1 ��ġ �޽��� ���÷��� 
       Display_Number_LCD( Servo1_Position, 3 );      // ��������1 ��ġ 3�ڸ�(000-180) ���÷��� 

       if( Servo1_Direction == FORWARD )        LcdPuts( " Fwd" );   // ��������1�� ������ ȸ������ �̸� 
                                                                     // ������ ȸ�� �޽��� ���÷��� 
       else if( Servo1_Direction == BACKWARD )  LcdPuts( " Rev" );   // ��������1�� ������ ȸ������ �̸� 
                                                                     // ������ ȸ�� �޽��� ���÷��� 

	  

    }   // ���ѷ��� while (1) �� �� 

}      // int main() �Լ��� �� 


///////////////////////////////////////////////////


ISR( TIMER2_OVF_vect )   // 10msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ �������α׷�
{                        // ���۸�尡 ����������̸� ��������1, ��������2�� �������̰� �ݺ����� ���۽����� ���� 

  TCNT2 = 256 - 156;          // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

  ///////  ��������1, ��������2 ���� ������ ////////////

  if( Operating_Mode == SEQUENCE_MODE )    // ���۸�尡 ����������̸� 
  {
      if( Step == 0 )                      // �ʱ���ġ�� ���ʹܰ� (Step 0�� �ʱ⿡ 1���� ���� ��)
      {

        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2�ʰ� ������� �ʾ����� (2�� ���� ȸ��)
        { 
           Servo1_Direction = BACKWARD;      // ��������1 ������ ȸ������ 
           Servo1_Position = 0;              // ��������1 ��ġ���(Servo1_Position)�� 0���� ����
           Servo1_Move( Servo1_Position );   // ��������1 0��(�ʱ������ġ)�� ���� 

           PORTA &=  ~0x04;                  // LED1 ON ( PA2 = 0 ) : ��������1 ȸ������ ǥ��
                                             // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

          

           
         }
         else                            // 2�� ��� �� 
         {
           Step++;                      // Step1 �� �Ѿ. 
         }
      }

      if( Step == 1 )      
	  {

        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2�ʰ� ������� �ʾ����� (2�� ���� ȸ��)
        { 
           Servo1_Direction = FORWARD;      // ��������1 ������ ȸ������ 
           Servo1_Position = 180;           // ��������1 ��ġ���(Servo1_Position)�� 180���� ����
           Servo1_Move( Servo1_Position );  // ��������1 0�� --> 180�� �� ȸ�� 

           PORTA &=  ~0x04;                 // LED1 ON ( PA2 = 0 ) : ��������1 ȸ������ ǥ��
                                            // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
 
           PORTA |=  0x08;                  // LED2 OFF ( PA3 = 1 ) : ��������2 �������� ǥ��
                                            // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                                // 2�� ��� �� 
        {
           Step=0;                          // Step2 �� �Ѿ. 
        }
      }

     

     

      
      

    }   // End of  if( Operating_Mode = SEQUENCE_MODE )

}       // End of  ISR( TIMER2_OVF_vect ) 

////////////////////////////////////////////////////////

void Servo1_Move( short sv_pos_cmd )    // ��������1�� �־��� ����(sv_pos_cmd) ��ŭ ȸ��. 0�� - 180���� ����
{
      if( sv_pos_cmd > 180 )   sv_pos_cmd = 180;   // ������(0�� - 180��)�� ����� �ʵ��� ���� 
      if( sv_pos_cmd < 0 )     sv_pos_cmd = 0; 

      OCR3A = ( 5 * sv_pos_cmd )/9  + 44 ;       // OC3A(PE3)������ ��µǴ� PWM ��ȣ�� �޽���(duty) ���� 
                                                 // �������Ͱ� ���� 

      //  �޽��� = 0.7msec = 16usec * 44,   ���� ��(0 ��)  (�޽��� = 0.7msec )
      //  �޽��� = 1.5msec = 16usec * 94 ,  ���(90 ��) (�޽��� = 1.5msec )
      //  �޽��� = 2.3msec = 16usec * 144 , ������ ��(180 ��) (�޽��� = 2.3msec ) 
}

void Servo2_Move( short sv_pos_cmd )   // ��������2�� �־��� ����(sv_pos_cmd) ��ŭ ȸ��. 0�� - 180���� ����
{
      if( sv_pos_cmd > 180 )   sv_pos_cmd = 180;   // ������(0�� - 180��)�� ����� �ʵ��� ���� 
      if( sv_pos_cmd < 0 )     sv_pos_cmd = 0; 

      OCR3B = ( 5 * sv_pos_cmd )/9  + 44 ;       // OC3B(PE4)������ ��µǴ� PWM ��ȣ�� �޽���(duty) ���� 
                                                 // �������Ͱ� ���� 

      //  �޽��� = 0.7msec = 16usec * 44,   ���� ��(0 ��)  (�޽��� = 0.7msec )
      //  �޽��� = 1.5msec = 16usec * 94 ,  ���(90 ��) (�޽��� = 1.5msec )
      //  �޽��� = 2.3msec = 16usec * 144 , ������ ��(180 ��) (�޽��� = 2.3msec ) 

}

/////////////////////////////////////////////////////

unsigned char Time_Delay_Polling( unsigned short d_time )  // �ð���� üũ�Լ�(�������)
{
    unsigned char  ret_val = 0;

    curr_delay++ ;  
    if( curr_delay >= d_time )   // 50msec * d_time ��� �� 
    {
       ret_val = 1; 
       curr_delay = 0 ;
    } 
    return  ret_val ;
}


/////////////////////////////////////                                                              
unsigned char Key_Input(void)   // �Լ� ȣ��� ���Ӱ� ������ key ���� ������(ä�͸� ���� �������.)
{                               // ������ key�� ���� �� 0(NO_KEY) �� ���ϵ�. �ѹ� ȣ��� ���� 50 msec �̻� �ð��� �ҿ�. 

    static unsigned char pin = NO_KEY; 
    unsigned char in, in1;

    in = key_scan();             // key �Է� �б� 
    while(1)                     // key �Է��� ����ȭ�ɶ����� ��ٸ�
    {
        msec_delay(50);          // ��ٿ�� �ð� ����     
        in1 = key_scan();        // key �Է� �б�     
        if( in == in1 ) break;
        in = in1;
    }
    if( !( in & 0xFF ) )         // key�� �������� �ʰ� ������  0 ( NO_KEY) ����
    {	
        pin= NO_KEY;				
        return NO_KEY;
    }
    if( pin == in )              // ������ key�� ��� �������� ������  0 ( NO_KEY) ����
    {
        return NO_KEY;  
    }
    pin = in;

    return  in;
}


//////////////////////////////////////////
unsigned char key_scan(void)   // Key_Input( ) �Լ����� ȣ���ϴ� ���� �Լ�. ������ ��� HW �Է���Ʈ�� ����� 
{                              // push ����ġ�� ���������� üũ(��ĵ)�ϴ� �Լ� ��. 
                               // key�� ���õ� HW ���� �� (key�� �ٸ� �Է���Ʈ�� ������ ��) �ݵ�� �����ؾ���.

    unsigned char   in = 0 ;
    
    if (       ( ~PINA  & 0x10  ) == 0x10  )        in = KEY1 ; 	  //   Key1 (PA4) �� ��������    	
    else if (  ( ~PINA  & 0x20  ) == 0x20  )        in = KEY2 ; 	  //   Key2 (PA5) �� ��������   		 
    else if (  ( ~PINA  & 0x40  ) == 0x40  )        in = KEY3 ; 	  //   Key3 (PA6) �� �������� 
        	 
    else                                             in = NO_KEY;   //	Key�� �������� ������ 
    
    return   in ;
    
}


void Display_Number_LCD( unsigned short num, unsigned char digit ) //��ȣ���� ����������(num)�� 10���� ���·� 
{                                                                  // ������ �ڸ���(digit) ��ŭ LCD �� ���÷��� �ϴ� �Լ� 
      HexToDec( num, 10); //10������ ��ȯ

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000�ڸ� ���ʷ��� 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000�ڸ� ���ʷ���
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100�ڸ� ���ʷ��� 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10�ڸ� ���ʷ���
      if( digit >= 1)  LcdPutchar(NumToAsc(cnumber[0]));    // 1�ڸ� ���÷���
}

void HexToDec( unsigned short num, unsigned short radix)   // num���� �Ѿ�� 16���� ������ �����͸� 10������ 
{       //  ��ȯ�Ͽ� ������ �ڸ����� �������� �迭 cnumber[0](1�ڸ�) - cnumber[4](10000�ڸ�)�� �����ϴ� �Լ�. 
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
	for(; n>0; n--)		         // 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		     // 1usec �ð� ����
}

