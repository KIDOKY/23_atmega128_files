
/////  < DC_Motor_Cont_2.c >  : Key(5��)�� �̿��Ͽ� DC����(2��)�� �ӵ��� �����ϴ� ATmega128 �ڵ�  ///// 

#include <avr/io.h>         // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.

// Key ���� �ɺ� ����
#define  NO_KEY  0
#define  KEY1     1
#define  KEY2     2
#define  KEY3     3
#define  KEY4     4
#define  KEY5     5

// ���� ���ۻ���(ȸ������, ����) ���� �ɺ� ����
#define  FORWARD   0
#define  REVERSE    1 
#define  STOP        2

// ���� ���۽����� ���� �ɺ� ����
#define  STOP_MODE  0
#define  RUN_MODE   1 

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

void DC_Motor1_Run_Fwd( short duty );     // DC ����1 ��ȸ��(PWM����) �Լ� 
void DC_Motor1_Run_Rev( short duty );     // DC ����1 ��ȸ��(PWM����) �Լ�  
void DC_Motor1_Stop( void );              // DC ����1 ���� �Լ�  
 
void DC_Motor2_Run_Fwd( short duty );     // DC ����2 ��ȸ��(PWM����) �Լ� 
void DC_Motor2_Run_Rev( short duty );     // DC ����2 ��ȸ��(PWM����) �Լ�  
void DC_Motor2_Stop( void );              // DC ����2 ���� �Լ�  

void DC_Motor1_PWM( short duty );         // DC ����1 PWM ���� �Լ�  
                                          // ��ȸ��(duty>0), ��ȸ��(duty<0), ����(duty=0) ��� ���� 
void DC_Motor2_PWM( short duty );         // DC ����2 PWM ���� �Լ�  
                                          // ��ȸ��(duty>0), ��ȸ��(duty<0), ����(duty=0) ��� ���� 

static volatile short  Duty1_Max = 100, Duty2_Max = 100, PWM_Period = 0;  //����1, ����2�� �ִ�PWM duty = 100[%]
static volatile short  Duty1_Min = 0, Duty2_Min = 0;   //����1, ����2�� �ּ�PWM duty = 0[%]
static volatile short  Motor1_Duty = 0, Motor2_Duty = 0, Step = 0 ;    //����1, ����2�� PWM duty [%]

static volatile unsigned short  curr_delay = 0;    // �ð���� üũ�Լ����� ���Ǵ� ī���� ���� 

static volatile unsigned char  DC_Motor1_Direction=0, DC_Motor2_Direction=0, Operating_Sequence=0, Key_cnt=0;


int main() 
{   
    unsigned char   Key_Data = 0 ;

    LcdInit();                     // LCd �ʱ�ȭ �Լ� 
    LcdMove(0,0);                 // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("DC Motor Control");  // LCD�� DC�������� �޽��� ���÷��� 
    LcdMove(1,0);                 // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Test Program");      // LCD�� DC�������� �޽��� ���÷���
    msec_delay(2000);            // 2��(2000msec) �ð�����

   ////////  Key 5�� ����� ���� HW ���� �������� ����  ////////////
   //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA &= ~0xF0;      // Key1(PA4), Key2(PA5), Key3(PA6), Key4(PA7): �Է���Ʈ�� ���� 
                           // �Է���Ʈ ����.  DDRA = 0b0000 ****, DDRA = DDRA & ~0b1111 0000(~0xF0)
    PORTA |= 0xF0;       // Key1(PA4), Key2(PA5), Key3(PA6), Key4(PA7)  : ����Ǯ�������� ����  
                           // PORTA = 0b1111 ****, PORTA = PORTA | 0b1111 0000( 0xF0 )

    DDRB &= ~0x01;      // Key5(PB0) : �Է���Ʈ�� ����   
                           // �Է���Ʈ ����.  DDRB = 0b**** ***0, DDRB = DDRB & ~0b0000 0001(~0x01)
    PORTB |= 0x01;       // Key5(PB0)  : ����Ǯ�������� ����  
                           // PORTB = 0b**** ***1, PORTB = PORTB | 0b0000 0001( 0x01 )
   // (����):  Key �Է���Ʈ ������ �����ϸ� key_scan() �Լ��� �����ؾ���. key_scan() �Լ� ���� ������ Ȯ���� ��. 

    ////////  DC���� 2�� ������ ���� HW ���� �������� ����  ////////////
    //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA |= 0x03;       // DC����1 ����������Ʈ(PA0), DC����2 ����������Ʈ(PA1) : �����Ʈ�� ����   
                        // DDRA = 0b**** **11, DDRA = DDRA | 0b0000 0011( 0x03 )

    DDRB |= 0x60;       // DC����1 �ӵ����� PWM��Ʈ(OC1A/PB5), DC����2 �ӵ����� PWM��Ʈ(OC1B/PB6): 
                        // �����Ʈ�� ����   
                        // DDRB = 0b*11* ****, DDRB = DDRB | 0b0110 0000( 0x60 )

    DDRA |= 0x0C;       // LED1(DC����1 ���ۻ���ǥ��)(PA2), LED2(DC����2 ���ۻ���ǥ��)(PA3) : �����Ʈ�� ����                          // DDRA = 0b**** 11**, DDRA = DDRA | 0b0000 1100( 0x0C )

    ////////  Timer1 ����( DC����1 PWM��ȣ(OC1A/PB5), DC����2 PWM��ȣ(OC1B/PB6) �߻� )   //////////// 
    ///////   PWM��ȣ ���ļ� = 5kHz (�ֱ� = 200usec )  //////////////////////
    ///////   ���� P238-245(�������� �׸��� ǥ12.2, ǥ12.4, ǥ12.5) ����    

    TCCR1A &= ~0x51;     // Fast PWM: ����ġ�� OC1A(PB5)), OC1B(PB6)���� 0�����ϰ� TOP���� 1�� ���
                         // (ǥ12.2 ����),   Fast PWM ( mode 14 ) ���� (ǥ12.4 ����)
                         // TCCR1A = 0b1010 **10 
                         // TCCR1A = TCCR1A & ~0b0101 0001(~0x51 )  
    TCCR1A |= 0xA2;      // TCCR1A = TCCR1A |   0b1010 0010( 0xA2 )  

    TCCR1B &= ~0x04;     // 64���� Ÿ�̸�1 ����(����Ŭ�� �ֱ�=64/(16*10^6Hz)=4usec ), Fast PWM(mode 14)����
                         // (ǥ12.4 - ǥ12.5 ����)
                         // TCCR1B = 0b***1 1011   
                         // TCCR1B = TCCR1B & ~0b0000 0100(~0x04 )  
    TCCR1B |= 0x1B;      // TCCR1B = TCCR1B |   0b0001 1011( 0x1B )  

    ICR1 = 50;           // PWM �ֱ�(���ļ�) ����( �ֱ�= 50*4usec = 200usec, ���ļ� = 1/(200usec) = 5kHz )
    PWM_Period = ICR1;   // PWM ��ȣ �ֱ⸦ ���������� ���� 

    Motor1_Duty = 0;         // DC����1 �ӵ� 0 ����( �ִ� = 100[%],  �ּ� = 0[%] )
    DC_Motor1_Stop( );       // DC����1 ����
    Motor2_Duty = 0;         // DC����2 �ӵ� 0 ����( �ִ� = 100[%],  �ּ� = 0[%] )
    DC_Motor2_Stop( );       // DC����2 ����

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

       if( Key_Data == KEY1 )             // Key 1�� ������
       {  
 	       Motor1_Duty += 5;                                          // DC����1 duty ���� 5�� ���� 
           if( Motor1_Duty > Duty1_Max )  Motor1_Duty = Duty1_Max;    // �ִ� ������ ����� Duty1_Max�� ���� 
       }

       else if( Key_Data == KEY2 )        // Key 2�� ������
       {  
            Motor1_Duty -= 5;                                        // DC����1 duty ���� 5�� ����
           if( Motor1_Duty < Duty1_Min )  Motor1_Duty = Duty1_Min;   // �ּ� ������ ����� Duty1_Min���� ���� 
       }
       else if( Key_Data == KEY3 )        // Key 3�� ������
       {  
  	       Motor2_Duty += 5;                                         // DC����2 duty ���� 5�� ���� 
           if( Motor2_Duty > Duty2_Max )  Motor2_Duty = Duty2_Max;  // �ִ� ������ ����� Duty2_Max�� ����  
       }
       else if( Key_Data == KEY4 )        // Key 4�� ������
       {  
           Motor2_Duty -= 5;                                        // DC����2 duty ���� 5�� ����
           if( Motor2_Duty < Duty2_Min )  Motor2_Duty = Duty2_Min;  // �ּ� ������ ����� Duty2_Min���� ����  
       } 

       else if( Key_Data == KEY5 )        // Key 5�� ������
       {  
           Key_cnt++;   
           if( Key_cnt >= 2 ) Key_cnt = 0; 

           if( Key_cnt == 0 ) Operating_Sequence = STOP_MODE;  // Key5�� ¦���� �������� ���� ������� 
           if( Key_cnt == 1 ) Operating_Sequence = RUN_MODE;   // Key5�� Ȧ���� �������� ���� ȸ����� 

           if( Operating_Sequence == RUN_MODE )   //DC����1,2�� ȸ�����(���۽����� ����)���۽� ���ú����ʱ�ȭ 
           {         
               Step = 0 ;         // DC����1, DC����2 ���۽����� �ܰ� �ʱ�ȭ 
               curr_delay = 0;    // �ð���� üũ�Լ����� ���Ǵ� ī���� ���� �ʱ�ȭ 
           }
           else if( Operating_Sequence == STOP_MODE )  // ���Ͱ� ������� ��  
           {         
              DC_Motor1_Direction = STOP;   // DC����1 �������� 
              DC_Motor1_Stop( );            // DC ����1 ����(PWM����) �Լ� ȣ�� 
              PORTA |=  0x04;               // LED1 OFF ( PA2 = 1 ) : DC����1 �������� ǥ��
                                            // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

              DC_Motor2_Direction = STOP;   // DC����2 �������� 
              DC_Motor2_Stop( );            // DC ����2 ����(PWM����) �Լ� ȣ��
              PORTA |=  0x08;               // LED2 OFF ( PA3 = 1 ) : DC����2 �������� ǥ��
                                            // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
           }
       }
       ///////  LCD�� ����1, ����2 ���ۻ��� ���÷���  ///////////

       LcdCommand( ALLCLR );              // LCD ȭ�� �����

       if( Operating_Sequence == STOP_MODE )   // ���� ���� ��� �̸� 
       {
	      LcdMove(0, 0);                  // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "Stop Mode!!" );       // ������� �޽��� ���÷��� 
	      LcdMove(1, 0);                  // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "M1:Stop, M2:Stop" );  // DC����1, DC����2 �������� �޽��� ���÷���  
       } 

       else if( Operating_Sequence == RUN_MODE )      // ���� ȸ�� ���(���۽����� ����) �̸� 
       { 
	      LcdMove(0, 0);                              // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 

          if( DC_Motor1_Direction == FORWARD )        // ����1�� ������ ȸ������ �̸� 
          {
              LcdPuts( "M1: Run Fwd " );              // ����1 ������ ȸ�� �޽��� ���÷��� 
              Display_Number_LCD( Motor1_Duty, 3 );   // ����1 �ӵ�(Duty) 3�ڸ� ���÷���
              LcdPutchar( '%' );                      // ���� % ���÷��� 
          }
          else if( DC_Motor1_Direction == REVERSE )   // ����1�� ������ ȸ������ �̸� 
          {
              LcdPuts( "M1: Run Rev " );              // ����1 ������ ȸ�� �޽��� ���÷��� 
              Display_Number_LCD( Motor1_Duty, 3 );   // ����1 �ӵ�(Duty) 3�ڸ� ���÷���
              LcdPutchar( '%' );                      // ���� % ���÷��� 
          }
          else if( DC_Motor1_Direction == STOP )      // ����1�� �������� �̸� 
          {
              LcdPuts( "M1: Stop " );                 // ����1 ���� �޽��� ���÷��� 
          }

	      LcdMove(1, 0);                              // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 

          if( DC_Motor2_Direction == FORWARD )        // ����2�� ������ ȸ������ �̸� 
          {
              LcdPuts( "M2: Run, Fwd " );             // ����2 ������ ȸ�� �޽��� ���÷��� 
              Display_Number_LCD( Motor2_Duty, 3 );   // ����2 �ӵ�(Duty) 3�ڸ� ���÷���
              LcdPutchar( '%' );                      // ���� % ���÷��� 
          }
          else if( DC_Motor2_Direction == REVERSE )   // ����2�� ������ ȸ������ �̸� 
          {
              LcdPuts( "M2: Run, Rev " );             // ����2 ������ ȸ�� �޽��� ���÷��� 
              Display_Number_LCD( Motor2_Duty, 3 );   // ����2 �ӵ�(Duty) 3�ڸ� ���÷���
              LcdPutchar( '%' );                      // ���� % ���÷��� 
          }
          else if( DC_Motor2_Direction == STOP )      // ����2�� �������� �̸� 
          {
              LcdPuts( "M2: Stop " );                 // ����2 ���� �޽��� ���÷��� 
          }

       }  // else if( Operating_Sequence == RUM_MODE ) �� ��

    }   // ���ѷ��� while (1) �� �� 

}      // int main() �Լ��� �� 



////////////////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )   // 10msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ �������α׷�
{                        // DC����1, DC����2 ���۽����� ���� 

    TCNT2 = 256 - 156;        // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

   ///////  DC����1, DC����2 ���� ������ ////////////

  if( Operating_Sequence == RUN_MODE )
  {

      if( Step == 0 )      
	  { 

         if( Time_Delay_Polling( 200 ) != 1 )    //10msec * 200=2000msec=2�ʰ� ������� �ʾ����� (2�� ���� ȸ��)
         { 
             DC_Motor1_Direction = FORWARD;      // DC����1 ������ ȸ������ 
             DC_Motor1_Run_Fwd( Motor1_Duty );   // DC ����1 ��ȸ��(PWM����) �Լ�(2�ʵ��� ��ȸ��) 
             PORTA &=  ~0x04;                    // LED1 ON ( PA2 = 0 ) : DC����1 ȸ������ ǥ��
                                                 // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

             DC_Motor2_Direction = REVERSE;      // DC����2 ������ ȸ������ 
             DC_Motor2_Run_Rev( Motor2_Duty );   // DC ����2 ��ȸ��(PWM����) �Լ�(2�ʵ��� ��ȸ��)  
             PORTA &=  ~0x08;                    // LED2 ON ( PA3 = 0 ) : DC����2 ȸ������ ǥ��
                                                 // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
          }
          else                             // 2�� ��� �� 
          {
             Step++;                      // Step1 �� �Ѿ. 
          }
      }

      if( Step == 1 )      
	  {   

         if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1�ʰ� ������� �ʾ�����(1�� ���� ����)
         { 
             DC_Motor1_Direction = STOP;      // DC����1 �������� 
             DC_Motor1_Stop( );               // DC ����1 ����(PWM����) �Լ� ȣ�� (1�ʵ��� ����) 
             PORTA |=  0x04;                  // LED1 OFF ( PA2 = 1 ) : DC����1 �������� ǥ��
                                              // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

             DC_Motor2_Direction = STOP;      // DC����2 �������� 
             DC_Motor2_Stop( );               // DC ����2 ����(PWM����) �Լ� ȣ�� (1�ʵ��� ����)
             PORTA |=  0x08;                  // LED2 OFF ( PA3 = 1 ) : DC����2 �������� ǥ��
                                              // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
         }
         else                             // 1�� ��� �� 
         {
             Step++;                      // Step2 �� �Ѿ. 
         }
      }

      if( Step == 2 )      
	  {   

         if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec= 2�ʰ� ������� �ʾ�����(2�� ���� ȸ��) 
         { 
             DC_Motor1_Direction = REVERSE;     // DC����1 ������ ȸ������ 
             DC_Motor1_Run_Rev( Motor1_Duty );  // DC ����1 ��ȸ��(PWM����) �Լ� ȣ��(2�ʵ��� ��ȸ��) 
             PORTA &=  ~0x04;                   // LED1 ON ( PA2 = 0 ) : DC����1 ȸ������ ǥ��
                                                // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 ) 

             DC_Motor2_Direction = FORWARD;     // DC����2 ������ ȸ������ 
             DC_Motor2_Run_Fwd( Motor2_Duty );  // DC ����2 ��ȸ��(PWM����) �Լ� ȣ��(2�ʵ��� ��ȸ��) 
             PORTA &=  ~0x08;                   // LED2 ON ( PA3 = 0 ) : DC����2 ȸ������ ǥ��
                                                // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
         }
         else                             // 2�� ��� �� 
         {
             Step++;                      // Step3 ���� �Ѿ. 
         }
      }

      if( Step == 3 )      
	  {   

         if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1�ʰ� ������� �ʾ�����(1�� ���� ����)
         { 
             DC_Motor1_Direction = STOP;   // DC����1 �������� 
             DC_Motor1_Stop( );            // DC ����1 ����(PWM����) �Լ� ȣ��  
             PORTA |=  0x04;               // LED1 OFF ( PA2 = 1 ) : DC����1 �������� ǥ��
                                           // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

             DC_Motor2_Direction = STOP;   // DC����2 �������� 
             DC_Motor2_Stop( );            // DC ����2 ����(PWM����) �Լ� ȣ��
             PORTA |=  0x08;               // LED2 OFF ( PA3 = 1 ) : DC����2 �������� ǥ��
                                           // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
         }
         else                       // 1�� ��� �� 
         {
             Step = 0;              // ���۽����� ����. �ʱ�ܰ�(Step = 0)�� ���ư��� �ٽ� �ݺ�. 
         }

      } 

    }   // End of  if( Operating_Sequence = RUN_MODE )


}       // End of  ISR( TIMER2_OVF_vect ) 

////////////////////////////////////////////////////////


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

    PORTA |= 0x01;               // ȸ�����⼳��(��ȸ��) :   DC����1 B����(-) = 1(High) ( PA0 = 1 )    
    OCR1A = PWM_Period - duty;   // ȸ���ӵ�(PWM Duty)���� : DC����1 A����(+) = PWM��ȣ(OC1A(PB5))  
}

void DC_Motor1_Stop( void )   // DC ����1 ���� �Լ� 
{
    PORTA &= ~0x01;      // ȸ�����⼳��(��ȸ��) :          DC����1 B����(-) = 0(Low) ( PA0 = 0 )    
    OCR1A = 0;           // ȸ���ӵ� 0(PWM Duty = 0) ���� : DC����1 A����(+) = PWM��ȣ(OC1A(PB5))  
}

void DC_Motor2_Run_Fwd( short duty )   // DC ����2 ��ȸ�� �Լ� 
{
    if( duty > Duty2_Max )     duty = Duty2_Max;  // duty ������ ����� Duty2_Max�� ���� 
    duty = ( duty*PWM_Period )/Duty2_Max;         // %������ duty ���� ���� ��������(OCR1B) ���� ������ ��ȯ


    PORTA &= ~0x02;      // ȸ�����⼳��(��ȸ��) :   DC����2 B����(-) = 0(Low) ( PA1 = 0 )    
    OCR1B = duty;        // ȸ���ӵ�(PWM Duty)���� : DC����2 A����(+) = PWM��ȣ(OC1B(PB6))  
}

void DC_Motor2_Run_Rev( short duty )   // DC ����2 ��ȸ�� �Լ� 
{
    if( duty > Duty2_Max )     duty = Duty2_Max;  // duty ������ ����� Duty2_Max�� ���� 
    duty = ( duty*PWM_Period )/Duty2_Max;         // %������ duty ���� ���� ��������(OCR1B) ���� ������ ��ȯ

    PORTA |= 0x02;               // ȸ�����⼳��(��ȸ��) :   DC����2 B����(-) = 1(High) ( PA1 = 1 )    
    OCR1B = PWM_Period - duty;   // ȸ���ӵ�(PWM Duty)���� : DC����2 A����(+) = PWM��ȣ(OC1B(PB6))  
}

void DC_Motor2_Stop( void )   // DC ����2 ���� �Լ� 
{
    PORTA &= ~0x02;      // ȸ�����⼳��(��ȸ��) :          DC����2 B����(-) = 0(Low) ( PA1 = 0 )    
    OCR1B = 0;           // ȸ���ӵ� 0(PWM Duty = 0) ���� : DC����2 A����(+) = PWM��ȣ(OC1B(PB6))  
}

void DC_Motor1_PWM( short duty )   // DC ����1 PWM ���� �Լ�  
{
   if ( duty > Duty1_Max )       duty = Duty1_Max;    // duty�� ���� �ִ���� ���� ũ�� Duty1_Max�� ���� 
   else if( duty < -Duty1_Max )  duty = -Duty1_Max;   // duty�� ���� �ּҹ��� ���� ������ -Duty1_Max�� ���� 

   if( duty > 0 )          DC_Motor1_Run_Fwd( duty );
   else if( duty == 0 )    DC_Motor1_Stop();
   else if( duty < 0 )     DC_Motor1_Run_Rev( -duty );
}

void DC_Motor2_PWM( short duty )   // DC ����2 PWM ���� �Լ�  
{
   if ( duty > Duty2_Max )       duty = Duty2_Max;   // duty�� ���� �ִ���� ���� ũ�� Duty2_Max�� ���� 
   else if( duty < -Duty2_Max )  duty = -Duty2_Max;   // duty�� ���� �ּҹ��� ���� ������ -Duty2_Max�� ���� 

   if( duty > 0 )         DC_Motor2_Run_Fwd( duty );
   else if( duty == 0 )   DC_Motor2_Stop();
   else if( duty < 0 )    DC_Motor2_Run_Rev( -duty );
}
 

/////////////////////////////////////   
                                                           
unsigned char Key_Input(void)   // �Լ� ȣ��� ���Ӱ� ������ key ���� ������(ä�͸� ���� �������.)
{                      // ������ key�� ���� �� 0(NO_KEY) �� ���ϵ�. �ѹ� ȣ��� ���� 50 msec �̻� �ð��� �ҿ�. 
    static unsigned char pin = NO_KEY; 
    unsigned char in, in1;

    in = key_scan();             // key �Է� �б� 
    while(1)                     // key �Է��� ����ȭ�ɶ����� ��ٸ�
    {
        msec_delay(50);         // ��ٿ�� �ð� ����     
        in1 = key_scan();        // key �Է� �б�     
        if( in == in1 ) break;
        in = in1;
    }
    if( !( in & 0xFF ) )    // key�� �������� �ʰ� ������  0 ( NO_KEY) ����
    {	
        pin= NO_KEY;				
        return NO_KEY;
    }
    if( pin == in )         // ������ key�� ��� �������� ������  0 ( NO_KEY) ����
    {
        return NO_KEY;  
    }
    pin = in;

    return  in;
}

//////////////////////////////////////////

unsigned char key_scan(void)  // Key_Input( ) �Լ����� ȣ���ϴ� ���� �Լ�. ������ ��� HW �Է���Ʈ�� ����� 
{                              // push ����ġ�� ���������� üũ(��ĵ)�ϴ� �Լ� ��. 
                               // key�� ���õ� HW ���� �� (key�� �ٸ� �Է���Ʈ�� ������ ��) �ݵ�� �����ؾ���.
    unsigned char   in = 0 ;
    
    if (       ( ~PINA  & 0x10  ) == 0x10  )        in = KEY1 ; 	  //   Key1 (PA4) �� ��������    	
    else if (  ( ~PINA  & 0x20  ) == 0x20  )        in = KEY2 ; 	  //   Key2 (PA5) �� ��������   		 
    else if (  ( ~PINA  & 0x40  ) == 0x40  )        in = KEY3 ; 	  //   Key3 (PA6) �� �������� 
    else if (  ( ~PINA  & 0x80  ) == 0x80  )        in = KEY4 ; 	  //   Key4 (PA7) �� ��������   		 
    else if (  ( ~PINB  & 0x01  ) == 0x01  )        in = KEY5 ; 	  //   Key5 (PB0) �� ��������     	 
    else                                             in = NO_KEY;   //	Key�� �������� ������ 
    
    return   in ;
    
}
/////////////////////////////////////////


unsigned char Time_Delay_Polling( unsigned short d_time )  // �ð���� üũ�Լ�(�������)
{
    static unsigned short  curr_delay = 0; 
    unsigned char  ret_val = 0;

    curr_delay++ ;  
    if( curr_delay >= d_time )   // 50msec * d_time ��� �� 
    {
       ret_val = 1; 
       curr_delay = 0 ;
    } 
    return  ret_val ;
}

//////////////////////////////////////////

void Display_Number_LCD( unsigned short num, unsigned char digit ) //��ȣ���� ����������(num)�� 10���� ���·� 
{                                                       // ������ �ڸ���(digit) ��ŭ LCD �� ���÷��� �ϴ� �Լ� 
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


char NumToAsc( unsigned char Num )   // Num���� �Ѿ�� 16���� 1�ڸ� ���ڸ� ���ڵ�����(ASCII �ڵ�)�� 
{                                        // ��ȯ�Ͽ� �����ϴ� �Լ�
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}


void msec_delay(int n)           // n msec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		// 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		// 1msec �ð� ����
}
void usec_delay(int n)            // n usec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		// 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		// 1usec �ð� ����
}

