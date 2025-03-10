
/////////   < DC_Motor_Cont_1.c >  : DC ���� 2���� �����ϴ� ATmega128 �ڵ�   //////////

#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.


void msec_delay(int n);    // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����

unsigned char Time_Delay_Polling( unsigned short d_time );   // �ð���� üũ�Լ�(�������)

void DC_Motor1_Run_Fwd( short duty );    // DC ����1 ��ȸ��(PWM����) �Լ� 
void DC_Motor1_Run_Rev( short duty );    // DC ����1 ��ȸ��(PWM����) �Լ�  
void DC_Motor1_Stop( void );             // DC ����1 ���� �Լ�  
 
void DC_Motor2_Run_Fwd( short duty );    // DC ����2 ��ȸ��(PWM����) �Լ� 
void DC_Motor2_Run_Rev( short duty );    // DC ����2 ��ȸ��(PWM����) �Լ�  
void DC_Motor2_Stop( void );             // DC ����2 ���� �Լ�  

void DC_Motor1_PWM( short duty );        // DC ����1 PWM ���� �Լ�  
                                         // ��ȸ��(duty>0), ��ȸ��(duty<0), ����(duty=0) ��� ���� 
void DC_Motor2_PWM( short duty );        // DC ����2 PWM ���� �Լ�  
                                         // ��ȸ��(duty>0), ��ȸ��(duty<0), ����(duty=0) ��� ���� 

static volatile short  Duty1_Max= 100, Duty2_Max=100, PWM_Period=0;    //����1, ����2�� �ִ�PWM duty = 100[%]
static volatile short  Motor1_Duty = 0, Motor2_Duty = 0, Step = 0 ;    //����1, ����2�� PWM duty [%]


int main() 
{   
    short  duty_1 = 0, duty_2 = 0 ; 

    LcdInit();                     // LCd �ʱ�ȭ �Լ� 

    LcdMove(0,0);                 // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("DC Motor Control");  // LCD�� DC�������� �޽��� ���÷��� 
    LcdMove(1,0);                 // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Test Program");      // LCD�� DC�������� �޽��� ���÷���
    msec_delay(2000);            // 2��(2000msec) �ð�����

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

       cli();                             // �������ͷ�Ʈ ����

 	   duty_1 = Motor1_Duty;              // DC����1 duty ���� ���� duty_1�� ���� 
 	   duty_2 = Motor2_Duty;              // DC����2 duty ���� ���� duty_2�� ���� 

       sei();                             // �������ͷ�Ʈ ���

       if( duty_1 == 0 && duty_2 == 0 )      // DC����1, DC����2 ��� ���������̸� 
       {  
          LcdCommand( ALLCLR );              // LCD ȭ�� �����
	      LcdMove(0, 0);                     // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "Motor1 : Stop" );        // DC����1 �������� �޽��� ���÷���  
	      LcdMove(1, 0);                     // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "Motor2 : Stop" );        // DC����2 �������� �޽��� ���÷��� 
       }

       else if( duty_1 == 0 && duty_2 != 0 )    // DC����1 ��������, DC����2 ȸ�������̸� 
       {  
          LcdCommand( ALLCLR );                 // LCD ȭ�� �����
	      LcdMove(0, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "Motor1 : Stop" );           // DC����1 �������� �޽��� ���÷���  
	      LcdMove(1, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "Motor2 : Run" );            // DC����2 ȸ������ �޽��� ���÷��� 
       }

       else if( duty_1 != 0 && duty_2 == 0 )    // DC����1 ȸ������, DC����2 ���������̸� 
       {  
          LcdCommand( ALLCLR );                 // LCD ȭ�� �����
	      LcdMove(0, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "Motor1 : Run" );            // DC����1 ȸ������ �޽��� ���÷���  
	      LcdMove(1, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "Motor2 : Stop" );           // DC����2 �������� �޽��� ���÷��� 
       }

       else if( duty_1 != 0 && duty_2 != 0 )    // DC����1, DC����2 ��� ȸ�������̸� 
       {  
          LcdCommand( ALLCLR );                 // LCD ȭ�� �����
	      LcdMove(0, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "Motor1 : Run" );            // DC����1 ȸ������ �޽��� ���÷���  
	      LcdMove(1, 0);                        // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "Motor2 : Run" );            // DC����2 ȸ������ �޽��� ���÷��� 
       }
      
       msec_delay(50);      // 50msec �ð����� (�������� LCD ���÷��̸� ����.)

   }   // ���ѷ��� while (1) �� �� 

}      // int main() �Լ��� �� 


////////////////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )   // 10msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ �������α׷�
{                        // DC����1, DC����2 ���۽����� ���� 

    TCNT2 = 256 - 156;        // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    ///////  DC����1, DC����2 ���� ������ ////////////

    if( Step == 0 )    
	{
        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec=2�ʰ� ������� �ʾ����� (2�� ���� ȸ��)
        { 
           Motor1_Duty = 80;                    // DC����1 duty(�ӵ�) = 80[%] ���� 
           DC_Motor1_Run_Fwd( Motor1_Duty );    // DC ����1 ��ȸ��(PWM����) �Լ�(2�ʵ��� ��ȸ��) 
           PORTA &=  ~0x04;                     // LED1 ON ( PA2 = 0 ) : DC����1 ȸ������ ǥ��
                                                // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )

           Motor2_Duty = 80;                    // DC����2 duty(�ӵ�) = 80[%] ����
           DC_Motor2_Run_Rev( Motor2_Duty );    // DC ����2 ��ȸ��(PWM����) �Լ�(2�ʵ��� ��ȸ��)  
           PORTA &=  ~0x08;                     // LED2 ON ( PA3 = 0 ) : DC����2 ȸ������ ǥ��
                                                // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
        }
        else                                    // 2�� ��� �� 
        {
           Step++;                              // Step1 �� �Ѿ. 
        }
    }

    if( Step == 1 )    
	{   
        if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1�ʰ� ������� �ʾ�����(1�� ���� ����)
        { 
           Motor1_Duty = 0;        // DC����1 duty = 0[%] ����(����) 
           DC_Motor1_Stop( );      // DC ����1 ����(PWM����) �Լ� ȣ�� (1�ʵ��� ����) 

           PORTA |=  0x04;         // LED1 OFF ( PA2 = 1 ) : DC����1 �������� ǥ��
                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
           Motor2_Duty = 0;        // DC����2 duty = 0[%] ����(����)  
           DC_Motor2_Stop( );      // DC ����2 ����(PWM����) �Լ� ȣ�� (1�ʵ��� ����)

           PORTA |=  0x08;         // LED2 OFF ( PA3 = 1 ) : DC����2 �������� ǥ��
                                   // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                       // 1�� ��� �� 
        {
           Step++;                 // Step2 �� �Ѿ. 
        }
    }

    if( Step == 2 )    
	{   
        if( Time_Delay_Polling( 200 ) != 1 )  //10msec * 200=2000msec= 2�ʰ� ������� �ʾ�����(2�� ���� ȸ��) 
        { 
           Motor1_Duty = 100;                  // DC����1 duty(�ӵ�) = 100[%] ���� (�ִ�ӵ�) 
           DC_Motor1_Run_Rev( Motor1_Duty );   // DC ����1 ��ȸ��(PWM����) �Լ� ȣ��(2�ʵ��� ��ȸ��) 

           PORTA &=  ~0x04;                    // LED1 ON ( PA2 = 0 ) : DC����1 ȸ������ ǥ��
                                               // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 ) 

           Motor2_Duty = 100;                  // DC����2 duty(�ӵ�) = 100[%] ���� (�ִ�ӵ�) 
           DC_Motor2_Run_Fwd( Motor2_Duty );   // DC ����2 ��ȸ��(PWM����) �Լ� ȣ��(2�ʵ��� ��ȸ��) 

           PORTA &=  ~0x08;                    // LED2 ON ( PA3 = 0 ) : DC����2 ȸ������ ǥ��
                                               // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
        }
        else                                   // 2�� ��� �� 
        {
           Step++;                             // Step3 ���� �Ѿ. 
        }
    }

    if( Step == 3 )    
	{   
        if( Time_Delay_Polling( 100 ) != 1 )  //10msec * 100=1000msec= 1�ʰ� ������� �ʾ�����(1�� ���� ����)
        { 
           Motor1_Duty = 0;        // DC����1 duty = 0[%] ����(����) (1�ʵ��� ����)
           DC_Motor1_Stop( );      // DC ����1 ����(PWM����) �Լ� ȣ��  

           PORTA |=  0x04;         // LED1 OFF ( PA2 = 1 ) : DC����1 �������� ǥ��
                                   // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )

           Motor2_Duty = 0;        // DC����2 duty = 0[%] ����(����) (1�ʵ��� ����)
           DC_Motor2_Stop( );      // DC ����2 ����(PWM����) �Լ� ȣ��

           PORTA |=  0x08;         // LED2 OFF ( PA3 = 1 ) : DC����2 �������� ǥ��
                                   // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
        }
        else                       // 1�� ��� �� 
        {
           Step = 0;              // ���۽����� ����. �ʱ�ܰ�(Step0)�� ���ư��� �ٽ� �ݺ�. 
        }

    } 

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

