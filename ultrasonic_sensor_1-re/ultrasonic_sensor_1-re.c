
// < ultrasonic_sensor_1.c > :�����ļ������(HC-SR04)�� �̿��Ͽ� ��ֹ������� �Ÿ��� �����ϴ� ATmega128 �ڵ� 

#include <avr/io.h>         // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 
#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���. 


void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // ��ȣ���� ������ ������ 10���� ���·� 
                                                                      // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);    // msec ���� �ð�����
void usec_delay(int n);    // usec ���� �ð�����

static volatile unsigned short    distance_1 = 0,  distance_1_prev = 0;
static volatile unsigned char     flag = 0;


int main() 
{   

    unsigned short  dist_1 = 0;


    LcdInit();                     // LCd �ʱ�ȭ �Լ� 

    LcdMove(0,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("UltrasonicSensor");   // LCD�� ���ڿ� ���÷��� 
    LcdMove(1,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Dist_1 =    cm");     // LCD�� ���ڿ� ���÷���

    //// �����ļ������(HC-SR04) ������ ���� HW ���� �������� ����  ////////////
    //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRB |= 0x08;        // �����ļ��� Trigger ��ȣ( PB3 : �����Ʈ ����  )
    PORTB &= ~0x08;      // PB3  : Low  ( Trigger ��ȣ OFF )  
    DDRE &= ~0x40;       // Echo ��ȣ( �ܺ����ͷ�Ʈ6, INT6(PE6) : �Է���Ʈ ���� )

    ////////////  Timer 0 ����  ( 10 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���� )  ///////////////
    // ���� P133-137(�������� �׸��� ǥ8.1-ǥ8.5) ����    

    TCCR0 &= ~0x48;         // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 0 ����(1024���� ���)
                            // TCCR0 = 0b*0**0111 
                            // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;          // TCCR0 =TCCR0 | 0b00000111( 0x07 ) 
	 
    TCNT0 = 256 - 156;      // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                            // �����÷����ͷ�Ʈ �ֱ� = 10msec
                            // 156 = 10msec/ 64usec,  TCNT0 = 256 - 156 

    TIMSK |= 0x01;          // Ÿ�̸�0 �����÷� ���ͷ�Ʈ ��� 

    ////////////////////////////////////////////////////////////////////////////////////////////    
    // Echo ��ȣ �޽��� �ð� ������ ���� Timer 3 ����
    // ���� PP238-244(�������� �׸��� ǥ12.4 - ǥ12.5 ����) 

    TCCR3A &= ~0x03;         // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 3 ����(���ֺ� 8) 
                             // ����Ŭ���ֱ� = 8/ (16x10^6) = 0.5 usec (0.5usec ������ ����) 
                             // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                             // TCCR3A = TCCR3A & ~0b00000011(~0x03)   
    TCCR3B &= ~0x1D;         // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;         // TCCR3B = TCCR3B | 0b00000010( 0x02 )  
	 
   //////////////////////////////////////////////////////////////////////////////////////////
   // �ܺ����ͷ�Ʈ 6( pin: INT6/PE6 ) :  �����ļ������ Echo ��ȣ �Է� 
   // ���� pp108-109 (�������� �׸�, ǥ 7.4-ǥ7.5 ���� )

    EICRB &= ~0x20;        // INT6 : �ϰ�����(falling edge) ��¿���(rising edge) ��ο��� ���ͷ�Ʈ �䱸
                           // EICRB = 0b**01**** 
                           // EICRB = EICRB & ~0b00100010(~0x20 ) 
    EICRB |=   0x10;       // EICRB = EICRB | 0b00010000( 0x10 ) 
 
    EIMSK |= 0x40;         // INT6 Enable(���) 
    sei();                 // ���� ���ͷ�Ʈ ���

///////////////////////////////////////////////////////////////////////////////////
	 
    while (1) 
    { 

       cli();                           // �������ͷ�Ʈ ���� 

 	   dist_1 = distance_1 ;            // �Ÿ��������� ���� dist_1�� ���� 

       sei();                           // �������ͷ�Ʈ ��� 

	   LcdMove(1, 9);                   // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 9��)
       Display_Number_LCD(dist_1, 3);   // �Ÿ�������(���� ��st_1)�� 100�ڸ����� ���÷��� 

   }

}      // int main() �Լ��� �� 

///////////////////////////////////////////////////////////////

ISR( TIMER0_OVF_vect )     //  10 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ �������α׷�
{                          //  50msec ���� �����Ľ�ȣ �߻� ��û��ȣ(Trigger ��ȣ) ���

    static unsigned short  time_index = 0; 


    TCNT0 = 256 - 156;     //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                           //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156

    time_index++ ; 

    if( time_index == 5 )   // 50 msec (=10msec * 5) ���� 
    {

       time_index = 0;    // �ʱ�ȭ

       //  �����ļ��� Ʈ���� ��ȣ ���(������ �߻�) 
	   PORTB |= 0x08;      // PA0 : High
	   usec_delay(20);     // 20usec ���� High ���� 
	   PORTB &= ~0x08;     // PA0 : Low    

       flag = 0;   
   }

}

//////////////////////////////////////////////////////////

ISR( INT6_vect )     // �ܺ����ͷ�Ʈ 6(INT6) ���� ���α׷�
{                    // Echo ��ȣ �޽����� �ð� ���� �� ���� ��ֹ������� �Ÿ� ��� 

      static unsigned short  count1 = 0, count2 = 0, del_T = 0; 


	  if( flag == 0 )        // Echo��ȣ�� ��¿������� ���ͷ�Ʈ �ɸ� �� 
	  {

		  count1 = TCNT3;    // ��¿��������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  flag = 1;          // flag ���� 1�� ����

	  } 
	  else if( flag == 1 )    // Echo��ȣ�� �ϰ��������� ���ͷ�Ʈ �ɸ� ��
	  { 

		  count2 = TCNT3;                     // �ϰ����������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  del_T = ( count2 - count1 ) / 2;    // Echo ��ȣ �޽����� �ð� ����(usec ����)
    	  distance_1 = del_T / 58;            // ���� ��ֹ������� �Ÿ����(cm ����)

          if( distance_1 > 380 )              // �ݻ�Ǵ� �����İ� ������� ������ 
		  {
		      distance_1 = distance_1_prev ;  // ���� �Ÿ������� ��� 
		  } 

          distance_1_prev = distance_1;       // ���� �Ÿ������� ���� ���� ������Ʈ  
		  flag = 0;                           // flag ���� 0���� ���� 
	  } 

} 

////////////////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned int num, unsigned char digit )   //��ȣ���� ����������(num)�� 10���� ���·� 
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

void msec_delay(int n)      // n msec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		    // 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		// 1msec �ð� ����
}

void usec_delay(int n)      // n usec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)		    // 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		// 1usec �ð� ����
}



