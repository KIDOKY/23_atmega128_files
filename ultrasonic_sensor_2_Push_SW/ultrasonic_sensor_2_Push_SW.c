
// < ultrasonic_sensor_2.c >  :   2���� �����ļ������(HC-SR04)�� ���ÿ� �����ϴ� ATmega128 �ҽ� �ڵ� 

#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 
#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.

#define NO_KEY    0


#define KEY1      1  
#define KEY2      2
#define KEY3      3            
 

// �Լ� ����

unsigned char key_scan(void);

unsigned char KeyInput(void);


void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // ��ȣ���� ������ ������ 10���� ���·� 
                                                                      // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);    // msec ���� �ð�����
void usec_delay(int n);    // usec ���� �ð�����

unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // �ð����� üũ�Լ�(�������)

static volatile unsigned short    distance_1 = 0, distance_2 = 0, distance_min = 300; 
static volatile unsigned short    distance_1_prev = 0, distance_2_prev = 0; 
static volatile unsigned char     sensor_count = 0, active_sensor_flag = 0;
static volatile  unsigned char    Warning_Flag = 0, flag = 0 ;
static volatile  unsigned short   Delay_Time = 0, curr_delay = 0;


int main() 
{   

    unsigned short  dist_1 = 0,  dist_2 = 0, pressed_key = 0, cnt = 0;


    LcdInit();                     // LCd �ʱ�ȭ �Լ� ȣ��

    LcdMove(0,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("Dist_1 =    cm");     // LCD�� ���ڿ� ���÷��� 

    LcdMove(1,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)  
    LcdPuts("Dist_2 =    cm");     // LCD�� ���ڿ� ���÷��� 

   //// 2���� �����ļ������(HC-SR04) ������ ���� HW ���� �������� ����  ////////////

  //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA |= 0x03;        // 2���� �����ļ��� Trigger ��ȣ( Trigger��ȣ1 : PA0, Trigger��ȣ2 : PA1 )�����Ʈ ����. 
                         // DDRA = 0b******11, DDRA = DDRA | 0b00000011(0x03)   
    PORTA &= ~0x03;      // PA0 : Low,  PA1 : Low  ( Trigger ��ȣ OFF )  
                         // PORTA = 0b******00, PORTA = PORTA & ~0b00000011(~0x03)  

    DDRE &= ~0x30;       // 2���� �����ļ��� Echo ��ȣ( �ܺ����ͷ�Ʈ4(INT4/PE4), �ܺ����ͷ�Ʈ5(INT5/PE5) 
                         // �Է���Ʈ ����.  DDRE = 0b**00****, DDRE = DDRE & ~0b00110000(~0x30) 
						   
    DDRA |= 0x08;        // ����(Buzzer) ( PA3 : �����Ʈ ���� )
                         // DDRA = 0b****1***, DDRA = DDRA | 0b00001000(0x08)   
    PORTA &= ~0x08;      // PA3  : Low  ( ���� OFF )  
                         // PORTA = 0b****0***, PORTA = PORTA | ~0b00001000(~0x08) 
						  
    DDRA |= 0x10;        // LED ( PA4 : �����Ʈ ���� )
                         // DDRA = 0b***1****, DDRA = DDRA | 0b00010000(0x10)   
    PORTA |= 0x10;       // PA4  : High ( LED OFF)    
                         // PORTA = 0b***1****, PORTA = PORTA | 0b00010000(0x10)  
	
	DDRB &= ~0x01;        // Push SW ( PB0 : �Է���Ʈ ���� )
                          // DDRB = 0b*******0, DDRA = DDRA & 0b00000001(~0x01)   
    PORTB |= 0x01;        // PORTB : 0b*******1	:	High ����Ǯ������ ������ ����    
                          // PORTB = PORTB | 0b00010000(0x10)

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

    ////////////////////////////////////////////////////////////////////////////     
    // Echo ��ȣ �޽��� �ð� ������ ���� Timer 3 ����
   // ���� PP238-244(�������� �׸��� ǥ12.4 - ǥ12.5 ����) 

    TCCR3A &= ~0x03;      // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 3 ����(���ֺ� 8) 
                          // ����Ŭ���ֱ� = 8/ (16x10^6) = 0.5 usec (0.5usec ������ ����) 
                          // TCCR3A = 0b******00,  TCCR3B = 0b***00010 
                          // TCCR3A = TCCR3A & ~0b00000011(~0x03)  
						   
    TCCR3B &= ~0x1D;      // TCCR3B = TCCR3B & ~0b00011101(~0x1D)                               
    TCCR3B |=  0x02;      // TCCR3B = TCCR3B | 0b00000010( 0x02 )  

   ///////////////////////////////////////////////////////////////////////////// 

   // �ܺ����ͷ�Ʈ 4( pin: INT4/PE4 ) ���� :  �����ļ������ Echo ��ȣ1�� �Էµ�.    
   // �ܺ����ͷ�Ʈ 5( pin: INT5/PE5 ) ���� :  �����ļ������ Echo ��ȣ2�� �Էµ�.  
   // ���� pp108-109 (�������� �׸�, ǥ 7.4-ǥ7.5 ���� )

    EICRB &= ~0x0A;       // INT4, INT5 : �ϰ�����(falling edge) ��¿���(rising edge) ��ο��� ���ͷ�Ʈ �䱸
                          // EICRB = 0b****0101 
                          // EICRB = EICRB & ~0b00001010(~0x0A ) 
    EICRB |=   0x05;      // EICRB = EICRB | 0b00000101( 0x05 ) 

    EIMSK |= 0x30;        // INT4, INT5 Enable(���) 
                          // EIMSK = 0b**11****,  EIMSK = EIMSK | 0b00110000( 0x30 ) 
    sei();                // ���� ���ͷ�Ʈ ���
 
	 
    while (1) 
    {
	   pressed_key = KeyInput();
	   
	   if(	pressed_key == KEY1 )
	   {
	   		cnt++;
			if( cnt == 2 ) cnt = 0;

			if( cnt == 1 )		//Key1 ��  Ȧ���� ��������
			{
				//TIMSK |= 0x01;		    // Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���
				sei();
				
				TCNT0 = 256 - 156;      // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,
										// �����÷����ͷ�Ʈ �ֱ� = 10msec
                              
				EIMSK |= 0x30;        // INT4, INT5 Enable(���) 
                          			  // EIMSK = 0b**11****,  EIMSK = EIMSK | 0b00110000( 0x30 ) 
				flag = 0;
				sensor_count = 0;
				active_sensor_flag = 0;
				Warning_Flag = 0;
				Delay_Time = 0;
				curr_delay = 0;
				PORTA &= ~0x01;

			}
			else if( cnt == 0 )	//Key1 �� ¦���� ��������
			{
			    cli();

				//TIMSK &= ~0x01; // Ÿ�̸�0 �����÷� ���ͷ�Ʈ����(�����Ľ�ȣ�� ���� ����)

				//EIMSK &= ~0x30;        // INT4, INT5 Enable(���) 
                          			   // EIMSK = 0b**00****,  EIMSK = EIMSK & 0b00110000( 0x30 ) 

				PORTA &= ~0x08;		   // PA3(����) OFF : ���� OFF
				PORTA |= 0x10;		   // PA4(LED) OFF : LED OFF
			}
	   
	   }
	   else if(	pressed_key == KEY2 )
	   {
	   		cnt++;
			if( cnt1 == 3 ) cnt1 = 0;

			if( cnt1 == 0 )		//Key2�� 1�� ��������
			{
				
			}

			else if( cnt1 == 1 ) // Key2�� 2�� ��������
			{

			}
			else if( cnt1 == 2 )  //Key2�� 3�� �������� 
			{

			}
	   }

       cli();                            // �������ͷ�Ʈ ����

 	   dist_1 = distance_1 ;            // �����ļ������1�� ������ �Ÿ��������� ���� dist_1�� ���� 
 	   dist_2 = distance_2 ;            // �����ļ������2�� ������ �Ÿ��������� ���� dist_2�� ���� 

       sei();                           // �������ͷ�Ʈ ���

	   LcdMove(0, 9);                   // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 9��)
       Display_Number_LCD(dist_1, 3);   // �����ļ������1�� ������ �Ÿ��������� 100�ڸ����� ���÷��� 
       LcdMove(1, 9);                   // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 9��)
       Display_Number_LCD(dist_2, 3);   // �����ļ������2�� ������ �Ÿ��������� 100�ڸ����� ���÷��� 

    } 

}      // int main() �Լ��� �� 




ISR( TIMER0_OVF_vect )     //  10 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ �������α׷�
{                          //  50msec ���� �����Ľ�ȣ �߻� ��û��ȣ(Trigger ��ȣ1, Trigger ��ȣ2) ���
                           //  ���� ��ֹ��� �Ÿ��� ������ ��� ������(�ܼ���) �߻� , LED ������.

    static unsigned short  time_index = 0 ; 


    TCNT0 = 256 - 156;     //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                           //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                           //  156 = 10msec/ 64usec,  TCNT0 = 256 - 156

    time_index++ ; 

    if( time_index == 5 )   // 50 msec (=10msec * 5) ���� 
    {

       time_index = 0;      // �ʱ�ȭ

       sensor_count++;          // ������ ���� ī���� �� ����        
	   if( sensor_count == 3 )  sensor_count = 1; 

       if ( sensor_count == 1 )        //  �����ļ��� Ʈ���� ��ȣ1 �߻�(������ 1 �߻�) 
	   {

	      PORTA |= 0x01;       // PA0 : High
	      usec_delay(20);      // 20usec ���� High ���� 
	      PORTA &= ~0x01;      // PA0 : Low   
     
	      active_sensor_flag = 1; 
		  flag = 0;

	   }
       else if ( sensor_count == 2 )   //  �����ļ��� Ʈ���� ��ȣ2 �߻�(������ 2 �߻�)
	   {

	      PORTA |= 0x02;       // PA1 : High
	      usec_delay(20) ;     // 20usec ���� High ���� 
	      PORTA &= ~0x02;      // PA1 : Low 

	      active_sensor_flag = 2;
		  flag = 0;

	   }

       ////////  ����� �߻�   ///////////// 

       // �����ļ������ 1�� 2�� ������ �Ÿ� �� �ּڰ� ���� 
       if( distance_1 <= distance_2  && distance_1 != 0 )       distance_min = distance_1;
       else if( distance_2 < distance_1  && distance_2 != 0 )   distance_min = distance_2;

       if( distance_min <=  40 ) Warning_Flag = 1 ;  // ������ �Ÿ��� 40 cm �����̸� ����� �߻� �÷��� set
       else                      Warning_Flag = 0 ;    
		
       Delay_Time = distance_min /10 + 1;  //�Ÿ��� ����ϴ� �ֱ�(=Delay_Time*50 msec )�� ���� ����� �߻�

	   if( Delay_Time <= 1)   Delay_Time = 1 ;   // ������ֱ� ���� : 0.1��
	   if( Delay_Time >= 4)   Delay_Time = 4 ;   // ������ֱ� ���� : 0.4�� 
 
       if( Warning_Flag == 1 )      // ���� ��ֹ��� �Ÿ��� ������ ��� ������(�ܼ���) �߻� , LED ������. 
	   {
           if( Time_Delay_Polling( Delay_Time ) == 1 )     // 50msec * Delay_Time ��� �� 
	       {
                 PORTA ^= 0x08;    // PA3(����) toggle :  ���� �ܼ��� 
		         PORTA ^= 0x10;    // PA4(LED) toggle :  LED ON, OFF �ݺ� 
	       }
	   }
       else if( Warning_Flag == 0 )  // ���� ��ֹ��� �Ÿ��� ������ ������ ���� OFF , LED OFF. 
	   {
            PORTA &= ~0x08;     // PA3(����) OFF : ���� OFF 
		    PORTA |= 0x10;      // PA4(LED) OFF :  LED  OFF 
	   }
      
   }

}


//////////////////////////////////////////////////////////////////



ISR( INT4_vect )     // �ܺ����ͷ�Ʈ 4(INT4) ���� ���α׷�
{                    // Echo ��ȣ1 �޽����� �ð� ���� �� ���� ��ֹ������� �Ÿ� ��� 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0 ;

    if ( active_sensor_flag == 1 )
    {

	  if( flag == 0 )          // Echo ��ȣ1�� ��¿������� ���ͷ�Ʈ �ɸ� �� 
	  {
		  count1 = TCNT3;      // ��¿��������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  flag = 1;            // flag ���� 1�� ����
	  } 
	  else if( flag == 1 )     // Echo ��ȣ1�� �ϰ��������� ���ͷ�Ʈ �ɸ� ��
	  { 
		  count2 = TCNT3;                    // �ϰ����������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  del_T = ( count2 - count1 ) / 2 ;  // Echo ��ȣ1 �޽����� �ð� ����(usec ����)
    	  distance_1 = del_T / 58;           // �����ļ������1�� ������ ���� ��ֹ������� �Ÿ����(cm ����)

          if( distance_1 > 380 )  distance_1 = distance_1_prev ;   // �ݻ�Ǵ� �����İ� ������� ������ 
		                                                           // ���� �Ÿ������� ��� 
          distance_1_prev = distance_1;       // ���� �Ÿ������� ���� ���� ������Ʈ  
		  flag = 0;                           // flag ���� 0���� ���� 
          active_sensor_flag = 0;             // active_sensor_flag ���� ����
	   } 

    }
} 


ISR( INT5_vect )     // �ܺ����ͷ�Ʈ 5(INT5) ���� ���α׷�
{                    // Echo ��ȣ2 �޽����� �ð� ���� �� ���� ��ֹ������� �Ÿ� ��� 

    static unsigned short  count1 = 0, count2 = 0, del_T = 0 ;

    if ( active_sensor_flag == 2 )
    {

	  if( flag == 0 )          // Echo��ȣ2�� ��¿������� ���ͷ�Ʈ �ɸ� �� 
	  {
		  count1 = TCNT3;      // ��¿��������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  flag = 1;            // flag ���� 1�� ����
	  } 
	  else if( flag == 1 )     // Echo��ȣ2�� �ϰ��������� ���ͷ�Ʈ �ɸ� ��
	  { 
		  count2 = TCNT3;                    // �ϰ����������� ī���ͷ������Ͱ�(TCNT3) ���� 
		  del_T = ( count2 - count1 ) / 2 ;  // Echo ��ȣ2 �޽����� �ð� ����(usec ����)
    	  distance_2 = del_T / 58;           // �����ļ������2�� ������ ���� ��ֹ������� �Ÿ����(cm ����)

          if( distance_2 > 380 )  distance_2 = distance_2_prev ;   // �ݻ�Ǵ� �����İ� ������� ������ 
		                                                           // ���� �Ÿ������� ��� 
          distance_2_prev = distance_2;       // ���� �Ÿ������� ���� ���� ������Ʈ  
		  flag = 0;                           // flag ���� 0���� ���� 
          active_sensor_flag = 0;             // active_sensor_flag ���� ����
	  } 

    }
}
 
///////////////////////////////////////////////////////////////////

//==============================================
// Ű�е��� ��ĵ �ڵ尪�� ��ȯ�Ѵ�.
//
// ���� �� : 0�� �ƴ� �� : ��ĵ �ڵ尪
//           0�� ��      : �Է°��� ����
//==============================================
unsigned char KeyInput(void)
{
    unsigned char in, in1;

    static unsigned char pin = NO_KEY;

    
    in = key_scan();            // key �Է� �б� 
    
    while(1)               // key �Է��� ����ȭ�ɶ����� ��ٸ�
    {
        //��ٿ�� �ð� ����
        msec_delay(10); msec_delay(10);  msec_delay(10);
        
        in1 = key_scan();         // key �Է� �б�    
        
        if(in==in1) break;
        in = in1;
    }
    
    if(!(in & 0xFF))    // key�� ������ �ʰ� ������  0 ( NO_KEY) ����
    {
        pin= 0;			
        return 0;
    }
    
    if(pin == in)     // ������ key�� ��� ������ ������  0 ( NO_KEY) ����
    {
        return 0;  
    }
    
    pin = in;
    
    return in;
}

 
//----------------------------------------
//��ĵ �ڵ尪�� ������
//----------------------------------------

unsigned char key_scan(void)
{
  
    unsigned char   in = 0 ;
    
    if (       ! ( PINB  & 0x01  )  )        in = KEY1 ; 	  //   PB0 key scan 	
    else if (  ! ( PINB  & 0x02  )  )        in = KEY2 ; 	  //   PB1 key scan  		
    else if (  ! ( PINB  & 0x04  )  )        in = KEY3 ; 	  //   PB2 key scan  	

    else                                     in = NO_KEY ; 	
    
    return in ;
    
}

///////////////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned int num, unsigned char digit )   //��ȣ���� ����������(num)�� 10���� ���·� 
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

char NumToAsc( unsigned char Num )   // Num���� �Ѿ�� 16���� 1�ڸ� ���ڸ� ���ڵ�����(ASCII �ڵ�)�� 
{                                        // ��ȯ�Ͽ� �����ϴ� �Լ�
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 
	return Num ;
}

unsigned char Time_Delay_Polling( unsigned short d_time )
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

////////////////////////////////////////////////////

void msec_delay(int n)               // n msec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)  _delay_ms(1);	 // 1msec �ð� ������ nȸ �ݺ�
}

void usec_delay(int n)               // n usec ��ŭ�� �ð����� �߻� �Լ� 
{	
	for(; n>0; n--)   _delay_us(1);	 // 1usec �ð� ������ nȸ �ݺ�
}

