
// < CDS_TMP_sensor_ADC_1.c >  : ��������, �µ�����(LM35), Key 3���� �̿��Ͽ� ������ �µ��� �����ϴ� ATmega128 �ڵ� 

#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���. 

// Key ���� �ɺ� ����
#define  NO_KEY  0
#define  KEY1    1
#define  KEY2    2
#define  KEY3    3

unsigned short ADC_Read( unsigned char ch ) ;    // AD��ȯ�� ä�ΰ��� �Ķ���ͷ� �޾Ƽ� �ش� ä���� ADC ���� 
                                                 // ���� �� �� ���� �����ϴ� �Լ� 

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
void Display_TMP_LCD( unsigned short tp );   // �µ��� 10���� ���·� �Ҽ��� �� 2�ڸ�, �Ҽ��� �Ʒ� 1�ڸ����� 
                                             // LCD �� ���÷��� �ϴ� �Լ�

void msec_delay(int n);     // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����


static volatile unsigned short   CDS_value = 0, CDS_adc_value = 0,  CDS_adc_value_avg = 0 ;
static volatile unsigned short   TMP_value = 0, TMP_adc_value = 0,  TMP_adc_value_avg = 0 ;


int main() 
{   

    unsigned short  Temperature = 0, CDS = 0  ;
    unsigned char   Key_Data = 0 ;

    LcdInit();                     // LCd �ʱ�ȭ �Լ� 

    LcdMove(0,0);                // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("ILLUM Sensor");     // LCD�� �������� �޽��� ���÷��� 
    LcdMove(1,0);                // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("TEMP Sensor");      // LCD�� �µ����� �޽��� ���÷���

////////  Key 3�� ����� ���� HW ���� �������� ����  ////////////
//  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)
    DDRA &= ~0x70;      // Key1(PA4), Key2(PA5), Key3(PA6) : �Է���Ʈ�� ����   
                           // �Է���Ʈ ����.  DDRA = 0b*000 ****, DDRA = DDRA & ~0b0111 0000(~0x70)
    PORTA |= 0x70;       // Key1(PA4), Key2(PA5), Key3(PA6) : ����Ǯ�������� ����  
                           // PORTA = 0b*111 ****, PORTA = PORTA | 0b0111 0000( 0x70 )

   ////////  �������� (CDS����) ������ ���� HW ���� �������� ����  ////////////
   //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA |= 0x04;          // LED1 ON/OFF ��ȣ( PA2 : �����Ʈ ����  )
                           // DDRA = 0b**** *1**, DDRA = DDRA | 0b0000 0100( 0x04 )
    DDRF &= ~0x04;         // ����������½�ȣ Vo ������ ( ADC2(PF2) : �Է���Ʈ ���� )  
                           // �Է���Ʈ ����.  DDRF = 0b**** *0**, DDRF = DDRF & ~0b0000 0100(~0x04)

    ////////  �µ����� (LM35) ������ ���� HW ���� �������� ����  ////////////
    //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRA |= 0x08;          // LED2 ON/OFF ��ȣ( PA3 : �����Ʈ ����  )
                           // DDRA = 0b**** 1***, DDRA = DDRA | 0b0000 1000( 0x08 )
    DDRF &= ~0x08;         // �µ�������½�ȣ Vo ������ ( ADC3(PF3) : �Է���Ʈ ���� )  
                           // �Է���Ʈ ����.  DDRF = 0b**** 0***, DDRF = DDRF & ~0b0000 1000(~0x08)
  
    ////////////  Timer 2 ����  ( 10 msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ ���� )  ///////////////
    // ���� P133-137(�������� �׸��� ǥ8.1-ǥ8.6) ����    

    TCCR2 &= ~0x4A;        // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 2 ����(1024���� ���)
                           // TCCR2 = 0b*0**0101 
                           // TCCR2 =TCCR2 & ~0b01001010(~0x4A )  
    TCCR2 |= 0x05;         // TCCR2 =TCCR2 | 0b00000101( 0x05 )  

    TCNT2 = 256 - 156;     // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                           // �����÷����ͷ�Ʈ �ֱ� = 2msec
                           // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156 

    TIMSK |= 0x40;         // Ÿ�̸�2 �����÷� ���ͷ�Ʈ ��� 
                           // TIMSK = 0b*1** ****, TIMSK = TIMSK | 0b0100 0000( 0x40 ) 

    sei();                 // ���� ���ͷ�Ʈ ���

    //////////   ADC( AD��ȯ�� ) ����  /////////////
    //////  (���� pp321-322 �������� ADMUX �׸�, ǥ15.1, ǥ15.2 ����)

    ADMUX &= ~0xE0;         // �������м���( AREF ), ADC��� ������ ���� 
                            // ADMUX = 0b000* ****  
                            // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 

    //////  (���� pp323-324 �������� ADCSRA �׸�, ǥ15.3 ����)
	ADCSRA |= 0x87;        // ADC ����(enable), ���������Ϸ�(Prescaler) ����: 128 ����
                           // ADCSRA = 0b1*** *111
                           // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 

    while (1) 
    { 

       cli();                               // �������ͷ�Ʈ ����

 	   CDS = CDS_adc_value;                 // ����� ������ ������ ���� ���� CDS�� ���� 
 	   Temperature = TMP_value;             // ����� �µ����� ���� Temperature�� ���� 

       sei();                               // �������ͷ�Ʈ ���

       Key_Data = Key_Input( );           // �Լ� Key_Input( )ȣ��. � Key�� ���������� üũ  

       if( Key_Data == KEY1 )                  // Key1 �� ���������� ���������� ���÷���
       {  
          LcdCommand( ALLCLR );               // LCD ȭ�� �����
	      LcdMove(0, 0);                      // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "ILLUMINANCE Mode" );      // ����������� �޽��� ���÷���  
	      LcdMove(1, 0);                      // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "CDS = " );                // ���� �޽��� ���÷���
          Display_Number_LCD( CDS, 3 );       // ������ ������ ���� 3�ڸ� �������� ���÷��� 
       }

       else if( Key_Data == KEY2 )             // Key2 �� ���������� �µ������� ���÷���
       {  
          LcdCommand( ALLCLR );                // LCD ȭ�� �����
	      LcdMove(0, 0);                       // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "TEMPERATURE Mode" );       // �µ�������� �޽��� ���÷���   
	      LcdMove(1, 0);                       // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "TMP = " );                 // �µ� �޽��� ���÷���
          Display_TMP_LCD( Temperature  );     // �µ��� �Ҽ��� �Ʒ� ù°�ڸ����� ���÷���  
       }

       else if( Key_Data == KEY3 )             // Key3 �� ���������� ����, �µ������� ��� ���÷���
       {  
          LcdCommand( ALLCLR );                // LCD ȭ�� �����
	      LcdMove(0, 0);                       // LCD�� �� ������ �ʱ� ��ġ ���� (0��, 0��) 
          LcdPuts( "CDS = " );                 // ���� �޽��� ���÷���
          Display_Number_LCD( CDS, 3 );        // ������ ������ ���� 3�ڸ� �������� ���÷���  
	      LcdMove(1, 0);                       // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 0��) 
          LcdPuts( "TMP = " );                 // �µ� �޽��� ���÷���
          Display_TMP_LCD( Temperature  );     // �µ��� �Ҽ��� �Ʒ� ù°�ڸ����� ���÷���  
       }
 

   }  // ���ѷ��� while (1) �� �� 


}      // int main() �Լ��� �� 


////////////////////////////////////////////////////////////////////


ISR( TIMER2_OVF_vect )    // 10msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ �������α׷�
{                         // ���ø��ֱ� 10msec(= 1 x 10msec) ���� ��������(ADC2)��, �µ�����(ADC3)�� ���� 

    static unsigned short  index_time  = 0,  tmp_cnt = 0, cds_cnt = 0;
    static unsigned long   TMP_adc_value_sum = 0,  CDS_adc_value_sum = 0;


    TCNT2 = 256 - 156;        // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156

    index_time++; 

    if( index_time  == 1 )    //  ���ø��ֱ� 10msec(= 1 x 10msec ) ���� ����, �µ� ���� 
    { 

        index_time = 0;    // ���� �ʱ�ȭ

       /////////////   ��������(CDS����)��ȣ(ADC2) ����(AD��ȯ)  ////////////////
       //////  (���� P321 �������� ADMUX �׸�, ǥ15.1, p322 ǥ15.2 ����)

       /////////////   ��������(CDS����)��ȣ(ADC2) ����(AD��ȯ)  ////////////////
       CDS_adc_value = ADC_Read( 2 ) ;  // ADC ä�� 2�� AD ��ȯ�� ������ ���� ���� �� ���������� ����  

       /////////////   �µ�������ȣ(ADC3) ����(AD��ȯ)  ////////////////
       TMP_adc_value  = ADC_Read( 3 ) ;  // ADC ä�� 3�� AD ��ȯ�� ������ ���� ���� �� ���������� ����  

 
      ////// ����� �����ϱ� ���� CDS ���� ADC���� ��հ��� ����.   ////////////////////

       CDS_adc_value_sum +=  CDS_adc_value; 
	   cds_cnt++;

	   if( cds_cnt == 16 )       // 16�� ���( 0.16�� = 16 * 10msec  ���� ��հ� ����.)
	   {
          CDS_adc_value_avg = CDS_adc_value_sum / cds_cnt  ;

          cds_cnt = 0;
          CDS_adc_value_sum = 0; 
	   }

       /////  ADC �������� ��հ��� CDS ���� ���������� ����   ////// 

       CDS_value = (unsigned short) CDS_adc_value_avg;  


      ////// ����� �����ϱ� ���� �µ����� ADC �� ��հ� ����.   ////////////////////

       TMP_adc_value_sum +=  TMP_adc_value; 
	   tmp_cnt++;

	   if( tmp_cnt == 128 )       // 128�� ���(1.28�� = 128 * 10msec  ���� ��հ� ����.)
	   {
          TMP_adc_value_avg = TMP_adc_value_sum / tmp_cnt  ;

          tmp_cnt = 0;
          TMP_adc_value_sum = 0; 
	   }

       /////  ADC �������� ��հ����κ��� �����µ�(����) ���  ////// 

       TMP_value = (unsigned long)  TMP_adc_value_avg * 1250 / 256 ;  
	   
	    
      ///////////////////////////////////////////////////////////////////////////////////////////////////

      if( CDS_value < 250 )           // ����� ������ ���� ���� 250 �̸��̸� LED1 (PA2) ON 
      {    
          PORTA &=  ~0x04;            // LED1 ON ( PA2 = 0 )
                                      // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
      }
      else if( CDS_value >= 250 )     // ����� ������ ���� ���� 250 �̻��̸� LED1 (PA2) OFF
      {
           PORTA |=  0x04;            // LED1 OFF ( PA2 = 1 )
                                      // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
      }

      if( TMP_value < 270 )           // ����� �µ��� 27.0C �̸��̸� LED2 (PA3) OFF 
      {    
           PORTA |=  0x08;            // LED2 OFF ( PA3 = 1 )
                                      // PORTA = 0b**** 1***,   PORTA = PORTA | 0b0000 1000( 0x08 )
      }
      else if( TMP_value >= 270 )    // ����� �µ��� 27.0C �̻��̸� LED2 (PA3) ON
      {
          PORTA &=  ~0x08;           // LED2 ON ( PA3 = 0 )
                                     // PORTA = 0b**** 0***,   PORTA = PORTA & ~0b0000 1000( ~0x08 )
      }

   }    // End of  if( index_time == 10 ) 
 
}       // End of  ISR( TIMER2_OVF_vect ) 

//////////////////////////////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch )   //  AD��ȯ�� ä�ΰ��� �Ķ����(ch)�� �޾Ƽ� �ش� ä���� ADC 
{                                                //  ���� ���� �� �� ���� �����ϴ� �Լ� 
    unsigned short ad_result  = 0 ;

    if( ch > 7 )  ch = 7;         // ADC ä���� 0 - 7 �̹Ƿ� ������ ����� ���� ä�� 7�� ����

    ADMUX &= ~0x1F;           // ADC ä�� ���� 
    ADMUX |=  ch;              // ADC ä��(ch) ����
    ADCSRA |= 0x40;            // AD ��ȯ ���� ( ADCSRA �������� bit6 = 1 )
    while( !( ADCSRA & 0x10) );  // AD ��ȯ�� �Ϸ�� ������ ��ٸ�. 
    ADCSRA |= 0x10;            // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) ����.
    ad_result = ADC;             // AD ��ȯ �Ϸ�� ������ ��( �������� ADC )�� ������ ����  

    return  ad_result;
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
    else                                             in = NO_KEY;   //	Key�� �������� ������ 
    
    return   in ;
    
}


/////////////////////////////////////////////////////////////

void Display_Number_LCD( unsigned short num, unsigned char digit )       // ��ȣ���� ������ ������ 10���� ���·� LCD �� ���÷��� 
{

	HexToDec( num, 10); //10������ ��ȯ 

	if( digit == 0 )     digit = 1 ;
	if( digit > 5 )      digit = 5 ;
 
    if( digit >= 5 )     LcdPutchar( NumToAsc(cnumber[4]) );  // 10000�ڸ� ���÷���
	
	if( digit >= 4 )     LcdPutchar(NumToAsc(cnumber[3]));    // 1000�ڸ� ���÷��� 

	if( digit >= 3 )     LcdPutchar(NumToAsc(cnumber[2]));    // 100�ڸ� ���÷��� 

	if( digit >= 2 )     LcdPutchar(NumToAsc(cnumber[1]));    // 10�ڸ� ���÷���

	if( digit >= 1 )     LcdPutchar(NumToAsc(cnumber[0]));    //  1�ڸ� ���÷���

}


void Display_TMP_LCD( unsigned short tp  )   // �µ��� 10���� ���·� �Ҽ��� �� 2�ڸ�, �Ҽ��� �Ʒ� 1�ڸ����� 
{                                            // LCD �� ���÷��� �ϴ� �Լ�

    HexToDec( tp, 10); //10������ ��ȯ 

    LcdPutchar(NumToAsc(cnumber[2]) );   // 10�ڸ� ���÷���
    LcdPutchar(NumToAsc(cnumber[1]));    // 1�ڸ� ���÷��� 
    LcdPuts( ".");                       // �Ҽ���(.) ���÷��� 
    LcdPutchar(NumToAsc(cnumber[0]));    // 0.1 �ڸ� ���÷��� 

    LcdPutchar( 0xDF );                  // LCD�� ���� �� ���÷���
    LcdPutchar('C');                     // LCD�� ���� C ���÷���

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
	for(; n>0; n--)		    // 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		// 1msec �ð� ����
}

void usec_delay(int n)
{	
	for(; n>0; n--)		    // 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		// 1usec �ð� ����
}



