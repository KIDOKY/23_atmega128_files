//< TMP_sensor_ADC_HW.c >  : �µ����� (LM35)�� �̿��Ͽ� �µ��� �����ϴ� ATmega128 �ڵ� 

#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 
#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���. 

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // ��ȣ���� ������ ������ 10���� ���·� 
                                                                     // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void Display_TMP_LCD( unsigned short tp );    // �µ��� 10���� ���·� �Ҽ��� �� 2�ڸ�, �Ҽ��� �Ʒ� 1�ڸ����� 
                                              // LCD �� ���÷��� �ϴ� �Լ�

void msec_delay(int n);     // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����

unsigned  short ADC_Read( unsigned char ch_no );	//AD ��ȯ�� �б� �Լ�

static volatile unsigned short   TMP_value = 0, TMP_adc_value = 0,  TMP_adc_value_avg = 0 ;

 
int main() 
{   
    unsigned short  Temperature = 0  ;

    LcdInit();                   // LCd �ʱ�ȭ �Լ� 

    LcdMove(0,0);                // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("TMP Sensor");       // LCD�� ���ڿ� ���÷��� 
    LcdMove(1,0);                // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)
    LcdPuts("TMP = ");           // LCD�� ���ڿ� ���÷��� 


    ////////  �µ����� (LM35) ������ ���� HW ���� �������� ����  ////////////
    //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRB |= 0x20;          // LED ON/OFF ��ȣ( PB5 : �����Ʈ ����  )
                           // DDRB = 0b**1* ****, DDRB = DDRB | 0b0010 0000( 0x20 ) 

    DDRF &= ~0x08;         // �µ�������½�ȣ Vo ������ ( ADC3(PF3) : �Է���Ʈ ���� )  
                           // �Է���Ʈ ����.  DDRF = 0b**** 0***, DDRF = DDRF & ~0b0000 1000(~0x08)
  
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



   //////////   ADC( AD��ȯ�� ) ����  /////////////

   //////  (���� pp321-322 �������� ADMUX �׸�, ǥ15.1, ǥ15.2 ����)
    ADMUX &= ~0xE0;          // �������м���( AREF ), ADC��� ������ ���� 
                             // ADMUX = 0b000* ****  
                             // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 ) 

   //////  (���� pp323-324 �������� ADCSRA �׸�, ǥ15.3 ����)
    ADCSRA |= 0x87;          // ADC ����(enable), ���������Ϸ�(Prescaler) ����: 128 ����
                             // ADCSRA = 0b1*** *111
                             // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 
    while (1) 
    { 

        cli();                                // �������ͷ�Ʈ ����

 	    Temperature = TMP_value;              // ����� �µ����� ���� Temperature�� ���� 

        sei();                                // �������ͷ�Ʈ ���

	    LcdMove(1, 6);                        // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 8��) 
        Display_TMP_LCD( Temperature  );      // �µ��� �Ҽ��� �Ʒ� ù°�ڸ����� ���÷���  


   }

}      // int main() �Լ��� �� 


////////////////////////////////////////////////////

ISR( TIMER2_OVF_vect )      // 10msec �ֱ��� Ÿ�̸�2 �����÷� ���ͷ�Ʈ �������α׷�
{                           // 10msec(= 1 x 10msec) ���� �µ�����(ADC3)�� ���� 

    static unsigned short  index_time  = 0, tmp_cnt = 0;
    static unsigned long   TMP_adc_value_sum = 0;


    TCNT2 = 256 - 156;        // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 10msec
                              // 156 = 10msec/ 64usec,  TCNT2 = 256 - 156

    index_time++; 

    if( index_time  == 1 )    //  10msec(= 1 x 10msec ) ���� �µ� ���� 
    { 

        index_time = 0;    // ���� �ʱ�ȭ

       /////////////   �µ�������ȣ(ADC3) ����(AD��ȯ)  ////////////////

       //////  (���� P321 �������� ADMUX �׸�, ǥ15.1, p322 ǥ15.2 ����)
       ADMUX &= ~0x1F;      // ADC ä�� ���� (ä�� 0, ADC0)
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F ) 

       ADMUX |=  0x00;      // ADC ä�� 3 ( ADC3 ) ���� 
                            // ADMUX = 0b**** **11  
                            // ADMUX = ADMUX | 0b0000 0011 ( 0x03 ) 

       ADCSRA |= 0x40;         // ADC ��ȯ ���� ( ADCSRA �������� bit6 = 1 )
                               // ADCSRA = 0b*1** ****  
                               // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

       while( !( ADCSRA & 0x10) );  // ADC ��ȯ�� �Ϸ�� ������ ��ٸ�. 
                                    // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) = 1 �̸� ��ȯ�Ϸ�.
 
       ADCSRA |= 0x10;           // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) ����.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 ) 

       TMP_adc_value = ADC_Read( 3 );  // ADC ��ȯ �Ϸ�� ������ ��( �������� ADC )�� ���������� ����  


      ////// ������ ���Ÿ� ���� ADC ���� ��հ��� ����.   ////////////////////
 
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
 
       if( TMP_value < 270 )       // ����� �µ��� 27.0C �̸��̸� LED(PB5) OFF 
       {    
          PORTB |=  0x20;         //  LED  OFF ( PB5 = 1)
                                  // PORTB = 0b**1* ****,   PORTB = PORTB | 0b0010 0000( 0x20 )
       }

       else if( TMP_value >= 270 )  // ����� �µ��� 27.0C �̻��̸� LED(PB5) ON
       {
           PORTB &= ~0x20;         //  LED ON ( PB5 = 0 )
                                   // PORTB = 0b**0* ****,   PORTB = PORTB & ~0b0010 0000( ~0x20 )
       }

   }    // End of  if( index_time == 10 ) 
 
}       // End of  ISR( TIMER2_OVF_vect ) 


/////////////////////////////////////////////////////////////////

unsigned short ADC_Read( unsigned char ch_no )		// AD ��ȯ�� �б� �Լ�
{
	if( ch_no >= 8 )	ch_no = 7;

	/////////////   �ݻ�� ���ܼ� ������ȣ ����(AD��ȯ)  ////////////////
    //////  (���� P321 �������� ADMUX �׸�, ǥ15.1, p322 ǥ15.2 ����) 

    ADMUX &= ~0x1F;         // ADC ä�� ����
                            // ADMUX = 0b***0 0000  
                            // ADMUX = ADMUX & ~0b0001 1111 (~0x1F )

	ADMUX |= ch_no;			// ä�� �ѹ� ����

    ADCSRA |= 0x40;         // ADC ��ȯ ���� ( ADCSRA �������� bit6 = 1 )
                            // ADCSRA = 0b*1** ****  
                            // ADCSRA = ADCSRA | 0b0100 0000 ( 0x40 ) 

    while( !( ADCSRA & 0x10) );  // ADC ��ȯ�� �Ϸ�� ������ ��ٸ�. 
                                 // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) = 1 �̸� ��ȯ�Ϸ�.
 
    ADCSRA |= 0x10;              // ADCSRA ���������� ADC ���ͷ�Ʈ �÷��׺�Ʈ(ADIF, bit4) ����.
                                 // ADCSRA = 0b***1 ****  
                                 // ADCSRA = ADCSRA | 0b0001 0000 ( 0x10 )

	return ADC;


}
/////////////////////////////////////////////////////////////////
 
void Display_Number_LCD( unsigned short num, unsigned char digit ) //��ȣ���� ����������(num)�� 10���� ���·� 
{   
                                                    // ������ �ڸ���(digit) ��ŭ LCD �� ���÷��� �ϴ� �Լ� 
      HexToDec( num, 10); //10������ ��ȯ

      if( digit < 1 )     digit = 1 ;
      if( digit > 5 )     digit = 5 ;

      if( digit >= 5)  LcdPutchar(NumToAsc(cnumber[4]));    // 10000�ڸ� ���ʷ��� 
      if( digit >= 4)  LcdPutchar(NumToAsc(cnumber[3]));    // 1000�ڸ� ���ʷ��� 
      if( digit >= 3)  LcdPutchar(NumToAsc(cnumber[2]));    // 100�ڸ� ���ʷ��� 
      if( digit >= 2)  LcdPutchar(NumToAsc(cnumber[1]));    // 10�ڸ� ���ʷ���
      if( digit >= 1)  LcdPutchar(NumToAsc(cnumber[0]));    // 1�ڸ� ���÷���

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

/////////////////////////////////////

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

////////////////////////////////////

