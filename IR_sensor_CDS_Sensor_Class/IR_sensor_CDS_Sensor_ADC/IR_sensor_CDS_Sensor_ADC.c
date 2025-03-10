
// < IR_sensor_CDS_Sensor_ADC.c >  : ���ܼ����� ���(TCRT5000), CDS������ �̿��Ͽ� ���� ��ֹ��� ����
 
#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���. 

#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.

#define NO_KEY    0

#define KEY1      1

unsigned char key_scan(void);
unsigned char KeyInput(void);

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit ) ;    // ��ȣ���� ������ ������ 10���� ���·� 
                                                                      // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);    // msec ���� �ð�����
void usec_delay(int n);    // usec ���� �ð�����

unsigned short ADC_Read( unsigned char ch_no );		// AD ��ȯ�� �б� �Լ�

static volatile unsigned short    IR_adc0_value = 0, IR_flag = 0;
static volatile unsigned short    CDS_value = 0, CDS_adc_value = 0,  CDS_adc_value_avg = 0 ;

static volatile unsigned short    distance_1 = 0, distance_2 = 0, distance_min = 300; 
static volatile unsigned short    distance_1_prev = 0, distance_2_prev = 0; 
static volatile unsigned char     sensor_count = 0, active_sensor_flag = 0;
static volatile  unsigned char    Warning_Flag = 0, flag = 0 ;
static volatile  unsigned short   Delay_Time = 0, curr_delay = 0;

int main() 
{   

    unsigned short  IR_data0 = 0, pressed_key = 0, cnt = 0;

    LcdInit();                     // LCd �ʱ�ȭ �Լ� 

    LcdMove(0,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("IR Sensor");          // LCD�� ���ڿ� ���÷��� 
    LcdMove(1,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("IR_ADC =     ");      // LCD�� ���ڿ� ���÷���

	////////  �������� (CDS����) ������ ���� HW ���� �������� ����  ////////////
    //  �Է�/��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)

    DDRF &= ~0x04;         // ����������½�ȣ Vo ������ ( ADC2(PF2) : �Է���Ʈ ���� )  
                           // �Է���Ʈ ����.  DDRF = 0b**** *0**, DDRF = DDRF & ~0b0000 0100(~0x04)


   ////////  ���ܼ��������(TCRT5000) ������ ���� HW ���� �������� ����  ////////////

   //  �Է�/��� ��Ʈ ���� 	(���� pp75-76 �������ͱ׸�, ǥ6.1 ����)
    DDRA |= 0x01;          // ���ܼ�LED ON/OFF ��ȣ( PA0 : �����Ʈ ����  )
                           // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 ) 

    PORTA &= ~0x01;        // ���ܼ�LED(EL-7L) OFF ( PA0 = 0 )
                           // PORTA = 0b**** ***0,   PORTA = PORTA & ~0b0000 0001( ~0x01 )

    DDRF &= ~0x02;         // ���ܼ�������½�ȣVo(����TR Emitter ��ȣ) ������ ( ADC1(PF0) : �Է���Ʈ ���� )  
                           // �Է���Ʈ ����.  DDRF = 0b**** **0*, DDRF = DDRF & ~0b0000 0010(~0x02)

	DDRA |= 0x08;		   // ���� PA3: ���
	PORTA &= ~0x08;		   // ���� OFF PA3 = 0

	DDRA |= 0x10;		   // LED PA4: ���
	PORTA |= 0x10;		   // LED OFF: PA4 = 1

	DDRA &= ~0x20;        // Push SW ( PA5 : �Է���Ʈ ���� )   
    PORTA |= 0x20;
    
    ////////////  Timer 0 ����  ( 2 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ ���� )  ///////////////
    // ���� P133-137(�������� �׸��� ǥ8.1-ǥ8.5) ����

    TCCR0 &= ~0x48;           // Normal mode(Ÿ�̸Ӹ��), Ÿ�̸� 0 ����(1024���� ���)
                              // TCCR0 = 0b*0**0111
                              // TCCR0 =TCCR0 & ~0b01001000(~0x48 )  
    TCCR0 |= 0x07;            // TCCR0 =TCCR0 | 0b00000111( 0x07 )  

    TCNT0 = 256 - 31;         // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 2msec
                              // 31 = 2msec/ 64usec,  TCNT0 = 256 - 31 

    TIMSK |= 0x01;            // Ÿ�̸�0 �����÷� ���ͷ�Ʈ ��� 
                              // TIMSK = 0b**** ***1, TIMSK = TIMSK | 0b0000 0001( 0x01 ) 

    sei();                    // ���� ���ͷ�Ʈ ���


    //////////   ADC( AD��ȯ�� ) ����  /////////////
   //////  (���� pp321-322 �������� ADMUX �׸�, ǥ15.1, ǥ15.2 ����)

    ADMUX &= ~0xE0;       // �������м���( AREF ), ADC��� ������ ���� 
                          // ADMUX = 0b000* ****  
                          // ADMUX = ADMUX & ~0b1110 0000( ~0xE0 )  

    //////  (���� pp323-324 �������� ADCSRA �׸�, ǥ15.3 ����)

	ADCSRA |= 0x87;       // ADC ����(enable), ���������Ϸ�(Prescaler) ����: 128 ����
                          // ADCSRA = 0b1*** *111
                          // ADCSRA = ADCSRA | 0b1000 0111( 0x87 ) 
	 
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

       cli();                               // �������ͷ�Ʈ ����

 	   IR_data0 = IR_adc0_value;            // ADC ���� ���� IR_data0 �� ���� 

       sei();                              // �������ͷ�Ʈ ���

	   LcdMove(1, 9);                     // LCD�� �� ������ �ʱ� ��ġ ���� (1��, 9��)
       Display_Number_LCD(IR_data0, 3);  // �ݻ�� ���ܼ���(ADC��) ���� IR_data0 �� 100�ڸ����� ���÷��� 

   }

}      // int main() �Լ��� �� 



ISR( TIMER0_OVF_vect )     //  2 msec �ֱ��� Ÿ�̸�0 �����÷� ���ͷ�Ʈ �������α׷�
{                          //  2msec ���� ���ܼ� LED ��� �ݻ�Ǵ� ���ܼ� �� ���� 

	static unsigned short  index_time  = 0, cds_cnt = 0;
    static unsigned long   CDS_adc_value_sum = 0;

    TCNT0 = 256 - 31;         // ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                              // �����÷����ͷ�Ʈ �ֱ� = 2msec
                              // 31 = 2msec/ 64usec,  TCNT0 = 256 - 31
	index_time++;

	if( index_time  == 5 )    //  ���ø��ֱ� 10msec(= 5 x 2msec ) ���� ����, �µ� ���� 
    { 

        index_time = 0;    // ���� �ʱ�ȭ
		CDS_adc_value = ADC_Read( 2 );      // AD ��ȯ �Ϸ�� CDS���� ������ ��( �������� ADC )�� ���������� ����

		////// ����� �����ϱ� ���� CDS ���� ADC���� ��հ��� ����.   ////////////////////

       CDS_adc_value_sum +=  CDS_adc_value; 
	   cds_cnt++;

	   if( cds_cnt == 16 )       // 16�� ���( 0.16�� = 16 * 10msec  ���� ��հ� ����.)
	   {
          CDS_adc_value_avg = CDS_adc_value_sum / cds_cnt;

          cds_cnt = 0;
          CDS_adc_value_sum = 0; 
	   }

       /////  ADC �������� ��հ��� CDS ���� ���������� ����   ////// 

       CDS_value = (unsigned short) CDS_adc_value_avg;

	    if( CDS_value < 250 )           // ����� ������ ���� ���� 500 �̸��̸� LED1 (PA2) ON 
      	{    
		  IR_flag = 1;					//IR ���� ON
          PORTA &=  ~0x04;            // LED1 ON ( PA2 = 0 )
                                      // PORTA = 0b**** *0**,   PORTA = PORTA & ~0b0000 0100( ~0x04 )
      	}
      	else if( CDS_value >= 250 )     // ����� ������ ���� ���� 500 �̻��̸� LED1 (PA2) OFF
      	{
		   IR_flag = 0;					//IR ���� OFF
           PORTA |=  0x04;            // LED1 OFF ( PA2 = 1 )
                                      // PORTA = 0b**** *1**,   PORTA = PORTA | 0b0000 0100( 0x04 )
      	}
	}

	if( IR_flag == 1 )
	{
		
    	/////  ���ܼ�LED(EL-7L) �ѱ�(ON)  //////

    	PORTA |=  0x01;        // ���ܼ�LED(EL-7L) ON ( PA0 = 1)
                           // PORTA = 0b**** ***1,   PORTA = PORTA | 0b0000 0001( 0x01 )
    	usec_delay(50);        // 50usec���� ��� �ѱ�(���ܼ��� �ݻ�Ǿ� ���ƿö�(����ȭ�� ��)���� ��� �Ҵ�.) 
	
		IR_adc0_value = ADC_Read( 1 );	// ADC ��ȯ �Ϸ�� ������ ��( �������� ADC )�� ���������� ����

    	/////  ���ܼ�LED(EL-7L) ����(OFF)  //////
    	PORTA &= ~0x01;        // ���ܼ�LED(EL-7L) OFF ( PA0 = 0 )
                           // PORTA = 0b**** ***0,   PORTA = PORTA & ~0b0000 0001( ~0x01 )

		if( IR_adc0_value > 600 )	// warning mode
		{
			PORTA |= 0x08;		   // ���� ON PA3 = 1
			PORTA &= ~0x10;		   // LED ON: PA4 = 0
		}
		else if( IR_adc0_value <= 600 )	//normal mode
		{
			PORTA &= ~0x08;		   // ���� OFF PA3 = 0
			PORTA |= 0x10;		   // LED OFF: PA4 = 1
		}
	}		// if( IR_flag == 1 )
}

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

//----------------------------------------
//��ĵ �ڵ尪�� ������
//----------------------------------------

unsigned char key_scan(void)
{
  
    unsigned char   in = 0 ;
    
    if (       ! ( PINA  & 0x20  )  )        in = KEY1 ; 	  //   PA5 key scan
    else                                     in = NO_KEY ; 	
    
    return in ;
    
}

///////////////////////////////////////////////////////////////////
 
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
