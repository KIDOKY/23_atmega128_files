
/////  < Bluetooth_Prog1=1.c >  : ����������(HC-06)�� �̿��Ͽ� ����Ʈ���� ����ϴ� ATmega128 �ڵ�  ///// 


#include <avr/io.h>          // ATmega128 �̿�� �ݵ�� �����ؾ���.(���� HW���� �������Ͱ� ����Ǿ� ����)
#include <avr/interrupt.h>   // ���ͷ�Ʈ �̿�� �ݵ�� �����ؾ���.  
#include <util/delay.h>      // �ð������Լ� �̿�� �ݵ�� �����ؾ���.
 
#include "lcd.h"             // LCD�� ���� ���÷����� �� �ݵ�� �����ؾ���.

void SerialPutChar_USART1( char ch );       // USART1 ���(Serial ���)��Ʈ�� 1����Ʈ �����͸� �۽��ϴ� �Լ�
void SerialPutString_USART1( char str[] );  // USART1 ���(Serial ���)��Ʈ�� ���ڿ� �����͸� �۽��ϴ� �Լ�

void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 	 
void Display_Number_LCD( unsigned short num, unsigned char digit );  // ��ȣ���� ������ ������ 10���� ���·� 
                                                                     // LCD �� ���÷���, digit: ���÷����� �ڸ��� 
void msec_delay(int n);     // msec ���� �ð�����
void usec_delay(int n);     // usec ���� �ð�����

static volatile char Send_Message_1[] = { "Received Data Count = " } ;     
static volatile  char  recv_cnt = 0, rdata=0, new_recv_flag = 0  ;               
static volatile unsigned char   Command_Error_Flag = 0 ; 


int main() 
{   

    LcdInit();                     // LCd �ʱ�ȭ �Լ� ȣ��

    LcdMove(0,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 0�� 0��)
    LcdPuts("Bluetooth Prog");     // LCD�� ������� �׽�Ʈ �޽��� ���÷��� 
    LcdMove(1,0);                  // LCD�� �� ������ �ʱ� ��ġ ����( 1�� 0��)   
    LcdPuts("Send Command.");      // LCD�� ������� �׽�Ʈ �޽��� ���÷���

    msec_delay(1000);             // 1��(1000msec) �ð����� 

//  ��� ��Ʈ ���� 	   (���� pp75-76 �������ͱ׸�, ǥ6.1 ����)   
 
    DDRA |= 0x01;      // LED1( ���������� �׽�Ʈ ��)(PA0) : �����Ʈ�� ����                         
                       // DDRA = 0b**** ***1, DDRA = DDRA | 0b0000 0001( 0x01 ) 

    PORTA |= 0x01;     // �ʱ⿡ LED1 OFF :  PA0 = 1 ���                        
                       // PORTA = 0b**** ***1, PORTA = PORTA | 0b0000 0001( 0x01 ) 

/////////  USART1 ��� ��Ʈ ���� (344�� - 351�� ���� �������Ϳ� ǥ ���� )  //////////////////   
 
    UCSR1A &= ~0x01;      // ���� ���μ�����Ÿ�� ��� ����. --> bit0=0  
                          // ���ۼӵ� �谡 ��� �����. --> bit1=1
                          // UCSR1A = 0b**** **10 --> 
			  // UCSR1A = UCSR1A & ~0b0000 0001 ( ~0x01 ) : 0 ���� 
    UCSR1A |=  0x02;	  // UCSR1A = UCSR1A & ~0b0000 0010 ( ~0x02 ) : 1 ���� 


    UCSR1B &= ~0x04;      // ����ũ��(������ ���ۺ�Ʈ �� ) = 8��Ʈ --> bit2(UCSZn2)=0 
                          // �ۼ��� ���(enable) --> bit4=1, bit3=1                           
                          // UCSR1B = 0b***1 10** --> 
                          // UCSR1B = UCSR1B & ~0b0000 0100 ( ~0x04 ) : 0����
    UCSR1B |= 0x18;       // UCSR1B = UCSR1B |   0b0001 1000 (  0x18 ) : 1����

    UCSR1C &= ~0x78;     // ����ũ��(������ ���ۺ�Ʈ �� ) = 8��Ʈ --> bit2(UCSZn1)=1,  bit1(UCSZn0)=1  
                         // ������Ʈ�� = 1��Ʈ ��� --> bit3 = 0   
                         // �񵿱� ��Ÿ�� ���� --> bit6=0,  �и�Ƽüũ��� ��� ����. --> bit5=0, bit4=0 
                         // UCSR1C = 0b*000 011* --> 
                         // UCSR1C = UCSR1C & ~0b0111 1000 ( ~0x78 ) : 0����
    UCSR1C |= 0x06;      // UCSR1C = UCSR1C |   0b0000 0110 (  0x06 ) : 1����

    // USART1 ��Ʈ�� ������Ʈ ���� ��������(UBRR1)�� 2����Ʈ(16��Ʈ) �������� �ε� ���⿡ ���� ������ 
    // 2����Ʈ ������ ���⸦ �� �� ���� ���� 1����Ʈ(UBRR1H) ����� ���� 1����Ʈ(UBRR1L)���⸦ ���� �ؾ� ��. 
    // ǥ17.8�� �����ϸ� �ý���Ŭ�� ���ļ��� 16MHz�� �� ������Ʈ 9600 bps�� �����Ϸ��� 
    // UBRR1(16��Ʈ) �������Ϳ� 207�� ���־�� ��. 207�� 8��Ʈ ���� �����̹Ƿ� UBRR1 ����������  
    // ���� 1����Ʈ(UBRR1L)�� 207�� ���ְ� ���� 1����Ʈ(UBRR1H)���� 0�� ���ָ� ��.
 
    UBRR1H = 0x00;      
    UBRR1L = 207;          // ������Ʈ(Baud Rate) = 9600 bps�� ���� ( 361��, ǥ 17.8 ���� )

    UCSR1B |= 0x80;        // UART1 �۽�(RX) �Ϸ� ���ͷ�Ʈ ���
                           // UCSR1B = 0b1*** ****, UCSR1B = UCSR1B | 0b1000 0000( 0x80 ) 
						                
    sei();                 // �������ͷ�Ʈ��� 


    while (1) 
    { 
       if( new_recv_flag == 1 )             // ���ο� �� ���ڵ�����(1����Ʈ)�� ���ŵǾ�����  
	   { 

	       /////////////  ���ŵ� ������(��ɾ�) ó��   //////////////

	       if( rdata == 'a' )          // ���ŵ� ���� �����Ͱ� ��a�� �̸� 
	       {
               PORTA |= 0x01;          // LED1 OFF 
	       }

	       else if( rdata == 'b' )     // ���ŵ� ���� �����Ͱ� 'b' �̸�
	       {
               PORTA &= ~0x01;         // LED1 ON
	       }

	       else if( rdata == 'c' )     // ���ŵ� ���� �����Ͱ� 'c' �̸�
	       {
                    PORTA ^= 0x01;         // LED1 Toggle
	       }

	       else if( rdata == 'd')      // ���ŵ� ���ڵ����Ͱ� ��d�� �̸�
	       {
               SerialPutString_USART1( "Received Data Count = " );     //  �޽����� �޴������� ����
 
		        HexToDec(recv_cnt,10);            // ���ŵ� ����Ʈ���� ��Ÿ���� ������ recv_cnt�� �������� ��ȯ

                // �������� ��ȯ�� ������ �ڸ����� ���ڵ�����(ASCII �ڵ�)�� ��ȯ�Ͽ� ����Ʈ������ ����

                SerialPutChar_USART1( NumToAsc(cnumber[2]) );  // ���� recv_cnt ���� ���� ����(100�ڸ�)���� ���� 
                SerialPutChar_USART1( NumToAsc(cnumber[1]) );  // ���� recv_cnt ���� 10�ڸ� ����
                SerialPutChar_USART1( NumToAsc(cnumber[0]) );  // ���� recv_cnt ���� 1�ڸ� ����
                SerialPutChar_USART1('\n');    // �޴������� ������ ���� �� Line Feed('\n')�� �׻� ���� �����ؾ���
	       } 

	       else                                //  ���ŵ� ���ڵ����Ͱ� ���ǵ��� ���� ����̸�(��, ��� ���� �̸�)
	       {
              SerialPutString_USART1( "Command Error!!  Try again.\n" ); //  ��� ���� �޽����� �޴������� ����

		      Command_Error_Flag = 1;          // ��� ���� �÷��� ��
	       }

        ////////////////  LCD ���÷��� //////////////////////////////////

           if( Command_Error_Flag == 0  )          // ��ɿ� ������ ������  
	       {  
                /////  ���ŵ� ����Ʈ��(���� ��) LCD ���÷���  /////////////////
                LcdCommand( ALLCLR ) ;    // LCD Clear

	            LcdMove(0,0);    
	            LcdPuts("Recv cnt = "); 
                Display_Number_LCD( recv_cnt, 3 ); //���ŵ� ����Ʈ�� recv_cnt�� �������� ��ȯ�Ͽ� LCD�� ���÷���
	            LcdMove(1,0);    
	            LcdPuts("Recv data = "); 
	            LcdPutchar( rdata );       // ���ŵ� ���� rdata�� LCD�� ���÷���

           }

	       else if( Command_Error_Flag == 1 )    // ��ɿ� ������ ������
	       {  

		      LcdCommand( 0x01) ;         // LCD Claear
		      LcdMove(0, 0 );             // LCD�� �����޽��� ���÷���
		      LcdPuts("Cmd Error!!"); 
		      LcdMove(1, 0 );
		      LcdPuts("Try Again."); 

		      Command_Error_Flag = 0 ;     // Command_Error_Flag ���� 
           }


           new_recv_flag = 0;                      // �� ����(���) ���� �÷��� ����

  
       }    //  if( new_recv_flag == 1 )  �� ��      // ���ο� �� ���ڵ�����(1����Ʈ)�� ���ſϷ� �� 


   }     // ���ѷ��� while (1) �� �� 

}        // int main() �Լ��� �� 

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

//////  USART1 ���(�ø������) ���ſϷ� ���ͷ�Ʈ ���� ���α׷�   ///////

ISR( USART1_RX_vect )      // 107��( �Ǵ� 108��) ǥ7.2 ���ͷ�Ʈ������ ��ũ�� ����
{

    rdata = UDR1;           // USART1 ���Ŵ���(RXD1 ��)�κ��� ���ŵ� 1����Ʈ �����͸� �������� rdata�� ����
 
    SerialPutChar_USART1(rdata);  //Echo: ���ŵ� �����͸� �ٷ� �޴������� �۽��Ͽ� ���ŵȵ����Ͱ� ��Ȯ���� Ȯ�� 
    SerialPutChar_USART1('\n');   //�޴������� ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ���

    recv_cnt++ ;            // ���ŵ� ������ ����Ʈ�� ���� �� ����

    new_recv_flag = 1;      // ���ο� ����(���)�� �޴������κ��� ���ŵǾ����� ��Ÿ���� ���� �÷��� ������ Set

}


////////////////////////////////////////////////////////////
// �� ���ڸ� �۽��Ѵ�.
///////////////////////////////////////////////////////////

void SerialPutChar_USART1(char ch)
{
	while( !( UCSR1A & 0x20 ) );	// ���۰� �� ��(���ο� �����͸� �۽� �����Ҷ�)�� ��ٸ�
                                        // UCSR1A ���������� �����ͷ������ͺ�( bit5(UDRE) )��Ʈ�� 
                                        // 1 �� �� ������ ��ٸ�.

  	UDR1 = ch;		        // ���ۿ� ���ڸ� ����
}


//////////////////////////////////////////////////////////////////
// ���ڿ��� �۽��Ѵ�.
// �Է�   : str - �۽��� ���ڿ��� ������ ������ �ּ�
/////////////////////////////////////////////////////////////////

void SerialPutString_USART1(char *str)
{

    while(*str != '\0')         // ���ŵ� ���ڰ� Null ����( 0x00 )�� �ƴϸ� 
    {

        SerialPutChar_USART1(*str++);
    }
}


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

