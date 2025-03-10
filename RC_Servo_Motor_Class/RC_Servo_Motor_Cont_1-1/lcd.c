#include<avr/io.h>
#include<util/delay.h>
#include "lcd.h"

#define LCD_PORT	PORTC
#define LCD_DDR		DDRC

static void checkbusy(void);
static void write_command(char command);
static void write_data(char ch);

void LcdInit(void)
{
	LCD_DDR = 0xff;
	_delay_ms(15);
	write_command(0x30);
	_delay_ms(5);
	write_command(0x30);
	_delay_ms(1);
	write_command(0x32);

	LcdCommand(FUNSET);
	LcdCommand(DISP_OFF);
	LcdCommand(ALLCLR);
	LcdCommand(ENTMOD);

	LcdCommand(DISP_ON);
}
void LcdCommand(char command)
{
	checkbusy();
	write_command(command);
	if(command==ALLCLR || command==HOME)
		_delay_ms(2);
}

void LcdPutchar(char ch)
{
	checkbusy();
	write_data(ch);
}

void LcdPuts(char* str)
{
   while(*str)
  {
   LcdPutchar(*str);
   str++;
  }
}

void LcdMove(char line, char pos)

{ 

//  pos = (line << 6) + pos; 

	if(line == 0 )       pos = 0x00 + pos ;
	else if( line == 1 ) pos = 0x40 + pos ;
	else if( line == 2 ) pos = 0x10 + pos ;
	else                 pos = 0x50 + pos ;

  pos |= 0x80;
 
  LcdCommand(pos);
}


void LcdNewchar(char ch, char font[])
{
 int i;

 ch <<=3;
 ch|= 0x40;

 LcdCommand(ch);

 for(i=0;i<8;i++)
    LcdPutchar(font[i]);
}

static void write_command(char command)

{
  char temp;
  temp = (command & 0xF0)|0x04;

LCD_PORT = temp;
LCD_PORT = temp & ~0x04;

temp = (command << 4) | 0x04;

LCD_PORT = temp;
LCD_PORT = temp & ~0x04;
_delay_us(1);
}

static void write_data(char ch)
{
	unsigned char temp;

	temp = (ch & 0xF0) | 0x05;

	LCD_PORT = temp;
	LCD_PORT = temp & ~0x04;

	temp = (ch<<4) | 0x05;

	LCD_PORT = temp;
	LCD_PORT = temp & ~0x04;
}
	static void checkbusy()
{
	_delay_us(10); _delay_us(10); _delay_us(10);
	_delay_us(10); _delay_us(10);
}
