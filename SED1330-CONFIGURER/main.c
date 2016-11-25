/*
 * SED1330-CONFIGURER.c
 *
 * Created: 2016-11-20 20:05:21
 * Author : saper
 */ 

#include <avr/io.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DEBUG_MODE 1


#include "libs/delay.h"
#include "libs/sed1330.h"
#include "libs/usart.h"
//#include "lcd-graphic.h"

//#include "graph-bmp.h" // logo :D
//#include "graph-bitmap.h" // big image

#define led_toggle PORTB ^= 0x01
#define led_off PORTB &= ~0x01
#define led_on PORTB |= 0x01

//#define SED_INIT_CONF_LEN 10
//#define sed_init_conf[SED_INIT_CONF_LEN] = {  }; 

#define RS_BUFF_SIZE 24
uint8_t rsBuffPtr = 0;
uint8_t rsBuff[RS_BUFF_SIZE];

void flushRSBuff(void) {
	for (uint8_t i=0;i<RS_BUFF_SIZE;i++) rsBuff[i]=0;
	rsBuffPtr=0;
	#ifdef DEBUG_MODE
		usart_send_strP(PSTR("<<<RS-FLUSH>>>"));
	#endif
}


uint8_t hex2dec(char hex) {
	if (hex > 0x2f && hex < 0x3a) return hex-0x30;
	if (hex > 0x40 && hex < 0x47) return hex-0x37;
	if (hex > 0x60 && hex < 0x67) return hex-0x57;
	return 0x0f;
}

uint8_t bcd2bin(uint8_t bcd) {
	uint8_t res=(bcd&0x0f);
	bcd >>= 4;
	res += (bcd&0x0f)*10;
	return res;
}

void usart_crlf(void) {
	usart_send_char(0x0d);
	usart_send_char(0x0a);
}

void fill_text1_chartest(void) {
	sed_clear(SED_TEXT1, 0x00);
	sed_cursor(0x0000);
	sed_command(SED_MEM_MWRITE);
	uint8_t c=33;
	for (uint16_t i=0;i<SED_SAD1_SIZE;i++) {
		sed_data(c++);
		if (c>127) c=32;
	}
}

// 9 bytes: SysSet[P1=0,P2=1,P4..P8=2..6] Scroll[P3=P6=7] HDOT_SCR[P1=8]
void sed_init2(uint8_t conf[]) {
	SED_PORT_DATA = 0xff;
	//DDR(SED_PORT_DATA) = 0xff;
	sed_data_out();
	SED_DDR_CTRL |= (1<<SED_PIN_RST | 1<<SED_PIN_CS | 1<<SED_PIN_WR | 1<<SED_PIN_RD | 1<<SED_PIN_A0);
	SED_PORT_CTRL |= (1<<SED_PIN_RST | 1<<SED_PIN_CS | 1<<SED_PIN_WR | 1<<SED_PIN_RD | 1<<SED_PIN_A0);
	// Reset :)
	delay1ms(20);
	//sed_cs_control = SED_CS_CTRL_AUTO;
	sed_set_cs();
	sed_set_rd();
	sed_set_wr();
	
	sed_clr_rst();
	
	delay1ms(5); // min. 200us
	sed_set_rst();
	delay1ms(10);
	// SYSTEM SET
	usart_send_strP(PSTR("\r\nSYS_SET[P1..8]="));
	sed_command(0x40); // system set
		// P1:
		//  *[0] M0 (0x01) - 0=Internal CG ROM, 1=External
		//  *[1] M1 (0x02) - 0=CG RAM1 32char, 1=64char CG RAM + CG RAM2
		//  *[2] M2 (0x04) - 0=8px char height , 1=16px char height <- External CG ROM
		//  *[3] W/S (0x08) - 0=Single panel LCD , 1=Dual panel LCD
		//  *[4] 1 (0x10) - 1= Always 1...
		//  *[5] IV (0x20) - 0=Screen top-line correction , 1=No correction
		//  *[6] TL (0x40) - 0=LCD Mode , 1=TV Mode (Only SED1336 ?)
		//  *[7] DR (0x80) - 0=Normal , 1=Additional Shift clock cycles
		sed_data(conf[0]); // M2:M0=000, WS=0, IV=1, TL=0, DR=0
		usart_send_hex_byte(conf[0]); usart_send_char(' ');
		// P2:
		//  *[2:0] FX (0x00...0x07) - Character field width + 1 (1...8)
		//  *[7] WF (0x80) - 0=16line AC Drive, 1=Two-frame AC Drive
		sed_data(conf[1]); // FX=7 (8px) or FX=5 (6px), WF=0 ::: FX >= DisplayWidth / CharsPerLine
		usart_send_hex_byte(conf[1]); usart_send_char(' ');
		// P3:
		//  *[3:0] FY (0x00...0x0F) - Height in pixels of character + 1 (1...16)
		sed_data(7); // FY=7 (8px)
		usart_send_hex_byte(7); usart_send_char(' ');
		// P4:
		//  *[7:0] CR (0x00...0xEF - 0...239) - "Section 3.2.1.12" :::  CR=RND(FX / 8) * CharsPerLine
		sed_data(conf[2]); //50
		usart_send_hex_byte(conf[2]); usart_send_char(' ');
		// P5:
		//  *[7:0] TCR (0x00...0xFF - 1...256) - must be: >= CR+4
		sed_data(conf[3]); // P5
		usart_send_hex_byte(conf[3]); usart_send_char(' ');
		// P6:
		//  *[7:0] LF (0x00...0xFF) - Height in lines of frame, height=LF+1
		sed_data(conf[4]); // LF=(239+2=dummy lines)+1=242
		usart_send_hex_byte(conf[4]); usart_send_char(' ');
		// P7 & P8 - AP - horizontal address range = 256 = 0x0100 => [15:8-7:0] 0x01-0x00
		//  *[15:0] AP - horizontal address range of the virtual screen (same as CR??)
		sed_data(conf[5]); // P7 - AP Low
		sed_data(conf[6]); // P8 - AP High
		usart_send_hex_byte(conf[5]); usart_send_char(' ');
		usart_send_hex_byte(conf[6]);

	// Memory map:
	// TEXT: Chars per display: 50*20=1000chars (bytes) per display
	// GRAPH: ((400*160)/8) = 8000bytes
	// SAD1 :> dispaly 1, Layer 1 ( TEXT ) - 20rows, 50char = 1000chars per panel
	// SAD2 :> display 1, Layer 2 ( Graphic ) ; 160 rows, 400cols = 64000 / 8bits = 8000 bytes per panel
	// SAD3 :> display 2, Layer 1 ( TEXT )
	// SAD4 :> display 2, Layer 2 ( Graphic ) - INVALID FOR THIS PANEL (WS=0)
	usart_send_strP(PSTR("\r\nSCROLL[P3P6]="));
	sed_command(0x44); // Scroll
		// P1:P2 = SAD1 L:H
		sed_data(SED_SAD1L); // P1
		sed_data(SED_SAD1H); // P2
		// P3 - SL1 - Screen block start address - numbers of lines per display
		sed_data(conf[7]); // P3
		usart_send_hex_byte(conf[7]); usart_send_char(' ');
		// P4:P5 = SAD2 L:H
		sed_data(SED_SAD2L); // P4
		sed_data(SED_SAD2H); // P5
		// P6 - SL2 - same as SL1
		sed_data(conf[7]); // P6
		// P7:P8 = SAD3 L:H
		//sed_data(SED_SAD3L); // P7
		//sed_data(SED_SAD3H); // P8
		// P9:P10 = SAD4 L:H
		//sed_data(SED_SAD4L); // P9
		//sed_data(SED_SAD4H); // P10
	usart_send_strP(PSTR("\r\nHDOT_SCR[P1]="));
	sed_command(0x5A); // HDOT SCR
		sed_data(conf[8]); // 0=no horizontal scroll of display
		usart_send_hex_byte(conf[8]); usart_crlf();
	sed_command(0x5B); // OVLAY
		// MX0,MX1 = [0:1] 0,0 0x00=OR (0x00) [ L1 | L2 | L3 ]
		// MX0,MX1 = [0:1] 0,1 0x01=EX-OR (0x01) [ (L1 ^ L2) | (L3 ^ L4) ] // L2 and L4 equal?
		// MX0,MX1 = [0:1] 1,0 0x02=AND (0x02) [ (L1 & L2) | (L3 & L4) ]
		// MX0,MX1 = [0:1] 1,1 0x03=PriorityOR (0x03) [ L1 > L2(L4??) > L3)
		// DM1,DM2 = [2:3] 0,0 0x00=Text mode for Layer 1 and 3 (Layer 1 Upper panel, Layer 1 lower panel) (0x00)
		// DM1,DM2 = [2:3] 1,1 0x0c=Graphic mode for Layer 1 and 3 (Layer 1 Upper panel, Layer 1 lower panel) (0x0C)
		// OV = [4] 0 - Mixed mode (text+graph), two layer (0x10)
		sed_data(0x01);
	sed_command(0x58); // disp ON/OFF => disable screen
		// FC0,1 = [0:1] (0x01,0x02) - cursor flash rate
		// FP0,1 = [2:3] (0x04,0x08) - SAD1 ON/OFF + flashing
		// FP2,3 = [4:5] (0x10,0x20) - SAD2,4 ON/OFF + flashing
		// FP4,5 = [6:7] (0x40,0x80) - SAD3 ON/OFF + flashing
		sed_data(0x04|0x10); // SAD1, SAD2&4, SAD3 = ON
	sed_command(0x5D); // CSR FORM
		sed_data(7); // CRX <= FX - cursor width (8px - same as char width)
		//sed_data(0x87); // CRY = FY-1 - cursor height (8px - same as char height) ; CM=1 - block cursor
		sed_data(0x80|7); // CRY = FY-1 - cursor height (8px - same as char height) ; CM=0 - line cursor
	sed_command(0x4C); // CSRDIR - Cursor move in right dir.
	// "zero" whole memory
	sed_command(SED_DRAW_CSRW);
		sed_data(0); sed_data(0);
	sed_command(SED_MEM_MWRITE);
		for (uint16_t t=0;t<0x8000;t++) sed_data(0);
	// for the end switch back on display
	sed_command(0x59); // disp ON/OFF => DISP_ON
		// FC0,1 = [0:1] (0x01,0x02) - cursor flash rate
		// FP0,1 = [2:3] (0x04,0x08) - SAD1 ON/OFF + flashing
		// FP2,3 = [4:5] (0x10,0x20) - SAD2,4 ON/OFF + flashing
		// FP4,5 = [6:7] (0x40,0x80) - SAD3 ON/OFF + flashing
		sed_data(0x04|0x10); // SAD1, SAD2&4
}

void processRSBuff(void) {
	uint8_t tmp=0;
	
	if (rsBuff[0] == 'c') {
		uint8_t conf[9];
		for (uint8_t i=0, j=1;i<9;i++,j+=2) {
			tmp = hex2dec(rsBuff[j]) & 0x0f;
			conf[i] = (tmp<<4) | (hex2dec(rsBuff[j+1]) & 0x0f);
		}
		usart_send_char('c');
		for (uint8_t i=0;i<9;i++) usart_send_hex_byte(conf[i]);
		usart_send_char(0x0d); usart_send_char(0x0a);
		usart_send_strP(PSTR("INIT\r\n"));
		sed_init2(conf);
		usart_send_strP(PSTR("FILL\r\n"));
		fill_text1_chartest();
		usart_send_strP(PSTR("DONE\r\n"));
		} else if (rsBuff[0] == 'p') {
		tmp = ((hex2dec(rsBuff[1])<<4) & 0xf0) | (hex2dec(rsBuff[2]) & 0x0f);
		if (tmp == 0) {
			usart_send_strP(PSTR("LCDOFF\r\n"));
			sed_onoff(0);
			} else {
			usart_send_strP(PSTR("LCDON\r\n"));
			sed_onoff(1);
		}
	}
	flushRSBuff();
}

int main(void)
{
	//uint16_t x,y;
	//uint16_t t16=0;
	
	DDRB |= 1<<0;
	
	sei();
	usart_config(25,USART_RX_ENABLE|USART_TX_ENABLE|USART_RX_INT_COMPLET,USART_MODE_8N1);
	
	usart_send_strP(PSTR("\r\n\r\n********** SED1330 + def init for 400x160px panel ********\r\n"));

	led_off;
	sed_init();

	sed_onoff(1);
	led_on;
	//uint8_t buff[10];
	//for (uint8_t i=0;i<10;i++) buff[i]=0;

	//sed_init2(buff);

	fill_text1_chartest();
	
	led_off;
	while(1) { 
		delay1ms(50);
		led_toggle;
		if (rsBuffPtr == 254) {
			processRSBuff();
		}
	}
}

SIGNAL(USART_RXC_vect) {
	char c = UDR;
	
	if (c == 0x0d) { // CR
			rsBuffPtr=254; // mark as buffer to be processed
	} else if (c == 0x1b) { // ESC
		flushRSBuff();
	} else {
		if (rsBuffPtr<RS_BUFF_SIZE) {
			rsBuff[rsBuffPtr]=c;
			rsBuffPtr++;
		}
	}
}