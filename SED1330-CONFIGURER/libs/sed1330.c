/* ****************************************************************************
   REWRITE HEADER
   **************************************************************************** */

#include <avr/io.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#include "delay.h"
#include "sed1330.h"

// ------- Font libray inclusion ----------------
#ifdef SED_FUNCTS_STRINGS
	#include "font.h"
#endif
// ----------------------------------------------
/*
#define sed_en_pulse() {\
			nop; \
			sed_set_en(); \
			nop; nop; nop; \
			nop; nop; nop; \
			sed_clr_en(); } */


// This define witch mode sed is entered: graphic mode use 8bit char width, 
// but text-mode use 6bit char-width and 5bit of right side are not used in this mode.
//uint8_t sed_mode=SED_MODE_NONE;
//uint8_t sed_cs_control=SED_CS_CTRL_AUTO;
//uint8_t sed_disp=SED_DISP1;


void sed_data(uint8_t data) {
	sed_data_out(); //SED_DDR_DATA = 0xff;
	SED_PORT_DATA = data;
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_clr_cs();
	sed_clr_A0();
	sed_clr_wr();
	delay1us(1);//nop; //nop; 
	//nop; nop; 
	// writting data to SED1330...
	//nop; nop;
	sed_set_wr();
	delay1us(1);//
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_set_cs();
	sed_data_in();
	
	while (sed_readBusy() == 0) {
		nop;
	}
}

void sed_command(uint8_t cmd) {
	sed_data_out(); //  SED_DDR_DATA = 0xff;
	SED_PORT_DATA = cmd;
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_clr_cs();
	sed_set_A0();
	sed_clr_wr();
	delay1us(1);//nop;// nop; 
	//nop; nop; 
	// writting command....
	//nop; nop;
	sed_set_wr();
	delay1us(1);//
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_set_cs();
	sed_data_in();
	
	while (sed_readBusy() == 0) {
		nop;
	}	
}

uint8_t sed_read(void) {
	// his might need some fixes
	uint8_t t;
	sed_data_in(); //SED_DDR_DATA = 0x00;
	SED_PORT_DATA = 0xff;
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_clr_cs();
	sed_set_A0();
	sed_set_wr();
	sed_clr_rd();
	//nop; nop;
	t = SED_PIN_DATA;
	//nop;
	//nop;
	//nop;
	delay1us(1);//nop;
	//nop;
	t = SED_PIN_DATA;
	//nop;
	//nop;
	//nop;
	delay1us(1);//nop;
	//nop;
	t = SED_PIN_DATA;
	//nop;
	delay1us(1);//nop;
	t = SED_PIN_DATA;
	sed_set_rd();
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_set_cs();
	 
	//DDR(SED_PORT_DATA) = 0xff;
	sed_data_in();
	SED_PORT_DATA = 0xff;
	return t;
}

uint8_t sed_read_memory(void) {
	// 
	uint8_t t;
	// write command
	//DDR(SED_PORT_DATA) = 0xff;
	sed_data_out();
	SED_PORT_DATA = SED_MEM_MREAD;
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_clr_cs();
	sed_set_A0();
	sed_set_rd();
	sed_clr_wr();
	delay1us(1);//
	//nop;// nop; 
	// writting command....
	//nop;// nop;
	sed_set_wr();
	//nop;
	sed_data_in();//SED_DDR_DATA = 0x00;
	SED_PORT_DATA = 0xff;
	//nop;
	sed_clr_rd();
	//nop; //nop;
	//t = PIN(SED_PORT_DATA);
	delay1us(1);//nop;
	//nop;
	t = SED_PIN_DATA;
	//nop;
	delay1us(1);//nop;
	t = SED_PIN_DATA;
	//nop;
	delay1us(1);//nop;
	t = SED_PIN_DATA;
	sed_set_rd();
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_set_cs();

	//DDR(SED_PORT_DATA) = 0xff;
	sed_data_in();
	SED_PORT_DATA = 0xff;
	return t;
}


uint8_t sed_readBusy(void) {
	uint8_t t;
	sed_data_in();// SED_PORT_DATA = 0x00;
	SED_PORT_DATA = 0xff;
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_clr_cs();
	sed_clr_A0();
	sed_clr_rd();
	//nop;
	//t = PIN(SED_PORT_DATA);
	//nop;
	//nop;
	//nop;
	delay1us(1);//nop;
	t = SED_PIN_DATA;
	//nop;
	//nop;
	//nop;
	delay1us(1);//nop;
	t = SED_PIN_DATA;
	sed_set_rd();
	//if (sed_cs_control == SED_CS_CTRL_AUTO)  
	sed_set_cs();
	t &= 0x40;
	//DDR(SED_PORT_DATA) = 0xff;
	sed_data_in();
	SED_PORT_DATA = 0xff;//*/
	return t;
} // */

/*void sed_initText(void) {	
	SED_PORT_DATA = 0xff;
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
	delay1ms(15);
	
	// SYSTEM SET
	sed_command(0x40); // system set
		// P1:
		//  *[0] M0  (0x01) - 0=Internal CG ROM, 1=External
		//  *[1] M1  (0x02) - 0=CG RAM1 32char, 1=64char CG RAM + CG RAM2
		//  *[2] M2  (0x04) - 0=8px char height , 1=16px char height <- External CG ROM
		//  *[3] W/S (0x08) - 0=Single panel LCD , 1=Dual panel LCD
		//  *[4] 1   (0x10) - 1= Always 1...
		//  *[5] IV  (0x20) - 0=Screen top-line correction , 1=No correction
		//  *[6] TL  (0x40) - 0=LCD Mode , 1=TV Mode (Only SED1336 ?)
		//  *[7] DR  (0x80) - 0=Normal , 1=Additional Shift clock cycles
		sed_data(0x10|0x20);
		// P2:
		//  *[2:0] FX (0x00...0x07) - Character field width + 1 (1...8)
		//  *[7] WF (0x80) - 0=16line AC Drive, 1=Two-frame AC Drive
			// CharsPerLine = 53 (6px char width when disp. width 320px)
		sed_data(0x80|5); // FX=7 (8px) or FX=5 (6px), WF=0 ::: FX >= DisplayWidth / CharsPerLine
		// P3:
		//  *[3:0] FY (0x00...0x0F) - Height in pixels of character + 1 (1...16)
		sed_data(7); // FY=7 (8px)
		// P4: 
		//  *[7:0] CR (0x00...0xEF - 0...239) - "Section 3.2.1.12" :::  CR=RND(FX / 8) * CharsPerLine
		#define SED_TEXT_CR 53
		sed_data(SED_TEXT_CR-1);
		// P5:
		//  *[7:0] TCR (0x00...0xFF - 1...256) - must be: >= CR+4
		sed_data(SED_TEXT_CR+4);
		// P6:
		//  *[7:0] LF (0x00...0xFF) - Height in lines of frame, height=LF+1
		sed_data(241); // LF=(239+2)+1=242  // 2 dummy lines need to be generated for display
		// P7 & P8 - AP - horizontal address range = 256 = 0x0100 => [15:8-7:0] 0x01-0x00
		//  *[15:0] AP - horiontal address range of the virtual screen (same as CR??)
		sed_data(SED_TEXT_CR); // P7 - AP Low
		sed_data(0); // P8 - AP High
	
	// Memory map:
	// Hint: Display 1 - upper panel; Display 2 - lower panel
	// SAD1 :> dispaly 1, Layer 1 ( TEXT )
	// SAD3 :> display 2, Layer 1 ( TEXT )
	// SAD2 :> display 1, Layer 2 ( Graphic ) ; 64 rows, 256cols = 16384 / 8bits = 2048bytes per panel
	// SAD4 :> display 2, Layer 2 ( Graphic )
	sed_command(0x44); // Scroll
		// WS=0 for EDMMPU3BGF
		#define SED_SAD_TEXT_SL 242
		#define SED_SAD1TEXT 0
		#define SED_SAD2TEXT 640
		#define SED_SAD3TEXT C80
		#define SED_SAD4TEXT 12C0
		// P1:P2 = SAD1 L:H
		sed_data(SED_SAD1TEXT&0x00ff); // P1
		sed_data(SED_SAD1TEXT>>8); // P2
		// P3 - SL1 - Screen block start address - numbers of lines per display
		sed_data(SED_SAD_TEXT_SL); // P3
		// P4:P5 = SAD2 L:H
		sed_data(SED_SAD2TEXT&0x00ff); // P4
		sed_data(SED_SAD2TEXT>>8); // P5
		// P6 - SL2 - same as SL1
		sed_data(SED_SAD_TEXT_SL); // P6
		// P7:P8 = SAD3 L:H
		sed_data(SED_SAD3TEXT&0x00ff); // P7
		sed_data(SED_SAD3TEXT>>8); // P8
		// P9:P10 = SAD4 L:H
		sed_data(SED_SAD4TEXT&0x00ff); // P9
		sed_data(SED_SAD4TEXT>>8); // P10
	sed_command(0x5A); // HDOT SCR
		sed_data(0x00); // 0=no horizontal scroll of display 
	sed_command(0x5B); // OVLAY
		// MX0,MX1 = [0:1] 0,0=OR (0x01,0x02)
		// DM1,DM2 = [2:3] 0,0=Text mode for Layer 1 and 3 (Layer 1 Upper panel, Layer 1 lower panel) (0x04,0x08)
		// OV = [4] 0 - Mixed mode (text+graph), two layer (0x10)
		sed_data(0x01); 
	sed_command(0x58); // disp ON/OFF => disable screen
		// FC0,1 = [0:1] (0x01,0x02) - cursor flash rate
		// FP0,1 = [2:3] (0x04,0x08) - SAD1 ON/OFF + flashing
		// FP2,3 = [4:5] (0x10,0x20) - SAD2,4 ON/OFF + flashing
		// FP4,5 = [6:7] (0x40,0x80) - SAD3 ON/OFF + flashing
		sed_data(0x04|0x10|0x40); // SAD1, SAD2&4, SAD3 = ON
	sed_command(0x5D); // CSR FORM
		sed_data(7); // CRX <= FX - cursor width (8px - same as char width)
		//sed_data(0x87); // CRY = FY-1 - cursor height (8px - same as char height) ; CM=1 - block cursor
		sed_data(0x80|7); // CRY = FY-1 - cursor height (8px - same as char height) ; CM=0 - line cursor
	sed_command(0x59); // disp ON/OFF => DISP_ON
		// FC0,1 = [0:1] (0x01,0x02) - cursor flash rate
		// FP0,1 = [2:3] (0x04,0x08) - SAD1 ON/OFF + flashing
		// FP2,3 = [4:5] (0x10,0x20) - SAD2,4 ON/OFF + flashing
		// FP4,5 = [6:7] (0x40,0x80) - SAD3 ON/OFF + flashing
		sed_data(0x04|0x10|0x40); // SAD1, SAD2&4, SAD3 = ON
	sed_command(0x4C); // CSRDIR - Cursor move in right dir.
	// zero memory
	sed_command(SED_DRAW_CSRW);
		sed_data(0); sed_data(0);
	sed_command(SED_MEM_MWRITE);
	for (uint16_t t=0;t<0x8000;t++) sed_data(0);
	sed_mode = SED_MODE_TEXT;
	sed_disp = SED_DISP1;
}
*/

// Polish fonts for text generator (CGRAM)
#include "sed1330-CGRAM-PL-FONT.h"
void sed_load_CGRAM(void) {
	// **********************************************************************
	// **********************************************************************
	// **********************************************************************
	// load "pl" chars to in video ram after SAD4 (2nd text layer - lower panel)

		#define SED_SAG_START ((uint16_t)(0))
		#define SED_SAG_START2 ((uint16_t)(0x0400))
		// setup CG-RAM...
		sed_command(SED_DISPLAY_CGRAM_ADR);
			sed_data(SED_SAG_START & 0xff); // SAG Low
			sed_data((SED_SAG_START>>8) & 0xff); // SAG high

		sed_command(SED_DRAW_CSRW);
			sed_data(SED_SAG_START2 & 0xff); // Low address
			sed_data((SED_SAG_START2>>8) & 0xff); // high address
		sed_command(SED_MEM_MWRITE);
		for (uint8_t t=0;t<pl_text_font_SIZE;t++) sed_data(pgm_read_byte(&pl_text_font[t]));
}

void sed_init(void) {
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
		sed_data(0x10|0x20); // M2:M0=000, WS=0, IV=1, TL=0, DR=0
		// P2:
		//  *[2:0] FX (0x00...0x07) - Character field width + 1 (1...8)
		//  *[7] WF (0x80) - 0=16line AC Drive, 1=Two-frame AC Drive
		sed_data(0x80|7); // FX=7 (8px) or FX=5 (6px), WF=0 ::: FX >= DisplayWidth / CharsPerLine
		// P3:
		//  *[3:0] FY (0x00...0x0F) - Height in pixels of character + 1 (1...16)
		sed_data(7); // FY=7 (8px)
		// P4: 
		//  *[7:0] CR (0x00...0xEF - 0...239) - "Section 3.2.1.12" :::  CR=RND(FX / 8) * CharsPerLine
		sed_data(49); //50 
		// P5:
		//  *[7:0] TCR (0x00...0xFF - 1...256) - must be: >= CR+4
		#define SED_TCR_VALUE 54
		sed_data(SED_TCR_VALUE); // P5
		// P6:
		//  *[7:0] LF (0x00...0xFF) - Height in lines of frame, height=LF+1
		#define SED_LF_VALUE 159
		sed_data(SED_LF_VALUE); // LF=(239+2=dummy lines)+1=242
		// P7 & P8 - AP - horizontal address range = 256 = 0x0100 => [15:8-7:0] 0x01-0x00
		//  *[15:0] AP - horiontal address range of the virtual screen (same as CR??)
		#define SED_AP_GRAPH (uint16_t)50
		#define SED_APL_GRAPH (SED_AP_GRAPH&0xff)
		#define SED_APH_GRAPH (SED_AP_GRAPH>>8)
		sed_data(SED_APL_GRAPH); // P7 - AP Low
		sed_data(SED_APH_GRAPH); // P8 - AP High
		
		#if (SED_XTAL/((SED_TCR_VALUE*9+1)*SED_LF_VALUE)) < 50
			#warning "SED1330 Crystal Oscillator Frequency is too low, lcd screen might flicker!"
		#endif
	// Memory map:
	// TEXT: Chars per display: 50*20=1000chars (bytes) per display
	// GRAPH: ((400*160)/8) = 8000bytes
	// SAD1 :> dispaly 1, Layer 1 ( TEXT ) - 20rows, 50char = 1000chars per panel
	// SAD2 :> display 1, Layer 2 ( Graphic ) ; 160 rows, 400cols = 64000 / 8bits = 8000 bytes per panel
	// SAD3 :> display 2, Layer 1 ( TEXT )
	// SAD4 :> display 2, Layer 2 ( Graphic ) - INVALID FOR THIS PANEL (WS=0)
	sed_command(0x44); // Scroll
		// P1:P2 = SAD1 L:H
		sed_data(SED_SAD1L); // P1
		sed_data(SED_SAD1H); // P2
		// P3 - SL1 - Screen block start address - numbers of lines per display
		sed_data(SED_SL); // P3
		// P4:P5 = SAD2 L:H
		sed_data(SED_SAD2L); // P4
		sed_data(SED_SAD2H); // P5
		// P6 - SL2 - same as SL1
		sed_data(SED_SL); // P6
		// P7:P8 = SAD3 L:H
		//sed_data(SED_SAD3L); // P7
		//sed_data(SED_SAD3H); // P8
		// P9:P10 = SAD4 L:H
		//sed_data(SED_SAD4L); // P9
		//sed_data(SED_SAD4H); // P10		
	sed_command(0x5A); // HDOT SCR
		sed_data(0x00); // 0=no horizontal scroll of display 
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


void sed_onoff(uint8_t on) {
	if (on == 1) {
		sed_command(0x59); // disp ON
		sed_data(0x04|0x10);//|0x40); // SAD1, SAD2&4, SAD3 = ON
	} else {
		sed_command(0x58); // disp OFF
		sed_data(0x00); // all SADx OFF
	}
}



void sed_clear(uint8_t sec, uint8_t fill) {
	uint16_t t,t2;
	if (sec == SED_TEXT1) {
		t = SED_SAD1_SIZE;
		t2 = SED_SAD1;
	//} else if (sec == SED_TEXT2) {
	//	t = SED_SAD3_SIZE;
	//	t2 = SED_SAD3;
	} else if (sec == SED_GRAPH1) {
		t = SED_SAD2_SIZE;
		t2 = SED_SAD2;
	} else {
		return;
	}
	sed_command(SED_DRAW_CSRW);
		sed_data(t2 & 0xff); // Low address
		sed_data((t2>>8) & 0xff); // high address
	sed_command(SED_MEM_MWRITE);
	while (t--) {
		sed_data(fill);
	}
}

void sed_setOverlayMode(uint8_t ov_mode) {
	sed_command(0x5B); // OVLAY
		// MX0,MX1 = [0:1] 0,0 0x00=OR (0x00) [ L1 | L2 | L3 ]
		// MX0,MX1 = [0:1] 0,1 0x01=EX-OR (0x01) [ (L1 ^ L2) | (L3 ^ L4) ] // L2 and L4 equal?
		// MX0,MX1 = [0:1] 1,0 0x02=AND (0x02) [ (L1 & L2) | (L3 & L4) ]
		// MX0,MX1 = [0:1] 1,1 0x03=PriorityOR (0x03) [ L1 > L2(L4??) > L3)
		// DM1,DM2 = [2:3] 0,0 0x00=Text mode for Layer 1 and 3 (Layer 1 Upper panel, Layer 1 lower panel) (0x00)
		// DM1,DM2 = [2:3] 1,1 0x0c=Graphic mode for Layer 1 and 3 (Layer 1 Upper panel, Layer 1 lower panel) (0x0C)
		// OV = [4] 0 - Mixed mode (text+graph), two layer (0x10)
		sed_data(ov_mode&0x03); 
}

uint8_t sed_txt_char(uint8_t x, uint8_t y, char c) {
	sed_txt_setPos(x,y);
	sed_command(SED_MEM_MWRITE);
		sed_data(c);
	return x+1;
}

uint8_t sed_txt_str(uint8_t x, uint8_t y, char* str) {
	char znak;
	sed_txt_setPos(y,x);
	sed_command(SED_MEM_MWRITE);
	while(0 != (znak = *(str++))) {
		sed_data(znak);
		x++;
	}
	return x;
}

uint8_t sed_txt_str_P(uint8_t x, uint8_t y, const char* str) {
	char znak;
	sed_txt_setPos(y,x);
	sed_command(SED_MEM_MWRITE);
	while (0 != (znak = pgm_read_byte(str++))) {
		sed_data(znak);
		x++;
	}
	return x;
}

uint8_t sed_txt_dec(uint8_t x, uint8_t y, int val) {
	char bufor[7];
	return sed_txt_str(x,y, itoa(val, bufor, 10));
}

void sed_txt_setPos(uint8_t y, uint8_t x) {
	// 30 lines, 40 chars per line...
	uint16_t t=0;
	
	if (x>39) x=39;
	if (y>29) y=29;
	
	t = y*40 + x;
	
	sed_command(SED_DRAW_CSRW);
	sed_data(t); // Low address
	sed_data(t>>8); // high address
}

uint8_t sed_txt_hex_byte(uint8_t x, uint8_t y, uint8_t val) {
	if (((val >> 4) & 0x0f) > 0x09) {
		sed_txt_char(x,y,'7'+((val >> 4) & 0x0f));
	} else {
		sed_txt_char(x,y,'0'+((val >> 4) & 0x0f));
	}
	x++;
	if ((val & 0x0f) > 0x09) {
		sed_txt_char(x,y,'7'+(val & 0x0f));
	} else {
		sed_txt_char(x,y,'0'+(val & 0x0f));
	}
	return ++x;
}

/* ********** OLD TEXT FUNCTIONS *****************
void sed_txt_char(char c) {
	sed_command(SED_MEM_MWRITE);
		sed_data(c);
}

void sed_txt_str(char* str) {
	char znak;
	sed_command(SED_MEM_MWRITE);
	while(0 != (znak = *(str++))) sed_data(znak);
}

void sed_txt_str_P(prog_char* str) {
	char znak;
	sed_command(SED_MEM_MWRITE);
	while (0 != (znak = pgm_read_byte(str++))) sed_data(znak);
}

void sed_txt_dec(int val) {
	char bufor[7];
	sed_txt_str(itoa(val, bufor, 10));
}

void sed_txt_setPos(uint8_t y, uint8_t x) {
	// 30 lines, 40 chars per line...
	uint16_t t=0;
	
	if (x>39) x=39;
	if (y>29) y=29;
	
	t = y*40 + x;
	
	t += y*40;
	
	sed_command(SED_DRAW_CSRW);
	sed_data(t); // Low address
	sed_data(t>>8); // high address
}

void sed_txt_hex_byte(uint8_t val) {
	if (((val >> 4) & 0x0f) > 0x09) {
		sed_txt_char('7'+((val >> 4) & 0x0f));
	} else {
		sed_txt_char('0'+((val >> 4) & 0x0f));
	}
	if ((val & 0x0f) > 0x09) {
		sed_txt_char('7'+(val & 0x0f));
	} else {
		sed_txt_char('0'+(val & 0x0f));
	}
}
*/
void sed_cursor(uint16_t addr) {
	sed_command(SED_DRAW_CSRW);
	sed_data(addr & 0xff); // Low address
	sed_data((addr>>8) & 0xff); // high address
	//sed_data((addr>>8) & 0xff); // high address
}

/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */
/* ****************************************************************************************************************** */

void sed_setPix(uint16_t x, uint8_t y, uint8_t mode) {
	uint16_t t;
	uint8_t tmp,xp;
	// position :)
	// 50 bytes per line: 400/8=50
	// graphic layer starts at 0x0500 SED_SAD2
	t = SED_SAD2 + (y*50) + (x/8);
	xp=x%8;
	
	sed_command(SED_DRAW_CSRW);
	sed_data(t);
	sed_data(t>>8);
	tmp = sed_read_memory();
	
	if (mode == SED_PX_ON) {
		tmp |= 0x80>>xp;
	} else if (mode == SED_PX_OFF) {
		tmp &= ~(0x80>>xp);
	} else if (mode == SED_PX_XOR) {
		if (tmp & (0x80>>xp)) {
			tmp &= ~(0x80>>xp);
		} else {
			tmp |= 0x80>>xp;
		}
	}
	sed_command(SED_DRAW_CSRW);
	sed_data(t);
	sed_data(t>>8);
	sed_command(SED_MEM_MWRITE);
	sed_data(tmp);	
}
