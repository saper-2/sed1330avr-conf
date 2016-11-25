/* ****************************************************************************
   * Name: Driver for SED1330 lcd controller for monochromatic LCD displays - *
   *        - header file                                                     *
   * Author: saper_2                                                          *
   * Contact:          @gmail.com                                             *
   * Date: 17.11.2016                                                         *
   * Version: 3.0 with special configuration for ITE-400160 400x160 lcd panel *
   * License:                                                                 *
   *   You are free to use this library in any kind project, but at least     *
   *   send me a info that YOU are using it. I'll be happy for memory :)      *
   * About: :)                                                                *
   *   This is a driver for SED1330 controller (S1D37000 or something like    *
   *   that...). Iside driver there are only placed functions to display text *
   *   on screen by using CGROM of SED1330. Other functions are already       *
   *   inside unified graphic library. SED is configured into two layer mode: *
   *   LAYER-1 is a text layer (text by CGROM of SED) and LAYER-2 is a        *
   *   graphic one (which is used for drawing by unified library).            *
   *   Panel .............................................................    *
   *   ............................................ Panel have resolution     *
   *   of 400 pix wide and 160 pix in height and it is a single panel (not    *
   *   splitted into two like in DMF6104...)                                  *
   *   Panel pinout is (looking at bottom with visible PCB at TOP,            *
   *   counting from left):                                                   *
   *    1 - xxxxxxxxxxxxxxxxxxxxx                                             *
   *    2 - xxxxxxxxx                                                         *
   *    3 - xx                                                                *
   *    4 - xx                                                                *
   *    5 - xx                                                                *
   *    6 - xx                                                                *
   *    7 - xxxxxxxV (this is same as ablove Vee)                             *
   *    8 - GND                                                               *
   *    9 - xxxxxxx                                                           *
   *   10 - xxx                                                               *
   *   11 - xxxxx                                                             *
   *   12 - xxxxx                                                             *
   *   13 - xxxx                                                              *
   *   14 - xxxxxxx                                                           *
   *                                                                          *
   * Bugs:                                                                    *
   *                                                                          *
   **************************************************************************** */
#ifndef _SED1330LCD_HEADER_
#define _SED1330LCD_HEADER_

/* ************************************************* */
// Hardware connections

#define SED_PORT_DATA PORTC
#define SED_PIN_DATA PINC
#define SED_DDR_DATA DDRC
#define SED_PORT_CTRL PORTA
#define SED_DDR_CTRL DDRA
#define SED_PIN_RST 0
#define SED_PIN_CS 1
#define SED_PIN_WR 2
#define SED_PIN_RD 3
#define SED_PIN_A0 4 // aka A0


#define sed_set_rst() SED_PORT_CTRL |= 1<<SED_PIN_RST;
#define sed_clr_rst() SED_PORT_CTRL &= ~(1<<SED_PIN_RST);

#define sed_set_cs() SED_PORT_CTRL |= 1<<SED_PIN_CS;
#define sed_clr_cs() SED_PORT_CTRL &= ~(1<<SED_PIN_CS);

#define sed_set_wr() SED_PORT_CTRL |= 1<<SED_PIN_WR;
#define sed_clr_wr() SED_PORT_CTRL &= ~(1<<SED_PIN_WR);


#define sed_set_rd() SED_PORT_CTRL |= 1<<SED_PIN_RD;
#define sed_clr_rd() SED_PORT_CTRL &= ~(1<<SED_PIN_RD);

#define sed_set_A0() SED_PORT_CTRL |= 1<<SED_PIN_A0;
#define sed_clr_A0() SED_PORT_CTRL &= ~(1<<SED_PIN_A0);

#define sed_data_in() SED_DDR_DATA = 0x00;
#define sed_data_out() SED_DDR_DATA = 0xff;

/* ************************************************* */

/* ********************************************************** */
/* ********************* REGISTERs ************************** */
// Registers
#define SED_SYSTEM_SET 0x40
#define SED_SLEEP_IN 0x53
#define SED_DISPLAY_ONOFF 0x58
	#define SED_DISPLAY_ON 0x01
	#define SED_DISPLAY_OFF 0x00
#define SED_DISPLAY_SCROLL 0x44
#define SED_DISPLAY_CSRFORM 0x5D
#define SED_DISPLAY_CGRAM_ADR 0x5C
#define SED_DISPLAY_CSRDIR 0x4C
	#define SED_DISPLAY_CSRDIR_LEFT 0x01
	#define SED_DISPLAY_CSRDIR_RIGHT 0x00
	#define SED_DISPLAY_CSRDIR_UP 0x02
	#define SED_DISPLAY_CSRDIR_DOWN 0x03
#define SED_DISPLAY_HDOT_SCR 0x5A
#define SED_DISPLAY_OVLAY 0x5B
#define SED_DRAW_CSRW 0x46
#define SED_DRAW_CSRR 0x47
#define SED_MEM_MWRITE 0x42
#define SED_MEM_MREAD 0x43
/* ****************************************************************** */
	//#define SED_SAD1 0x1000
	#define SED_SAD1_SIZE 1200 //1000 // 0x04B0
	#define SED_SAD2_SIZE 9600 //8000 // 0x2580
	#define SED_SAD3_SIZE 1200 //1000
	//#define SED_SAD4_SIZE 9600
	// address start offset
	#define SED_SAD1 0
	#define SED_SAD2 0x0500
	//#define SED_SAD3 0x3000
	//#define SED_SAD4 0x3500
	#define SED_SL 159 // 159 might bee too; Screen block start address - numbers of lines per display
	
	#define SED_SAD1L (SED_SAD1&0xff)
	#define SED_SAD1H (SED_SAD1>>8)
	#define SED_SAD2L (SED_SAD2&0xff)
	#define SED_SAD2H (SED_SAD2>>8)
	#define SED_SAD3L (SED_SAD3&0xff)
	#define SED_SAD3H (SED_SAD3>>8)
/* ****************************************************************** */
#define SED_WIDTH 400
#define SED_HEIGHT 160
#define SED_XTAL 4000000

#define SED_TEXT1 1
#define SED_GRAPH1 2

void sed_data(uint8_t data);
void sed_command(uint8_t cmd);
uint8_t sed_read(void);
uint8_t sed_readBusy(void);

// -----------------------------------
void sed_load_CGRAM(void);
void sed_init(void);
void sed_onoff(uint8_t on);
void sed_clear(uint8_t sec, uint8_t fill);
//void sed_cursor(uint8_t sec, uint16_t pos);

#define SED_OVERLAY_MODE_OR 0x00
#define SED_OVERLAY_MODE_EXOR 0x01
#define SED_OVERLAY_MODE_AND 0x02
#define SED_OVERLAY_MODE_PRIORITY_OR 0x03
void sed_setOverlayMode(uint8_t ov_mode);


// new text functions
uint8_t sed_txt_char(uint8_t x, uint8_t y, char c);
uint8_t sed_txt_str(uint8_t x, uint8_t y, char* str);
uint8_t sed_txt_str_P(uint8_t x, uint8_t y, const char* str);
uint8_t sed_txt_dec(uint8_t x, uint8_t y, int val);
void sed_txt_setPos(uint8_t y, uint8_t x);
uint8_t sed_txt_hex_byte(uint8_t x, uint8_t y, uint8_t val);
/* ***** old text functions ******
void sed_txt_char(char c);
void sed_txt_str(char* str);
void sed_txt_str_P(prog_char* str);
void sed_txt_dec(int val);
void sed_txt_setPos(uint8_t y, uint8_t x);
void sed_txt_hex_byte(uint8_t val);
*/

void sed_cursor(uint16_t addr);

// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------

#define SED_GRAPHIC_FUNCTIONS_ENABLED 0

// #*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#

#define SED_PX_ON 1
#define SED_PX_OFF 0
#define SED_PX_XOR 2
void sed_setPix(uint16_t x, uint8_t y, uint8_t mode);

#if SED_GRAPHIC_FUNCTIONS_ENABLED == 1

#define SED_TEXT_PIX_ON 1 // only set pixels witch need to be on
#define SED_TEXT_PIX_OFF 0 // off pixeles witch need to be on
#define SED_TEXT_PIX_XOR 2 // xor pixels of whole char
#define SED_TEXT_PIX_ON_BG_OFF 3 // "black" char on "white" background
#define SED_TEXT_PIX_OFF_BG_ON 4 // "white" char on "black" background

#define SED_FUNCT_LINE
void sed_lineH(uint16_t x, uint8_t y, uint16_t w, uint8_t px_mode);
void sed_lineV(uint16_t x, uint8_t y, uint8_t h, uint8_t px_mode);
void sed_line(uint16_t x1, uint8_t y1, uint16_t x2, uint8_t y2, uint8_t px_mode);
#define SED_FUNCT_RECT
void sed_rect(uint16_t x1, uint8_t y1, uint16_t x2, uint8_t y2, uint8_t px_mode);
#define SED_FUNCT_RECT_FILL
void sed_rectFill(uint16_t x1, uint8_t y1, uint16_t x2, uint8_t y2, uint8_t px_mode);
#define SED_FUNCTS_STRINGS
uint8_t sed_char(uint16_t x, uint8_t y, char znak, uint8_t chr_mode);
uint8_t sed_str(uint16_t x, uint8_t y, char* str, uint8_t txt_mode);
uint8_t sed_str_P(uint16_t x, uint8_t y, char* str, uint8_t txt_mode);
uint8_t sed_dec(uint16_t x, uint8_t y, int val, uint8_t txt_mode);
#define SED_FUNCT_HEX_BYTE
uint8_t sed_hex_byte(uint16_t x, uint8_t y, uint8_t val, uint8_t txt_mode);
#define SED_FUNCT_LOAD
void sed_load(prog_uint8_t* pData, uint16_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t px_mode);
// If image is not loaded check image size and position agnist screen size!

#define XCHG(a, b, var_type) \
{\
	var_type temp##a##b; \
	temp##a##b = b; \
	b = a; \
	a = temp##a##b;	\
}

#endif // SED_GRAPHIC_FUNCTIONS_ENABLED

#endif // _SED1330LCD_HEADER_

