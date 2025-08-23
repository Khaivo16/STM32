#include "LCD_I2C.h"
#include <inttypes.h>
#include <sys.h>
#include <myiic.h>

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set:
//    DL = 1; 8-bit interface data
//    N = 0; 1-line display
//    F = 0; 5x8 dot character font
// 3. Display on/off control:
//    D = 0; Display off
//    C = 0; Cursor off
//    B = 0; Blinking off
// 4. Entry mode set:
//    I/D = 1; Increment by 1
//    S = 0; No shift
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).


	uint8_t _addr;
	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;
	uint8_t _cols;
	uint8_t _rows;
	uint8_t _charsize;
	uint8_t _backlightval;
	
void LCD_LiquidCrystal_I2C(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
	_addr = lcd_addr;
	_cols = lcd_cols;
	_rows = lcd_rows;
	//_charsize = charsize;
	_backlightval = LCD_NOBACKLIGHT;
	LCD_begin();
}

void LCD_begin() {
	//LCD_begin();
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

	if (_rows > 1) {
		_displayfunction |= LCD_2LINE;
	}
	
	// for some 1 line displays you can select a 10 pixel high font
	if ((_charsize != 0) && (_rows == 1)) {
		_displayfunction |= LCD_5x10DOTS; 
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
	delay_ms(50);

	// Now we pull both RS and R/W low to begin commands
	LCD_expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	delay_ms(1000);

	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	// we start in 8bit mode, try to set 4 bit mode
	LCD_write4bits(0x03 << 4);
	delay_us(4500); // wait min 4.1ms

	// second try
	LCD_write4bits(0x03 << 4);
	delay_us(4500); // wait min 4.1ms

	// third go!
	LCD_write4bits(0x03 << 4);
	delay_us(150);

	// finally, set to 4-bit interface
	LCD_write4bits(0x02 << 4);

	// set # lines, font size, etc.
	LCD_command(LCD_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	LCD_display();

	// clear it off
	LCD_clear();

	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	LCD_command(LCD_ENTRYMODESET | _displaymode);

	LCD_home();
}

/********** high level commands, for the user! */
void LCD_clear(){
	LCD_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delay_us(2000);  // this command takes a long time!
}

void LCD_home(){
	LCD_command(LCD_RETURNHOME);  // set cursor position to zero
	delay_us(2000);  // this command takes a long time!
}

void LCD_setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > _rows) {
		row = _rows-1;    // we count rows starting w/0
	}
	LCD_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LCD_noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LCD_display() {
	_displaycontrol |= LCD_DISPLAYON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LCD_noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LCD_cursor() {
	_displaycontrol |= LCD_CURSORON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LCD_noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LCD_blink() {
	_displaycontrol |= LCD_BLINKON;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LCD_scrollDisplayLeft(void) {
	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LCD_scrollDisplayRight(void) {
	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LCD_leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	LCD_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LCD_rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	LCD_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LCD_autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	LCD_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LCD_noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	LCD_command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LCD_createChar(uint8_t location, uint8_t charmap[]) {
	int i=0;
	location &= 0x7; // we only have 8 locations 0-7
	LCD_command(LCD_SETCGRAMADDR | (location << 3));
	for (i=0; i<8; i++) {
		LCD_write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void LCD_noBacklight(void) {
	_backlightval=LCD_NOBACKLIGHT;
	LCD_expanderWrite(0);
}

void LCD_backlight(void) {
	_backlightval=LCD_BACKLIGHT;
	LCD_expanderWrite(0);
}
u8 LCD_getBacklight() {
  return _backlightval == LCD_BACKLIGHT;
}


/*********** mid level commands, for sending data/cmds */

void LCD_command(uint8_t value) {
	LCD_send(value, 0);
}

//size_t LCD_write(uint8_t value) {
//	LCD_send(value, Rs);
//	return 1;
//}

void LCD_write(uint8_t value){
	LCD_send(value, Rs);
}


/************ low level data pushing commands **********/

// write either command or data
void LCD_send(uint8_t value, uint8_t mode) {
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value<<4)&0xf0;
	LCD_write4bits((highnib)|mode);
	LCD_write4bits((lownib)|mode);
}

void LCD_write4bits(uint8_t value) {
	LCD_expanderWrite(value);
	LCD_pulseEnable(value);
}

void LCD_expanderWrite(uint8_t _data){
//	Wire.beginTransmission(_addr);
//	Wire.write((int)(_data) | _backlightval);
//	Wire.endTransmission();
	I2C_Write(I2C1, _addr,0x00, (int)(_data) | _backlightval);
}

void LCD_pulseEnable(uint8_t _data){
	LCD_expanderWrite(_data | En);	// En high
	delay_us(1);		// enable pulse must be >450ns

	LCD_expanderWrite(_data & ~En);	// En low
	delay_us(50);		// commands need > 37us to settle
}

void LCD_load_custom_character(uint8_t char_num, uint8_t *rows){
	LCD_createChar(char_num, rows);
}

void LCD_setBacklight(uint8_t new_val){
	if (new_val) {
		LCD_backlight();		// turn backlight on
	} else {
		LCD_noBacklight();		// turn backlight off
	}
}

void LCD_printstr(const char *str){
	//This function is not identical to the function used for "real" I2C displays
	//it's here so the user sketch doesn't have to be changed
	uint8_t i=0;
  while(str[i])
  {
    LCD_write(str[i]);
    i++;
  }
}


