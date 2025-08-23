#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <inttypes.h>
#include <sys.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0x04  // Enable bit
#define Rw 0x02  // Read/Write bit
#define Rs 0x01  // Register select bit

/**
 * This is the driver for the Liquid Crystal LCD displays that use the I2C bus.
 *
 * After creating an instance of this class, first call begin() before anything else.
 * The backlight is on by default, since that is the most likely operating mode in
 * most cases.
 */

	/**
	 * Constructor
	 *
	 * @param lcd_addr	I2C slave address of the LCD display. Most likely printed on the
	 *					LCD circuit board, or look in the supplied LCD documentation.
	 * @param lcd_cols	Number of columns your LCD display has.
	 * @param lcd_rows	Number of rows your LCD display has.
	 * @param charsize	The size in dots that the display has, use LCD_5x10DOTS or LCD_5x8DOTS.
	 */
	void LCD_LiquidCrystal_I2C(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows);

	/**
	 * Set the LCD display in the correct begin state, must be called before anything else is done.
	 */
	void LCD_begin(void);

	 /**
	  * Remove all the characters currently shown. Next print/write operation will start
	  * from the first position on LCD display.
	  */
	void LCD_clear(void);

	/**
	 * Next print/write operation will will start from the first position on the LCD display.
	 */
	void LCD_home(void);

	 /**
	  * Do not show any characters on the LCD display. Backlight state will remain unchanged.
	  * Also all characters written on the display will return, when the display in enabled again.
	  */
	void noDisplay(void);

	/**
	 * Show the characters on the LCD display, this is the normal behaviour. This method should
	 * only be used after noDisplay() has been used.
	 */
	void LCD_display(void);

	/**
	 * Do not blink the cursor indicator.
	 */
	void LCD_noBlink(void);

	/**
	 * Start blinking the cursor indicator.
	 */
	void LCD_blink(void);

	/**
	 * Do not show a cursor indicator.
	 */
	void LCD_noCursor(void);

	/**
 	 * Show a cursor indicator, cursor can blink on not blink. Use the
	 * methods blink() and noBlink() for changing cursor blink.
	 */
	void LCD_cursor(void);

	void LCD_scrollDisplayLeft(void);
	void LCD_scrollDisplayRight(void);
	void LCD_printLeft(void);
	void LCD_printRight(void);
	void LCD_leftToRight(void);
	void LCD_rightToLeft(void);
	void LCD_shiftIncrement(void);
	void LCD_shiftDecrement(void);
	void LCD_noBacklight(void);
	void LCD_backlight(void);
	u8 LCD_getBacklight(void);
	void LCD_autoscroll(void);
	void LCD_noAutoscroll(void);
	void LCD_createChar(uint8_t, uint8_t[]);
	void LCD_setCursor(uint8_t, uint8_t);
	void LCD_command(uint8_t);

//	inline void blink_on() { LCD_blink(); }
//	inline void blink_off() { LCD_noBlink(); }
//	inline void cursor_on() { LCD_cursor(); }
//	inline void cursor_off() { LCD_noCursor(); }

// Compatibility API function aliases
	void LCD_setBacklight(uint8_t new_val);				// alias for backlight() and nobacklight()
	void LCD_load_custom_character(uint8_t char_num, uint8_t *rows);	// alias for createChar()
	void LCD_printstr(const char *str);

	void LCD_write(uint8_t value);
	void LCD_send(uint8_t, uint8_t);
	void LCD_write4bits(uint8_t);
	void LCD_expanderWrite(uint8_t);
	void LCD_pulseEnable(uint8_t);
	


#endif // LCD_I2C_H


