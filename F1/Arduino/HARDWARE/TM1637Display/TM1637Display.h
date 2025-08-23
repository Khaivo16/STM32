//  Author: avishorp@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef __TM1637DISPLAY__
#define __TM1637DISPLAY__
#ifdef __cplusplus
 extern "C" {
#endif 
	 
#include <inttypes.h>
#include <sys.h>

#define SEG_A   B00000001
#define SEG_B   B00000010
#define SEG_C   B00000100
#define SEG_D   B00001000
#define SEG_E   B00010000
#define SEG_F   B00100000
#define SEG_G   B01000000
#define SEG_DP  B10000000

#define DEFAULT_BIT_DELAY  100
//////////////////////////////////
const uint8_t SEG_DONE[] = {
	SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_C | SEG_E | SEG_G,                           // n
	SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};
const int SIZE_DONE = sizeof(SEG_DONE)/sizeof(SEG_DONE[0]);
const uint8_t SEG_OUI[] = {
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,           // U
	SEG_F | SEG_E                                    // I
};
const int SIZE_OUI = sizeof(SEG_OUI)/sizeof(SEG_OUI[0]);
const uint8_t SEG_oui[] = {
	SEG_C | SEG_D | SEG_E | SEG_G,                   // o
	SEG_C | SEG_D | SEG_E,                           // u
	SEG_E                                            // i
};
const int SIZE_oui = sizeof(SEG_oui)/sizeof(SEG_oui[0]);
const uint8_t SEG_NON[] = {
	SEG_A | SEG_B | SEG_C | SEG_E | SEG_F,           // N
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_A | SEG_B | SEG_C | SEG_E | SEG_F            // N
};
const int SIZE_NON = sizeof(SEG_NON)/sizeof(SEG_NON[0]);
const uint8_t SEG_non[] = {
	SEG_C | SEG_E | SEG_G,                           // n
	SEG_C | SEG_D | SEG_E | SEG_G,                   // o
	SEG_C | SEG_E | SEG_G                            // n
};
const int SIZE_non = sizeof(SEG_non)/sizeof(SEG_non[0]);
const uint8_t SEG_Fin[] = {
	SEG_A | SEG_E | SEG_F | SEG_G,                   // F
	SEG_E,                                           // i
	SEG_C | SEG_E | SEG_G                            // n
};
const int SIZE_Fin = sizeof(SEG_Fin)/sizeof(SEG_Fin[0]);
const uint8_t SEG_End[] = {
	SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,           // E
	SEG_C | SEG_E | SEG_G,                           // n
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G            // d
};
const int SIZE_End = sizeof(SEG_End)/sizeof(SEG_End[0]);
const uint8_t SEG_Go[] = {
	SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,           // G
	SEG_C | SEG_D | SEG_E | SEG_G                    // o
};
const int SIZE_Go = sizeof(SEG_Go)/sizeof(SEG_Go[0]);
const uint8_t SEG_nul[] = {
	SEG_C | SEG_E | SEG_G,                           // n
	SEG_C | SEG_D | SEG_E,                           // u
    SEG_F | SEG_E                                    // l
};
const int SIZE_nul = sizeof(SEG_nul)/sizeof(SEG_nul[0]);
const uint8_t SEG_Err[] = {
	SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,           // E
	SEG_E | SEG_G,                                   // r
    SEG_E | SEG_G                                    // r
};
const int SIZE_Err = sizeof(SEG_Err)/sizeof(SEG_Err[0]);	 
	 

class TM1637Display {

public:
  //! Initialize a TM1637Display object, setting the clock and
  //! data pins.
  //!
  //! @param pinClk - The number of the digital pin connected to the clock pin of the module
  //! @param pinDIO - The number of the digital pin connected to the DIO pin of the module
  //! @param bitDelay - The delay, in microseconds, between bit transition on the serial
  //!                   bus connected to the display
  TM1637Display(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay = DEFAULT_BIT_DELAY);

  //! Sets the brightness of the display.
  //!
  //! The setting takes effect when a command is given to change the data being
  //! displayed.
  //!
  //! @param brightness A number from 0 (lowes brightness) to 7 (highest brightness)
  //! @param on Turn display on or off
  void setBrightness(uint8_t brightness, bool on = true);

  //! Display arbitrary data on the module
  //!
  //! This function receives raw segment values as input and displays them. The segment data
  //! is given as a byte array, each byte corresponding to a single digit. Within each byte,
  //! bit 0 is segment A, bit 1 is segment B etc.
  //! The function may either set the entire display or any desirable part on its own. The first
  //! digit is given by the @ref pos argument with 0 being the leftmost digit. The @ref length
  //! argument is the number of digits to be set. Other digits are not affected.
  //!
  //! @param segments An array of size @ref length containing the raw segment values
  //! @param length The number of digits to be modified
  //! @param pos The position from which to start the modification (0 - leftmost, 3 - rightmost)
  void setSegments(const uint8_t segments[], uint8_t length = 4, uint8_t pos = 0);

  //! Clear the display
  void clear();

  //! Display a decimal number
  //!
  //! Dispaly the given argument as a decimal number.
  //!
  //! @param num The number to be shown
  //! @param leading_zero When true, leading zeros are displayed. Otherwise unnecessary digits are
  //!        blank. NOTE: leading zero is not supported with negative numbers.
  //! @param length The number of digits to set. The user must ensure that the number to be shown
  //!        fits to the number of digits requested (for example, if two digits are to be displayed,
  //!        the number must be between 0 to 99)
  //! @param pos The position of the most significant digit (0 - leftmost, 3 - rightmost)
  void showNumberDec(int num, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);

  //! Display a decimal number, with dot control
  //!
  //! Dispaly the given argument as a decimal number. The dots between the digits (or colon)
  //! can be individually controlled.
  //!
  //! @param num The number to be shown
  //! @param dots Dot/Colon enable. The argument is a bitmask, with each bit corresponding to a dot
  //!        between the digits (or colon mark, as implemented by each module). i.e.
  //!        For displays with dots between each digit:
  //!        * 0.000 (B10000000)
  //!        * 00.00 (B01000000)
  //!        * 000.0 (B00100000)
  //!        * 0.0.0.0 (B11100000)
  //!        For displays with just a colon:
  //!        * 00:00 (B01000000)
  //!        For displays with dots and colons colon:
  //!        * 0.0:0.0 (B11100000)
  //! @param leading_zero When true, leading zeros are displayed. Otherwise unnecessary digits are
  //!        blank. NOTE: leading zero is not supported with negative numbers.
  //! @param length The number of digits to set. The user must ensure that the number to be shown
  //!        fits to the number of digits requested (for example, if two digits are to be displayed,
  //!        the number must be between 0 to 99)
  //! @param pos The position of the most significant digit (0 - leftmost, 3 - rightmost)
  void showNumberDecEx(int num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);

  //! Display a hexadecimal number, with dot control
  //!
  //! Dispaly the given argument as a hexadecimal number. The dots between the digits (or colon)
  //! can be individually controlled.
  //!
  //! @param num The number to be shown
  //! @param dots Dot/Colon enable. The argument is a bitmask, with each bit corresponding to a dot
  //!        between the digits (or colon mark, as implemented by each module). i.e.
  //!        For displays with dots between each digit:
  //!        * 0.000 (B10000000)
  //!        * 00.00 (B01000000)
  //!        * 000.0 (B00100000)
  //!        * 0.0.0.0 (B11100000)
  //!        For displays with just a colon:
  //!        * 00:00 (B01000000)
  //!        For displays with dots and colons colon:
  //!        * 0.0:0.0 (B11100000)
  //! @param leading_zero When true, leading zeros are displayed. Otherwise unnecessary digits are
  //!        blank
  //! @param length The number of digits to set. The user must ensure that the number to be shown
  //!        fits to the number of digits requested (for example, if two digits are to be displayed,
  //!        the number must be between 0 to 99)
  //! @param pos The position of the most significant digit (0 - leftmost, 3 - rightmost)
  void showNumberHexEx(uint16_t num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);

  //! Translate a single digit into 7 segment code
  //!
  //! The method accepts a number between 0 - 15 and converts it to the
  //! code required to display the number on a 7 segment display.
  //! Numbers between 10-15 are converted to hexadecimal digits (A-F)
  //!
  //! @param digit A number between 0 to 15
  //! @return A code representing the 7 segment image of the digit (LSB - segment A;
  //!         bit 6 - segment G; bit 7 - always zero)
  uint8_t encodeDigit(uint8_t digit);
	/* Définition des nombres en segments (pour utilisation des virgules) */
	uint8_t chiffre(char signe);
	
	/* Fonction qui affiche un mot prédéfini sur l'afficheur */
	void display_message(const uint8_t segments[], int taille);
	
	/* Fonction qui affiche le meme motif sur toutes les cases de l'afficheur */
	void display_all(uint8_t segment);
	
	/* Affiche un nombre avec partie décimale entre -999 et 9999.
	Par défaut, le nombre commence à gauche et toutes les cases sont remplies,
	y compris avec des zeros ajoutés à droite. Si ndecdigits (nombre de décimales
	imposées) est transmis (valeur entre 0 et 3), alors le bon nombre de décimales
	sont collées à droite et le reste du nombre suit avec des cases vides à gauche
	si besoin.
	Au delà de ]-1000;10000[, la notation scientifique sera utilisée, avec un
	seul chiffre significatif et une puissance de 10. Pour économiser l'affichage
	le signe "+" de la puissance ne sera pas affiché car, de toute façon, les
	nombre très près de 0 sont affichés en simple notation décimale.
	Si le nombre dépase la capacité d'affichage en notation scientifique,
	la fonction renvoie false, sinon elle renvoie true.
	*/
	bool display_number(double nombre, int ndecdigits = -1);

protected:
   void bitDelay();

   void start();

   void stop();

   bool writeByte(uint8_t b);

   void showDots(uint8_t dots, uint8_t* digits);
   
   void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);


private:
	uint8_t m_pinClk;
	uint8_t m_pinDIO;
	uint8_t m_brightness;
	unsigned int m_bitDelay;
};

#ifdef __cplusplus
}
#endif

#endif // __TM1637DISPLAY__

