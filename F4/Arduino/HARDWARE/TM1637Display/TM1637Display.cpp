
extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include <TM1637Display.h>
//#include <Arduino.h>
#include "IString.h"

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  B00111111,    // 0
  B00000110,    // 1
  B01011011,    // 2
  B01001111,    // 3
  B01100110,    // 4
  B01101101,    // 5
  B01111101,    // 6
  B00000111,    // 7
  B01111111,    // 8
  B01101111,    // 9
  B01110111,    // A
  B01111100,    // b
  B00111001,    // C
  B01011110,    // d
  B01111001,    // E
  B01110001     // F
  };

static const uint8_t minusSegments = B01000000;

TM1637Display::TM1637Display(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	m_bitDelay = bitDelay;

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void TM1637Display::setBrightness(uint8_t brightness, bool on)
{
	m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
}

void TM1637Display::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	  writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}

void TM1637Display::clear()
{
    uint8_t data[] = { 0, 0, 0, 0 };
	setSegments(data);
}

void TM1637Display::showNumberDec(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
  showNumberDecEx(num, 0, leading_zero, length, pos);
}

void TM1637Display::showNumberDecEx(int num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
}

void TM1637Display::showNumberHexEx(uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void TM1637Display::showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
    bool negative = false;
	if (base < 0) {
	    base = -base;
		negative = true;
	}


    uint8_t digits[4];

	if (num == 0 && !leading_zero) {
		// Singular case - take care separately
		for(uint8_t i = 0; i < (length-1); i++)
			digits[i] = 0;
		digits[length-1] = encodeDigit(0);
	}
	else {
		//uint8_t i = length-1;
		//if (negative) {
		//	// Negative number, show the minus sign
		//    digits[i] = minusSegments;
		//	i--;
		//}
		
		for(int i = length-1; i >= 0; --i)
		{
		    uint8_t digit = num % base;
			
			if (digit == 0 && num == 0 && leading_zero == false)
			    // Leading zero is blank
				digits[i] = 0;
			else
			    digits[i] = encodeDigit(digit);
				
			if (digit == 0 && num == 0 && negative) {
			    digits[i] = minusSegments;
				negative = false;
			}

			num /= base;
		}

		if(dots != 0)
		{
			showDots(dots, digits);
		}
    }
    setSegments(digits, length, pos);
}

void TM1637Display::bitDelay()
{
	delayMicroseconds(m_bitDelay);
}

void TM1637Display::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}

void TM1637Display::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}

bool TM1637Display::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();

  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);


  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();

  return ack;
}

void TM1637Display::showDots(uint8_t dots, uint8_t* digits)
{
    for(int i = 0; i < 4; ++i)
    {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t TM1637Display::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}
//////////////////////////////////////////////////////////////////

/* Définition des nombres en segments (pour utilisation des virgules) */
uint8_t TM1637Display::chiffre(char signe) {
  switch(signe) {
    case '0' : 
      return B00111111;
      //break;
    case '1' :
      return B00000110;
      //break;
    case '2' :
      return B01011011;
      //break;
    case '3' :
      return B01001111;
      //break;
    case '4' :
      return B01100110;
      //break;
    case '5' :
      return B01101101;
      //break;
    case '6' :
      return B01111101;
      //break;
    case '7' :
      return B00000111;
      //break;
    case '8' :
      return B01111111;
      //break;
    case '9' :
      return B01101111;
      //break;
    case 'p' :
      return B10000000; // point pour case en cours
      //break;
    case '-' :
      return B01000000;
    case 'e' :
      return B01111001;
    case '_' :
      return 0;
      //break;
    default : return 0;
  } 
}

/* Fonction qui affiche un mot prédéfini sur l'afficheur */
void TM1637Display::display_message(const uint8_t segments[], int taille) {
  uint8_t data[] = { 0, 0, 0, 0 };
  for (int i=0; i<taille; i++) {
    data[i] = segments[i];
  }
  this->setSegments(data);
}

/* Fonction qui affiche le meme motif sur toutes les cases de l'afficheur */
void TM1637Display::display_all(uint8_t segment) {
  uint8_t data[] = { segment, segment, segment, segment };
  this->setSegments(data);
}

/* Affiche un nombre avec partie décimale entre -999 et 9999.
Par défaut, le nombre commence à gauche et toutes les cases sont remplies,
y compris avec des zeros ajoutés à droite. Si ndecdigits (nombre de décimales
imposées) est transmis (valeur entre 0 et 3), alors le bon nombre de décimales
sont collées à droite et le reste du nombre suit avec des cases vides à gauche
si besoin.
*/
//int int_pow(int base, int exp)//dung ok
//{
//    int result = 1;
//    while (exp)
//    {
//        if (exp % 2)
//           result *= base;
//        exp /= 2;
//        base *= base;
//    }
//    return result;
//}
bool TM1637Display::display_number(double nombre, int ndecdigits) {
	double nent, ndec;
	char entierc[4];
	char decimalc[5];
	bool neg = false;
	char signe;
	int compteur, pospoint;
	const int lim = 4; //le nombre de cases disponibles sur l'afficheur

	// pour les grands nombres
	if (nombre <= -pow(10.0,lim - 1) || nombre >= pow(10.0,lim)) {
		//représenter nombre avec notation scientifique (presque)
		int ordre = log10(abs(nombre));
		if ((ordre > 99 && nombre > 0) || (ordre > 9 && nombre < 0)) {
			//là on peut plus rien c'est trop grand !
			this->display_message(SEG_Err, SIZE_Err);
			return false;
		}
		int premierChiffre = abs(nombre) / pow(10.0, ordre);
		
		String nombre_str = "";
		if (nombre < 0) {
			nombre_str += '-';
		}
		char premierChiffre_str [2];
		itoa (premierChiffre,premierChiffre_str,10);
		nombre_str += String(premierChiffre_str);
		nombre_str += String('e');
		char ordre_str [3];
		itoa (ordre,ordre_str,10);
		nombre_str += String(ordre_str);
		
		uint8_t monchiffre[lim];
		for (int i = 0; i < lim; i++) {
			monchiffre[i] = this->chiffre('_');
		}
		for (int i = 0; i<nombre_str.length(); i++) {
			signe = nombre_str.charAt(i);
			monchiffre[i] = this->chiffre(signe);
		}
		
		this->setSegments(monchiffre);
		
		return true;
	}

	if (nombre < 0) {
		neg = true;
		nombre = -nombre;
	}

	uint8_t monchiffre[lim];
	for (int i = 0; i < lim; i++) {
		monchiffre[i] = this->chiffre('0');
	}
	//uint8_t monchiffre[] = { chiffre('0'), chiffre('0'), chiffre('0'), chiffre('0') };

	ndec = modf (nombre , &nent);
	ndec = ndec * pow (10.0,lim-1); // precision à 3 chiffres apès la virgule
	dtostrf(nent, lim, 0, entierc);
	dtostrf(ndec, lim, 0, decimalc);
	String entier = String(entierc);
	String decimal = String(decimalc);

	pospoint = lim;
	if (neg) {
		monchiffre[0] = this->chiffre('-');
		compteur = 1;
	}
	else {
		compteur = 0;
	}
	for (int i = 0; i<lim; i++) {
		signe = entier.charAt(i);
		if (signe != ' ') {
			monchiffre[compteur] = this->chiffre(signe);
			compteur++;
		}
	}
	pospoint = compteur;
	for (int i = 0; i<lim; i++) {
		signe = decimal.charAt(i);
		if (signe != ' ' && compteur < lim) {
			monchiffre[compteur] = this->chiffre(signe);
			compteur++;
		}
	}
	if (pospoint < lim) {
		pospoint--;
		monchiffre[pospoint] = monchiffre[pospoint] | this->chiffre('p');
	}
	if (ndecdigits > -1) {
		int decal = lim - pospoint - 1; //chiffres après la virgule avant traitement
		if (decal > ndecdigits) {
			//il faut enlever des décimales - on ne peut en ajouter puisque les zero à droite sont ajoutés par défaut
			for (int i = 0; i < decal - ndecdigits; i++) {
				for (int k = lim - 1; k >= 1; k--) { // décalage vers la droite de 1 case
					monchiffre[k] = monchiffre[k - 1];
				}
				monchiffre[0] = this->chiffre('_');
			}   
		} 
	}
	if (ndecdigits == 0 && pospoint < lim) {
		monchiffre[lim - 1] = monchiffre[lim - 1] ^ this->chiffre('p'); //enlève le point de la dernière case s'il y est
	}

	this->setSegments(monchiffre);
	return true;
}

