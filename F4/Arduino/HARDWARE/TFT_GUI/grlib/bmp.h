#ifndef _BMP_H
#define _BMP_H

#include "ff.h"

typedef enum {
	BR_OK = 0,
	BR_EOF,
	BR_HEADER_ERR,
	BR_DIB_ERR,
	BR_PAR_ERR
} BRESULT;

typedef enum {
	BITMAPINFOHEADER
} BHEADERTYPE;

BRESULT openBMP(FIL *fil,unsigned int x, unsigned int y) ;

#endif
