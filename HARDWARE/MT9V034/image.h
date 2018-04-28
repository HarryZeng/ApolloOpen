#ifndef IMAGE_H_
#define IMAGE_H_

#include <stdint.h>
#include "settings.h"
#include "stdbool.h"


typedef long BOOL;
typedef long LONG;
typedef unsigned char BYTE;
typedef unsigned long DWORD;
typedef unsigned short WORD;

#define WIDTH 320
#define HEIGHT 240
 
typedef struct tagBMPFILEHEADER {
        WORD    bfType;
        DWORD   bfSize;
        WORD    bfReserved1;
        WORD    bfReserved2;
        DWORD   bfOffBits;
} BMPFILEHEADER;

typedef struct tagBMPINFOHEADER {
        DWORD      biSize;
        LONG       biWidth;
        LONG       biHeight;
        WORD       biPlanes;
        WORD       biBitCount;
        DWORD      biCompression;
        DWORD      biSizeImage;
        LONG       biXPelsPerMeter;
        LONG       biYPelsPerMeter;
        DWORD      biClrUsed;
        DWORD      biClrImportant;
} BMPINFOHEADER;

typedef struct tagRGB{
		BYTE b;
        BYTE g;
        BYTE r;
} RGB;  // 定义位图数据

void bayer2rgb24_bingo(unsigned char *dst, unsigned char *src, long width, long height);
void BilinerInterpolation(BYTE raw_dat[HEIGHT][WIDTH],RGB p[HEIGHT][WIDTH]);
void bayer2rgb(unsigned char *dst, unsigned char *src, long width, long height);

#endif /* MT9V34_H_ */