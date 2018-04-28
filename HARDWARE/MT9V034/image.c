
#include "image.h"

#define	IMG_HSIZE	752
#define	IMG_VSIZE	480

#define min(x,y) ((x)>(y)?(y):(x))
#define max(x,y) ((x)>(y)?(x):(y))
//-----------------------------------------------------------
//   �¼�����			raw2rgb
//-----------------------------------------------------------
	bool	h_flg;			//ˮƽ������
	bool	v_flg;			//��ֱ������
	bool	raw2rgb_flag;	//RAW to RGB888	
	bool	gray8_flag;		//8 Bit Gray����
	

//void bayer2rgb(unsigned char *dst, unsigned char *src, uint16_t DstWidth,uint16_t DstHeight ,uint16_t SrcWidth, uint16_t SrcHeight)
//{
//	int SrcX=0,SrcY=0;
//	int x,y;
//	int i,j;
//	x=(SrcX+0.5)*SrcWidth/DstWidth-0.5;
//	
//	y=(SrcY+0.5)*SrcHeight/DstHeight-0.5;
//}

	int yy,xx;
	int y = 0;
	int x = 0;
	unsigned char	red 	= 0,
								green	= 0,
								blue 	= 0;
		unsigned char	hg 	= 0 ,
								vg 	= 0 ,
								db1	= 0 ,
								db2 = 0 ;
	
void bayer2rgb24_bingo(unsigned char *dst, unsigned char *src, long width, long height)
{
	unsigned char	*bayer, 
								*image;

	unsigned char OddLINE_OddPOINT		=	2;	//00;	//odd lines + odd point
	unsigned char OddLINE_EvenPOINT		=	3;	//01;	//odd lines + even point
	unsigned char EvenLINE_OddPOINT		=	0;	//10;	//even lines + odd point
	unsigned char EvenLINE_EvenPOINT	=	1;	//11;	//even lines + even point

	bayer = src;					//raw
	image = dst;					//rgb24

	for( yy = 0; yy < height; yy++ )
	{ 
		//if(h_flg == false)//true)	//��֪��ô������ˣ����Է�����
		if(0)
		{
			y = height - yy - 1;	
		}
		else
		{
			y = yy;
		}

		for( xx = 0; xx < width; xx++ )
		{	
			//if(v_flg == true)
			if(0)
			{
				x = width - xx -1;	
			}
			else
			{
				x = xx;
			}

////////////////////////////////////////////////////////////////////////////////////////////
			if(y == 0 || y == height -1)	//��һ��,���һ��
			{
				red		=  bayer[x + y*width];
				green 	=  bayer[x + y*width];
				blue 	=  bayer[x + y*width];
			}
			else if((((y%2)<<1) + (x%2)) == OddLINE_OddPOINT)			//������ & ������ ,�����ݶ�  ????
			{
				hg 	= bayer[abs(x   + (y-1)*width)] 	- bayer[abs(x   + (y+1)*width)];
				vg 	= bayer[abs(x-1 +  y*width)] 			- bayer[abs(x+1 +  y*width)];
				db1 = bayer[abs(x-1 + (y-1)*width)] 	- bayer[abs(x+1 + (y+1)*width)];
				db2 = bayer[abs(x-1 + (y+1)*width)] 	- bayer[abs(x+1 + (y-1)*width)];

				blue = bayer[x  + y*width];

				if(hg < vg)
					green = (bayer[x + (y-1)*width] + bayer[x + (y+1)*width])/2;
				else if(hg > vg)
					green = (bayer[abs(x-1 + y*width)] + bayer[x+1 + y*width])/2;
				else
					green = (bayer[x + (y-1)*width] + bayer[x + (y+1)*width]
						   + bayer[abs(x-1 +  y*width)]+ bayer[x+1 + y*width] ) /4;

				if(db1 < db2)
					red = (bayer[abs(x-1 + 	(y-1)*width)] + bayer[x+1 + (y+1)*width])/2;
				else if (db1 > db2)
					red = (bayer[abs(x-1 + 	(y+1)*width)] + bayer[x+1 + (y-1)*width]) /2;
				else
					red = (bayer[abs(x-1 + 	(y-1)*width)] + bayer[x+1 + (y+1)*width] 
						 + bayer[x+1 + (y-1)*width] + bayer[x-1 + 	(y+1)*width])/4;					
			}
			else if((((y%2)<<1) + (x%2)) == EvenLINE_OddPOINT)			//ż���� & ������  ��ɫ����
			{
				red 	= (bayer[x-1 + y*width] + bayer[x+1 	+ (y)*width])/2;
				green 	=  bayer[x 	 + y*width];
				blue 	= (bayer[x 	 +  (y-1) *width] + bayer[x + 	(y+1)*width])/2;
			}
			else if((((y%2)<<1) + (x%2)) == EvenLINE_EvenPOINT)			//ż���� & ż����,�����ݶ�  ��ɫ����
			{
				hg 	= bayer[abs(x   + (y-1)*width)] 	- bayer[abs(x   + (y+1)*width)];
				vg 	= bayer[abs(x-1 + y*width)] 		- bayer[abs(x+1 + y*width)];

				db1 = bayer[abs(x-1 + (y-1)*width)] 	- bayer[abs(x+1 + (y+1)*width)];
				db2 = bayer[abs(x-1 + (y+1)*width)] 	- bayer[abs(x+1 + (y-1)*width)];

				red = bayer[x + y*width];

				if(hg < vg)
					green = (bayer[x + (y-1)*width] + bayer[x + (y+1)*width])/2;
				else if(hg > vg)
					green = (bayer[abs(x-1 + y*width)] + bayer[x+1 + y*width])/2;
				else
					green = (bayer[x + (y-1)*width] + bayer[x + (y+1)*width]
							+bayer[abs(x-1 +  y*width)]	+ bayer[x+1 + 	y*width]) /4 ;

				if(db1 < db2)
					blue = (bayer[abs(x-1 + (y-1)*width)] + bayer[x+1 + (y+1)*width])/2;
				else if (db1 > db2)
					blue = (bayer[abs(x-1 + (y+1)*width)] + bayer[x+1 + (y-1)*width]) /2;
				else
					blue = (bayer[abs(x-1 + (y-1)*width)] + bayer[x+1 + (y+1)*width] 
							+ bayer[x+1 + (y-1)*width] + bayer[x-1 + (y+1)*width])/4;					
			}
			else if((((y%2)<<1) + (x%2)) == OddLINE_EvenPOINT)			//������ & ż����,��ɫ����
			{
				red 	= (bayer[x	 + (y-1)*width] + bayer[x + (y+1)*width])/2;
				green 	=  bayer[x 	 +  y*width];
				blue 	= (bayer[abs(x-1 +  y *width)] + bayer[x+1 +  y*width])/2;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////
			image[0]	= min(max(blue,0),255);
			image[1]	= min(max(green,0),255);
			image[2]	= min(max(red,0),255);
			image += 3;
		}		
	}
}


void bayer2rgb(unsigned char *dst, unsigned char *src, long width, long height)
{
	unsigned char	*bayer, 
								*image;

	bayer = src;					//raw
	image = dst;					//rgb

	for( y = 0; y < height; y++ ) //����
	{ 
		for( x = 0; x < width; x++ )	//����
		{	
////////////////////////////////////////////////////////////////////////////////////////////
			if(x== 0 || x==width-1  || y==0 || y == height -1 ) //��һ�У���һ�У��߽�
			{
				red		=  bayer[x + y*width];
				green =  bayer[x + y*width];
				blue 	=  bayer[x + y*width];
			}
			else if((x%2==0) && (y%2==1))			//ż���� & ������  ��ɫ����
			{
				red 	= (bayer[x-1 + y*width] + bayer[x+1 	+ (y)*width])/2;
				green 	=  bayer[x 	 + y*width];
				blue 	= (bayer[x 	 +  (y-1) *width] + bayer[x + 	(y+1)*width])/2;
			}
			else if((x%2==1) && (y%2==0))			//������ & ż����,��ɫ����
			{
				red 	= (bayer[x	 + (y-1)*width] + bayer[x + (y+1)*width])/2;
				green 	=  bayer[x 	 +  y*width];
				blue 	= (bayer[(x-1 +  y *width)] + bayer[x+1 +  y*width])/2;
			}
			else 	//ż���� & ż����, ������ & ������,��ɫ����
			{
				red = bayer[x + y*width];
				green = (bayer[x + (y-1)*width] + bayer[x + (y+1)*width]+bayer[abs(x-1 +  y*width)]	+ bayer[x+1 + 	y*width]) /4 ;
				blue = (bayer[(x-1 + (y-1)*width)] + bayer[x+1 + (y+1)*width] + bayer[x+1 + (y-1)*width] + bayer[x-1 + (y+1)*width])/4;					
			}
			
			///////////////////////////////////////////////////////////////////////////////////////////////////////
			image[0]	= min(max(blue,0),255);
			image[1]	= min(max(green,0),255);
			image[2]	= min(max(red,0),255);
			image += 3;
		}		
	}
}