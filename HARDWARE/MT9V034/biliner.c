//generate BMP file ,extract a raw rgb format ,then using biliner interpolation to creat three arrays standing for RGB and generate a new BMP file.
//

#include "stdlib.h"
#include "string.h"
#include "image.h"


void GenRGBimage(RGB p[HEIGHT][WIDTH])
{ BYTE i,j;
  for(i=0;i<HEIGHT;i++ ){
	  for(j=0;j<WIDTH;j++)
	  
			  {  if(j<50)
	           {  p[i][j].r =200;
			     p[i][j].g =0;
			  	 p[i][j].b =0;			  		 
			   }	
			  else if(j<100)
			  { p[i][j].r =0;
			     p[i][j].g =200;
			  	 p[i][j].b =0;
	         }
			  else if(j<150)
			    {p[i][j].r =0;
			     p[i][j].g =0;
			  	 p[i][j].b =200;}
			  else{
			     p[i][j].r =200;
			     p[i][j].g =200;
			  	 p[i][j].b =0;}
          
	   }     
}
}
/*
void GenRaw( BYTE raw[HEIGHT][WIDTH],RGB p[HEIGHT][WIDTH])
{
BYTE	 ibit;
BYTE     jbit;
BYTE     ijbit;
BYTE     i,j;
 
  for( i=0;i<HEIGHT;i++ ){
	 for( j=0;j<WIDTH;j++){
	       ibit=i%2;
		   jbit=j%2;
	        ijbit=(1-ibit)*2+jbit;
		   switch(ijbit)
		   {  case 0:
		          raw[i][j]=p[i][j].r;
				   break;
			   case 1:
				   raw[i][j]=p[i][j].g;
				   break;
			   case 2:
				    raw[i][j]=p[i][j].g;
				   break;
               case 3:
				   raw[i][j]=p[i][j].b;
				   break;
			   default:
				   raw[i][j]=0;   
	       }
		  //   if(j==WIDTH-1)
		//  printf("%d\n",raw[i][j]);
		//  else
		//   printf("%d\\",raw[i][j]);
        }
      }
}
*/
void GenRaw( BYTE raw[HEIGHT][WIDTH],RGB p[HEIGHT][WIDTH])
{
BYTE	 ibit;
BYTE     jbit;
BYTE     ijbit;
BYTE     i,j;
 
  for( i=0;i<HEIGHT;i++ ){
	 for( j=0;j<WIDTH;j++){
	       ibit=i%2;
		   jbit=j%2;
	        ijbit=(1-ibit)*2+jbit;
		   switch(ijbit)
		   {  case 0:
		          raw[i][j]=p[i][j].b;
				   break;
			   case 1:
				   raw[i][j]=p[i][j].g;
				   break;
			   case 2:
				    raw[i][j]=p[i][j].g;
				   break;
               case 3:
				   raw[i][j]=p[i][j].r;
				   break;
			   default:
				   raw[i][j]=0;   
	       }
	
        }
      }
}



void BilinerInterpolation(BYTE raw_dat[HEIGHT][WIDTH],RGB p[HEIGHT][WIDTH])
{
	BYTE	 	 ibit;
	BYTE     jbit;
	BYTE     ijbit;
	BYTE     i,j;
	
 for( i=0;i<HEIGHT;i++ ){
  for( j=0;j<WIDTH;j++){
	 ibit=i%2;
    jbit=j%2;
    ijbit=(1-ibit)*2+jbit;
     if((i==0)||(i==HEIGHT-1)||(j==0)||(j==WIDTH-1))
		{;}
	 else{
       switch(ijbit)
       {
				case 3:
         {  p[i][j].r=(raw_dat[i-1][j-1]+raw_dat[i-1][j+1]+raw_dat[i+1][j-1]+raw_dat[i+1][j+1])/4;
            p[i][j].g=(raw_dat[i-1][j]+raw_dat[i][j-1]+raw_dat[i][j+1]+raw_dat[i+1][j])/4;
            p[i][j].b=raw_dat[i][j];
					} 
					break;
				case 1:
					{
							p[i][j].r=(raw_dat[i][j-1]+raw_dat[i][j+1])/2;
							p[i][j].g=raw_dat[i][j];
							p[i][j].b=(raw_dat[i-1][j]+raw_dat[i+1][j])/2;
					}
					break;	       
				case 2:
					{
						 p[i][j].r=(raw_dat[i-1][j]+raw_dat[i+1][j])/2;
						 p[i][j].g=raw_dat[i][j];
						 p[i][j].b=(raw_dat[i][j-1]+raw_dat[i][j+1])/2;
					}       
					break;
				case 0:
					{
						p[i][j].r=raw_dat[i][j];
						p[i][j].g=(raw_dat[i-1][j]+raw_dat[i][j-1]+raw_dat[i][j+1]+raw_dat[i+1][j])/4;
						p[i][j].b=(raw_dat[i-1][j-1]+raw_dat[i-1][j+1]+raw_dat[i+1][j-1]+raw_dat[i+1][j+1])/4;
					}
					break;
				default: 
					{  
						p[i][j].r=100;
						p[i][j].g=0;
						p[i][j].b=0;
					}
        }
	 }	    		 
  }
}  
}
void BoundaryProcess(RGB p[HEIGHT][WIDTH])
{BYTE     i,j;
  for( i=0;i<HEIGHT;i++ ){
  for( j=0;j<WIDTH;j++){
	 if(j==0)
      {p[i][j].r=p[i][j+1].r;
	   p[i][j].g=p[i][j+1].g;
	   p[i][j].b=p[i][j+1].b;}
	  if(j==WIDTH-1)
      {p[i][j].r=p[i][j-1].r;
	   p[i][j].g=p[i][j-1].g;
	   p[i][j].b=p[i][j-1].b;}
	  if(i==0)
	  {p[i][j].r=p[i+1][j].r;
	   p[i][j].g=p[i+1][j].g;
	   p[i][j].b=p[i+1][j].b;
	  }
	 if(i==HEIGHT-1)
	  {p[i][j].r=p[i-1][j].r;
	   p[i][j].g=p[i-1][j].g;
	   p[i][j].b=p[i-1][j].b;}
  }  
  }
}
void biliner_main(vid) 
{
//    
//	   RGB         pRGB[HEIGHT][WIDTH];
//	   RGB   BilinerRGB[HEIGHT][WIDTH];
//       BYTE  raw_dat[HEIGHT][WIDTH];
//	   BYTE  r_dat[HEIGHT][WIDTH];
//	   BYTE  g_dat[HEIGHT][WIDTH];
//	   BYTE  b_dat[HEIGHT][WIDTH];
//      BYTE     i,j;
//       FILE *fp,*fpr,*fpg,*fpb;
//      loadBMP(pRGB);
//      //load image
//     // GenRGBimage(pRGB);
//       //genrate image
//   //  Snapshot( ( BYTE*)pRGB, WIDTH,HEIGHT, "E:\\RAW2RGB\\image_pattern\\lenna.bmp" );

//	 GenRaw(raw_dat,pRGB);
//     fp=fopen("E:\\RAW2RGB\\simulation\\test.txt","wb+");
//	 fwrite(raw_dat,sizeof(raw_dat),1,fp);
//   // BilinerInterpolation(raw_dat,BilinerRGB);
//    fpr=fopen("E:\\RAW2RGB\\simulation\\pattern_r.txt","rb");
//	 fpg=fopen("E:\\RAW2RGB\\simulation\\pattern_g.txt","rb");
//	   fpb=fopen("E:\\RAW2RGB\\simulation\\pattern_b.txt","rb");
//	   fread(r_dat,sizeof(r_dat),1,fpr);
//       fread(g_dat,sizeof(g_dat),1,fpg);
//	   fread(b_dat,sizeof(b_dat),1,fpb);
////for( i=0;i<HEIGHT;i++ ){
////  for( j=0;j<WIDTH;j++){
////  r_dat[i][j]=200;
////  g_dat[i][j]=0;
////  b_dat[i][j]=0;
////  }
////}



//  for( i=0;i<HEIGHT;i++ ){
//  for( j=0;j<WIDTH;j++){
//	 BilinerRGB[i][j].r=r_dat[i][j];
//	 BilinerRGB[i][j].g=g_dat[i][j];
//	 BilinerRGB[i][j].b=b_dat[i][j];
//  }
//  }
//    
//	//BoundaryProcess(BilinerRGB);
//  
// //  Snapshot( ( BYTE*)BilinerRGB, WIDTH,HEIGHT, "E:\\RAW2RGB\\simulation\\pattern1.bmp" );
// //  getchar();
}