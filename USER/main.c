#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "ltdc.h"
#include "sdram.h"
#include "w25qxx.h"
#include "nand.h"  
#include "mpu.h"
#include "ov5640.h"
#include "pcf8574.h"
#include "dcmi.h"
#include "sdmmc_sdcard.h"
#include "usmart.h"
#include "malloc.h"
#include "ff.h"
#include "exfuns.h"
#include "fontupd.h"
#include "text.h"
#include "piclib.h"	
#include "string.h"		
#include "math.h"
#include "mt9v034.h"
#include "image.h"
/************************************************
 ALIENTEK ������STM32F7������ ʵ��46
 �����ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/
#define bufsize  752*480
//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.
u32 startX,startY,endX,endY,runX,runY;
extern uint32_t MT9V034_Width,MT9V034_height;

#define jpeg_buf_size   8*1024*1024		//����JPEG���ݻ���jpeg_buf�Ĵ�С(8M�ֽ�)
#define jpeg_line_size	2*1024			//����DMA��������ʱ,һ�����ݵ����ֵ

u32 *dcmi_line_buf[2];					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л���  
u8 *jpeg_data_buf;						//JPEG���ݻ���buf 

extern uint16_t version ;
extern uint8_t Frame_Flag;

u32 startX,startY,endX,endY,xlength,ydispaly,xdisplay;

//RGB rgb_data[HEIGHT][WIDTH];
//BYTE raw_data[HEIGHT][WIDTH];
u32 display_i;

void raw_data_process(void)
{
	u16 rlen;			//ʣ�����ݳ���
	u32 *pbuf;
	u32 y;
	
	startX = 0;
	startY = 0;
	
	endX = MT9V034_Width;
	endY = MT9V034_height;

	DCMI_Stop(); 			//ֹͣDMA����
	
	//bayer2rgb24_bingo(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	//bayer2rgb(&dcmi_line_buf[0][0],jpeg_data_buf,MT9V034_Width,MT9V034_height);
	//BilinerInterpolation(raw_data,rgb_data);
	
//	display_i = 0;
//	xlength=endX-startX+1;	
//	
//	for(xdisplay=startY;xdisplay<=endY;xdisplay++)
//	{
//		LCD_SetCursor(startX,xdisplay);      				//���ù��λ�� 
//		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	  
//		for(ydispaly=0;ydispaly<xlength;ydispaly++)
//			LCD->LCD_RAM = jpeg_data_buf[display_i++];	//��ʾ��ɫ 	 
//		//	for(j=0;j<xlength;j++)LCD->LCD_RAM=RED;	//��ʾ��ɫ 
//	}
	LCD_SetCursor(0,0);      				//���ù��λ�� 
	LCD_WriteRAM_Prepare();     			//��ʼд��GRAM
	
	for(display_i=0;display_i<MT9V034_Width*MT9V034_height;display_i++)
	{
		//LCD->LCD_RAM=jpeg_data_buf[display_i];	//��ʾ��ɫ 	
			//LCD->LCD_RAM=display_i%255;	//��ʾ��ɫ 	
			//LCD->LCD_RAM=((display_i%255)&0x3f)<<5;
			//LCD->LCD_RAM=(display_i&0x3f)<<5;
			//LCD->LCD_RAM=((display_i&0x1f)<<0)|((display_i&0x3f)<<5)|((display_i&0x3f)<<11);
			//LCD->LCD_RAM=((display_i&0x1f)<<0)|((display_i&0x3f)<<5)|((display_i&0x1f)<<11);
				//LCD->LCD_RAM=((display_i&0x1f)<<0);
				//LCD->LCD_RAM=((display_i&0x3f)<<5);
				//LCD->LCD_RAM=((display_i&0x1f)<<11);
			//LCD->LCD_RAM=((display_i&0x1f)<<0)|((display_i&0x3f)<<5)|((display_i&0x1f)<<11);
			//LCD->LCD_RAM=(((display_i>>3)&0x1f)<<0)|(((display_i>>2)&0x3f)<<5)|(((display_i>>3)&0x1f)<<11);
			//y=640*480;
			LCD->LCD_RAM=(((jpeg_data_buf[display_i]>>3)&0x1f)<<0)|(((jpeg_data_buf[display_i]>>2)&0x3f)<<5)|(((jpeg_data_buf[display_i]>>3)&0x1f)<<11);
	}

//for(display_i=0;display_i<MT9V034_Width*MT9V034_height;display_i++)
//	{
//		LCD->LCD_RAM=display_i%255;	//��ʾ��ɫ 	 
//	}

	DCMI_Start(); 			//�������� 
		
	Frame_Flag=1;
}


//�л�ΪOV5640ģʽ
void sw_ov5640_mode(void)
{ 
	GPIO_InitTypeDef GPIO_Initure;
 	OV5640_WR_Reg(0X3017,0XFF);	//����OV5650���(����������ʾ)
	OV5640_WR_Reg(0X3018,0XFF); 

	//GPIOC8/9/11�л�Ϊ DCMI�ӿ�
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
    GPIO_Initure.Alternate=GPIO_AF13_DCMI;      //����ΪDCMI   
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //��ʼ��   
} 


int main(void)
{
	u32 k=0;
	u8 res,fac;							 
	u8 *pname;					//��·�����ļ��� 
	u8 key;						//��ֵ		   
	u8 i;						 
	u8 sd_ok=1;					//0,sd��������;1,SD������. 
 	u8 scale=1;					//Ĭ����ȫ�ߴ�����
	u8 msgbuf[15];				//��Ϣ������ 
	u16 outputheight=0;
	
    Cache_Enable();                 //��L1-Cache
    MPU_Memory_Protection();        //������ش洢����
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
	uart_init(115200);		        //���ڳ�ʼ��
	usmart_dev.init(108); 		    //��ʼ��USMART
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������
    SDRAM_Init();                   //��ʼ��SDRAM
    LCD_Init();                     //��ʼ��LCD
		
	PCF8574_Init();                 //��ʼ��PCF8574
    //OV5640_Init();				    //��ʼ��OV5640
		
	//sw_sdcard_mode();				//�����л�ΪOV5640ģʽ
	W25QXX_Init();					//��ʼ��W25Q256
 	my_mem_init(SRAMIN);			//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);			//��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMDTCM);			//��ʼ��TCM�ڴ�� 
	exfuns_init();					//Ϊfatfs��ر��������ڴ�  
 	f_mount(fs[0],"0:",1); 			//����SD��  
	POINT_COLOR=RED;      	
	while(font_init()) 				//����ֿ�
	{
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}
 	Show_Str(30,50,200,16,"������STM32F4/F7������",16,0);	 			    	 
	Show_Str(30,70,200,16,"�����ʵ��",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:����(bmp��ʽ)",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:����(jpg��ʽ)",16,0);			    	 
	Show_Str(30,130,200,16,"KEY2:�Զ��Խ�",16,0);					    	 
	Show_Str(30,150,200,16,"WK_UP:FullSize/Scale",16,0);				    	 
	Show_Str(30,170,200,16,"2016��7��31��",16,0);
	res=f_mkdir("0:/PHOTO");		//����PHOTO�ļ���
	if(res!=FR_EXIST&&res!=FR_OK) 	//�����˴���
	{
		res=f_mkdir("0:/PHOTO");		//����PHOTO�ļ���		
		Show_Str(30,190,240,16,"SD������!",16,0);
		delay_ms(200);				  
		Show_Str(30,190,240,16,"���չ��ܽ�������!",16,0);
		delay_ms(200);				  
		sd_ok=0;  	
	} 	
	dcmi_line_buf[0]=mymalloc(SRAMIN,jpeg_line_size*4);	//Ϊjpeg dma���������ڴ�	
	dcmi_line_buf[1]=mymalloc(SRAMIN,jpeg_line_size*40);	//Ϊjpeg dma���������ڴ�	
	jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//Ϊjpeg�ļ������ڴ�(���4MB)
 	pname=mymalloc(SRAMIN,30);//Ϊ��·�����ļ�������30���ֽڵ��ڴ�	 
 	while(pname==NULL||!dcmi_line_buf[0]||!dcmi_line_buf[1]||!jpeg_data_buf)	//�ڴ�������
 	{	    
		Show_Str(30,190,240,16,"�ڴ����ʧ��!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}   	

 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(30,230,version,5,16,1);
	LCD_Set_Window(0,0,MT9V034_Width,MT9V034_height);
	DCMI_Init();			//DCMI����

	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA����,MCU��,����
	DCMI_DMA_Init((u32)jpeg_data_buf,0,MT9V034_Width*MT9V034_height,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);			//DCMI DMA����,MCU��,����
	DCMI_Start(); 			//��������

	while(1)
	{
		if(i==20)//DS0��˸.
		{
			i=0;
			LED0_Toggle;
 		}
	}
}


