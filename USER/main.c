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
extern uint16_t MT9V034_Width,MT9V034_height;

unsigned char *raw_buf;					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л���  
unsigned char *rgb_buf;					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л��� 

extern uint16_t version ;
extern uint8_t Frame_Flag;

u32 startX,startY,endX,endY,xlength,j,i;

//RGB rgb_data[HEIGHT][WIDTH];
//BYTE raw_data[HEIGHT][WIDTH];

void raw_data_process(void)
{
	u32 i;
	u16 rlen;			//ʣ�����ݳ���
	u32 *pbuf;
	
	startX = 0;
	startY = 0;
	
	endX = MT9V034_Width;
	endY = MT9V034_height;

	DCMI_Stop(); 			//ֹͣDMA����
	
		//bayer2rgb24_bingo(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	
		bayer2rgb(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	//BilinerInterpolation(raw_data,rgb_data);
	
	xlength=endX-startX+1;	 
	for(i=startY;i<=endY;i++)
	{
		LCD_SetCursor(startX,i);      				//���ù��λ�� 
		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	  
		for(j=0;j<xlength;j++)LCD->LCD_RAM=rgb_buf[i];	//��ʾ��ɫ 	    
	} 
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
	
	u8 res,fac;							 
	u8 *pname;					//��·�����ļ��� 
	u8 key;						//��ֵ		   
					 
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

    SDRAM_Init();                   //��ʼ��SDRAM
    LCD_Init();                     //��ʼ��LCD
		PCF8574_Init();                 //��ʼ��PCF8574
		sw_ov5640_mode();
		
	LCD_Display_Dir(1);		//0->Ϊ���� ,1->����
	LCD_Set_Window(0,0,752,480);
		
 	my_mem_init(SRAMIN);			//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);			//��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMDTCM);			//��ʼ��TCM�ڴ�� 
	exfuns_init();					//Ϊfatfs��ر��������ڴ�  
	POINT_COLOR=RED;      	


	raw_buf= mymalloc(SRAMIN,bufsize);	//Ϊjpeg dma���������ڴ�	
	rgb_buf= mymalloc(SRAMIN,bufsize);	//Ϊjpeg dma���������ڴ�	

 	Show_Str(30,50,200,16,"������STM32F4/F7������",16,0);	 			    	 
	Show_Str(30,70,200,16,"�����ʵ��",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:����(bmp��ʽ)",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:����(jpg��ʽ)",16,0);			    	 
	Show_Str(30,130,200,16,"KEY2:�Զ��Խ�",16,0);					    	 
	Show_Str(30,150,200,16,"WK_UP:FullSize/Scale",16,0);				    	 
	Show_Str(30,170,200,16,"2016��7��31��",16,0);

	DCMI_Init();			//DCMI����
 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(640,240,version,5,16,1);
	
	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA����,MCU��,����
	DCMI_DMA_Init((u32)raw_buf[0],0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  	

	DCMI_Start(); 			//��������
		
 	while(1)
	{
		
		//LCD_ShowxNum(680,450,sizeof(jpeg_data_buf),5,16,1);
		//LCD_Fill(0,0,640,480,BLUE);
		//MT9V034_TEST();
		
//		xlength=endX-startX+1;	 
//		for(i=startY;i<=endY;i++)
//		{
//			LCD_SetCursor(startX,i);      				//���ù��λ�� 
//			LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	  
//			for(j=0;j<xlength;j++)LCD->LCD_RAM=RED;	//��ʾ��ɫ 	    
//		} 
		
//		LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM	  
//		xlength=endX * endY;	 
//		for(i=startY;i<=xlength;i++)
//		{
//			LCD->LCD_RAM=RED;	//��ʾ��ɫ 	    
//		}
		
//				Frame_Flag=1;				
//				LCD_SetCursor(0,0);
//				LCD_WriteRAM_Prepare();
	
//							for(k=0;k<lcddev.width/2;k++)
//							{
//								LCD_WriteRAM(dcmi_line_buf[0][k]);
//							}
		//LCD_WriteRAM(dcmi_line_buf[0][i]);
//		for(k=0;k<(480*320);k++)
//		{
//			LCD->LCD_RAM = 655;
//		}
		
		delay_ms(25);
		i++;
		if(i==20)//DS0��˸.
		{
			i=0;
			LED0_Toggle;
 		}
	}
}


