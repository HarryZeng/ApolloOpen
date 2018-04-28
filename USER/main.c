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
 ALIENTEK 阿波罗STM32F7开发板 实验46
 照相机实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/
#define bufsize  752*480
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
u32 startX,startY,endX,endY,runX,runY;
extern uint16_t MT9V034_Width,MT9V034_height;

unsigned char *raw_buf;					//RGB屏时,摄像头采用一行一行读取,定义行缓存  
unsigned char *rgb_buf;					//RGB屏时,摄像头采用一行一行读取,定义行缓存 

extern uint16_t version ;
extern uint8_t Frame_Flag;

u32 startX,startY,endX,endY,xlength,j,i;

//RGB rgb_data[HEIGHT][WIDTH];
//BYTE raw_data[HEIGHT][WIDTH];

void raw_data_process(void)
{
	u32 i;
	u16 rlen;			//剩余数据长度
	u32 *pbuf;
	
	startX = 0;
	startY = 0;
	
	endX = MT9V034_Width;
	endY = MT9V034_height;

	DCMI_Stop(); 			//停止DMA搬运
	
		//bayer2rgb24_bingo(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	
		bayer2rgb(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	//BilinerInterpolation(raw_data,rgb_data);
	
	xlength=endX-startX+1;	 
	for(i=startY;i<=endY;i++)
	{
		LCD_SetCursor(startX,i);      				//设置光标位置 
		LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
		for(j=0;j<xlength;j++)LCD->LCD_RAM=rgb_buf[i];	//显示颜色 	    
	} 
	DCMI_Start(); 			//启动传输 
	Frame_Flag=1;
}


//切换为OV5640模式
void sw_ov5640_mode(void)
{ 
	GPIO_InitTypeDef GPIO_Initure;
 	OV5640_WR_Reg(0X3017,0XFF);	//开启OV5650输出(可以正常显示)
	OV5640_WR_Reg(0X3018,0XFF); 

	//GPIOC8/9/11切换为 DCMI接口
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    GPIO_Initure.Alternate=GPIO_AF13_DCMI;      //复用为DCMI   
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //初始化   
} 


int main(void)
{
	
	u8 res,fac;							 
	u8 *pname;					//带路径的文件名 
	u8 key;						//键值		   
					 
	u8 sd_ok=1;					//0,sd卡不正常;1,SD卡正常. 
 	u8 scale=1;					//默认是全尺寸缩放
	u8 msgbuf[15];				//消息缓存区 
	u16 outputheight=0;
	
    Cache_Enable();                 //打开L1-Cache
    MPU_Memory_Protection();        //保护相关存储区域
    HAL_Init();				        //初始化HAL库
    Stm32_Clock_Init(432,25,2,9);   //设置时钟,216Mhz 
    delay_init(216);                //延时初始化
		uart_init(115200);		        //串口初始化
		usmart_dev.init(108); 		    //初始化USMART
    LED_Init();                     //初始化LED

    SDRAM_Init();                   //初始化SDRAM
    LCD_Init();                     //初始化LCD
		PCF8574_Init();                 //初始化PCF8574
		sw_ov5640_mode();
		
	LCD_Display_Dir(1);		//0->为竖屏 ,1->横屏
	LCD_Set_Window(0,0,752,480);
		
 	my_mem_init(SRAMIN);			//初始化内部内存池
	my_mem_init(SRAMEX);			//初始化外部内存池
	my_mem_init(SRAMDTCM);			//初始化TCM内存池 
	exfuns_init();					//为fatfs相关变量申请内存  
	POINT_COLOR=RED;      	


	raw_buf= mymalloc(SRAMIN,bufsize);	//为jpeg dma接收申请内存	
	rgb_buf= mymalloc(SRAMIN,bufsize);	//为jpeg dma接收申请内存	

 	Show_Str(30,50,200,16,"阿波罗STM32F4/F7开发板",16,0);	 			    	 
	Show_Str(30,70,200,16,"照相机实验",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:拍照(bmp格式)",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:拍照(jpg格式)",16,0);			    	 
	Show_Str(30,130,200,16,"KEY2:自动对焦",16,0);					    	 
	Show_Str(30,150,200,16,"WK_UP:FullSize/Scale",16,0);				    	 
	Show_Str(30,170,200,16,"2016年7月31日",16,0);

	DCMI_Init();			//DCMI配置
 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(640,240,version,5,16,1);
	
	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA配置,MCU屏,竖屏
	DCMI_DMA_Init((u32)raw_buf[0],0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  	

	DCMI_Start(); 			//启动传输
		
 	while(1)
	{
		
		//LCD_ShowxNum(680,450,sizeof(jpeg_data_buf),5,16,1);
		//LCD_Fill(0,0,640,480,BLUE);
		//MT9V034_TEST();
		
//		xlength=endX-startX+1;	 
//		for(i=startY;i<=endY;i++)
//		{
//			LCD_SetCursor(startX,i);      				//设置光标位置 
//			LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
//			for(j=0;j<xlength;j++)LCD->LCD_RAM=RED;	//显示颜色 	    
//		} 
		
//		LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
//		xlength=endX * endY;	 
//		for(i=startY;i<=xlength;i++)
//		{
//			LCD->LCD_RAM=RED;	//显示颜色 	    
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
		if(i==20)//DS0闪烁.
		{
			i=0;
			LED0_Toggle;
 		}
	}
}


