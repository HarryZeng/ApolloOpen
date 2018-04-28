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
extern uint32_t MT9V034_Width,MT9V034_height;

#define jpeg_buf_size   8*1024*1024		//定义JPEG数据缓存jpeg_buf的大小(8M字节)
#define jpeg_line_size	2*1024			//定义DMA接收数据时,一行数据的最大值

u32 *dcmi_line_buf[2];					//RGB屏时,摄像头采用一行一行读取,定义行缓存  
u8 *jpeg_data_buf;						//JPEG数据缓存buf 

extern uint16_t version ;
extern uint8_t Frame_Flag;

u32 startX,startY,endX,endY,xlength,ydispaly,xdisplay;

//RGB rgb_data[HEIGHT][WIDTH];
//BYTE raw_data[HEIGHT][WIDTH];
u32 display_i;

void raw_data_process(void)
{
	u16 rlen;			//剩余数据长度
	u32 *pbuf;
	u32 y;
	
	startX = 0;
	startY = 0;
	
	endX = MT9V034_Width;
	endY = MT9V034_height;

	DCMI_Stop(); 			//停止DMA搬运
	
	//bayer2rgb24_bingo(rgb_buf,raw_buf,MT9V034_Width,MT9V034_height);
	//bayer2rgb(&dcmi_line_buf[0][0],jpeg_data_buf,MT9V034_Width,MT9V034_height);
	//BilinerInterpolation(raw_data,rgb_data);
	
//	display_i = 0;
//	xlength=endX-startX+1;	
//	
//	for(xdisplay=startY;xdisplay<=endY;xdisplay++)
//	{
//		LCD_SetCursor(startX,xdisplay);      				//设置光标位置 
//		LCD_WriteRAM_Prepare();     			//开始写入GRAM	  
//		for(ydispaly=0;ydispaly<xlength;ydispaly++)
//			LCD->LCD_RAM = jpeg_data_buf[display_i++];	//显示颜色 	 
//		//	for(j=0;j<xlength;j++)LCD->LCD_RAM=RED;	//显示颜色 
//	}
	LCD_SetCursor(0,0);      				//设置光标位置 
	LCD_WriteRAM_Prepare();     			//开始写入GRAM
	
	for(display_i=0;display_i<MT9V034_Width*MT9V034_height;display_i++)
	{
		//LCD->LCD_RAM=jpeg_data_buf[display_i];	//显示颜色 	
			//LCD->LCD_RAM=display_i%255;	//显示颜色 	
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
//		LCD->LCD_RAM=display_i%255;	//显示颜色 	 
//	}

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
	u32 k=0;
	u8 res,fac;							 
	u8 *pname;					//带路径的文件名 
	u8 key;						//键值		   
	u8 i;						 
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
    KEY_Init();                     //初始化按键
    SDRAM_Init();                   //初始化SDRAM
    LCD_Init();                     //初始化LCD
		
	PCF8574_Init();                 //初始化PCF8574
    //OV5640_Init();				    //初始化OV5640
		
	//sw_sdcard_mode();				//首先切换为OV5640模式
	W25QXX_Init();					//初始化W25Q256
 	my_mem_init(SRAMIN);			//初始化内部内存池
	my_mem_init(SRAMEX);			//初始化外部内存池
	my_mem_init(SRAMDTCM);			//初始化TCM内存池 
	exfuns_init();					//为fatfs相关变量申请内存  
 	f_mount(fs[0],"0:",1); 			//挂载SD卡  
	POINT_COLOR=RED;      	
	while(font_init()) 				//检查字库
	{
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
		delay_ms(200);				  
	}
 	Show_Str(30,50,200,16,"阿波罗STM32F4/F7开发板",16,0);	 			    	 
	Show_Str(30,70,200,16,"照相机实验",16,0);				    	 
	Show_Str(30,90,200,16,"KEY0:拍照(bmp格式)",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:拍照(jpg格式)",16,0);			    	 
	Show_Str(30,130,200,16,"KEY2:自动对焦",16,0);					    	 
	Show_Str(30,150,200,16,"WK_UP:FullSize/Scale",16,0);				    	 
	Show_Str(30,170,200,16,"2016年7月31日",16,0);
	res=f_mkdir("0:/PHOTO");		//创建PHOTO文件夹
	if(res!=FR_EXIST&&res!=FR_OK) 	//发生了错误
	{
		res=f_mkdir("0:/PHOTO");		//创建PHOTO文件夹		
		Show_Str(30,190,240,16,"SD卡错误!",16,0);
		delay_ms(200);				  
		Show_Str(30,190,240,16,"拍照功能将不可用!",16,0);
		delay_ms(200);				  
		sd_ok=0;  	
	} 	
	dcmi_line_buf[0]=mymalloc(SRAMIN,jpeg_line_size*4);	//为jpeg dma接收申请内存	
	dcmi_line_buf[1]=mymalloc(SRAMIN,jpeg_line_size*40);	//为jpeg dma接收申请内存	
	jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//为jpeg文件申请内存(最大4MB)
 	pname=mymalloc(SRAMIN,30);//为带路径的文件名分配30个字节的内存	 
 	while(pname==NULL||!dcmi_line_buf[0]||!dcmi_line_buf[1]||!jpeg_data_buf)	//内存分配出错
 	{	    
		Show_Str(30,190,240,16,"内存分配失败!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//清除显示	     
		delay_ms(200);				  
	}   	

 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(30,230,version,5,16,1);
	LCD_Set_Window(0,0,MT9V034_Width,MT9V034_height);
	DCMI_Init();			//DCMI配置

	//DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA配置,MCU屏,竖屏
	DCMI_DMA_Init((u32)jpeg_data_buf,0,MT9V034_Width*MT9V034_height,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);			//DCMI DMA配置,MCU屏,竖屏
	DCMI_Start(); 			//启动传输

	while(1)
	{
		if(i==20)//DS0闪烁.
		{
			i=0;
			LED0_Toggle;
 		}
	}
}


