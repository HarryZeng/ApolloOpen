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
/************************************************
 ALIENTEK 阿波罗STM32F7开发板 实验46
 照相机实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

vu8 bmp_request=0;						//bmp拍照请求:0,无bmp拍照请求;1,有bmp拍照请求,需要在帧中断里面,关闭DCMI接口.
u8 ovx_mode=0;							//bit0:0,RGB565模式;1,JPEG模式 
u16 curline=0;							//摄像头输出数据,当前行编号
u16 yoffset=0;							//y方向的偏移量

#define jpeg_buf_size   8*1024*1024		//定义JPEG数据缓存jpeg_buf的大小(8M字节)
#define jpeg_line_size	2*1024			//定义DMA接收数据时,一行数据的最大值

u32 *dcmi_line_buf[2];					//RGB屏时,摄像头采用一行一行读取,定义行缓存  
u32 *jpeg_data_buf;						//JPEG数据缓存buf 

volatile u32 jpeg_data_len=0; 			//buf中的JPEG有效数据长度 
volatile u8 jpeg_data_ok=0;				//JPEG数据采集完成标志 
										//0,数据没有采集完;
										//1,数据采集完了,但是还没处理;
										//2,数据已经处理完成了,可以开始下一帧接收
		 
//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.

void Cam_Start(void)
{
//    LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();		        //开始写入GRAM
    __HAL_DMA_ENABLE(&DMADMCI_Handler); //使能DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能

}
void Cam_Spot(void)
{
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //关闭捕获
    while(DCMI->CR&0X01);               //等待传输完成
    __HAL_DMA_DISABLE(&DMADMCI_Handler);//关闭DMA
		
}


extern uint16_t version ;
extern uint8_t Frame_Flag;

void jpeg_data_process(void)
{
//	u16 i;
//	u16 rlen;			//剩余数据长度
//	u32 *pbuf;
//	curline=yoffset;	//行数复位
//	if(ovx_mode&0X01)	//只有在JPEG格式下,才需要做处理.
//	{
//		if(jpeg_data_ok==0)	//jpeg数据还未采集完?
//		{
//			DMA2_Stream1->CR&=~(1<<0);		//停止当前传输
//			while(DMA2_Stream1->CR&0X01);	//等待DMA2_Stream1可配置 
//			rlen=jpeg_line_size-DMA2_Stream1->NDTR;//得到剩余数据长度	
//			pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾,继续添加
//			if(DMA2_Stream1->CR&(1<<19))for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的剩余数据
//			else for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的剩余数据 
//			jpeg_data_len+=rlen;			//加上剩余长度
//			jpeg_data_ok=1; 				//标记JPEG数据采集完按成,等待其他函数处理
//		}
//		if(jpeg_data_ok==2)	//上一次的jpeg数据已经被处理了
//		{
//			DMA2_Stream1->NDTR=jpeg_line_size;//传输长度为jpeg_buf_size*4字节
//			DMA2_Stream1->CR|=1<<0;			//重新传输
//			jpeg_data_ok=0;					//标记数据未采集
//			jpeg_data_len=0;				//数据重新开始
//		}
//	}else
//	{  
//		if(bmp_request==1)	//有bmp拍照请求,关闭DCMI
//		{
//			DCMI_Stop();	//停止DCMI
//			bmp_request=0;	//标记请求处理完成.
//		}
//		LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();				//开始写入GRAM  
//	}  
} 

//处理JPEG数据
//当采集完一帧JPEG数据后,调用此函数,切换JPEG BUF.开始下一帧采集.
void raw_data_process(void)
{
	uint32_t k;
	u16 i;
	u16 rlen;			//剩余数据长度
	u32 *pbuf;
	curline=yoffset;	//行数复位

//		DMA2_Stream1->CR&=~(1<<0);		//停止当前传输
//		while(DMA2_Stream1->CR&0X01);	//等待DMA2_Stream1可配置 
//		rlen=jpeg_line_size-DMA2_Stream1->NDTR;//得到剩余数据长度	
//		pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾,继续添加
//		if(DMA2_Stream1->CR&(1<<19))
//			for(i=0;i<rlen;i++)
//				pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的剩余数据
//		else 
//			for(i=0;i<rlen;i++)
//				pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的剩余数据 
//		jpeg_data_len+=rlen;			//加上剩余长度
//		jpeg_data_ok=1; 				//标记JPEG数据采集完按成,等待其他函数处理
//	
//			LCD_WriteRAM(yuv422_to_Gray(temp_l));
//			DCMI_Stop(); 			//停止DMA搬运
//			LCD_SetCursor(0,0);
//			for(k=0;k<lcddev.width/2;k++)
//					LCD_WriteRAM(dcmi_line_buf[0][i]);
				//delay_ms(2);
//				LCD_SetCursor(0,0);
//				LCD_WriteRAM_Prepare();
//				while(Frame_Flag)
//				{
//					delay_ms(5);
//				}
//				DCMI_Start(); 			//启动传输 
//				Frame_Flag=1;				
//				

}

//jpeg数据接收回调函数
void jpeg_dcmi_rx_callback(void)
{  
	u16 i;
	u32 *pbuf;
	pbuf=jpeg_data_buf+jpeg_data_len;//偏移到有效数据末尾
	if(DMA2_Stream1->CR&(1<<19))//buf0已满,正常处理buf1
	{ 
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[0][i];//读取buf0里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移
	}else //buf1已满,正常处理buf0
	{
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[1][i];//读取buf1里面的数据
		jpeg_data_len+=jpeg_line_size;//偏移 
	} 
} 

//RGB屏数据接收回调函数
void rgblcd_dcmi_rx_callback(void)
{  
	u16 *pbuf;
	if(DMA2_Stream1->CR&(1<<19))//DMA使用buf1,读取buf0
	{ 
		pbuf=(u16*)dcmi_line_buf[0]; 
	}else 						//DMA使用buf0,读取buf1
	{
		pbuf=(u16*)dcmi_line_buf[1]; 
	} 	
	LTDC_Color_Fill(0,curline,lcddev.width-1,curline,pbuf);//DM2D填充 
	if(curline<lcddev.height)curline++;
	if(bmp_request==1&&curline==(lcddev.height-1))//有bmp拍照请求,关闭DCMI
	{
		DCMI_Stop();	//停止DCMI
		bmp_request=0;	//标记请求处理完成.
	}
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
//切换为SD卡模式
void sw_sdcard_mode(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	OV5640_WR_Reg(0X3017,0X00);	//关闭OV5640全部输出(不影响SD卡通信)
	OV5640_WR_Reg(0X3018,0X00);  
	
 	//GPIOC8/9/11切换为 SDIO接口
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //高速
    GPIO_Initure.Alternate=GPIO_AF12_SDMMC1;    //复用为SDIO  
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);   
}
//文件名自增（避免覆盖）
//mode:0,创建.bmp文件;1,创建.jpg文件.
//bmp组合成:形如"0:PHOTO/PIC13141.bmp"的文件名
//jpg组合成:形如"0:PHOTO/PIC13141.jpg"的文件名
void camera_new_pathname(u8 *pname,u8 mode)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		if(mode==0)sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		else sprintf((char*)pname,"0:PHOTO/PIC%05d.jpg",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		if(res==FR_NO_FILE)break;		//该文件名不存在=正是我们需要的.
		index++;
	}
}  
//OV5640拍照jpg图片
//返回值:0,成功
//    其他,错误代码
u8 ov5640_jpg_photo(u8 *pname)
{
	FIL* f_jpg; 
	u8 res=0,headok=0;
	u32 bwr;
	u32 i,jpgstart,jpglen;
	u8* pbuf;
	f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//开辟FIL字节的内存区域 
	if(f_jpg==NULL)return 0XFF;				//内存申请失败.
	ovx_mode=1;
	jpeg_data_ok=0;
	sw_ov5640_mode();						//切换为OV5640模式 
	OV5640_JPEG_Mode();						//JPEG模式  
	OV5640_OutSize_Set(16,4,2592,1944);		//设置输出尺寸(500W)  
	dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG接收数据回调函数
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,DMA_MDATAALIGN_WORD,DMA_MINC_ENABLE);//DCMI DMA配置    
	DCMI_Start(); 			//启动传输 
	while(jpeg_data_ok!=1);	//等待第一帧图片采集完
	jpeg_data_ok=2;			//忽略本帧图片,启动下一帧采集 
	while(jpeg_data_ok!=1);	//等待第二帧图片采集完,第二帧,才保存到SD卡去. 
	DCMI_Stop(); 			//停止DMA搬运
	ovx_mode=0; 
	sw_sdcard_mode();		//切换为SD卡模式
	res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//模式0,或者尝试打开失败,则创建新文件	 
	if(res==0)
	{
		printf("jpeg data size:%d\r\n",jpeg_data_len*4);//串口打印JPEG文件大小
		pbuf=(u8*)jpeg_data_buf;
		jpglen=0;	//设置jpg文件大小为0
		headok=0;	//清除jpg头标记
		for(i=0;i<jpeg_data_len*4;i++)//查找0XFF,0XD8和0XFF,0XD9,获取jpg文件大小
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//找到FF D8
			{
				jpgstart=i;
				headok=1;	//标记找到jpg头(FF D8)
			}
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//找到头以后,再找FF D9
			{
				jpglen=i-jpgstart+2;
				break;
			}
		}
		if(jpglen)			//正常的jpeg数据 
		{
			pbuf+=jpgstart;	//偏移到0XFF,0XD8处
			res=f_write(f_jpg,pbuf,jpglen,&bwr);
			if(bwr!=jpglen)res=0XFE; 
			
		}else res=0XFD; 
	}
	jpeg_data_len=0;
	f_close(f_jpg); 
	sw_ov5640_mode();		//切换为OV5640模式
	OV5640_RGB565_Mode();	//RGB565模式  
	if(lcdltdc.pwidth!=0)	//RGB屏
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB屏接收数据回调函数
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  
	}else					//MCU 屏
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA配置,MCU屏,竖屏
	}
	myfree(SRAMIN,f_jpg); 
	return res;
}  


int main(void)
{
	u32 k;
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
		
	sw_sdcard_mode();				//首先切换为OV5640模式
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
	dcmi_line_buf[1]=mymalloc(SRAMIN,jpeg_line_size*4);	//为jpeg dma接收申请内存	
	jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//为jpeg文件申请内存(最大4MB)
 	pname=mymalloc(SRAMIN,30);//为带路径的文件名分配30个字节的内存	 
 	while(pname==NULL||!dcmi_line_buf[0]||!dcmi_line_buf[1]||!jpeg_data_buf)	//内存分配出错
 	{	    
		Show_Str(30,190,240,16,"内存分配失败!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//清除显示	     
		delay_ms(200);				  
	}   	
//	while(OV5640_Init())//初始化OV5640
//	{
//		Show_Str(30,190,240,16,"OV5640 错误!",16,0);
//		delay_ms(200);
//	    LCD_Fill(30,190,239,206,WHITE);
//		delay_ms(200);
//	}	
 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(30,230,version,5,16,1);
	
//	//自动对焦初始化
//	OV5640_RGB565_Mode();	//RGB565模式 
//	OV5640_Focus_Init(); 
//	OV5640_Light_Mode(0);	//自动模式
//	OV5640_Color_Saturation(3);//色彩饱和度0
//	OV5640_Brightness(4);	//亮度0
//	OV5640_Contrast(3);		//对比度0
//	OV5640_Sharpness(33);	//自动锐度
//	OV5640_Focus_Constant();//启动持续对焦
	DCMI_Init();			//DCMI配置
	if(lcdltdc.pwidth!=0)	//RGB屏
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB屏接收数据回调函数
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  
	}else					//MCU 屏
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA配置,MCU屏,竖屏
		//DCMI_DMA_Init((u32)dcmi_line_buf[0],0,lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA配置  	
	}
	/********设置分辨率***********/
	if(lcddev.height>800)
	{
		yoffset=(lcddev.height-800)/2;
		outputheight=800; 
		//OV5640_WR_Reg(0x3035,0X51);//降低输出帧率，否则可能抖动
	}else 
	{
		yoffset=0;
		outputheight=lcddev.height;
	}
	curline=yoffset;		//行数复位
	//OV5640_OutSize_Set(16,4,lcddev.width,outputheight);	//满屏缩放显示
	
	//LCD_Set_Window(0,0,640,480);
	
		DCMI_Start(); 			//启动传输
		//ovx_mode = 1;
	//LCD_Clear(GRAY);
 	while(1)
	{
				DCMI_Start(); 			//启动传输
				//Cam_Start();	
				delay_ms(20);
				while(Frame_Flag)
				{
					delay_ms(50);
				}
				Frame_Flag=1;	
				DCMI_Stop(); 			//停止DMA搬运
					LCD_SetCursor(0,0);
					LCD_WriteRAM_Prepare();
//					for(k=0;k<lcddev.width/2;k++)
//							LCD_WriteRAM(655);	//LCD->LCD_RAM = dcmi_line_buf[0][k];	LCD_WriteRAM(65535);

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
		
		//delay_ms(25);
//		i++;
//		if(i==20)//DS0闪烁.
//		{
//			i=0;
//			LED0_Toggle;
// 		}
	}
}
