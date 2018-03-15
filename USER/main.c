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
 ALIENTEK ������STM32F7������ ʵ��46
 �����ʵ��-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

vu8 bmp_request=0;						//bmp��������:0,��bmp��������;1,��bmp��������,��Ҫ��֡�ж�����,�ر�DCMI�ӿ�.
u8 ovx_mode=0;							//bit0:0,RGB565ģʽ;1,JPEGģʽ 
u16 curline=0;							//����ͷ�������,��ǰ�б��
u16 yoffset=0;							//y�����ƫ����

#define jpeg_buf_size   8*1024*1024		//����JPEG���ݻ���jpeg_buf�Ĵ�С(8M�ֽ�)
#define jpeg_line_size	2*1024			//����DMA��������ʱ,һ�����ݵ����ֵ

u32 *dcmi_line_buf[2];					//RGB��ʱ,����ͷ����һ��һ�ж�ȡ,�����л���  
u32 *jpeg_data_buf;						//JPEG���ݻ���buf 

volatile u32 jpeg_data_len=0; 			//buf�е�JPEG��Ч���ݳ��� 
volatile u8 jpeg_data_ok=0;				//JPEG���ݲɼ���ɱ�־ 
										//0,����û�вɼ���;
										//1,���ݲɼ�����,���ǻ�û����;
										//2,�����Ѿ����������,���Կ�ʼ��һ֡����
		 
//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.

void Cam_Start(void)
{
//    LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();		        //��ʼд��GRAM
    __HAL_DMA_ENABLE(&DMADMCI_Handler); //ʹ��DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI����ʹ��

}
void Cam_Spot(void)
{
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //�رղ���
    while(DCMI->CR&0X01);               //�ȴ��������
    __HAL_DMA_DISABLE(&DMADMCI_Handler);//�ر�DMA
		
}


extern uint16_t version ;
extern uint8_t Frame_Flag;

void jpeg_data_process(void)
{
//	u16 i;
//	u16 rlen;			//ʣ�����ݳ���
//	u32 *pbuf;
//	curline=yoffset;	//������λ
//	if(ovx_mode&0X01)	//ֻ����JPEG��ʽ��,����Ҫ������.
//	{
//		if(jpeg_data_ok==0)	//jpeg���ݻ�δ�ɼ���?
//		{
//			DMA2_Stream1->CR&=~(1<<0);		//ֹͣ��ǰ����
//			while(DMA2_Stream1->CR&0X01);	//�ȴ�DMA2_Stream1������ 
//			rlen=jpeg_line_size-DMA2_Stream1->NDTR;//�õ�ʣ�����ݳ���	
//			pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ,�������
//			if(DMA2_Stream1->CR&(1<<19))for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1�����ʣ������
//			else for(i=0;i<rlen;i++)pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0�����ʣ������ 
//			jpeg_data_len+=rlen;			//����ʣ�೤��
//			jpeg_data_ok=1; 				//���JPEG���ݲɼ��갴��,�ȴ�������������
//		}
//		if(jpeg_data_ok==2)	//��һ�ε�jpeg�����Ѿ���������
//		{
//			DMA2_Stream1->NDTR=jpeg_line_size;//���䳤��Ϊjpeg_buf_size*4�ֽ�
//			DMA2_Stream1->CR|=1<<0;			//���´���
//			jpeg_data_ok=0;					//�������δ�ɼ�
//			jpeg_data_len=0;				//�������¿�ʼ
//		}
//	}else
//	{  
//		if(bmp_request==1)	//��bmp��������,�ر�DCMI
//		{
//			DCMI_Stop();	//ֹͣDCMI
//			bmp_request=0;	//������������.
//		}
//		LCD_SetCursor(0,0);  
//		LCD_WriteRAM_Prepare();				//��ʼд��GRAM  
//	}  
} 

//����JPEG����
//���ɼ���һ֡JPEG���ݺ�,���ô˺���,�л�JPEG BUF.��ʼ��һ֡�ɼ�.
void raw_data_process(void)
{
	uint32_t k;
	u16 i;
	u16 rlen;			//ʣ�����ݳ���
	u32 *pbuf;
	curline=yoffset;	//������λ

//		DMA2_Stream1->CR&=~(1<<0);		//ֹͣ��ǰ����
//		while(DMA2_Stream1->CR&0X01);	//�ȴ�DMA2_Stream1������ 
//		rlen=jpeg_line_size-DMA2_Stream1->NDTR;//�õ�ʣ�����ݳ���	
//		pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ,�������
//		if(DMA2_Stream1->CR&(1<<19))
//			for(i=0;i<rlen;i++)
//				pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1�����ʣ������
//		else 
//			for(i=0;i<rlen;i++)
//				pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0�����ʣ������ 
//		jpeg_data_len+=rlen;			//����ʣ�೤��
//		jpeg_data_ok=1; 				//���JPEG���ݲɼ��갴��,�ȴ�������������
//	
//			LCD_WriteRAM(yuv422_to_Gray(temp_l));
//			DCMI_Stop(); 			//ֹͣDMA����
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
//				DCMI_Start(); 			//�������� 
//				Frame_Flag=1;				
//				

}

//jpeg���ݽ��ջص�����
void jpeg_dcmi_rx_callback(void)
{  
	u16 i;
	u32 *pbuf;
	pbuf=jpeg_data_buf+jpeg_data_len;//ƫ�Ƶ���Ч����ĩβ
	if(DMA2_Stream1->CR&(1<<19))//buf0����,��������buf1
	{ 
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[0][i];//��ȡbuf0���������
		jpeg_data_len+=jpeg_line_size;//ƫ��
	}else //buf1����,��������buf0
	{
		for(i=0;i<jpeg_line_size;i++)pbuf[i]=dcmi_line_buf[1][i];//��ȡbuf1���������
		jpeg_data_len+=jpeg_line_size;//ƫ�� 
	} 
} 

//RGB�����ݽ��ջص�����
void rgblcd_dcmi_rx_callback(void)
{  
	u16 *pbuf;
	if(DMA2_Stream1->CR&(1<<19))//DMAʹ��buf1,��ȡbuf0
	{ 
		pbuf=(u16*)dcmi_line_buf[0]; 
	}else 						//DMAʹ��buf0,��ȡbuf1
	{
		pbuf=(u16*)dcmi_line_buf[1]; 
	} 	
	LTDC_Color_Fill(0,curline,lcddev.width-1,curline,pbuf);//DM2D��� 
	if(curline<lcddev.height)curline++;
	if(bmp_request==1&&curline==(lcddev.height-1))//��bmp��������,�ر�DCMI
	{
		DCMI_Stop();	//ֹͣDCMI
		bmp_request=0;	//������������.
	}
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
//�л�ΪSD��ģʽ
void sw_sdcard_mode(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	OV5640_WR_Reg(0X3017,0X00);	//�ر�OV5640ȫ�����(��Ӱ��SD��ͨ��)
	OV5640_WR_Reg(0X3018,0X00);  
	
 	//GPIOC8/9/11�л�Ϊ SDIO�ӿ�
    GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;  
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��
    GPIO_Initure.Pull=GPIO_PULLUP;              //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
    GPIO_Initure.Alternate=GPIO_AF12_SDMMC1;    //����ΪSDIO  
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);   
}
//�ļ������������⸲�ǣ�
//mode:0,����.bmp�ļ�;1,����.jpg�ļ�.
//bmp��ϳ�:����"0:PHOTO/PIC13141.bmp"���ļ���
//jpg��ϳ�:����"0:PHOTO/PIC13141.jpg"���ļ���
void camera_new_pathname(u8 *pname,u8 mode)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		if(mode==0)sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		else sprintf((char*)pname,"0:PHOTO/PIC%05d.jpg",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//���Դ�����ļ�
		if(res==FR_NO_FILE)break;		//���ļ���������=����������Ҫ��.
		index++;
	}
}  
//OV5640����jpgͼƬ
//����ֵ:0,�ɹ�
//    ����,�������
u8 ov5640_jpg_photo(u8 *pname)
{
	FIL* f_jpg; 
	u8 res=0,headok=0;
	u32 bwr;
	u32 i,jpgstart,jpglen;
	u8* pbuf;
	f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//����FIL�ֽڵ��ڴ����� 
	if(f_jpg==NULL)return 0XFF;				//�ڴ�����ʧ��.
	ovx_mode=1;
	jpeg_data_ok=0;
	sw_ov5640_mode();						//�л�ΪOV5640ģʽ 
	OV5640_JPEG_Mode();						//JPEGģʽ  
	OV5640_OutSize_Set(16,4,2592,1944);		//��������ߴ�(500W)  
	dcmi_rx_callback=jpeg_dcmi_rx_callback;	//JPEG�������ݻص�����
	DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],jpeg_line_size,DMA_MDATAALIGN_WORD,DMA_MINC_ENABLE);//DCMI DMA����    
	DCMI_Start(); 			//�������� 
	while(jpeg_data_ok!=1);	//�ȴ���һ֡ͼƬ�ɼ���
	jpeg_data_ok=2;			//���Ա�֡ͼƬ,������һ֡�ɼ� 
	while(jpeg_data_ok!=1);	//�ȴ��ڶ�֡ͼƬ�ɼ���,�ڶ�֡,�ű��浽SD��ȥ. 
	DCMI_Stop(); 			//ֹͣDMA����
	ovx_mode=0; 
	sw_sdcard_mode();		//�л�ΪSD��ģʽ
	res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//ģʽ0,���߳��Դ�ʧ��,�򴴽����ļ�	 
	if(res==0)
	{
		printf("jpeg data size:%d\r\n",jpeg_data_len*4);//���ڴ�ӡJPEG�ļ���С
		pbuf=(u8*)jpeg_data_buf;
		jpglen=0;	//����jpg�ļ���СΪ0
		headok=0;	//���jpgͷ���
		for(i=0;i<jpeg_data_len*4;i++)//����0XFF,0XD8��0XFF,0XD9,��ȡjpg�ļ���С
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))//�ҵ�FF D8
			{
				jpgstart=i;
				headok=1;	//����ҵ�jpgͷ(FF D8)
			}
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD9)&&headok)//�ҵ�ͷ�Ժ�,����FF D9
			{
				jpglen=i-jpgstart+2;
				break;
			}
		}
		if(jpglen)			//������jpeg���� 
		{
			pbuf+=jpgstart;	//ƫ�Ƶ�0XFF,0XD8��
			res=f_write(f_jpg,pbuf,jpglen,&bwr);
			if(bwr!=jpglen)res=0XFE; 
			
		}else res=0XFD; 
	}
	jpeg_data_len=0;
	f_close(f_jpg); 
	sw_ov5640_mode();		//�л�ΪOV5640ģʽ
	OV5640_RGB565_Mode();	//RGB565ģʽ  
	if(lcdltdc.pwidth!=0)	//RGB��
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB���������ݻص�����
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  
	}else					//MCU ��
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA����,MCU��,����
	}
	myfree(SRAMIN,f_jpg); 
	return res;
}  


int main(void)
{
	u32 k;
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
		
	sw_sdcard_mode();				//�����л�ΪOV5640ģʽ
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
	dcmi_line_buf[1]=mymalloc(SRAMIN,jpeg_line_size*4);	//Ϊjpeg dma���������ڴ�	
	jpeg_data_buf=mymalloc(SRAMEX,jpeg_buf_size);		//Ϊjpeg�ļ������ڴ�(���4MB)
 	pname=mymalloc(SRAMIN,30);//Ϊ��·�����ļ�������30���ֽڵ��ڴ�	 
 	while(pname==NULL||!dcmi_line_buf[0]||!dcmi_line_buf[1]||!jpeg_data_buf)	//�ڴ�������
 	{	    
		Show_Str(30,190,240,16,"�ڴ����ʧ��!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,190,240,146,WHITE);//�����ʾ	     
		delay_ms(200);				  
	}   	
//	while(OV5640_Init())//��ʼ��OV5640
//	{
//		Show_Str(30,190,240,16,"OV5640 ����!",16,0);
//		delay_ms(200);
//	    LCD_Fill(30,190,239,206,WHITE);
//		delay_ms(200);
//	}	
 	Show_Str(30,210,230,16,"MT9V034",16,0); 
	mt9v034_context_configuration();
	LCD_ShowxNum(30,230,version,5,16,1);
	
//	//�Զ��Խ���ʼ��
//	OV5640_RGB565_Mode();	//RGB565ģʽ 
//	OV5640_Focus_Init(); 
//	OV5640_Light_Mode(0);	//�Զ�ģʽ
//	OV5640_Color_Saturation(3);//ɫ�ʱ��Ͷ�0
//	OV5640_Brightness(4);	//����0
//	OV5640_Contrast(3);		//�Աȶ�0
//	OV5640_Sharpness(33);	//�Զ����
//	OV5640_Focus_Constant();//���������Խ�
	DCMI_Init();			//DCMI����
	if(lcdltdc.pwidth!=0)	//RGB��
	{
		dcmi_rx_callback=rgblcd_dcmi_rx_callback;//RGB���������ݻص�����
		DCMI_DMA_Init((u32)dcmi_line_buf[0],(u32)dcmi_line_buf[1],lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  
	}else					//MCU ��
	{
		DCMI_DMA_Init((u32)&LCD->LCD_RAM,0,1,DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE);			//DCMI DMA����,MCU��,����
		//DCMI_DMA_Init((u32)dcmi_line_buf[0],0,lcddev.width/2,DMA_MDATAALIGN_HALFWORD,DMA_MINC_ENABLE);//DCMI DMA����  	
	}
	/********���÷ֱ���***********/
	if(lcddev.height>800)
	{
		yoffset=(lcddev.height-800)/2;
		outputheight=800; 
		//OV5640_WR_Reg(0x3035,0X51);//�������֡�ʣ�������ܶ���
	}else 
	{
		yoffset=0;
		outputheight=lcddev.height;
	}
	curline=yoffset;		//������λ
	//OV5640_OutSize_Set(16,4,lcddev.width,outputheight);	//����������ʾ
	
	//LCD_Set_Window(0,0,640,480);
	
		DCMI_Start(); 			//��������
		//ovx_mode = 1;
	//LCD_Clear(GRAY);
 	while(1)
	{
				DCMI_Start(); 			//��������
				//Cam_Start();	
				delay_ms(20);
				while(Frame_Flag)
				{
					delay_ms(50);
				}
				Frame_Flag=1;	
				DCMI_Stop(); 			//ֹͣDMA����
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
//		if(i==20)//DS0��˸.
//		{
//			i=0;
//			LED0_Toggle;
// 		}
	}
}
