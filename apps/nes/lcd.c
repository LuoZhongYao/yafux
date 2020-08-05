/*
 * Copyright (c) 2020 ZhongYao Luo <luozhongyao@gmail.com>
 *
 * SPDX-License-Identifier:
 */
#include "lcd.h"
#include "stm32f10x.h"
#include <logging.h>

extern void delay(unsigned ms);
/* TFT地址结构体 */
typedef volatile struct
{
	uint16_t reg;
	uint16_t data;
} LCD_TypeDef;

/* 使用NOR/SRAM的 Bank1.sector4,地址位HADDR[27,26]=11 A10作为数据命令区分线 */
/* 注意设置时STM32内部会右移一位对其! 111110=0X3E */
#define LCD_BASE        ((uint32_t)(0x6C000000 | 0x000007FE))
#define TFT             ((LCD_TypeDef *) LCD_BASE)

/* 定义屏的大小 */
#define LCD_XMAX 239		//设置TFT屏的大小
#define LCD_YMAX 399

/* 定义颜色的宏 */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0		 //定义颜色的宏

static void LCD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 打开时钟使能 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE
	                      | RCC_APB2Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	/* FSMC_A10(G12) 和RS（G0）*/
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
	                              | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_8
								  | GPIO_Pin_9 | GPIO_Pin_10 |GPIO_Pin_11
								  | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
								  | GPIO_Pin_15 );

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9
	                               | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12
								   | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void LCD_FSMC_Config(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  FSMC_ReadTimingInitStructure; 
	FSMC_NORSRAMTimingInitTypeDef  FSMC_WriteTimingInitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC,ENABLE);	//使能FSMC时钟


	FSMC_ReadTimingInitStructure.FSMC_AddressSetupTime = 0x01;	 //地址建立时间（ADDSET）为2个HCLK 1/36M=27ns
	FSMC_ReadTimingInitStructure.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（ADDHLD）模式A未用到	
	FSMC_ReadTimingInitStructure.FSMC_DataSetupTime = 0x0f;		 // 数据保存时间为16个HCLK,因为液晶驱动IC的读数据的时候，速度不能太快，尤其对1289这个IC。
	FSMC_ReadTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_ReadTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 


	FSMC_WriteTimingInitStructure.FSMC_AddressSetupTime = 0x15;	 //地址建立时间（ADDSET）为16个HCLK  
	FSMC_WriteTimingInitStructure.FSMC_AddressHoldTime = 0x15;	 //地址保持时间		
	FSMC_WriteTimingInitStructure.FSMC_DataSetupTime = 0x05;		 //数据保存时间为6个HCLK	
	FSMC_WriteTimingInitStructure.FSMC_BusTurnAroundDuration = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_CLKDivision = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_DataLatency = 0x00;
	FSMC_WriteTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A  

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;//  这里我们使用NE4 ，也就对应BTCR[6],[7]。
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
	FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//存储器数据宽度为16bit   
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  存储器写使能
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_ReadTimingInitStructure; //读写时序
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_WriteTimingInitStructure;  //写时序

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  // 使能BANK1 

}

static inline void LCD_data_write(const uint16_t *data, unsigned size)
{
	unsigned i;
	for (i = 0; i < size; i++) {
		TFT->data = data[i];
	}
}

static inline void LCD_data_read(uint16_t *data, unsigned size)
{
	unsigned i;
	for (i = 0; i < size; i++) {
		data[i] = TFT->data;
	}
}

static inline void LCD_reg_write(uint16_t reg)
{
	TFT->reg = reg;
}

static void lcd_write(uint16_t reg, const uint16_t *data, unsigned size)
{
	LCD_reg_write(reg);
	LCD_data_write(data, size);
}

#define LCD_WRITE_ARRAY(reg, ...)				\
	do {										\
		uint16_t _buf[] = {__VA_ARGS__};		\
		lcd_write(reg, _buf, sizeof(_buf) / sizeof(uint16_t));	\
	} while (0)

static void lcd_write16(uint16_t reg, uint16_t val)
{
	lcd_write(reg, &val, 1);
}

static void lcd_read(uint16_t reg, uint16_t *data, unsigned size)
{
	LCD_reg_write(reg);
	LCD_data_read(data, size);
}

void lcd_init(void)
{
	uint16_t val;
	uint16_t buf[15];

	LCD_GPIO_Config();
	LCD_FSMC_Config();

	delay(50);

	lcd_read(0xd0, &val, 1);
	lcd_read(0xd0, &val, 1);
	BLOGD("Chip ID: %x\n", val);

	LCD_WRITE_ARRAY(0xe9, 0x20);
	LCD_reg_write(0x11);
	delay(50);

	LCD_WRITE_ARRAY(0x3a, 0x55);

	LCD_WRITE_ARRAY(0xd1, 0x00, 0x65, 0x1f);
	LCD_WRITE_ARRAY(0xd0, 0x07, 0x07, 0x80);
	LCD_WRITE_ARRAY(0x36, 0x4c);
	LCD_WRITE_ARRAY(0xc1, 0x10, 0x10, 0x02, 0x02);
	LCD_WRITE_ARRAY(0xc0, 0x00, 0x35, 0x00, 0x00, 0x01, 0x02);
	LCD_WRITE_ARRAY(0xc4, 0x03);
	LCD_WRITE_ARRAY(0xc5, 0x01);
	LCD_WRITE_ARRAY(0xd2, 0x01, 0x22);
	LCD_WRITE_ARRAY(0xe7, 0x38);
	LCD_WRITE_ARRAY(0xf3, 0x08, 0x12, 0x12, 0x08);
	LCD_WRITE_ARRAY(0xc8, 0x01, 0x52, 0x37, 0x10, 0x0d, 0x01, 0x04, 0x51, 0x77, 0x01, 0x01, 0x0d, 0x08, 0x80, 0x00);
	LCD_WRITE_ARRAY(0x29);
}
