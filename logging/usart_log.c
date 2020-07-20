#include "logging.h"
#include "stm32f10x.h"

#ifdef ENABLE_LOGGING
void USART1_IRQHandler (void)
{
    uint8_t dat;

    if(USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        dat = USART_ReceiveData(USART1);
    }    
}

static void USART1_NVIC_RxConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 设置NVIC参数 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应优先级为0
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   //打开USART1的全局中断
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		      //使能。

    NVIC_Init(&NVIC_InitStructure);
}

int _write(int fd, char* ptr, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		USART_SendData(USART1, ptr[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET); //等待发送完毕
	}		

	return len;
}

void logging_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 打开RCC时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 设置TXD的GPIO参数 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;               //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                     //串口输出PA9

	/* 初始化串口输入IO */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

#if 0
	/* 设置RXD的GPIO参数 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          //模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                     //串口输入PA10

	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

	/* 设置USART的参数 */
	USART_InitStructure.USART_BaudRate = 115200;                  //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //数据长8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;             //无效验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//失能硬件流
	USART_InitStructure.USART_Mode = USART_Mode_Tx/* | USART_Mode_Rx */; //开启发送和接受模式
	
    /* 初始化USART1 */
	USART_Init(USART1, &USART_InitStructure);
	
	/* 使能USART1 */
	USART_Cmd(USART1, ENABLE);
#ifdef USE_USART1RX_INTERRUPT  
    USART1_NVIC_RxConfig();
    USART_ITConfig(USART1, USART_IT_RXNE ,ENABLE);
#endif 		
}

#endif
