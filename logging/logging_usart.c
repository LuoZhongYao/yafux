#include "logging.h"
#include "stm32f10x.h"
#include <stdio.h>

#ifdef CONFIG_LOGGING_USART
void USART3_IRQHandler (void)
{
    uint8_t dat;

    if(USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        dat = USART_ReceiveData(USART3);
    }    
}

static void USART3_NVIC_RxConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ����NVIC���� */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;   //��USART1��ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		      //ʹ�ܡ�

    NVIC_Init(&NVIC_InitStructure);
}

int _write(int fd, char* ptr, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		USART_SendData(USART3, ptr[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET); //�ȴ��������
	}		

	return len;
}

static void logging_usart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ��RCCʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* ����TXD��GPIO���� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;               //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                     //�������PB10

	/* ��ʼ����������IO */
	GPIO_Init(GPIOB, &GPIO_InitStructure);

#if 0
	/* ����RXD��GPIO���� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          //ģ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                     //��������PA10

	GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

	/* ����USART�Ĳ��� */
	USART_InitStructure.USART_BaudRate = 115200;                  //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //���ݳ�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;             //��Ч��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ʧ��Ӳ����
	USART_InitStructure.USART_Mode = USART_Mode_Tx/* | USART_Mode_Rx */; //�������ͺͽ���ģʽ
	
    /* ��ʼ��USART3 */
	USART_Init(USART3, &USART_InitStructure);
	
	/* ʹ��USART3 */
	USART_Cmd(USART3, ENABLE);
#ifdef USE_USART1RX_INTERRUPT  
    USART3_NVIC_RxConfig();
    USART_ITConfig(USART3, USART_IT_RXNE ,ENABLE);
#endif 		
}

SYS_LOG_DEF(logging_usart_init, vprintf);

#endif
