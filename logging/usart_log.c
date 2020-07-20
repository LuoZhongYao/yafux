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

    /* ����NVIC���� */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //��Ӧ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   //��USART1��ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		      //ʹ�ܡ�

    NVIC_Init(&NVIC_InitStructure);
}

int _write(int fd, char* ptr, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		USART_SendData(USART1, ptr[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET); //�ȴ��������
	}		

	return len;
}

void logging_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ��RCCʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* ����TXD��GPIO���� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;               //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                     //�������PA9

	/* ��ʼ����������IO */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

#if 0
	/* ����RXD��GPIO���� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;          //ģ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                     //��������PA10

	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

	/* ����USART�Ĳ��� */
	USART_InitStructure.USART_BaudRate = 115200;                  //������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //���ݳ�8λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;          //1λֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;             //��Ч��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ʧ��Ӳ����
	USART_InitStructure.USART_Mode = USART_Mode_Tx/* | USART_Mode_Rx */; //�������ͺͽ���ģʽ
	
    /* ��ʼ��USART1 */
	USART_Init(USART1, &USART_InitStructure);
	
	/* ʹ��USART1 */
	USART_Cmd(USART1, ENABLE);
#ifdef USE_USART1RX_INTERRUPT  
    USART1_NVIC_RxConfig();
    USART_ITConfig(USART1, USART_IT_RXNE ,ENABLE);
#endif 		
}

#endif
