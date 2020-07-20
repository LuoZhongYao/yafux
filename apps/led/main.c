#include "stm32f10x.h"

void LED_Config(void)
{
	/*打开GPIOC的时钟*/
	RCC->APB2ENR |= 0x00000010;

    /* 设置GPIOC的低8位为通用推挽输出 */
    GPIOC->CRL |= 0x33333333;
    
    /* 初始化为Io口 */
    GPIOC->ODR |= 0xFF00; 
}

void LED_SetState(u8 stateValue)
{    
    /* 设置LED灯的状态, GPIO一次设置16位，将其值强制转换位16位 */
    GPIOC->BSRR = (u16)stateValue & 0x00FF;
    GPIOC->BRR =  ~((u16)stateValue & 0x00FF);                  
}

int main(void)
{
	int i;
	uint8_t v = 0;
	LED_Config();
	while(1) {
		LED_SetState(~(1 << (v++ & 7)));
		for(i=0; i<0x000FFFFF; i++);
	}
	return 0;
}
