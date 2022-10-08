/* September 25, 2022
 * by Mr.H
 * main.c
*/

//#include "Drivers/Inc/stm32f4xx_gpio_driver.h"
#include "stdint.h" // for uint8_t and others
#include "stm32f411.h"
#include "stm32f4xx_gpio_driver.h"
#define BTN_PRESSED 1
uint8_t worker = 10;

void init();
int main(void)
{
	/* main be here */
	init();
	while(1);
	/*while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
		}
	}*/
}

void init()
{	
	/* Making PD13 as output (orange light built-in on Devboard) */
	GPIO_Handle_t GPIO_Handle;
	GPIO_Handle.pGPIOx = GPIOD;

	GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	
	GPIO_PClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_Handle);
	
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);

	/* Set PA0 as input and use as interrupt */
	GPIO_Handle.pGPIOx = GPIOA;

	GPIO_Handle.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_PClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_Handle);
	
	GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);
	/* Now set up the interrupt for the button */

	
	//uint8_t irq_no = 6; // PA0 is on EXTI0 line
	//uint32_t* pirqISER = (uint32_t*)0xE000E100;
	//enable it
	//*pirqISER |= (1 << irq_no);

	// enable EXTI0
	//uint32_t* RCCADDY = (uint32_t*)0x40023800;
	//*(RCCADDY + 0x44U) |= (1 << 14); //syscfg clock enable
	//uint32_t* EXTIADDR = (uint32_t*)0x40013C00;
	//*EXTIADDR |= (1 << 0); //EXTI IMR enable
	//*(EXTIADDR + 0x0C) |= (1 << 0); //EXTI Falling edge enable or something
	//uint32_t* SYSCFGADDY = (uint32_t*)0x40013800;
	
	/**NVIC_ISER0 |= (1 << 6);
	EXTI->FTSR |= (1 << 0);
	EXTI->RTSR &= ~(1 << 0);
	SYSCFG_PCLK_EN();
	SYSCFG->EXTICR[0] = 0;
	EXTI->IMR |= (1 << 0);
	*/
}

void Ext10_Handler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
	for(int i = 0; i < 500000; i++){;}
}
