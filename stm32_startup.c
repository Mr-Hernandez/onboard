#include <stdint.h>

#define SRAM_START       0x20000000U
#define SRAM_SIZE        (128U * 1024U) //128KB
#define SRAM_END         ((SRAM_START) + (SRAM_SIZE))

#define STACK_START      SRAM_END   

extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;

extern uint32_t _sbss;
extern uint32_t _ebss;


// main prototyp
int main(void);
/* function prototypes for the STM32F411ve system exception and IRQ handlers*/

void Reset_Handler(void);

void NMI_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void SVCAll_Handler 	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMoniter_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void WWDG_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI16PVD_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI21_TAMP_STAMP_Handler	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI22_RTC_WKUP_Handler	(void) __attribute__ ((weak, alias("Default_Handler")));
void Flash_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Rcc_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Ext10_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Ext11_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Ext12_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Ext13_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void Ext14_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream0_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream1_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream2_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream3_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream4_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream5_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Stream6_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void ADC_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_Handler	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_Handler	         	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI17_RTC_ALARM_Handler	(void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI18_OTGS_FS_WKUP_Handler	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_STREAM7_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void SDIO_Handler       		(void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM0_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM1_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM2_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM3_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM4_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void OTG_FS_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM5_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM6_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_STREAM7_Handler		(void) __attribute__ ((weak, alias("Default_Handler")));
void USART6_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_EV_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void I2C3_ER_Handler	    	(void) __attribute__ ((weak, alias("Default_Handler")));
void FPU_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI4_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));
void SPI5_Handler	        	(void) __attribute__ ((weak, alias("Default_Handler")));



uint32_t vectors[] __attribute__((section(".isr_vector"))) = {
    STACK_START,    
    (uint32_t)Reset_Handler,
    (uint32_t)NMI_Handler,
    (uint32_t)HardFault_Handler,
    (uint32_t)MemManage_Handler,
    (uint32_t)BusFault_Handler,
    (uint32_t)UsageFault_Handler,
    0,
    0,
    0,
    0,
    (uint32_t)SVCAll_Handler,
    (uint32_t)DebugMoniter_Handler,
    0,
    (uint32_t)PendSV_Handler,
    (uint32_t)SysTick_Handler,
    (uint32_t)WWDG_Handler,
    (uint32_t)EXTI16PVD_Handler,
    (uint32_t)EXTI21_TAMP_STAMP_Handler,
    (uint32_t)EXTI22_RTC_WKUP_Handler,
    (uint32_t)Flash_Handler,
    (uint32_t)Rcc_Handler,
    (uint32_t)Ext10_Handler,
    (uint32_t)Ext11_Handler,
    (uint32_t)Ext12_Handler,
    (uint32_t)Ext13_Handler,
    (uint32_t)Ext14_Handler,
    (uint32_t)DMA1_Stream0_Handler,
    (uint32_t)DMA1_Stream1_Handler,
    (uint32_t)DMA1_Stream2_Handler,
    (uint32_t)DMA1_Stream3_Handler,
    (uint32_t)DMA1_Stream4_Handler,
    (uint32_t)DMA1_Stream5_Handler,
    (uint32_t)DMA1_Stream6_Handler,
    (uint32_t)ADC_Handler,
    (uint32_t)EXTI9_5_Handler,
    (uint32_t)TIM1_BRK_TIM9_Handler,
    (uint32_t)TIM1_UP_TIM10_Handler,
    (uint32_t)TIM1_TRG_COM_TIM11_Handler,
    (uint32_t)TIM1_CC_Handler,
    (uint32_t)TIM2_Handler,
    (uint32_t)TIM3_Handler,
    (uint32_t)TIM4_Handler,
    (uint32_t)I2C1_EV_Handler,
    (uint32_t)I2C1_ER_Handler,
    (uint32_t)I2C2_EV_Handler,
    (uint32_t)I2C2_ER_Handler,
    (uint32_t)SPI1_Handler,
    (uint32_t)SPI2_Handler,
    (uint32_t)USART1_Handler,
    (uint32_t)USART2_Handler,
    (uint32_t)EXTI15_10_Handler,
    (uint32_t)EXTI17_RTC_ALARM_Handler,
    (uint32_t)EXTI18_OTGS_FS_WKUP_Handler,
    (uint32_t)DMA1_STREAM7_Handler,
    (uint32_t)SDIO_Handler,
    (uint32_t)TIM5_Handler,
    (uint32_t)SPI3_Handler,
    (uint32_t)DMA2_STREAM0_Handler,
    (uint32_t)DMA2_STREAM1_Handler,
    (uint32_t)DMA2_STREAM2_Handler,
    (uint32_t)DMA2_STREAM3_Handler,
    (uint32_t)DMA2_STREAM4_Handler,
    (uint32_t)OTG_FS_Handler,
    (uint32_t)DMA2_STREAM5_Handler,
    (uint32_t)DMA2_STREAM6_Handler,
    (uint32_t)DMA2_STREAM7_Handler,
    (uint32_t)USART6_Handler,
    (uint32_t)I2C3_EV_Handler,
    (uint32_t)I2C3_ER_Handler,
    (uint32_t)FPU_Handler,
    (uint32_t)SPI4_Handler,
    (uint32_t)SPI5_Handler
    };

void Default_Handler(void)
{
    while(1);
}

void Reset_Handler(void)
{
    // copy .data section to SRAM
    uint32_t size =(uint32_t)&_edata - (uint32_t)&_sdata;

    uint8_t *pDst = (uint8_t*)&_sdata; // sram
    uint8_t *pSrc = (uint8_t*)&_etext; // 

    for(uint32_t i = 0; i < size; i++)
    {
        *pDst++ = *pSrc++;
    }
    // Init the .bss section to zero in SRAM
    size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    pDst = (uint8_t*)&_sbss;
    for(uint32_t i = 0; i < size; i++)
    {
        *pDst++ = 0; 
    }

    // call init function of std. library (if needed)

    // call main()
    main();
}
