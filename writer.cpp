#include <iostream>
#include <fstream>

int main(){
    std::string m_list[69] {
        "0", // STACK_START "Reset",
        "NMI",
        "HardFault",
        "MemManage",
        "BusFault",
        "UsageFault",
        "0",
        "SVCAll",
        "DebugMoniter",
        "0",
        "PendSV",
        "SysTick",
        "WWDG",
        "EXTI16PVD",
        "EXTI21_TAMP_STAMP",
        "EXTI22_RTC_WKUP",
        "Flash",
        "Rcc",
        "Ext10",
        "Ext11",
        "Ext12",
        "Ext13",
        "Ext14",
        "DMA1_Stream0",
        "DMA1_Stream1",
        "DMA1_Stream2",
        "DMA1_Stream3",
        "DMA1_Stream4",
        "DMA1_Stream5",
        "DMA1_Stream6",
        "ADC",
        "EXTI9_5",
        "TIM1_BRK_TIM9",
        "TIM1_UP_TIM10",
        "TIM1_TRG_COM_TIM11",
        "TIM1_CC",
        "TIM2",
        "TIM3",
        "TIM4",
        "I2C1_EV",
        "I2C1_ER",
        "I2C2_EV",
        "I2C2_ER",
        "SPI1",
        "SPI2",
        "USART1",
        "USART2",
        "EXTI15_10",
        "EXTI17_RTC_ALARM",
        "EXTI18_OTGS_FS_WKUP",
        "DMA1_STREAM7",
        "SDIO",
        "TIM5",
        "SPI3",
        "DMA2_STREAM0",
        "DMA2_STREAM1",
        "DMA2_STREAM2",
        "DMA2_STREAM3",
        "DMA2_STREAM4",
        "OTG_FS",
        "DMA2_STREAM5",
        "DMA2_STREAM6",
        "DMA2_STREAM7",
        "USART6",
        "I2C3_EV",
        "I2C3_ER",
        "FPU",
        "SPI4",
        "SPI5"
    };
    std::string proto;
    std::string vList;

    for(int i = 0; i < sizeof(m_list)/sizeof(m_list[0]) ; i++){
        if(m_list[i] == "0"){
            vList.append(m_list[i] + ",\n");
       } else {
           vList.append("(uint32_t)" + m_list[i] + "_Handler,\n");
           proto.append("void " + m_list[i] + "_Handler\t\t"
                   + "(void) __attribute__ ((weak, alias(\"Default_Handler\")));\n");
           //remove comma at end manually
        }
    }


    std::ofstream outfile;
    outfile.open ("writer_output.txt");
    outfile << proto;
    outfile << "\n\n";
    outfile << vList;
    outfile.close();

    return 0;

}
