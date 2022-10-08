target remote localhost:3333
monitor reset init
monitor flash write_image erase final.elf
monitor reset halt
file final.elf
monitor halt

define NVIC_ISER
p "ISER0 - ISER7"
monitor mdw 0xE000E100 8
end

define RCC
p "CR PLLCFGR CFGR CIR AHB1RSTR AHB2RSTR RESERVE1 RESERVED2 APB1RSTR APB2RSTR RESEVED3 RESERVED4 AHB1ENR AHB2ENR RESERVED RESERVED5 APB1ENR APB2ENR RESERVED6 RESERVED7 AHB1LPENR AHB2LPENR RESERVED8 RESERVED9 APB1LPENR APB2LPENR REERVED10 RESERVED11 BDCR CSR RESERVED12 RESERVED13 SSCGR PLLI2SCFGR RESERVED14 DCKCFGR"
monitor mdw 0x40023800 36
end

define EXTI
p "IMR EMR RTSR FTSR SWIER PR"
monitor mdw 0x40013C00 6
end

define SYSCFG
p "MEMRMP PMC EXTICR1 EXTICR2 EXTICR3 EXTICR4 RESERVED1? RESERVED2? CMPCR"
monitor mdw 0x40013800 9
end

define GPIOA
p "MODER OTYPER OSPEEDR PUPDR IDR ODR BSRR LCKR AFRL AFRH"
monitor mdw 0x40020000 10
end

define GPIOB
p "MODER OTYPER OSPEEDR PUPDR IDR ODR BSRR LCKR AFRL AFRH"
monitor mdw 0x40020400 10
end

define GPIOC
p "MODER OTYPER OSPEEDR PUPDR IDR ODR BSRR LCKR AFRL AFRH"
monitor mdw 0x40020800 10
end

define GPIOD
p "MODER OTYPER OSPEEDR PUPDR IDR ODR BSRR LCKR AFRL AFRH"
monitor mdw 0x40020C00 10
end


