#include "stm32f0xx.h"

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}


/**
 * PA5 : SCK
 * PA7 : SDI
 * PA9 : g/cx 
 * PA11 : reset (active low reset)
 * PA15 : cs
*/
void init_spi1() {
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

    //   automatic cs (hardware nss)
    // GPIOA -> MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER9 | GPIO_MODER_MODER11);
    // GPIOA -> MODER |= (GPIO_MODER_MODER15_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); //alternate function
    // GPIOA -> MODER |= GPIO_MODER_MODER9_0 | GPIO_MODER_MODER11_0; //output
    // GPIOA -> AFR[1] &= ~GPIO_AFRH_AFSEL15;
    // GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
    // GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR11_0;

    //manually controlling cs
    GPIOA -> MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER9 | GPIO_MODER_MODER11);
    GPIOA -> MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); //pa 5 and 7 alternate function 
    GPIOA -> MODER |= GPIO_MODER_MODER15_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER11_0; //pa 15, 9, 11 output
    GPIOA -> AFR[1] &= ~GPIO_AFRH_AFSEL15;
    GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
    GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR11_0;
    SPI1 -> CR1 &= ~SPI_CR1_SPE;

    //set lowest baud rate = 48000000 / 256
    SPI1 -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;

    //16 bits/pixel communication
    //8 bit data (pg 26 datasheet)
    SPI1 -> CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

    SPI1 -> CR1 |= SPI_CR1_MSTR;
    // SPI1 -> CR2 |= SPI_CR2_NSSP | SPI_CR2_SSOE; //only needed for auto cs
    // SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);  // only needed for auto cs

    SPI1 -> CR2 |= SPI_CR2_TXDMAEN;

    SPI1 -> CR1 |= SPI_CR1_SPE;
}


int main() {

}