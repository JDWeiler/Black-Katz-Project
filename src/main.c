#include "stm32f0xx.h"

void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

void init_spi1() {
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

//   //set pa5(SCK), pa7(SDI), and pa15(CS) to alternate mode and pa9(g/cx) to output
  GPIOA -> MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER9);
  GPIOA -> MODER |= (GPIO_MODER_MODER15_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); 
  GPIOA -> MODER |= GPIO_MODER_MODER9_0;
  GPIOA -> AFR[1] &= ~GPIO_AFRH_AFSEL15;
  GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);

    //manually controlling pa15(cs)
    // GPIOA -> MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER9);
    // GPIOA -> MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1); 
    // GPIOA -> MODER |= GPIO_MODER_MODER15_0 | GPIO_MODER_MODER9_0;
    // GPIOA -> AFR[1] &= ~GPIO_AFRH_AFSEL15;
    // GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);

    SPI1 -> CR1 &= ~SPI_CR1_SPE;

    //set lowest baud rate = 48000000 / 256
    SPI1 -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;

    //16 bits/pixel communication
    //8 bit data (pg 26 datasheet)
    SPI1 -> CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_3;

    SPI1 -> CR1 |= SPI_CR1_MSTR;
    SPI1 -> CR2 |= SPI_CR2_NSSP | SPI_CR2_SSOE; //might not need...manual CS could be better than hardware NSS
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);  // Set SPI mode 0

    SPI1 -> CR2 |= SPI_CR2_TXDMAEN;

    SPI1 -> CR1 |= SPI_CR1_SPE;
}

void pull_dcx_high() {
    GPIOA -> BSRR |= 1 << 9;
}

void pull_dcx_low() {
    GPIOA -> BSRR |= 1 << (9 + 16);
}

//only need if we wanna do manual cs 
// void set_cs_high() {
//     GPIOA -> BSRR |= 1 << 15;
// }

// void set_cs_low() {
//     GPIOA -> BSRR |= 1 << (15 + 16);
// }

/**
 * pull d/cx (pa9) pin low to activate comamnds
*/
void spi_cmd(unsigned int cmd) {
    // set_cs_low();
    pull_dcx_low();

    while(!(SPI1 -> SR & SPI_SR_TXE));

    SPI1 -> DR = cmd;

    while (SPI1->SR & SPI_SR_BSY);
    // set_cs_high();
}

/**
 * pull d/cx pin high to activate data transmission
*/
void spi_data(unsigned int data) {
    // set_cs_low();
    pull_dcx_high();

    while(!(SPI1 -> SR & SPI_SR_TXE));

    SPI1 -> DR = (data >> 8) & 0xFF; //first 8 bits
    while (SPI1 -> SR & SPI_SR_BSY);
    
    SPI1 -> DR = data & 0xFF; //second 8 bits
    while (SPI1 -> SR & SPI_SR_BSY);

    // set_cs_high();
}

void init_LCD() {
    nano_wait(1000000);

    spi_cmd(0x01); //software reset
    nano_wait(120000000); //120 ms 

    spi_cmd(0x11); //sleep out mode
    nano_wait(5000000); //5 ms
    
    spi_cmd(0x36); //memory access control command
    spi_data(0x48); //everything set to 0 
    // OR 
    //spi_data(0x3600);

    spi_cmd(0x3A); //pixel format set
    spi_data(0x55); //set DPI and DBI = 16 bits/pixel
    //OR
    //spi_data(3A55);

    spi_cmd(0x29); //display ON
    nano_wait(5000000);
}

void fill_screen(uint16_t color) {
    spi_cmd(0x2A);  // Column address set
    spi_data(0x00);  // Start at 0
    spi_data(0x00);  // Start at 0
    spi_data(0x00);  // End at 239 (for a 240x320 display)
    spi_data(0xEF);  // End at 239 (for a 240x320 display)

    spi_cmd(0x2B);  // Row address set
    spi_data(0x00);  // Start at 0
    spi_data(0x00);  // Start at 0
    spi_data(0x01);  // End at 319
    spi_data(0x3F);  // End at 319

    spi_cmd(0x2C);  // Memory write

    for (int i = 0; i < 240 * 320; i++) {
        spi_data(color);  // Fill with the chosen color
    }
}

int main() {
    init_spi1();
    init_LCD(); 
    
    //display_hello();
    fill_screen(0x000F);

    while (1);  
}