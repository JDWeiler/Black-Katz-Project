#include "stm32f0xx.h"

/**
 * PA5 : SCK
 * PA6 : MISO
 * PA7 : MOSI 
*/
void init_spi1_slow() {
  RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

  GPIOA -> MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
  GPIOA -> MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); //pa5, 6, 7 alternate function 
  GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);

  SPI1 -> CR1 &= ~SPI_CR1_SPE;

  //set lowest baud rate = 48000000 / 256
  SPI1 -> CR1 &= ~(SPI_CR1_BR);
  SPI1 -> CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_BR_0;

  //8 bit data 
  SPI1 -> CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

  SPI1 -> CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

  SPI1 -> CR2 |= SPI_CR2_FRXTH;

  SPI1 -> CR1 |= SPI_CR1_SPE;
}

/**
 * reset PA8
*/
void enable_sdcard() {
  GPIOA -> BSRR |= 1 << (8 + 16);
}
/**
 * set PA8
*/
void disable_sdcard() {
  GPIOA ->  BSRR |= 1 << 8;
}

void init_sdcard_io() {
  init_spi1_slow();

  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA -> MODER &= ~GPIO_MODER_MODER8;
  GPIOA -> MODER |= GPIO_MODER_MODER8_0;

  disable_sdcard();
}

void sdcard_io_high_speed() {
  SPI1 -> CR1 &= ~SPI_CR1_SPE;

  SPI1 -> CR1 &= ~(SPI_CR1_BR);
  SPI1 -> CR1 |= SPI_CR1_BR_0; //12MHz

  SPI1 -> CR1 |= SPI_CR1_SPE;
}

void init_lcd_display() {
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;

  GPIOA -> MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13);
  GPIOA -> MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;

  init_spi1_slow();
  sdcard_io_high_speed();
}
/**
 * PA11 : DC
 * PA12 : RESET
 * PA13 : LCD CS
*/
void init_lcd_spi() {
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA -> MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13);
  GPIOA -> MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
  GPIOA -> BSRR |= 1 << (13);
  init_spi1_slow();
  sdcard_io_high_speed();
}


//THIS IS FOR GPIOB ----------------------------------------------------
// #include "stm32f0xx.h"

// /**
//  * PA5 : SCK
//  * PA6 : MISO
//  * PA7 : MOSI 
// */
// void init_spi1_slow() {
//   RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
//   RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

//   GPIOB -> MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
//   GPIOB -> MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1); //pa3, 4, 5 alternate function 
//   GPIOB -> AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
  
//   SPI1 -> CR1 &= ~SPI_CR1_SPE;

//   //set lowest baud rate = 48000000 / 256
//   SPI1 -> CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_BR_0;

//   //8 bit data 
//   SPI1 -> CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

//   SPI1 -> CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

//   SPI1 -> CR2 |= SPI_CR2_FRXTH;

//   SPI1 -> CR1 |= SPI_CR1_SPE;
// }

// /**
//  * reset PB2
// */
// void enable_sdcard() {
//   GPIOB -> BSRR |= 1 << (2 + 16);
// }
// /**
//  * set PB2
// */
// void disable_sdcard() {
//   GPIOB ->  BSRR |= 1 << 2;
// }

// void init_sdcard_io() {
//   init_spi1_slow();

//   RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
//   GPIOB -> MODER &= ~GPIO_MODER_MODER2;
//   GPIOB -> MODER |= GPIO_MODER_MODER2_0;

//   disable_sdcard();
// }

// void sdcard_io_high_speed() {
//   SPI1 -> CR1 &= ~SPI_CR1_SPE;

//   SPI1 -> CR1 &= ~(SPI_CR1_BR);
//   SPI1 -> CR1 |= SPI_CR1_BR_0; //12MHz

//   SPI1 -> CR1 |= SPI_CR1_SPE;
// }

// void init_lcd_display() {
//   RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

//   GPIOB -> MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
//   GPIOB -> MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0;

//   init_spi1_slow();
//   sdcard_io_high_speed();
// }

// /**
//  * PB14 : DC
//  * PB11 : RESET
//  * PB8 : LCD CS
// */
// void init_lcd_spi() {
//   RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

//   GPIOB -> MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER8 | GPIO_MODER_MODER14);
//   GPIOB -> MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER14_0;

//   GPIOB -> BSRR |= 1 << 8;
  
//   init_spi1_slow();
//   sdcard_io_high_speed();
// }