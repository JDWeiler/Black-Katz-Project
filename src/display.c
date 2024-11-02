#include "stm32f0xx.h"

/**
 * PB3 : SCK
 * PB4 : SDI
 * PB5 : g/cx 
*/
void init_spi1_slow() {
  RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN; 
  RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

  GPIOB -> MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
  GPIOA -> MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1); //pa 3, 4, 5 alternate function 
  GPIOA -> AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
  
  SPI1 -> CR1 &= ~SPI_CR1_SPE;

  //set lowest baud rate = 48000000 / 256
  SPI1 -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;

  //8 bit data 
  SPI1 -> CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

  SPI1 -> CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;

  SPI1 -> CR2 |= SPI_CR2_FRXTH;

  SPI1 -> CR1 |= SPI_CR1_SPE;
}

void enable_sdcard() {
  GPIOB ->  BSRR |= 1 << 2 + 16;
}

void disable_sdcard() {
  GPIOB ->  BSRR |= 1 << 2;
}

void init_sdcard_io() {
  init_spi1_slow();
  RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
  
  GPIOA -> MODER |= GPIO_MODER_MODER2_0;

  disable_sdcard();
}

void sdcard_io_high_speed() {
  SPI1 -> CR1 &= ~SPI_CR1_SPE;

  SPI1 -> CR1 |= SPI_CR1_BR_0; //12MHz

  SPI1 -> CR1 |= SPI_CR1_SPE;
}

void init_lcd_display() {
  RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

  GPIOB -> BSRR |= (1 << 8 + 16) | (1 << 11 + 16) | (1 << 14 + 16);

  init_spi1_slow();
  sdcard_io_high_speed();
}