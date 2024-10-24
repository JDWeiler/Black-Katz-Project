#include "stm32f0xx.h"


void init_spi2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;   

    //migiht need to add miso pb14

    GPIOB->MODER &= ~((0x3 << (12 * 2)) | (0x3 << (13 * 2)) | (0x3 << (15 * 2)));  
    GPIOB->MODER |= (0x2 << (12 * 2)) | (0x2 << (13 * 2)) | (0x2 << (15 * 2));  

    // PB12, PB13, and PB15 to AF0 (SPI2)
    GPIOB->AFR[1] &= ~((0xF << ((12 - 8) * 4)) | (0xF << ((13 - 8) * 4)) | (0xF << ((15 - 8) * 4)));
    GPIOB->AFR[1] |= (0x0 << ((12 - 8) * 4)) | (0x0 << ((13 - 8) * 4)) | (0x0 << ((15 - 8) * 4));  

    SPI2->CR1 &= ~SPI_CR1_SPE;

    SPI2->CR1 |= (0x7 << 3);  //might need to change baud rate

    SPI2->CR1 |= SPI_CR1_MSTR;

    // 16-bit
    SPI2->CR2 |= (0xF << 8);

    SPI2->CR2 |= SPI_CR2_SSOE;

    SPI2->CR2 |= SPI_CR2_NSSP;

    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    SPI2->CR1 |= SPI_CR1_SPE;
}


//added this just becuz
void spi2_setup_dma(void) {
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    

    DMA1_Channel5 -> CCR &= ~DMA_CCR_EN;

    DMA1_Channel5 -> CMAR = 6; //CMAR IDRK
    DMA1_Channel5 -> CPAR = (uint32_t) &(SPI2 -> DR);

    DMA1_Channel5 -> CNDTR = 8;

    DMA1_Channel5 -> CCR |= DMA_CCR_DIR;

    DMA1_Channel5 -> CCR |= DMA_CCR_MINC;
    
    DMA1_Channel5 -> CCR &= ~((0x3) << (10));
    DMA1_Channel5 -> CCR |= (0x1) << (10); 
    DMA1_Channel5 -> CCR &= ~((0x3) << (8));
    DMA1_Channel5 -> CCR |= (0x1) << (8);
    DMA1_Channel5 -> CCR |= DMA_CCR_CIRC;

    SPI2->CR2 |= (0x1) << 1;
}

void spi2_enable_dma(void) {
    DMA1_Channel5 -> CCR |= DMA_CCR_EN;
}

void init_spi1() { 
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~((0x3 << (15 * 2)) | (0x3 << (5 * 2)) | (0x3 << (7 * 2)));  // Clear 
    GPIOA->MODER |= (0x2 << (15 * 2)) | (0x2 << (5 * 2)) | (0x2 << (7 * 2));

    GPIOA->AFR[1] &= ~((0xF << ((15 - 8) * 4)));
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (7 * 4)));

    /* GPIOA->AFR[1] |= (0x0 << ((15 - 8) * 4));  
    GPIOA->AFR[0] |= (0x0 << (5 * 4)) | (0x0 << (7 * 4)); */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;

    SPI1->CR1 |= (0x7 << 3); 
    SPI1->CR1 |= SPI_CR1_MSTR;
    
    SPI1->CR2 |= (0x9 << 8); 
    SPI1->CR2 &= ~(0x6 << 8);

    SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_spi1(void) {
    
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;   
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;  // Enable SPI1 clock

    // SPI1 (PA5: SCK, PA7: MOSI)
    GPIOA->MODER &= ~((0x3 << (5 * 2)) | (0x3 << (7 * 2)));  
    GPIOA->MODER |= (0x2 << (5 * 2)) | (0x2 << (7 * 2));     

    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (7 * 4)));    

    //DEPENDS ON DISPLAY
    SPI1->CR1 &= ~SPI_CR1_SPE;        
    SPI1->CR1 |= SPI_CR1_MSTR;        
    SPI1->CR1 |= (0x7 << 3);          

    // If  display doesn’t need the clock to idle high or sample data on the second clock edge get rid fr 
    SPI1->CR1 |= SPI_CR1_CPOL;        
    SPI1->CR1 |= SPI_CR1_CPHA;        

    SPI1->CR2 |= (0xF << 8);  // 8 bit or 16 bit??        
    SPI1->CR2 |= SPI_CR2_SSOE;        

    SPI1->CR1 |= SPI_CR1_SPE;         

}

void spi_send_command(uint8_t cmd) {
    while (!(SPI1->SR & SPI_SR_TXE)); // Wait transmit buffer is empty
    SPI1->DR = cmd;                   // Send command
    while (SPI1->SR & SPI_SR_BSY);    // Wait SPI not longer busy
}