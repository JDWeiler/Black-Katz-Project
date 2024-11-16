#include "stm32f0xx.h"
#include "../include/commands.h"
#include "../include/lcd.h"
#include "../include/fifo.h"
#include "../include/tty.h"
#include <stdio.h>


/**
 * 
 * 
 * 
 * FROM LAB 7 -------------------------------------------
 * 
 * 
 * 
*/

void internal_clock();

void init_usart5() {
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    GPIOC -> MODER &= ~GPIO_MODER_MODER12;
    GPIOC -> MODER |= GPIO_MODER_MODER12_1;

    GPIOD -> MODER &= ~GPIO_MODER_MODER2;
    GPIOD -> MODER |= GPIO_MODER_MODER2_1;

    GPIOC -> AFR[1] &= 0xFFF0FFFF;
    GPIOC -> AFR[1] |= 0x00020000;
    GPIOD -> AFR[0] &= 0xFFFFF0FF;
    GPIOD -> AFR[0] |= 0x00000200;

    RCC -> APB1ENR |= RCC_APB1ENR_USART5EN;

    USART5 -> CR1 &= ~USART_CR1_UE;

    USART5 -> CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);

    USART5 -> CR2 &= ~USART_CR2_STOP;

    USART5 -> CR1 &= ~USART_CR1_PCE;

    USART5 -> CR1 &= ~USART_CR1_OVER8;

    USART5 -> BRR = 0x1A1;

    USART5 -> CR1 |= USART_CR1_TE | USART_CR1_RE;

    USART5 -> CR1 |= USART_CR1_UE;

    while(!((USART5 -> ISR & USART_ISR_REACK) && (USART5 -> ISR & USART_ISR_TEACK))) {}
}

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    // TODO
    USART5 -> CR1 |= USART_CR1_RXNEIE;
    USART5 -> CR3 |= USART_CR3_DMAR;
    NVIC -> ISER[0] |= 1 << 29;

    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off
    
    DMA2_Channel2 -> CMAR = &serfifo;
    DMA2_Channel2 -> CPAR = &(USART5 -> RDR);
    DMA2_Channel2 -> CNDTR = FIFOSIZE;

    DMA2_Channel2 -> CCR &= ~DMA_CCR_DIR;
    DMA2_Channel2 -> CCR &= ~(DMA_CCR_TEIE | DMA_CCR_HTIE);
    DMA2_Channel2 -> CCR |= DMA_CCR_MINC;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_PINC;
    DMA2_Channel2 -> CCR |= DMA_CCR_CIRC;
    DMA2_Channel2 -> CCR &= ~DMA_CCR_MEM2MEM;
    DMA2_Channel2 -> CCR |= DMA_CCR_PL;

    DMA2_Channel2->CCR |= DMA_CCR_EN;

}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {

    // Wait for a newline to complete the buffer.
    while(fifo_newline(&input_fifo) == 0) {
        asm volatile ("wfi");
    }
    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
    // TODO
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5 -> TDR = '\r';
    }

    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
    return interrupt_getchar();
}

void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}






static const unsigned char dino_bitmap[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x06, 0xff, 0x00, 0x00, 0x0e, 0xff, 0x00,
    0x00, 0x0f, 0xff, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x0f, 0xc0, 0x00,
    0x00, 0x0f, 0xfc, 0x00, 0x40, 0x0f, 0xc0, 0x00, 0x40, 0x1f, 0x80, 0x00, 0x40, 0x7f, 0x80, 0x00,
    0x60, 0xff, 0xe0, 0x00, 0x71, 0xff, 0xa0, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x7f, 0xff, 0x80, 0x00,
    0x7f, 0xff, 0x80, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x0f, 0xfe, 0x00, 0x00,
    0x03, 0xfc, 0x00, 0x00, 0x01, 0xdc, 0x00, 0x00, 0x01, 0x8c, 0x00, 0x00, 0x01, 0x8c, 0x00, 0x00,
    0x01, 0x0c, 0x00, 0x00, 0x01, 0x8e, 0x00, 0x00
};

void LCD_DrawBitmap(int x, int y, const unsigned char *bitmap, int width, int height, uint16_t color) {
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (bitmap[i * (width / 8) + (j / 8)] & (128 >> (j & 7))) {

            
                LCD_DrawPoint(x + j, y + i, color);
            }
        }
    }
}


int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    

    LCD_Setup();
    LCD_Clear(0x0000);
    // LCD_DrawPoint(0, 200, 0xffff);
    // LCD_DrawFillRectangle(0, 0, 200, 300, 0xffff);

    // LCD_DrawChar(60, 150, 0x0000, 0xffff, 'B', 16, 0);
    // LCD_DrawChar(80, 150, 0x0000, 0xffff, 'L', 16, 0);
    // LCD_DrawChar(100, 150, 0x0000, 0xffff, 'A', 16, 0);
    // LCD_DrawChar(120, 150, 0x0000, 0xffff, 'C', 16, 0);
    // LCD_DrawChar(140, 150, 0x0000, 0xffff, 'K', 16, 0);
    // LCD_DrawChar(160, 150, 0x0000, 0xffff, 'C', 16, 0);
    // LCD_DrawChar(180, 150, 0x0000, 0xffff, 'A', 16, 0);
    // LCD_DrawChar(200, 150, 0x0000, 0xffff, 'T', 16, 0);

    // LCD_DrawRectangle(0, 0, 200, 200, 0x0f0f);
    LCD_DrawBitmap(0, 0, dino_bitmap, 200, 200, 0x0000);
    // command_shell();

  
}
