#include "stm32f0xx.h"
#include "../include/commands.h"
#include "../include/lcd.h"
#include "../include/fifo.h"
#include "../include/tty.h"
#include "bitmaps.c"
// #include "../include/movement.h"
// #include "bitmaps.c"
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


// NEW STUFF NOT FROM LAB 7

int dino_y = 224; // starting height
//TODO :: REPLACE THIS WITH BUTTON INTERRUPT FLAG
int dino_is_jumping = 0; //0 if dino not jumping, 1 if dino is jumping...
int direction = 1; // 0 = down, 1 = up
int cacti_exists = 0;
int cacti_x = 180; // starting x value
int game_over = 0; // 0 = game on ; 1 = game over

// 96x96 dino
#define DINO_HEIGHT 96
#define DINO_WIDTH 96
#define CACTI_HEIGHT 48
#define CACTI_WIDTH 48
#define DINO_VELOCITY 3
#define CACTI_VELOCITY 6

//100 hz game refresh rate (i dont think we can actually do 100 hx though bc it takes to long to refresh stuff)
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4799;
    TIM7->ARR = 99;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC -> ISER[0] = 1 << TIM7_IRQn;

    // TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7 -> CR1 |= TIM_CR1_CEN;
}

void refresh_game() {
    if(dino_is_jumping) {
        if(dino_y <= 120) direction = 0;

        if(direction) { 
            dino_y -= DINO_VELOCITY;
        } else {
            dino_y += DINO_VELOCITY;
        }

        update_dino(dino_bitmap, DINO_WIDTH, DINO_HEIGHT, dino_y, direction);
    
        if(dino_y == 224) {   
            //catch the dino once its done jumping to prevent it from jumping again
            dino_is_jumping = 0; 
            direction = 1;
        }
    } else {
        LCD_DrawPictureNew(0, dino_y, dino_bitmap, DINO_WIDTH, DINO_HEIGHT);
    }

    if(cacti_exists) {
        cacti_x -= CACTI_VELOCITY;
        update_cacti(cacti_bitmap, CACTI_WIDTH, CACTI_HEIGHT, cacti_x);
        
        // collision detection
        if(cacti_x <= 65 && cacti_x > 0) {
            if(!dino_is_jumping) {
                game_over = 1; 
                cacti_exists = 0;
                LCD_DrawPictureNew(0, 0, player2win, 240, 320);
            } else {
                cacti_x -= CACTI_VELOCITY;
                update_cacti(cacti_bitmap, CACTI_WIDTH, CACTI_HEIGHT, cacti_x);
            }
        }

        if(cacti_x <= 0) {
            cacti_x = 180;
            cacti_exists = 0;
            LCD_DrawFillRectangle(0, 272, 48, 320, 0x0000);
        }
    }
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF && !game_over) {
        TIM7->SR &= ~TIM_SR_UIF;
        refresh_game();
    }
}

void togglexn(GPIO_TypeDef *port, int n) {
  if(port->ODR & (1 << n)) { //change 1 to 0
    port->ODR &= ~(1 << n);
  } else{ //change 0 to 1
    port->ODR |= (1 << n);
  } 
}

// FROM LAB 2 FOR THE BUTTON TRIGGERED INTERRUPTS
void initb() {
    RCC-> AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB -> MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER2 | GPIO_MODER_MODER6); // set pins 0,2 as input
    GPIOB -> MODER |= GPIO_MODER_MODER6_0;
    GPIOB -> PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR2); // reset pupdr
    // GPIOB -> PUPDR |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR2_1; // pull down
}

void init_exti() {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;

  EXTI->RTSR |= EXTI_RTSR_TR0;
  EXTI->RTSR |= EXTI_RTSR_TR2;

  EXTI->IMR |= EXTI_IMR_IM0;
  EXTI->IMR |= EXTI_IMR_IM2;

  NVIC->ISER[0] |= 1<<5;
  NVIC->ISER[0] |= 1<<6;

}

void EXTI0_1_IRQHandler() {
  EXTI->PR = EXTI_PR_PR0;
  dino_is_jumping = 1;
//   dino_y = 224;
  togglexn(GPIOB, 6);
}

void EXTI2_3_IRQHandler() {
  EXTI->PR = EXTI_PR_PR2;
  cacti_exists = 1;
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

    initb();
    init_exti();
    init_tim7();

    // LCD_DrawChar(0, 0, 0x0000, 0xffff, 'd', 16, 0);
    // for(;;) {}
}