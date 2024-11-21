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
int countdown = 10;

// 96x96 dino
#define DINO_HEIGHT 96
#define DINO_WIDTH 96
#define CACTI_HEIGHT 48
#define CACTI_WIDTH 48
#define DINO_VELOCITY 6
#define CACTI_VELOCITY 5

//100 hz game refresh rate (i dont think we can actually do 100 hx though bc it takes to long to refresh stuff)
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 4799;
    TIM7->ARR = 499;
    TIM7->DIER |= TIM_DIER_UIE;

    NVIC -> ISER[0] |= 1 << TIM7_IRQn;

    // TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7 -> CR1 |= TIM_CR1_CEN;
}
void sound_on(){
    TIM3->CR1 |= TIM_CR1_CEN;
}
void sound_off(){
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

void refresh_game() {
    if(countdown == 0){
        game_over = 1;
        
        LCD_DrawPictureNew(0, 0, player2win, 240, 320);
        LCD_Clear(RED);
        sound_off();
    }
    if(dino_is_jumping && !game_over) {
        sound_on();
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
        
    } else if(!game_over){
        LCD_DrawPictureNew(0, dino_y, dino_bitmap, DINO_WIDTH, DINO_HEIGHT);
        sound_off();
    }

    if(cacti_exists && !game_over) {
        cacti_x -= CACTI_VELOCITY;
        update_cacti(cacti_bitmap, CACTI_WIDTH, CACTI_HEIGHT, cacti_x);
        
        // collision detection
        if(cacti_x <= 35 && cacti_x > 0) {
            if(dino_y >= 178) {
                game_over = 1; 
                cacti_exists = 0;
                LCD_DrawPictureNew(0, 0, player2win, 240, 320);
                sound_off();
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


void setup_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;      
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;    

    
    GPIOC->MODER &= ~(0x000FF000);           
    GPIOC->MODER |= (0x000AA000);            
    
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL6;       
    // Configure TIM3 for 4kHz PWM
    TIM3->PSC = 47;                  
    TIM3->ARR = 1923;                      

    // Set PWM mode 1 for TIM3 Channel 1 (PC6)
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);        // Clear the OC1M bits
    TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // Set PWM mode 1 for OC1
    TIM3->CCER |= TIM_CCER_CC1E;          

   
    TIM3->CCR1 = 200;                                 // Start the timer

}


//===========================================================================
// Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000, 0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700, 0x0800};
extern const char font[];


void small_delay(void) {
    nano_wait(50000);
}

void setup_bb(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  // Enable GPIOA clock

    // Configure PA4 (CS) as output
    GPIOA->MODER &= ~(0x3 << (4 * 2)); // Clear mode bits for PA4
    GPIOA->MODER |= (0x1 << (4 * 2)); // Set PA4 to output mode

    // Configure PA5 (SCK) as output
    GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear mode bits for PA5
    GPIOA->MODER |= (0x1 << (5 * 2)); // Set PA5 to output mode

    // Configure PA7 (SDI) as output
    GPIOA->MODER &= ~(0x3 << (7 * 2)); // Clear mode bits for PA7
    GPIOA->MODER |= (0x1 << (7 * 2)); // Set PA7 to output mode

    // Initialize GPIO outputs
    GPIOA->ODR |= (1 << 4);   // Set PA4 (CS) high
    GPIOA->ODR &= ~(1 << 5);  // Set PA5 (SCK) low
    GPIOA->ODR &= ~(1 << 7);  // Set PA7 (SDI) low
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================

void bb_write_bit(int val) {
    // Set SDI (PA7)
    if (val) {
        GPIOA->ODR |= (1 << 7);  // Set PA7 (SDI) high
    } else {
        GPIOA->ODR &= ~(1 << 7); // Set PA7 (SDI) low
    }
    small_delay();

    // Toggle SCK (PA5)
    GPIOA->ODR |= (1 << 5);   // Set PA5 (SCK) high
    small_delay();
    GPIOA->ODR &= ~(1 << 5);  // Set PA5 (SCK) low
}

//===========================================================================
// Set CS (PB12) low,
// write 16 bits using bb_write_bit,
// then set CS high.
//===========================================================================
void bb_write_halfword(int halfword) {
   GPIOA->ODR &= ~(1 << 4);  // Clear PA4 (CS)

    for (int i = 15; i >= 0; i--) {
        int bit = (halfword >> i) & 1;
        bb_write_bit(bit);
    }

    GPIOA->ODR |= (1 << 4);
}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void) {
    for(;;){
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(100000); // wait 1 ms between digits
        }
    }
}

//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 4 or 5.
//===========================================================================
/* void init_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 4799;
    TIM3->ARR = 99;

    TIM3 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] |= 1 << TIM3_IRQn;
    TIM3 -> CR1 = TIM_CR1_CEN;
}
 */
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->PSC = 47999;
    TIM6->ARR = 999;

    TIM6 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] |= 1 << TIM6_IRQn;
    TIM6 -> CR1 = TIM_CR1_CEN;
}


void update_display(int value) {
    int hundreds = value / 100;
    int tens = (value / 10) % 10;
    int ones = value % 10;
    
    msg[5] = (msg[5] & ~0x7F) | font[hundreds +'0']; 
    msg[6] = (msg[6] & ~0x7F) | font[tens +'0'];
    msg[7] = (msg[7] & ~0x7F) | font[ones +'0']; 

}

void TIM6_IRQHandler(void){
    TIM6->SR &= ~TIM_SR_UIF;
    if (countdown > 0) {
        countdown--;
    }
    update_display(countdown);
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
    init_tim6();
    init_tim7();
    setup_tim3();

    setup_bb();
    
    update_display(countdown);

    drive_bb();
    // LCD_DrawChar(0, 0, 0x0000, 0xffff, 'd', 16, 0);
    // for(;;) {}
}