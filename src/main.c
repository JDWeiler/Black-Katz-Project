/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 3, 2024
  * @brief   ECE 362 Lab 6 Student template
  ******************************************************************************
*/


#include "stm32f0xx.h"

void set_char_msg(int, char);
void nano_wait(unsigned int);
void game(void);
void internal_clock();
void check_wiring();
void autotest();

//===========================================================================
// Configure GPIOC
//===========================================================================



uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000, 0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700, 0x0800};
extern const char font[];
int countdown = 120;
//===========================================================================
// Configure PB12 (CS), PB13 (SCK), and PB15 (SDI) for outputs
//===========================================================================
void setup_bb(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(0x3 << (12 * 2)); 
    GPIOB->MODER |= (0x1 << (12 * 2));   

    GPIOB->MODER &= ~(0x3 << (13 * 2));  
    GPIOB->MODER |= (0x1 << (13 * 2));   

    GPIOB->MODER &= ~(0x3 << (15 * 2));  
    GPIOB->MODER |= (0x1 << (15 * 2));  

    GPIOB->ODR |= (1 << 12);   
    GPIOB->ODR &= ~(1 << 13);  
    GPIOB->ODR &= ~(1 << 15);
}

void small_delay(void) {
    nano_wait(50000);
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================
void bb_write_bit(int val) {
    // CS (PB12)
    // SCK (PB13)
    // SDI (PB15)
    if (val) {
        GPIOB->ODR |= (1 << 15);  
    } else {
        GPIOB->ODR &= ~(1 << 15);  
    }
    small_delay();

    
    GPIOB->ODR |= (1 << 13);
    small_delay();

    
    GPIOB->ODR &= ~(1 << 13);
}

//===========================================================================
// Set CS (PB12) low,
// write 16 bits using bb_write_bit,
// then set CS high.
//===========================================================================
void bb_write_halfword(int halfword) {
    GPIOB->ODR &= ~(1 << 12);  // Clear PB12 (CS)

    for (int i = 15; i >= 0; i--) {
        int bit = (halfword >> i) & 1;
        bb_write_bit(bit);
    }

    GPIOB->ODR |= (1 << 12);
}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void) {
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(100000); // wait 1 ms between digits
        }
}

//============================================================================
// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
// Copy this from lab 4 or lab 5.
//============================================================================
void init_tim15(void) {
    RCC -> APB2ENR |= RCC_APB2ENR_TIM15EN;

    TIM15 -> PSC = 47;
    TIM15 -> ARR = 999;
    
    TIM15 -> DIER |= TIM_DIER_UDE;
    TIM15 -> CR1 = TIM_CR1_CEN;
}


//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 4 or 5.
//===========================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->PSC = 47999;
    TIM7->ARR = 999;

    TIM7 -> DIER |= TIM_DIER_UIE;
    NVIC -> ISER[0] |= 1 << TIM7_IRQn;
    TIM7 -> CR1 = TIM_CR1_CEN;

}


void update_display(int value) {
    int hundreds = value / 100;
    int tens = (value / 10) % 10;
    int ones = value % 10;
    
    msg[5] = (msg[5] & ~0x7F) | font[hundreds +'0'];
    msg[6] = (msg[6] & ~0x7F) | font[tens +'0'];
    msg[7] = (msg[7] & ~0x7F) | font[ones +'0'];

}

void TIM7_IRQHandler(void){
    TIM7->SR &= ~TIM_SR_UIF;
    if (countdown > 0) {
        countdown--;
    }
    update_display(countdown);
}

void init_buttons(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

   // pa0 input with pull-up
    GPIOA->MODER &= ~(0x3 << (0 * 2)); 
    GPIOA->PUPDR &= ~(0x3 << (0 * 2));  
    GPIOA->PUPDR |= (0x1 << (0 * 2));   

    // pa1 as input with pull-up
    GPIOA->MODER &= ~(0x3 << (1 * 2));  
    GPIOA->PUPDR &= ~(0x3 << (1 * 2));  
    GPIOA->PUPDR |= (0x1 << (1 * 2));   
}

//is jump button pressed??
uint8_t is_jump_button_pressed(void) {
    return !(GPIOA->IDR & (1 << 0));  //1 press
}

//dino send
uint8_t is_obstacle_button_pressed(void) {
    return !(GPIOA->IDR & (1 << 1));  //1 press
}

void setup_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~(0x3 << (6 * 2));
    GPIOC->MODER |= (0x2 << (6 * 2));
    GPIOC->AFR[0] &= ~(0xF << (6 * 4));

    TIM3->PSC = 47999;
    TIM3->ARR = 999;
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |= (0x6 << 4);
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void play_buzzer(void) {
    TIM3->CCR1 = 500;
}

void stop_buzzer(void) {
    TIM3->CCR1 = 0;
}
void pl_buzzer(void) {
    // Simulate the buzzer by blinking an LED (for testing)
    GPIOC -> ODR |= (1 << 8);  // Turn on LED (or buzzer in the future)
    nano_wait(1000000);       // Delay to simulate the sound duration
    //GPIOC->ODR &= ~(1 << 7); // Turn off LED (or buzzer in the future)
    nano_wait(1000000);       // Delay between blinks
}
//===========================================================================
// Main function
//===========================================================================

int main(void) {
     internal_clock();
    pl_buzzer();
    // Initialize ports and timers
    setup_bb();
    init_tim7();
    init_buttons();
    setup_tim3();
    
    // Display the initial countdown value
    update_display(countdown);
    drive_bb();
}
