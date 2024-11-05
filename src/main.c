#include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>

#define COUNT 120

char disp[9] = "        ";
int countdown_value = COUNT;
uint8_t col = 0;

void init_tim7(void);
void TIM7_IRQHandler(void);
void show_char(int n, char c);
void drive_column(int c);

extern uint8_t font[];

void init_tim7(void) {
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    // Set prescaler and auto-reload values to generate 1 second interval
    TIM7->PSC = 47999;   // Prescaler to slow down timer clock
    TIM7->ARR = 999;     // Auto-reload value to generate 1-second interval
    TIM7->DIER |= TIM_DIER_UIE; // Enable update interrupt
    NVIC_EnableIRQ(TIM7_IRQn);  // Enable TIM7 interrupt in NVIC
    TIM7->CR1 |= TIM_CR1_CEN;   // Start TIM7
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {  // Check if update interrupt flag is set
        TIM7->SR &= ~TIM_SR_UIF;  // Clear update interrupt flag
        
        // Update countdown value
        if (countdown_value > 0) {
            countdown_value--;
        }
        
        // Update display string
        snprintf(disp, 9, " %4d   ", countdown_value);

        // Show character on 7-segment display
        char current_char = disp[col];
        show_char(col, current_char);
        col = (col + 1) % 8; // Move to the next column
        drive_column(col);
    }
}

void show_char(int n, char c) {
    if (n < 0 || n > 7) return;
    // Assuming GPIOB is used to drive segments
    GPIOB->ODR = font[(int)c] | (n << 8);
}

void drive_column(int c) {
    // Assuming GPIOC is used to drive column selection
    c &= 0x3;
    GPIOC->BRR = (0xF << 4);  // Clear all columns
    GPIOC->BSRR = (1 << (c + 4));  // Set the current column
}

int main(void) {
    // Initialize Timer 7
    init_tim7();
    
    // Main loop (does nothing, all work is done in interrupt)
    while (1) {
        // Idle loop
    }
    
    return 0;
}