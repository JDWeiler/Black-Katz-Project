#include "stm32f0xx.h"

#define MAX_OBSTACLES 4
#define WIDTH 128
#define COUNT 120

uint8_t font[] = {
    0x3F,  // '0' -> 0b00111111 (a, b, c, d, e, f)
    0x06,  // '1' -> 0b00000110 (b, c)
    0x5B,  // '2' -> 0b01011011 (a, b, g, e, d)
    0x4F,  // '3' -> 0b01001111 (a, b, g, c, d)
    0x66,  // '4' -> 0b01100110 (f, g, b, c)
    0x6D,  // '5' -> 0b01101101 (a, f, g, c, d)
    0x7D,  // '6' -> 0b01111101 (a, f, g, e, d, c)
    0x07,  // '7' -> 0b00000111 (a, b, c)
    0x7F,  // '8' -> 0b01111111 (a, b, c, d, e, f, g)
    0x6F,  // '9' -> 0b01101111 (a, b, c, d, f, g)
    0x77,  // 'A' -> 0b01110111 (a, b, c, e, f, g)
    0x7C,  // 'B' -> 0b01111100 (c, d, e, f, g)
    0x39,  // 'C' -> 0b00111001 (a, d, e, f)
    0x5E,  // 'D' -> 0b01011110 (b, c, d, e, g)
    0x79,  // 'E' -> 0b01111001 (a, d, e, f, g)
    0x71,  // 'F' -> 0b01110001 (a, e, f, g)
    0x00   // ' ' -> 0b00000000 (All segments off)
};

char disp[9] = "        ";

void init_spi2(void);
void init_tim7(void);
void setup_tim3(void);
void play_buzzer(void);
void stop_buzzer(void);
void show_char(int n, char c);
void drive_column(int c);
void write_display(void);
void update_obstacles(void);
void update_dino_position(void);
void check_game_time(void);
void TIM7_IRQHandler(void);

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

///
///
///
///
///
///
/// TIMER
///
///
///
///
///
///

// Timer 7 keeps track of regular stuff and the 2 min thing
// Tim 15 is slower 
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = 47;
    TIM7->ARR = 999;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
}

int countdown_value = COUNT;
uint8_t col = 0;
void TIM7_IRQHandler(void) {
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;
        update_obstacles();
        update_dino_position();
        check_game_time();

        if (countdown_value > 0) {
            countdown_value--;
        }

        snprintf(disp, 9, " %4d   ", countdown_value);


        char current_char = disp[col];
        show_char(col, current_char);
        col = (col + 1) % 8; 
        drive_column(col);
    }
}


void check_game_time(void) {
    if (countdown_value <= 0) {
        handle_game_over();
    }
}

void handle_game_over(void) {
    snprintf(disp, 9, "GAMEOVER");

    while (1) {
       //restart
    }
}

void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    TIM15->PSC = 4799;
    TIM15->ARR = 999;
    TIM15->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM15_IRQn);
    TIM15->CR1 |= TIM_CR1_CEN;
}

void TIM15_IRQHandler(void) {
    if (TIM15->SR & TIM_SR_UIF) {
        TIM15->SR &= ~TIM_SR_UIF;
        update_obstacles();
    }
}


void show_char(int n, char c) {
    if (n < 0 || n > 7) return;
    GPIOB->ODR = font[(int)c] | (n << 8);
}

void drive_column(int c) {
    c &= 0x3;
    GPIOC->BRR = (0xF << 4);
    GPIOC->BSRR = (1 << (c + 4));
}

void write_display() {
    snprintf(disp, 9, " %4d   ", countdown_value);
}
///
///
///
///
///
///
/// GPIO
///
///
///
///
///
///

void init_buttons(void) {
    // i used pa0 and pa1 
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

// DID IT SEND OBSTACLES
uint8_t is_obstacle_button_pressed(void) {
    return !(GPIOA->IDR & (1 << 1));  //1 press
}

void setup_tim3(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~(0x3 << (6 * 2));
    GPIOC->MODER |= (0x2 << (6 * 2));
    GPIOC->AFR[0] &= ~(0xF << (6 * 4));
    GPIOC->AFR[0] |= (0x1 << (6 * 4));

    TIM3->PSC = 47;
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


int dino_y = 0;              
int jump_height = 20;   
int jump_vel = 5;  
int is_jumping = 0;
int obstacle_x[MAX_OBSTACLES]; // Array to track x-position of obstacles
int obst_vel = 2;               // obstacle speed
int num_obstacles = 0;
int dino_x = 10;             // Dinosaur's x position fixed right now
int dino_width = 20;          
int dino_height = 20;        
int obstacle_width = 10;     
int obstacle_height = 10;

///
///
///
///
///
///
/// GAME LOGIC
///
///
///
///
///
///



//obstacle fr
void player2_spawn_obstacle(void) {
    if (num_obstacles < MAX_OBSTACLES) {
        obstacle_x[num_obstacles] = WIDTH;  // obstacle at right sde x = 128 might have to change that fr
        num_obstacles++;
    }
}

void update_obstacles(void) {
    for (int i = 0; i < num_obstacles; i++) {
        obstacle_x[i] -= obst_vel;  // Move obstacle left
        if (obstacle_x[i] <= 0) {
            // off-screenr remove it 
            obstacle_x[i] = obstacle_x[num_obstacles - 1];  // Move last obstacle ot here
            num_obstacles--;  
        }
    }
}

int check_collision(void) {
    for (int i = 0; i < num_obstacles; i++) {
        // Horizontal collision
        if (obstacle_x[i] < dino_x + dino_width && obstacle_x[i] + obstacle_width > dino_x) {
            // Vertical collision: 
            if (dino_y < obstacle_height) {
                return 1;  // Collision 
            }
        }
    }
    return 0;  // No collision
}

void game_loop(void) {
    while (1) {
        
        if (is_jump_button_pressed()) {
            play_buzzer();
            player1_jump();
            nano_wait(5000000);
            stop_buzzer();
        }

        if (is_obstacle_button_pressed()) {
            play_buzzer();
            player2_spawn_obstacle();
            nano_wait(500000);
            stop_buzzer();
        } 
    }
}



int main(void) {
    // Initialize 
    init_spi2();             
    init_tim7();             
    init_tim15();            
    init_buttons();  
    setup_tim3();        
    
    game_loop();             

    return 0;
}