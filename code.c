#include "stm32f4xx.h"  // Device header

void GPIO_Config(void);
void delay_us(uint32_t us);
void interrupt_config(void);
void echo_generate(void);
void TIM2_Config(void);

volatile int distance_cm = 0;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint8_t led_state_pb9 = 0; // Toggle state for PB9

int main(void) {
    GPIO_Config();       // Initialize GPIO pins for TRIG, ECHO, and LEDs
    interrupt_config();  // Configure an interrupt for the ECHO pin
    TIM2_Config();       // Configure TIM2 for timing measurements

    while (1) {     
        echo_generate();  // Generate echo signal to measure distance

        // Control LEDs based on distance
        if (distance_cm <= 100) {
            GPIOC->ODR |= (1U << 14);       // Turn on LED connected to PC14
            GPIOB->ODR ^= (1U<<9);
						GPIOC->ODR |= (1U<<13);
        } else if (distance_cm <= 1000) {
            GPIOC->ODR |= (1U << 14);       // Turn on LED connected to PC14
            GPIOB->ODR &= ~(1U << 9);       // Ensure PB9 is off
            GPIOC->ODR |= (1U << 13);      // Ensure PC13 is off
        } else {
            GPIOC->ODR &= ~(1U << 14);      // Turn off PC14
            GPIOB->ODR &= ~(1U << 9);       // Turn off PB9
            GPIOC->ODR &= ~(1U << 13);       // Turn on PC13
        }

        for (volatile int i = 0; i < 100000; i++);  // Simple delay for debounce
    }
}

// Configure GPIO pins for LEDs, TRIG, and ECHO
void GPIO_Config(void) {
    // Enable GPIOA, GPIOB, and GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // Set PA0 (TRIG) as output
    GPIOA->MODER |= (1 << 0);       // Set PA0 as output
    GPIOA->OTYPER &= ~(1 << 0);     // Output push-pull
    GPIOA->OSPEEDR |= (3 << 0);     // Set high speed

    // Set PA1 (ECHO) as input
    GPIOA->MODER &= ~(3 << 2);      // Set PA1 as input

    // Set PC14, PB9, and PC13 as outputs for LEDs
    GPIOC->MODER |= (1 << 28);      // Set PC14 as output
    GPIOB->MODER |= (1 << 18);      // Set PB9 as output
    GPIOC->MODER |= (1 << 26);      // Set PC13 as output
}

// Configure EXTI interrupt for PA1 (ECHO pin)
void interrupt_config(void) {
    RCC->APB2ENR |= (1U << 14);     // Enable SYSCFG clock

    SYSCFG->EXTICR[0] = 0;          // Clear EXTICR[0]
    SYSCFG->EXTICR[0] |= (0U << 4); // Set EXTI1 line to PA1

    EXTI->IMR |= (1U << 1);         // Unmask interrupt on EXTI1
    EXTI->FTSR |= (1U << 1);        // Select falling edge trigger for PA1
    EXTI->RTSR |= (1U << 1);        // Select rising edge trigger for PA1
    NVIC_EnableIRQ(EXTI1_IRQn);     // Enable EXTI1 interrupt in NVIC
}

// Delay function for microsecond timing using SysTick
void delay_us(uint32_t us) {
    volatile uint32_t count = 84 * us;  // 1 microsecond delay
    while (count--) {
        __NOP();  // No operation
    }
}

// Generate a 10us pulse on TRIG to start ultrasonic measurement
void echo_generate(void) {
    GPIOA->ODR |= (1 << 0);   // Set PA0 HIGH
    delay_us(10);             // Wait for 10 microseconds
    GPIOA->ODR &= ~(1 << 0);  // Set PA0 LOW
}

// EXTI1 interrupt handler for capturing ECHO pulse duration
void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U << 1)) {         // Check if interrupt was caused by PA1
        EXTI->PR |= (1U << 1);          // Clear pending interrupt flag

        if (GPIOA->IDR & (1 << 1)) {    // Rising edge detected (pulse start)
            TIM2->CNT = 0;              // Reset TIM2 counter
            TIM2->CR1 |= TIM_CR1_CEN;   // Start TIM2
        } else {                        // Falling edge detected (pulse end)
            TIM2->CR1 &= ~TIM_CR1_CEN;  // Stop TIM2
            end_time = TIM2->CNT;       // Capture pulse duration in microseconds

            // Calculate distance in cm
            distance_cm = (end_time * 0.0343) / 2;  // Speed of sound = 0.0343 cm/us, divide by 2 for round trip
        }
    }
}

// Configure TIM2 for timing measurements
void TIM2_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock

    TIM2->PSC = 84 - 1;                  // Prescaler to 1 MHz (84 MHz / 84 = 1 MHz)
    TIM2->ARR = 0xFFFFFFFF;              // Set auto-reload to maximum
    TIM2->CNT = 0;                       // Reset counter
    TIM2->CR1 |= TIM_CR1_URS;            // Update request source
}
