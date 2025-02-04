#include "stm32f4xx.h"

// Function prototypes
void GPIO_Config(void);
void ADC_Config(void);
uint16_t Read_ADC(uint8_t channel);
void Delay(uint32_t time);

int main(void) {
    uint16_t ir_value;
    uint16_t water_value;
    uint16_t ir_threshold = 1000;    // Adjust based on IR sensor output
    uint16_t water_threshold = 819;  // 20% of 4095 (12-bit ADC)

    // Initialize peripherals
    GPIO_Config();
    ADC_Config();

    while (1) {
        // Read ADC values
        ir_value = Read_ADC(0);    // IR sensor (PA0)
        water_value = Read_ADC(3); // Water sensor (PA3)

        // IR sensor logic
        if (ir_value > ir_threshold) {
            GPIOA->BSRR = GPIO_BSRR_BS7;  // Turn PA7 LED ON
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR7;  // Turn PA7 LED OFF
        }

        // Water sensor logic
        if (water_value > water_threshold) {
            GPIOA->BSRR = GPIO_BSRR_BS5;  // Turn PA5 LED ON
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR5;  // Turn PA5 LED OFF
        }

        Delay(500); // Delay for stability
    }
}

void GPIO_Config(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA0 (IR sensor) and PA3 (Water sensor) as analog inputs
    GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1); // PA0 analog
    GPIOA->MODER |= (GPIO_MODER_MODE3_0 | GPIO_MODER_MODE3_1); // PA3 analog

    // Configure PA5 (Water LED) and PA7 (IR LED) as output
    GPIOA->MODER |= (GPIO_MODER_MODE5_0); // PA5 output
    GPIOA->MODER |= (GPIO_MODER_MODE7_0); // PA7 output
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7); // Push-pull mode
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR7); // High speed
}

void ADC_Config(void) {
    // Enable ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure ADC prescaler (divide by 8)
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    // Configure ADC1 for single conversion mode
    ADC1->CR1 = 0; // 12-bit resolution
    ADC1->CR2 = ADC_CR2_ADON; // Enable ADC
}

uint16_t Read_ADC(uint8_t channel) {
    // Select ADC channel
    ADC1->SQR3 = channel;

    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion to complete
    while (!(ADC1->SR & ADC_SR_EOC));

    // Return the converted value
    return ADC1->DR;
}

void Delay(uint32_t time) {
    // Simple delay loop
    for (volatile uint32_t i = 0; i < time * 1000; i++);
}
