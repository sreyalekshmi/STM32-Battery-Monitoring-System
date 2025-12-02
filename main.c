#include "stm32f4xx.h"
#include <stdint.h>

/* ---------------------------------------------------------
   7-SEGMENT DISPLAY DIGIT MAP (Common Cathode)
   PB0–PB7 → Tens digit output
   PC0–PC7 → Ones digit output
   --------------------------------------------------------- */
uint8_t SSD_Num[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

int tens = 0;
int ones = 0;

/* ---------------------------------------------------------
   Display number on the 7-segment display
   --------------------------------------------------------- */
void displayNumber(void)
{
    /* Ones → PC0–PC7 */
    GPIOC->ODR &= ~0xFF;
    GPIOC->ODR |= SSD_Num[ones];

    /* Tens → PB0–PB7 */
    GPIOB->ODR &= ~0xFF;
    GPIOB->ODR |= SSD_Num[tens];
}

/* ---------------------------------------------------------
   ADC1 INITIALIZATION (PA1 = Channel 1)
   --------------------------------------------------------- */
void ADC1_Init(void)
{
    /* ADC prescaler */
    ADC->CCR &= ~(3 << 16);
    ADC->CCR |=  (1 << 16);

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    /* Sampling time for Channel 1 */
    ADC1->SMPR2 &= ~(7 << 3);
    ADC1->SMPR2 |=  (5 << 3);

    /* Regular sequence length = 1 conversion */
    ADC1->SQR1 &= ~(0xF << 20);

    /* Select Channel 1 (PA1) */
    ADC1->SQR3 &= ~(0x1F);
    ADC1->SQR3 |=  (1 << 0);

    ADC1->CR2 |= ADC_CR2_ADON;  // ADC ON

    for (volatile int i = 0; i < 5000; i++);
}

/* ---------------------------------------------------------
   Read ADC1 value
   --------------------------------------------------------- */
uint16_t ADC1_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;         // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));     // Wait for completion
    return ADC1->DR;
}

/* ---------------------------------------------------------
   Delay in milliseconds using SysTick
   --------------------------------------------------------- */
void delayMs(int n)
{
    SysTick->LOAD = (SystemCoreClock/1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5;   // Enable SysTick

    for (int i = 0; i < n; i++)
        while ((SysTick->CTRL & (1<<16)) == 0);

    SysTick->CTRL = 0;   // Disable
}

/* ---------------------------------------------------------
   MAIN FUNCTION
   --------------------------------------------------------- */
int main(void)
{
    /* Enable GPIOA, GPIOB, GPIOC, ADC1 clocks */
    RCC->AHB1ENR |= (1<<0) | (1<<1) | (1<<2);
    RCC->APB2ENR |= (1<<14);

    /* PA1 = ADC input (Analog mode) */
    GPIOA->MODER |=  (3 << (1*2));
    GPIOA->PUPDR &= ~(3 << (1*2));

    /* PA5, PA6, PA7 = LED outputs */
    GPIOA->MODER |=  ((1<<(5*2)) | (1<<(6*2)) | (1<<(7*2)));
    GPIOA->OTYPER &= ~((1<<5)|(1<<6)|(1<<7));
    GPIOA->PUPDR  &= ~((3<<(5*2))|(3<<(6*2))|(3<<(7*2)));

    /* PA0 = Push button input */
    GPIOA->MODER &= ~(3 << (0*2));
    GPIOA->PUPDR &= ~(3 << (0*2));

    /* PC0–PC7 = Ones SSD */
    GPIOC->MODER &= ~0x0000FFFF;
    GPIOC->MODER |=  0x00005555;
    GPIOC->OTYPER &= ~0x00FF;
    GPIOC->PUPDR  &= ~0x0000FFFF;

    /* PB0–PB7 = Tens SSD */
    GPIOB->MODER &= ~0x0000FFFF;
    GPIOB->MODER |=  0x00005555;
    GPIOB->OTYPER &= ~0x00FF;
    GPIOB->PUPDR  &= ~0x0000FFFF;

    ADC1_Init();

    /* Initial SSD display = 00 */
    tens = 0;
    ones = 0;
    displayNumber();

    /* -----------------------------------------------------
       MAIN LOOP
       ----------------------------------------------------- */
    while (1)
    {
        /* Manual Measurement via Push Button */
        if (GPIOA->IDR & (1<<0))
        {
            uint16_t adc_val = ADC1_Read();
            int percent = (adc_val * 100) / 4095;
            if (percent > 99) percent = 99;

            tens = percent / 10;
            ones = percent % 10;
            displayNumber();

            /* LED Status */
            if (percent > 70) {             // HIGH
                GPIOA->BSRR = (1<<5);
                GPIOA->BSRR = (1<<(6+16));
                GPIOA->BSRR = (1<<(7+16));
            }
            else if (percent > 30) {        // MEDIUM
                GPIOA->BSRR = (1<<(5+16));
                GPIOA->BSRR = (1<<6);
                GPIOA->BSRR = (1<<(7+16));
            }
            else {                           // LOW
                GPIOA->BSRR = (1<<(5+16));
                GPIOA->BSRR = (1<<(6+16));
                GPIOA->BSRR = (1<<7);
            }

            delayMs(200);  // Debounce
            continue;
        }

        /* Periodic Measurement (1 second) */
        delayMs(1000);

        uint16_t adc_val = ADC1_Read();
        int percent = (adc_val * 100) / 4095;
        if (percent > 99) percent = 99;

        tens = percent / 10;
        ones = percent % 10;
        displayNumber();

        /* LED Status */
        if (percent > 70) {
            GPIOA->BSRR = (1<<5);
            GPIOA->BSRR = (1<<(6+16));
            GPIOA->BSRR = (1<<(7+16));
        }
        else if (percent > 30) {
            GPIOA->BSRR = (1<<(5+16));
            GPIOA->BSRR = (1<<6);
            GPIOA->BSRR = (1<<(7+16));
        }
        else {
            GPIOA->BSRR = (1<<(5+16));
            GPIOA->BSRR = (1<<(6+16));
            GPIOA->BSRR = (1<<7);
        }
    }
}
