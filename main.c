#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#define PPR 924
#define DT 0.01f   // 10ms
#define WHEEL_DIAMETER_MM 96.0f
#define PI 3.1415926f

volatile float mm_per_pulse;

/* Encoder counters */
volatile int32_t pos_1 = 0, pos_2 = 0, pos_3 = 0, pos_4 = 0;
volatile int32_t cnt_1 = 0, cnt_2 = 0, cnt_3 = 0, cnt_4 = 0;

/* RPM + Distance */
volatile float rpm_1 = 0, rpm_2 = 0, rpm_3 = 0, rpm_4 = 0;
volatile float dist_1 = 0, dist_2 = 0, dist_3 = 0, dist_4 = 0;

/* Time */
volatile uint32_t time_ms = 0;

/* UART timer */
volatile uint16_t uart_timer = 0;

/* Commands */
volatile float cmd_1 = 0, cmd_2 = 0, cmd_3 = 0, cmd_4 = 0;

/* UART RX buffer */
char rx_buf[64];
uint8_t rx_idx = 0;
//==================USART=====================
void UART1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER |= (2 << (9*2)) | (2 << (10*2));
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8);
    /* Baudrate 9600 @ 100MHz */
    //USART1->BRR =  (104 << 4) | 2;
    /* Baudrate 115200 @ 16MHz */
    USART1->BRR =  (8 << 4) | 11;
    /* Baudrate 9600 @ 16MHz */
    //USART1->BRR = (104 << 4) | 3;   // 0x683

    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
    {
        char c = USART1->DR;

        if (c == '\n' || c == '\r')
        {
            rx_buf[rx_idx] = 0;
            rx_idx = 0;

            if (rx_buf[0] == 'C')
            {
                float a,b,c,d;
                if (sscanf(rx_buf+1, "%f %f %f %f", &a,&b,&c,&d) == 4)
                {
                    cmd_1 = a;
                    cmd_2 = b;
                    cmd_3 = c;
                    cmd_4 = d;
                }
            }
        }
        else
        {
            if (rx_idx < 63) rx_buf[rx_idx++] = c;
        }
    }
}

void UART_SendString(char *s)
{
    while (*s)
    {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *s++;
    }
}

void UART_SendFloat(float f)
{
    char buf[32];
    sprintf(buf, "%.2f ", f);
    UART_SendString(buf);
}

void UART_SendInt(int v)
{
    char buf[16];
    sprintf(buf, "%d ", v);
    UART_SendString(buf);
}
void Encoder_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(3*2)) | (3<<(4*2)));
    GPIOB->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(3*2)) | (3<<(4*2)));
    // ========PULL UP ==========
    GPIOA->PUPDR |= (1<<(0*2)) | (1<<(1*2)) | (1<<(3*2)) | (1<<(4*2));
    GPIOB->PUPDR |= (1<<(0*2)) | (1<<(1*2)) | (1<<(3*2)) | (1<<(4*2));
    // -------- BÁNH 1: PA0 -> EXTI0 --------
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PA;
    EXTI->IMR  |= EXTI_IMR_IM0;
    EXTI->RTSR |= EXTI_RTSR_TR0;
   // EXTI->FTSR |= EXTI_FTSR_TR0;  
    NVIC_EnableIRQ(EXTI0_IRQn);

    // -------- BÁNH 2: PB1 -> EXTI1 --------
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PB;
    EXTI->IMR  |= EXTI_IMR_IM1;
    EXTI->RTSR |= EXTI_RTSR_TR1;
    //EXTI->FTSR |= EXTI_FTSR_TR1;   
    NVIC_EnableIRQ(EXTI1_IRQn);

    // -------- BÁNH 3: PB3 -> EXTI3 --------
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3;
    SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI3_PB;
    EXTI->IMR  |= EXTI_IMR_IM3;
    EXTI->RTSR |= EXTI_RTSR_TR3;
   // EXTI->FTSR |= EXTI_FTSR_TR3;   
    NVIC_EnableIRQ(EXTI3_IRQn);

    // -------- BÁNH 4: PA4 -> EXTI4 --------
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |=  SYSCFG_EXTICR2_EXTI4_PA;
    EXTI->IMR  |= EXTI_IMR_IM4;
    EXTI->RTSR |= EXTI_RTSR_TR4;
   // EXTI->FTSR |= EXTI_FTSR_TR4;  
    NVIC_EnableIRQ(EXTI4_IRQn);
}
//==================Encoder 1 – PA0 (A), PA1 (B)===============
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        EXTI->PR = EXTI_PR_PR0;

        if (GPIOA->IDR & (1<<1)) { cnt_1++; pos_1++; dist_1 += mm_per_pulse; }
        else { cnt_1--; pos_1--; dist_1 -= mm_per_pulse; }
    }
}
//===================Encoder 2 – PB1 (A), PB0 (B)====================
void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR1)
    {
        EXTI->PR = EXTI_PR_PR1;

        if (GPIOB->IDR & (1<<0)) { cnt_2++; pos_2++; dist_2 += mm_per_pulse; }
        else { cnt_2--; pos_2--; dist_2 -= mm_per_pulse; }
    }
}
//======================Encoder 3 – PB3 (A), PB4 (B)========================
void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR3)
    {
        EXTI->PR = EXTI_PR_PR3;

        if (GPIOB->IDR & (1<<4)) { cnt_3++; pos_3++; dist_3 += mm_per_pulse; }
        else { cnt_3--; pos_3--; dist_3 -= mm_per_pulse; }
    }
}
//===========================Encoder 4 – PA4 (A), PA3(B)=====================
void EXTI4_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR4)
    {
        EXTI->PR = EXTI_PR_PR4;

        if (GPIOA->IDR & (1<<3)) { cnt_4++; pos_4++; dist_4 += mm_per_pulse; }
        else { cnt_4--; pos_4--; dist_4 -= mm_per_pulse; }
    }
}
void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 16000 - 1;   // 16MHz / 16000 = 1kHz (1ms)
    TIM2->ARR = 10 - 1;      // 10ms

    TIM2->DIER |= TIM_DIER_UIE;  // enable interrupt
    TIM2->CR1  |= TIM_CR1_CEN;   // start

    NVIC_EnableIRQ(TIM2_IRQn);
}
//===================Timer 10ms – tính RPM=====================
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        rpm_1 = (cnt_1 * 60.0f) / (PPR * DT);
        rpm_2 = (cnt_2 * 60.0f) / (PPR * DT);
        rpm_3 = (cnt_3 * 60.0f) / (PPR * DT);
        rpm_4 = (cnt_4 * 60.0f) / (PPR * DT);

        cnt_1 = cnt_2 = cnt_3 = cnt_4 = 0;

        uart_timer++;
    }
}
void TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 16000 - 1;   // 1kHz
    TIM3->ARR = 1 - 1;       // 1ms

    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1  |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}
//=====================Timer 1ms – d?m th?i gian=================
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        time_ms++;
    }
}
int main(void)
{
    UART1_Init();
    Encoder_Init();
    TIM2_Init();   // 10ms
    TIM3_Init();   // 1ms

    mm_per_pulse = (PI * WHEEL_DIAMETER_MM) / PPR;

    UART_SendString("STM32 EXTI Encoder System Ready\r\n");

    while (1)
    {
        if (uart_timer >= 50)   // 500ms
        {
            uart_timer = 0;

            UART_SendString("S ");
            UART_SendFloat(rpm_1);
            UART_SendFloat(rpm_2);
            UART_SendFloat(rpm_3);
            UART_SendFloat(rpm_4);
						UART_SendString("distance\r\n");
            UART_SendInt((int)dist_1);
            UART_SendInt((int)dist_2);
            UART_SendInt((int)dist_3);
            UART_SendInt((int)dist_4);
						UART_SendString("timer \r\n");
            UART_SendInt(time_ms);
            UART_SendString("\r\n");
        }
    }
}
