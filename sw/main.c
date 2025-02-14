#include "stm32l011xx.h"
#include <stdio.h>

#define BUFFER_SIZE 2  // Two channels: USB_Current (PA0), USB_Voltage (PA1)

volatile uint16_t adc_buffer[BUFFER_SIZE];  // Buffer for ADC data

/* Private functions */
static void clock_init(void);
static void gpio_init(void);
static void tim21_init(void);
static void adc_dma_init(void);
static void usart2_init(void);
static void send_data(uint16_t current, uint16_t voltage);

static void clock_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  // Enable MSI (2.097 MHz) and set range to 5
  RCC->CR |= RCC_CR_MSION;  // Enable MSI
  while (!(RCC->CR & RCC_CR_MSIRDY));  // Wait until MSI is ready

  RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | (5 << RCC_ICSCR_MSIRANGE_Pos);  // Set MSI range to 5 (2.097 MHz)
  while (!(RCC->CR & RCC_CR_MSIRDY));  // Wait until MSI stabilizes

  // Set system clock to MSI (2.097 MHz)
  RCC->CFGR = 0;  // Default SYSCLK = MSI

  // Enable LSI for future use (watchdog, RTC if needed)
  RCC->CSR |= RCC_CSR_LSION;
  while (!(RCC->CSR & RCC_CSR_LSIRDY));

  // Read access latency needs to be programmed according to CPU clock
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
}

static void gpio_init(void)
{
  // Enable GPIOA clock
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // PA0 (USB_Current) & PA1 (USB_Voltage) as Analog Inputs
  GPIOA->MODER |= (3 << (0 * 2)) | (3 << (1 * 2));  // Analog mode

  // PA9 (USART2_TX) as Alternate Function
  GPIOA->MODER &= ~(3 << (9 * 2));  // Clear mode
  GPIOA->MODER |= (2 << (9 * 2));   // Set to AF mode
  GPIOA->AFR[1] |= (4 << ((9 - 8) * 4));  // AF4 for USART2_TX
}

static void tim21_init(void)
{
  // Enable TIM21 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;

  // Set prescaler and ARR for 5ms period
  TIM21->PSC = 209;  // (2.097 MHz / (209+1)) = 10 kHz
  TIM21->ARR = 49;   // 10 kHz / 50 = 200 Hz (5ms period)

  // Enable update interrupt
  TIM21->DIER |= TIM_DIER_UIE;

  // Enable TIM21 interrupt in NVIC
  NVIC_EnableIRQ(TIM21_IRQn);

  // Start TIM21
  TIM21->CR1 |= TIM_CR1_CEN;
}

static void adc_dma_init(void)
{
  // Enable ADC & DMA clocks
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
  RCC->AHBENR  |= RCC_AHBENR_DMAEN;

  // Configure ADC (12-bit resolution, continuous mode disabled)
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;  // 12-bit resolution
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;  // Enable DMA
  ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1;  // Select PA0 & PA1
  ADC1->SMPR = 2;  // Sampling time (approx 7.5 cycles)

  // Configure DMA for ADC1
  DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;  // Source: ADC data register
  DMA1_Channel1->CMAR = (uint32_t) adc_buffer;  // Destination: adc_buffer
  DMA1_Channel1->CNDTR = BUFFER_SIZE;  // Number of transfers
  DMA1_Channel1->CCR = DMA_CCR_MINC  // Memory increment mode
             | DMA_CCR_CIRC  // Circular mode
             | DMA_CCR_PL_1  // High priority
             | DMA_CCR_EN;   // Enable DMA

  // Enable ADC
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY));  // Wait until ADC is ready

  // Enable DMA IRQ
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void usart2_init(void)
{
  // Enable USART2 clock
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // TODO : set USART2 clock to PCLK1 in RCC

  // Configure USART2: 115200 baud (assuming 16 MHz clock)
  USART2->BRR = (16000000 / 115200);
  USART2->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable TX and USART

  while (!(USART2->ISR & USART_ISR_TEACK));  // Wait for TX to be ready
}

static void send_data(uint16_t current, uint16_t voltage)
{
  char msg[20];
  int len = snprintf(msg, sizeof(msg), "%u,%u\n", current, voltage);

  for (int i = 0; i < len; i++)
  {
    while (!(USART2->ISR & USART_ISR_TXE));  // Wait for TX buffer empty
    USART2->TDR = msg[i];  // Send byte
  }
}

void TIM21_IRQHandler(void)
{
  if (TIM21->SR & TIM_SR_UIF)
  {
    TIM21->SR &= ~TIM_SR_UIF;  // Clear update flag
    ADC1->CR |= ADC_CR_ADSTART;  // Start ADC conversion
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if (DMA1->ISR & DMA_ISR_TCIF1)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF1;  // Clear DMA transfer complete flag

    // Send ADC results via USART
    send_data(adc_buffer[0], adc_buffer[1]);
  }
}

void HardFault_Handler(void)
{
  while(1);
}

int main(void)
{
  SCB->VTOR = (uint32_t)0x08000000;

  clock_init();
  gpio_init();
  tim21_init();
  adc_dma_init();
  usart2_init();

  while (1)
  {
    __WFI();  // Wait for interrupt (low power)
  }
}
