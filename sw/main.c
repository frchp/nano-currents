#include "stm32l011xx.h"

#define PCLK_FREQ ((uint32_t)2097152U)

#define UART_MSG_SIZE     (4u)
#define UART_BYTES_IN_U16 (2u)

#define ADC_BUFFER_SIZE   (2)  // Two channels: USB_Current (PA0), USB_Voltage (PA1)
#define ADC_REF_MV        (3300u)
#define ADC_REF_LSB       (0xFFF) // 12bits ADC
#define _ADC_LSB_TO_MV(x) ((x * ADC_REF_MV)/ADC_REF_LSB)

#define CURRENT_AMP_GAIN      (500u)
#define CURRENT_AMP_SHUNT     (1u) // TODO : Check value
#define CURRENT_UA_MA_FACTOR  (1000u)
#define _CURRENT_MV_TO_UA(x)  (uint16_t)((x * CURRENT_UA_MA_FACTOR)/(CURRENT_AMP_GAIN * CURRENT_AMP_SHUNT))

#define VOLTAGE_AMP_GAIN      (1u)
#define VOLTAGE_AMP_IN_RATIO  (0.5f)
#define VOLTAGE_AMP_INV_RATIO (2u) // = 1/VOLTAGE_AMP_IN_RATIO
#define _VOLTAGE_MV_TO_MV(x)  (uint16_t)((x * VOLTAGE_AMP_INV_RATIO)/VOLTAGE_AMP_GAIN)

volatile uint16_t g_au16adcBuffer[ADC_BUFFER_SIZE];  // Buffer for ADC data
volatile uint8_t g_u8AdcRdyFlag = 0u;

typedef struct {
  union
  {
    char m_acRawBuffer[UART_MSG_SIZE];
    struct
    {
      uint16_t m_u16Voltage;
      uint16_t m_u16Current;
      #if (UART_MSG_SIZE % UART_BYTES_IN_U16 != 0)
        uint8_t m_au8Padding[(UART_MSG_SIZE % UART_BYTES_IN_U16)];
      #endif
    } m_sValues;
  } m_uData;

  uint8_t m_u8TxIdx;
} UartPacket_t;
volatile UartPacket_t g_sUartPacket;

/* Private functions */
static void clock_init(void);
static void gpio_init(void);
static void tim21_init(void);
static void adc_dma_init(void);
static void usart2_init(void);
static void send_data(uint16_t current, uint16_t voltage);
static uint16_t adc_to_current_microamp(uint16_t current_lsb);
static uint16_t adc_to_voltage_millivolt(uint16_t voltage_lsb);
static void send_next_char(void);

static void clock_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  PWR->CR |= PWR_CR_DBP;

  RCC->CR |= RCC_CR_MSION;  // Enable MSI
  while (!(RCC->CR & RCC_CR_MSIRDY));  // Wait until MSI is ready

  RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | (RCC_ICSCR_MSIRANGE_5);  // Set MSI range to 5 (2.097 MHz)
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

  // PA0 (USB_Current) & PA2 (USB_Voltage) as Analog Inputs
  GPIOA->MODER |= (3 << GPIO_MODER_MODE0_Pos) | (3 << GPIO_MODER_MODE2_Pos);  // Analog mode

  // PA9 (USART2_TX) as Alternate Function
  GPIOA->MODER &= ~GPIO_MODER_MODE9_Msk;  // Clear mode
  GPIOA->MODER |= GPIO_MODER_MODE9_1;   // Set to AF mode
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

  // Set ADC Clock Prescaler
  ADC1_COMMON->CCR |= ADC_CCR_PRESC_0; // 0b0001 => /2 prescaler

  // Configure ADC (12-bit resolution, continuous mode disabled)
  ADC1->CFGR1 &= ~ADC_CFGR1_RES;  // 12-bit resolution
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;  // Enable DMA
  ADC1->CFGR2 |= ADC_CFGR2_CKMODE_0;  // PCLK/2
  ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL2;  // Select PA0 & PA2
  ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1;  // Sampling time (approx 39.5 cycles)

  // Configure DMA for ADC1
  DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;  // Source: ADC data register
  DMA1_Channel1->CMAR = (uint32_t) g_au16adcBuffer;  // Destination: g_au16adcBuffer
  DMA1_Channel1->CNDTR = ADC_BUFFER_SIZE;  // Number of transfers
  DMA1_Channel1->CCR = DMA_CCR_MINC  // Memory increment mode
             | DMA_CCR_CIRC  // Circular mode
             | DMA_CCR_PL_1  // High priority
             | DMA_CCR_EN   // Enable DMA
             | DMA_CCR_MSIZE_0 // 16 bits array
             | DMA_CCR_PSIZE_1 // 32 bits ADC reg
             | DMA_CCR_TCIE; // Enable TC interrupt

  // Enable ADC
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY))
  {
    // Wait until ADC is ready
  }

  // Enable DMA IRQ
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void usart2_init(void)
{
  // Enable USART2 clock
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  // APB clock is set as USART2 clock by default in RCC->CCIPR

  // Configure USART2: 115200 baud (assuming 16 MHz clock)
  USART2->BRR = (PCLK_FREQ / 115200);
  USART2->CR1 = USART_CR1_TE | USART_CR1_UE;  // Enable TX and USART

  while (!(USART2->ISR & USART_ISR_TEACK));  // Wait for TX to be ready

  // Enable USART2 interrupt in NVIC
  NVIC_EnableIRQ(USART2_IRQn);
}

static void send_data(uint16_t current, uint16_t voltage)
{
  // Fill packet
  g_sUartPacket.m_u8TxIdx = 0u;
  g_sUartPacket.m_uData.m_sValues.m_u16Current = current;
  g_sUartPacket.m_uData.m_sValues.m_u16Voltage = voltage;

  // Enable TXE interrupt to know when to refill peripheral
  while (!(USART2->ISR & USART_ISR_TXE));  // Wait for TX buffer empty
  USART2->CR1 |= USART_CR1_TXEIE;

  send_next_char();
}

static uint16_t adc_to_current_microamp(uint16_t current_lsb)
{
  return (_CURRENT_MV_TO_UA(_ADC_LSB_TO_MV(current_lsb)));
}

static uint16_t adc_to_voltage_millivolt(uint16_t voltage_lsb)
{
  return (_VOLTAGE_MV_TO_MV(_ADC_LSB_TO_MV(voltage_lsb)));
}

static void send_next_char(void)
{
  USART2->TDR = g_sUartPacket.m_uData.m_acRawBuffer[g_sUartPacket.m_u8TxIdx++];
  if (g_sUartPacket.m_u8TxIdx == UART_MSG_SIZE)
  {
    // All data have been transmitted
    USART2->CR1 &= ~USART_CR1_TXEIE;
  }
}

/* Interrupts handlers */

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

    // Set flag to asynchronize data handling
    g_u8AdcRdyFlag = 1u;
  }
}

void USART2_IRQHandler(void)
{
  if (USART2->ISR & USART_ISR_TXE)
  {
    // Flag is cleared by a write to TDR register
    send_next_char();
  }
}

void HardFault_Handler(void)
{
  while(1);
}

/* Main routine */

int main(void)
{
  SCB->VTOR = (uint32_t)0x08000000;

  clock_init();
  gpio_init();
  adc_dma_init();
  usart2_init();
  tim21_init(); // Start timer last, when every other peripheral is init

  while (1)
  {
    __WFI();  // Wait for interrupt (low power)
    if(g_u8AdcRdyFlag != 0u)
    {
      g_u8AdcRdyFlag = 0u;
      // Send ADC results via USART
      send_data(adc_to_current_microamp(g_au16adcBuffer[0]), adc_to_voltage_millivolt(g_au16adcBuffer[1]));
    }
  }
}
