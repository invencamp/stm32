#include "stm32l073xx.h"
#include "core_cm0plus.h"
#include <string.h>
#include <stdio.h>
uint32_t SystemCoreClock = 2097152U;
void SystemInit (void){}
	
volatile uint32_t uwTick;
	void SysTick_Handler(){
		uwTick+=1;
	}
	void TIM2_us_init(void) {
    // Enable TIM2 clock (APB1)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // reset timer
    TIM2->CR1 = 0;
    TIM2->PSC = (SystemCoreClock / 1000000UL) - 1; // prescaler for 1 MHz -> 1 tick = 1 us
    TIM2->ARR = 0xFFFF; // max
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN; // enable
}
void delay_us(uint16_t us)
{
    TIM2->CNT = 0;
    while (TIM2->CNT < us);
}
	void delayMicroseconds(uint32_t us)
{
    uint32_t startTick = SysTick->VAL;  // Ð?c giá tr? hi?n t?i c?a SysTick
    uint32_t ticks_per_us = SystemCoreClock / 1000000U; // s? chu k? trong 1 µs
    uint32_t load = SysTick->LOAD + 1;  // Giá tr? d?m t?i da
    uint32_t us_ticks = us * ticks_per_us; // t?ng s? tick c?n ch?

    uint32_t elapsedTicks = 0;
    uint32_t oldTick = startTick;

    while (elapsedTicks < us_ticks)
    {
        uint32_t newTick = SysTick->VAL;
        if (newTick <= oldTick)
            elapsedTicks += (oldTick - newTick);
        else
            elapsedTicks += (load - newTick + oldTick);
        oldTick = newTick;
    }
}
		#define DHT22_PORT GPIOA
#define DHT22_PIN  (0x0002U)
	
	
	typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;
	GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}
typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be a combination of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
                            This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
} GPIO_InitTypeDef;
#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
#define OUTPUT_TYPE_Pos                         4U
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
#define  GPIO_MODE_INPUT                        MODE_INPUT                                                  /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    (MODE_OUTPUT | OUTPUT_PP) 
#define  GPIO_SPEED_FREQ_LOW              (0x00000000U)  
#define EXTI_MODE_Pos                           16U
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define __HAL_RCC_SYSCFG_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_SYSCFGEN))
#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0U :\
                                      ((__GPIOx__) == (GPIOB))? 1U :\
                                      ((__GPIOx__) == (GPIOC))? 2U :\
                                      ((__GPIOx__) == (GPIOD))? 3U :\
                                      ((__GPIOx__) == (GPIOE))? 4U :\
                                      ((__GPIOx__) == (GPIOH))? 5U : 6U)
																			
#define TRIGGER_MODE_Pos                         20U
#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)
#define  GPIO_NOPULL        (0x00000000U) 
void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;


  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Init->Pin) & (1U << position);

    if (iocurrent)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if (((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) ||
          ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF))
      {

        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
      }

      if ((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
      {


        /* Activate the Pull-up or Pull down resistor for the current IO */
        temp = GPIOx->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));
        temp |= ((GPIO_Init->Pull) << (position * 2U));
        GPIOx->PUPDR = temp;
      }

      /* In case of Alternate function mode selection */
      if ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
      {


        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3U];
        temp &= ~(0xFUL << ((uint32_t)(position & 0x07UL) * 4U));
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07U) * 4U));
        GPIOx->AFR[position >> 3U] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if ((GPIO_Init->Mode & EXTI_MODE) != 0x00U)
      {
        /* Enable SYSCFG Clock */
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[position >> 2U];
        CLEAR_BIT(temp, (0x0FUL) << (4U * (position & 0x03U)));
        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4 * (position & 0x03U)));
        SYSCFG->EXTICR[position >> 2U] = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTI->RTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_RISING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->FTSR = temp;

        temp = EXTI->EMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_EVT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->EMR = temp;

        /* Clear EXTI line configuration */
        temp = EXTI->IMR;
        temp &= ~((uint32_t)iocurrent);
        if ((GPIO_Init->Mode & EXTI_IT) != 0x00U)
        {
          temp |= iocurrent;
        }
        EXTI->IMR = temp;
      }
    }
    position++;
  }
}
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{


  if (PinState != GPIO_PIN_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin ;
  }
}
void Set_Pin_Output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT22_Start (void)
{
	Set_Pin_Output(DHT22_PORT, DHT22_PIN); // set the pin as output
	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
	delay_us(1200);   // wait for > 1ms

	HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
	delay_us (20);   // wait for 30us
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
}
uint8_t DHT22_Check_Response (void)
{
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);   // set as input
	uint8_t Response = 0;
	delay_us (40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) // if the pin is low
	{
		delay_us (80);   // wait for 80us

		if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;  // if the pin is high, response is ok
		else Response = -1;
	}

	while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go low
	return Response;
}
uint8_t DHT22_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));   // wait for the pin to go high
		delay_us (40);   // wait for 40 us

		if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)));  // wait for the pin to go low
	}

	return i;
}




//UART_HandleTypeDef huart2;
uint8_t tx_buffer[27]="Welcome\n\r";
uint8_t rx_indx;
char rx_data[8];
char rx_buffer[100];
char *str = rx_data;
char *cmd = rx_buffer;
uint8_t Rh_byte1,Rh_byte2,Temp_byte1,Temp_byte2;
uint16_t SUM,TEMP,RH;
float Temperature=0;
float Humidity=0;
uint8_t Presence=0;
void TIM6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = (SystemCoreClock / 1000000U) - 1; // prescaler d? 1 tick = 1 µs
    TIM6->ARR = 0xFFFF;
    TIM6->CR1 |= TIM_CR1_CEN;
}

int main(void)
{	
	TIM2_us_init();
	TIM6_Init();
	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL=0b111;
	
  RCC->IOPENR |=1;
	RCC->APB1ENR |= (1 << 17); 
	GPIOA->MODER &= ~((0b11 <<(5*2))|(0b11 << (2*2))|(0b11 << (3*2)));
	GPIOA->MODER |= (1<<(5*2))|(0b10 << (2*2))|(0b10 <<(3*2));
GPIOA->AFR[0] |= (4 << (2 * 4)) | (4 << (3 * 4));
USART2->BRR = SystemCoreClock / 9600;
USART2->CR1 = (1<<3) | (1<<2) | 1;// | 1<<5;
//NVIC->ISER[0] = (1 << USART2_IRQn); // Kích ho?t ng?t USART2
		float temp, hum;
		char text[32];
	
  //HAL_TIM_Base_Start(&htim6);  // for us Delay
//  uint32_t s = uwTick;
//		while((uwTick - s) <2000){}
	
  while (1)
  {
		//DHT22_ReadData(&temp, &hum);

    //sprintf(text, "T=%.1fC, H=%.1f%%\r\n", temp, hum);
//		
		
		DHT22_Start();
      Presence = DHT22_Check_Response();
      Rh_byte1 = DHT22_Read ();
      Rh_byte2 = DHT22_Read ();
      Temp_byte1 = DHT22_Read ();
      Temp_byte2 = DHT22_Read ();
      SUM = DHT22_Read();

      TEMP = ((Temp_byte1<<8)|Temp_byte2);
      RH = ((Rh_byte1<<8)|Rh_byte2);

      Temperature = (float) (TEMP/10.0);
      Humidity = (float) (RH/10.0);
			
			uint32_t start = uwTick;
		while((uwTick - start) <2000){}
      //HAL_Delay(2000);
			
		sprintf(text, "T=%.1fC, H=%.1f%%\r\n", Temperature, Humidity);
		
    for (int i = 0; i < strlen(text); i++) {
        USART2->TDR = text[i];
        while (!(USART2->ISR & (1 << 7))); // ch? TDR tr?ng
    }

		
		  for (int i = 0; i < sizeof(tx_buffer); i++) {
    USART2->TDR = tx_buffer[i];  
    while (!(USART2->ISR & (1 << 7))); // Ch? cho d?n khi TDR tr?ng
}
		
//		//delay_us(2000);
GPIOA->BSRR = 1 << 5; // Set
delay_us(65000);
//		uint32_t start = uwTick;
//		while((uwTick - start) <65){}
			GPIOA->BRR = 1<<5 ;//reset
delay_us(65000);
//			uint32_t s = uwTick;
//		while((uwTick - s) <65){}
			
		//GPIOA->ODR ^= (1<<5);

  }
}
//void GPIO_Output(void)
//{
//    // 1. B?t clock GPIOA (ho?c port tuong ?ng)
//    RCC->IOPENR |= RCC_IOPENR_IOPAEN;

//    // 2. C?u hình PA5 làm Output push-pull
//    DHT22_PORT->MODER &= ~(0x3 << (DHT22_PIN * 2));  // Xóa c?u hình cu
//    DHT22_PORT->MODER |=  (0x1 << (DHT22_PIN * 2));  // 01: Output mode

//    DHT22_PORT->OTYPER &= ~(1 << DHT22_PIN);         // 0: Push-pull
//    DHT22_PORT->OSPEEDR |=  (0x2 << (DHT22_PIN * 2)); // 10: High speed
//    DHT22_PORT->PUPDR   &= ~(0x3 << (DHT22_PIN * 2)); // Không kéo lên/xu?ng
//}

//void GPIO_Input(void)
//{
//    // 1. B?t clock GPIOA (ho?c port tuong ?ng)
//    RCC->IOPENR |= RCC_IOPENR_IOPAEN;

//    // 2. C?u hình PA5 làm Input floating
//    DHT22_PORT->MODER &= ~(0x3 << (DHT22_PIN * 2));  // 00: Input mode
//    DHT22_PORT->PUPDR &= ~(0x3 << (DHT22_PIN * 2));  // 00: Floating (không kéo lên/xu?ng)
//}

//uint8_t DHT22_CheckResponse(void)
//{
//    uint8_t response = 0;
//		delay(40);
//    if (!(DHT22_PORT->IDR & (1 << DHT22_PIN)))
//    {
//			delay(80);
//        if (DHT22_PORT->IDR & (1 << DHT22_PIN)) response = 1;
//				delay_us(80);
//    }
//    return response;
//}

//uint8_t DHT22_ReadByte(void)
//{
//    uint8_t i = 0, j;
//    for (j = 0; j < 8; j++)
//    {
//        while (!(DHT22_PORT->IDR & (1 << DHT22_PIN))); // d?i lên cao
//				delay(40);
//        if (DHT22_PORT->IDR & (1 << DHT22_PIN))
//            i = (i << 1) | 1;
//        else
//            i = (i << 1);
//        while (DHT22_PORT->IDR & (1 << DHT22_PIN)); // d?i xu?ng th?p
//    }
//    return i;
//}

//uint8_t DHT22_ReadData(float *temperature, float *humidity)
//{
//    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, checksum;
//    uint16_t sum;

//    DHT22_Start();
//    if (DHT22_CheckResponse())
//    {
//        Rh_byte1 = DHT22_ReadByte();
//        Rh_byte2 = DHT22_ReadByte();
//        Temp_byte1 = DHT22_ReadByte();
//        Temp_byte2 = DHT22_ReadByte();
//        checksum = DHT22_ReadByte();

//        sum = Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
//        if (sum == checksum)
//        {
//            *humidity = ((Rh_byte1 << 8) + Rh_byte2) / 10.0;
//            *temperature = (((Temp_byte1 & 0x7F) << 8) + Temp_byte2) / 10.0;
//            if (Temp_byte1 & 0x80) *temperature *= -1;
//            return 0; // OK
//        }
//        else return 2; // checksum sai
//    }
//    else return 1; // không ph?n h?i
//}
/*
void PWM_Init(void)
{
    // 1. B?t clock cho GPIOA và TIM2
    RCC->IOPENR |= (1 << 0);     // GPIOAEN
    RCC->APB1ENR |= (1 << 0);    // TIM2EN

    // 2. PA0 -> TIM2_CH1 (AF5)
    GPIOA->MODER &= ~(0b11 << (0 * 2));
    GPIOA->MODER |=  (0b10 << (0 * 2));   // Alternate function
    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] |=  (5 << (0 * 4));     // AF5: TIM2_CH1

    // 3. C?u hình PWM
    // ví d? n?u timer clock = SystemCoreClock
TIM2->PSC = (SystemCoreClock / 1000000U) - 1; // prescale d? timer ch?y 1MHz
TIM2->ARR = 1000 - 1; // 1kHz PWM
TIM2->CCR1 = 500;


    TIM2->CCMR1 &= ~(7 << 4);
    TIM2->CCMR1 |=  (6 << 4);   // PWM mode 1
    TIM2->CCMR1 |=  (1 << 3);   // Preload enable
    TIM2->CCER  |=  (1 << 0);   // Enable output

    TIM2->CR1  |=  (1 << 7);    // ARPE enable
    TIM2->EGR  |=  (1 << 0);    // Update event
    TIM2->CR1  |=  (1 << 0);    // Counter enable
}


void ADC1_Init(void)
{
    // 1. B?t clock cho GPIOA và ADC
    RCC->IOPENR |= (1 << 0);   // Enable GPIOA clock
    RCC->APB2ENR |= (1 << 9);  // Enable ADC1 clock

    // 2. C?u hình PA0 làm analog (ADC_IN0)
    GPIOA->MODER |= (0b11 << (0 * 2));  // Analog mode (11)
    GPIOA->PUPDR &= ~(0b11 << (0 * 2)); // No pull-up/pull-down

    // 3. B?t b? ngu?n ADC (d?c trung STM32L0)
    if ((ADC1->CR & ADC_CR_ADEN) == 0) // N?u ADC chua b?t
    {
        ADC1->CR |= ADC_CR_ADEN;        // B?t ADC
        while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Ch? s?n sàng
    }

    // 4. Hi?u chu?n ADC
    ADC1->CR |= ADC_CR_ADCAL;           // B?t d?u hi?u chu?n
    while (ADC1->CR & ADC_CR_ADCAL);    // Ch? hi?u chu?n xong

    // 5. C?u hình kênh và th?i gian l?y m?u
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;   // Ch?n kênh 0 (PA0)
    ADC1->SMPR |= ADC_SMPR_SMP_2;       // Th?i gian m?u = 160.5 cycles (?n d?nh hon)

    // 6. B?t ADC (l?i, sau hi?u chu?n)
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Ch? s?n sàng
}


uint16_t ADC1_Read(void)
{
    ADC1->CR |= ADC_CR_ADSTART;              // B?t d?u chuy?n d?i
    while (!(ADC1->ISR & ADC_ISR_EOC));      // Ch? xong
    return (uint16_t)ADC1->DR;               // Tr? k?t qu?
}

void USART2_IRQHandler(void)
{
    
   *str = USART2->RDR;
	uint8_t i;
	  if(rx_indx == 0){
		for(i=0;i<100;i++)
			rx_buffer[i]=0;
			GPIOA->BRR = 1<<5 ;//reset
		}
		if(*str != 13){ // rxdata[0] !=13
		   rx_buffer[rx_indx++] = rx_data[0];
		}
		else{
		rx_indx = 0;
			USART2->TDR = *"\n\r";//Transmit
			const char *cmd = "LED ON";
			char *ptr = rx_buffer;  
			while (*ptr && (*ptr == *cmd)) { 
				ptr++;
				cmd++;
			}
			if (*cmd == '\0') {  // N?u duy?t h?t "LED ON" mà không có sai khác
				GPIOA->BSRR = 1 << 5; // Set
			}
		}	
		USART2->TDR = *str;//Transmit
}
*/