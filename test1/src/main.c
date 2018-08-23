
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define GPIO_Pin_n GPIO_PIN_15
#define GPIO_MODE             0x00000003U
#define EXTI_MODE             0x10000000U
#define GPIO_MODE_IT          0x00010000U
#define GPIO_MODE_EVT         0x00020000U
#define RISING_EDGE           0x00100000U
#define FALLING_EDGE          0x00200000U
#define GPIO_OUTPUT_TYPE      0x00000010U

#define GPIO_NUMBER           16U


int main(void) {
	RCC_OscInitTypeDef  RCC_OscInitStructval;
	RCC_OscInitTypeDef* RCC_OscInitStruct = &RCC_OscInitStructval;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__IO uint32_t tmpreg = 0x00U;
	((RCC_TypeDef *) RCC_BASE)->APB1ENR |= RCC_APB1ENR_PWREN;

	/* Delay after an RCC peripheral clock enabling */
	tmpreg = (((RCC_TypeDef *) RCC_BASE)->APB1ENR) & RCC_APB1ENR_PWREN;


	tmpreg = 0x00U;

	((PWR_TypeDef *) PWR_BASE)->CR &= (~(0x1U << (14U)) | (0x1U << (14U)));
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = PWR->CR & PWR_CR_VOS;


	RCC_OscInitStructval.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStructval.HSEState = RCC_HSE_ON;
	RCC_OscInitStructval.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStructval.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStructval.PLL.PLLM = 8;
	RCC_OscInitStructval.PLL.PLLN = 336;
	RCC_OscInitStructval.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStructval.PLL.PLLQ = 4;

	/*------------------------------- HSE Configuration ------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{

	   /* Set the new HSE configuration ---------------------------------------*/
	   __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

	   /* Check the HSE State */
	   if((RCC_OscInitStruct->HSEState) != RCC_HSE_OFF)
	   {
		 /* Wait till HSE is ready */
		 while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET);
	   }
	}

	/*-------------------------------- PLL Configuration -----------------------*/

	if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
	{
	 /* Check if the PLL is used as system clock or not */
	 if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
	 {
	   if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
	   {


		 /* Wait till PLL is ready */
		 while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET);

		 /* Configure the main PLL clock source, multiplication and division factors. */
		RCC->PLLCFGR= (RCC_OscInitStruct->PLL.PLLSource                                            | \
								  RCC_OscInitStruct->PLL.PLLM                                                 | \
								  (RCC_OscInitStruct->PLL.PLLN << POSITION_VAL(RCC_PLLCFGR_PLLN))             | \
								  (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U) << POSITION_VAL(RCC_PLLCFGR_PLLP)) | \
								  (RCC_OscInitStruct->PLL.PLLQ << POSITION_VAL(RCC_PLLCFGR_PLLQ)));

		 /* Enable the main PLL. */
		(*(__IO uint32_t *) RCC_CR_PLLON_BB = ENABLE);

		 /* Wait till PLL is ready */
		 while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET);
	   }
	 }
	}
	//----------------------------------

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK);



	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	uint32_t ticks = HAL_RCC_GetHCLKFreq()/1000;
	if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
	{
	return (1UL);                                                   /* Reload value impossible */
	}

	SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
	SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
				   SysTick_CTRL_TICKINT_Msk   |
				   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */


	SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);


	GPIO_InitTypeDef GPIO_InitStruct;

	//__HAL_RCC_GPIOH_CLK_ENABLE();
	tmpreg = 0x00U;
	/* GPIO Ports Clock Enable */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIOHEN;

	//__HAL_RCC_GPIOD_CLK_ENABLE();
	tmpreg = 0x00U;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = RCC->AHB1ENR & RCC_AHB1ENR_GPIODEN;


	/*Configure GPIO pin Output Level */
	GPIOD->BSRR = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;

	/*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;




	GPIO_InitTypeDef * GPIO_Init = &GPIO_InitStruct;
	GPIO_TypeDef* GPIOx = GPIOD;


	uint32_t position;
	uint32_t ioposition = 0x00U;
	uint32_t iocurrent = 0x00U;
	uint32_t temp = 0x00U;

	/* Configure the port pins */
	for(position = 0U; position < 16U; position++)
	{
	  /* Get the IO position */
	  ioposition = 0x01U << position;
	  /* Get the current IO position */
	  iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

	  if(iocurrent == ioposition)
	  {
		/*--------------------- GPIO Mode Configuration ------------------------*/
		/* In case of Alternate function mode selection */
		if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
		{

		  /* Configure Alternate function mapped with the current IO */
		  temp = GPIOx->AFR[position >> 3U];
		  temp &= ~(0xFU << ((uint32_t)(position & 0x07U) * 4U)) ;
		  temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & 0x07U) * 4U));
		  GPIOx->AFR[position >> 3U] = temp;
		}

		/* Configure IO Direction mode (Input, Output, Alternate or Analog) */
		temp = GPIOx->MODER;
		temp &= ~(GPIO_MODER_MODER0 << (position * 2U));
		temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
		GPIOx->MODER = temp;

		/* In case of Output or Alternate function mode selection */
		if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
		   (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
		{

		  /* Configure the IO Speed */
		  temp = GPIOx->OSPEEDR;
		  temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2U));
		  temp |= (GPIO_Init->Speed << (position * 2U));
		  GPIOx->OSPEEDR = temp;

		  /* Configure the IO Output Type */
		  temp = GPIOx->OTYPER;
		  temp &= ~(GPIO_OTYPER_OT_0 << position) ;
		  temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
		  GPIOx->OTYPER = temp;
		}

		/* Activate the Pull-up or Pull down resistor for the current IO */
		temp = GPIOx->PUPDR;
		temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2U));
		temp |= ((GPIO_Init->Pull) << (position * 2U));
		GPIOx->PUPDR = temp;


	  }
	}


	  //----------------------------------------------

    while (1) {
    	  GPIOD->ODR ^= GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    	  uint32_t tickstart = HAL_GetTick();
    	  uint32_t wait = 1000;

    	  /* Add a period to guarantee minimum wait */
    	  if (wait < HAL_MAX_DELAY)
    	  {
    	     wait++;
    	  }


    	  while((HAL_GetTick() - tickstart) < wait)
    	  {
    	  }
    }
}
