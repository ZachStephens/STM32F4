
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define GPIO_Pin_n GPIO_PIN_15

int main(void) {
	  RCC_OscInitTypeDef  RCC_OscInitStructval;
	  RCC_OscInitTypeDef* RCC_OscInitStruct = &RCC_OscInitStructval;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	  __HAL_RCC_PWR_CLK_ENABLE();

	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  RCC_OscInitStructval.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStructval.HSEState = RCC_HSE_ON;
	  RCC_OscInitStructval.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStructval.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStructval.PLL.PLLM = 8;
	  RCC_OscInitStructval.PLL.PLLN = 336;
	  RCC_OscInitStructval.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStructval.PLL.PLLQ = 4;

	   /* Check the parameters */
	   assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));
	   /*------------------------------- HSE Configuration ------------------------*/
	   if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	   {
	     /* Check the parameters */
	     assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));
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
	   /* Check the parameters */
	   assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
	   if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
	   {
	     /* Check if the PLL is used as system clock or not */
	     if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
	     {
	       if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
	       {
	         /* Check the parameters */
	         assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
	         assert_param(IS_RCC_PLLM_VALUE(RCC_OscInitStruct->PLL.PLLM));
	         assert_param(IS_RCC_PLLN_VALUE(RCC_OscInitStruct->PLL.PLLN));
	         assert_param(IS_RCC_PLLP_VALUE(RCC_OscInitStruct->PLL.PLLP));
	         assert_param(IS_RCC_PLLQ_VALUE(RCC_OscInitStruct->PLL.PLLQ));

	         /* Disable the main PLL. */
	         __HAL_RCC_PLL_DISABLE();

	         /* Wait till PLL is ready */
	         while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET);

	         /* Configure the main PLL clock source, multiplication and division factors. */
	         WRITE_REG(RCC->PLLCFGR, (RCC_OscInitStruct->PLL.PLLSource                                            | \
	                                  RCC_OscInitStruct->PLL.PLLM                                                 | \
	                                  (RCC_OscInitStruct->PLL.PLLN << POSITION_VAL(RCC_PLLCFGR_PLLN))             | \
	                                  (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U) << POSITION_VAL(RCC_PLLCFGR_PLLP)) | \
	                                  (RCC_OscInitStruct->PLL.PLLQ << POSITION_VAL(RCC_PLLCFGR_PLLQ))));
	         /* Enable the main PLL. */
	         __HAL_RCC_PLL_ENABLE();

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



	  //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	  /* Check the parameters */
	  assert_param(IS_SYSTICK_CLK_SOURCE(CLKSource));

      SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);


	  GPIO_InitTypeDef GPIO_InitStruct;

	  //__HAL_RCC_GPIOH_CLK_ENABLE();
	  __IO uint32_t tmpreg = 0x00U;
	  /* GPIO Ports Clock Enable */
	  do {

		  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
		  /* Delay after an RCC peripheral clock enabling */
		  tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);
		  UNUSED(tmpreg);
	   } while(0U);
	  //__HAL_RCC_GPIOD_CLK_ENABLE();

	  do {
	  		  tmpreg = 0x00U;
	  		  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
	  		  /* Delay after an RCC peripheral clock enabling */
	  		  tmpreg = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
	  		  UNUSED(tmpreg);
	  	   } while(0U);


	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

	  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    while (1) {
    	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
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
