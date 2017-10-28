/*
 * app_lcd.c
 *
 *  Created on: 28 Oct 2017
 *      Author: wang_p
 */


#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"

#include "../Utilities/Log/lcd_log.c"

/**
  * @brief  Setup the BSP.
  * @param  None
  * @retval None
  */
void LCD_APP_init(void)
{

  /* Configure LED1, LED2, LED3 and LED4 */
//  BSP_LED_Init(LED1);
//  BSP_LED_Init(LED2);
//  BSP_LED_Init(LED3);
//  BSP_LED_Init(LED4);

  /* Set Systick Interrupt to the highest priority */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);

//  /* Init MFX */
//  BSP_IO_Init();
//  /* Enable MFX interrupt for ETH MII pin */
//  BSP_IO_ConfigPin(MII_INT_PIN, IO_MODE_IT_FALLING_EDGE);

  /* Initialize the LCD */
  BSP_LCD_Init();

  /* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Initialize LCD Log module */
  LCD_LOG_Init();

  /* Show Header and Footer texts */
  LCD_LOG_SetHeader((uint8_t *)"TCP Test");
  LCD_LOG_SetFooter((uint8_t *)"STM32756G-EVAL board");

  LCD_UsrLog ("  State: Ethernet Initialization ...\n");

}
