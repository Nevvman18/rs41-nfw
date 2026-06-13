/*
 * clock_override.cpp - external crystal clock source for RS41-NFW
 *
 * Overrides the WEAK SystemClock_Config() supplied by the STM32duino
 * variant so the firmware uses the on-board 24 MHz passive crystal (HSE)
 * instead of the internal HSI.  No changes to variant/board-manager
 * files are needed - the linker automatically prefers a non-weak symbol.
 *
 * Enabled only when USE_EXTERNAL_CRYSTAL is defined in CONFIG.h.
 * Board detection uses ARDUINO_GENERIC_* defines from the STM32duino variant.
 *
 * RSM4x4 (STM32L412RBTxP): 24 MHz crystal on PH0/PH1 → PLL → 80 MHz SYSCLK
 *   PLL: HSE(24) / M=3 → VCOin=8 MHz  × N=20 → VCO=160 MHz  / R=2 → 80 MHz
 *
 * RSM4x2 (STM32F100C8T6B): 24 MHz crystal on PD0/PD1 → 24 MHz SYSCLK (direct)
 *   HSE feeds SYSCLK directly - STM32F100 max is 24 MHz, no PLL needed.
 *
 * NOTE: HAL's HAL_RCC_GetSysClockFreq() relies on the compile-time constant
 * HSE_VALUE which defaults to 8 MHz for generic STM32 boards.  With a 24 MHz
 * crystal this causes HAL_RCC_ClockConfig() to compute SystemCoreClock as
 * 8 MHz (F100) or ~26.7 MHz (L412) and then immediately misconfigure SysTick
 * via HAL_InitTick() - making millis()/delay() run ~3× too fast.
 * Fix: after HAL_RCC_ClockConfig() returns, write the correct SystemCoreClock
 * and call SysTick_Config() to recalibrate the tick before anything else runs.
 */

#include <Arduino.h>

// Comment out to fall back to the internal HSI oscillator.
#define USE_EXTERNAL_CRYSTAL

#ifdef USE_EXTERNAL_CRYSTAL

extern uint32_t SystemCoreClock;

// ── RSM4x4 · STM32L412 ───────────────────────────────────────────────────────
#if defined(ARDUINO_GENERIC_L412RBIXP) || defined(ARDUINO_GENERIC_L412RBTXP)

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  // VOS range 1 required for 80 MHz (matches variant default)
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  // HSE (passive crystal) + PLL → 80 MHz
  //   VCOin  = 24 / 3  =  8 MHz  (spec: 4-16 MHz)  ✓
  //   VCO    =  8 × 20 = 160 MHz (spec: 64-344 MHz) ✓
  //   SYSCLK = 160 / 2 =  80 MHz                    ✓
  // HSI must stay on: MCO1 on PA8 sources from HSI to clock the RPM411 sensor.
  // On L412, HSI is off after reset and the variant's SystemClock_Config() is
  // the one that enables it - our override must do it explicitly.
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM            = 3;
  RCC_OscInitStruct.PLL.PLLN            = 20;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  // HAL_RCC_ClockConfig() called HAL_InitTick() with wrong SystemCoreClock
  // (HSE_VALUE=8MHz → computed ~26.7 MHz).  Fix both the variable and SysTick.
  SystemCoreClock = 80000000UL;
  SysTick_Config(SystemCoreClock / 1000UL);
}

// ── RSM4x2 · STM32F100 ───────────────────────────────────────────────────────
#elif defined(ARDUINO_GENERIC_F100C8TX) || defined(ARDUINO_GENERIC_F100CBTX)

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  // HSE (passive crystal) at 24 MHz used directly as SYSCLK - no PLL needed
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // HSE → SYSCLK = 24 MHz; 0 wait states valid up to 24 MHz on F100
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }

  // HAL_RCC_ClockConfig() called HAL_InitTick() with wrong SystemCoreClock
  // (HSE_VALUE=8MHz → returned 8 MHz for HSE source).  Fix both.
  SystemCoreClock = 24000000UL;
  SysTick_Config(SystemCoreClock / 1000UL);
}

#endif // board selector

#endif // USE_EXTERNAL_CRYSTAL
