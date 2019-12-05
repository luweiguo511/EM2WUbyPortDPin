/**************************************************************************//**
 * @file
 * @brief Demonstrates basic interrupt functionality on GPIO using push buttons
 * @version 0.0.1
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2019 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"
 * for details. Before using this software for any purpose, you must agree to the
 * terms of that agreement.
 *
 ******************************************************************************/

/* README
This example is meant to run on a WSTK (BRD4001) with a daughter card (BRD4181)

GPIO_INTERRUPT 
This examples demonstrates how to set up inputs and configure external
interrupts based on these inputs. Push buttons PB0 (Port D pin 02) and PB1
(Port D pin 03) are the external inputs. The external interrupt is set up on a
falling edge. Since the pin numbers on PB0 and PB1 are distinct, we will use the
default GPIO_IntConfig. If for some reason, both the buttons you have chosen are
the same pin (e.g. Port D pin 0 and Port C pin 0), you might want to use
GPIO_ExtIntConfig instead and map them to distinct external interrupts.

PB0 controls LED0 and PB1 controls LED1. Every interrupt toggles the LED. 

Because EFR32MG21 can only wake from EM2/3 from GPIO interrupts on ports A or B,
push buttons on the WSTK (connected to port D) must be jumped to an interrupt
capable port. For this example, jump pins 7 and 9 to pins 14 and 12, respectively,
on the Expansion Header of the WSTK.
*/

#include "em_chip.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "bsp.h"

// PB0 port/pin definition
#define  BUTTON_0_PORT  gpioPortD
#define  BUTTON_0_PIN   2

#define  BUTTON_1_PORT  gpioPortD
#define  BUTTON_1_PIN   3

#define  EM4WU_EM4WUEN_NUM   (9)                       // PD2 is EM4WUEN pin 9
#define  EM4WU_EM4WUEN_MASK  (1 << EM4WU_EM4WUEN_NUM)

/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons and PA0 output.
 *****************************************************************************/
static void gpioSetup(void)
{
  /* Configure Button PB0 as input and enable interrupt */
  GPIO_PinModeSet(BUTTON_0_PORT, BUTTON_0_PIN, gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(BUTTON_1_PORT, BUTTON_1_PIN, gpioModeInputPullFilter, 1);

  GPIO_EM4EnablePinWakeup(EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);

  GPIO->IEN = EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT;

  /* Enable ODD interrupt to catch button press that changes slew rate */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* Configure pB00 as a push pull output for LED drive */
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 1);
}

/**************************************************************************//**
 * @brief GPIO Interrupt handler for even pins.
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Check if button 0 was pressed */
  if (interruptMask & (EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT))
  {
    GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
  }
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* Initialize Push Buttons and PA0 */
  gpioSetup();

  // toggle catch; allows debugger to re-attach and reprogram
  while (GPIO_PinInGet(BUTTON_1_PORT, BUTTON_1_PIN) == 1);
  while (GPIO_PinInGet(BUTTON_1_PORT, BUTTON_1_PIN) == 0);

  // indicate we're past the catch
  GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);

  while(1)
  {
    /* Enter EM3 until one of the push-button interrupts triggers a wake-up */
    EMU_EnterEM2(false);
  }
}
