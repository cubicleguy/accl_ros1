//==============================================================================
//
//  hcl_gpio.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call generic
//  functions to work on multiple hardware platforms, this is generic
//  implementation for GPIO function which may be needed for optionally
//  connecting pins RESET#, SPI Chipselect, DataReady, EXT of the sensor.
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================

#include "hcl.h"
#include "hcl_gpio.h"

/*****************************************************************************
** Function name:       gpioInit
** Description:         Initialize generic GPIO library (if any)
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int gpioInit(void) {
  /* if GPIO pins will be used, insert initialization code here */

  return SUCCESS;
}

/*****************************************************************************
** Function name:       gpioInit
** Description:         Release generic GPIO interface (if any)
** Parameters:          None
** Return value:        SUCCESS or FAIL
*****************************************************************************/
int gpioRelease(void) {
  /* if GPIO pins will be used, insert cleanup code here */

  return SUCCESS;
}

/*****************************************************************************
** Function name:       gpioSet
** Description:         Generic GPIO pin set to HIGH
** Parameters:          Pin number
** Return value:        None
*****************************************************************************/
void gpioSet(uint8_t pin) {
  /* if GPIO pins will be used, insert platform specific code to set a
   * GPIO here */

  return;
}

/*****************************************************************************
** Function name:       gpioClr
** Description:         Generic GPIO pin set to LOW
** Parameters:          Pin number
** Return value:        None
*****************************************************************************/
void gpioClr(uint8_t pin) {
  /* if GPIO pins will be used, insert platform specific code to clear a
   * GPIO here */

  return;
}

/*****************************************************************************
** Function name:       gpioGetPinLevel
** Description:         Generic read GPIO pin status
** Parameters:          Pin number
** Return value:        1 = GPIO pin HIGH, 0 = GPIO pin LOW
*****************************************************************************/
uint8_t gpioGetPinLevel(uint8_t pin) {
  /* if GPIO pins will be used, insert platform specific code to read
   * GPIO and return pin level here */

  return 0;
}
