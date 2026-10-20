#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include <stdbool.h>

/** Start receiving input frames from LPUART1.
 * Call after BSP_COM_Init() and input_state_init().
 * @return true if UART receive was started; false if startup failed.
 */
bool transmission_controller_start(void);

/** Check whether reception has entered a nonrecoverable fault state.
 * @return true if the controller can no longer continue receiving.
 */
bool transmission_controller_has_fault(void);

#endif /* TRANSMISSION_CONTROLLER_H */
