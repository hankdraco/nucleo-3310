#ifndef TRANSMISSION_CONTROLLER_H
#define TRANSMISSION_CONTROLLER_H

#include <stdbool.h>

/*
 * Rozpoczyna odbiór danych z LPUART1.
 * Wywołać po BSP_COM_Init() i input_state_init().
 */
bool transmission_controller_start(void);

/*
 * Informuje, czy wystąpił błąd uniemożliwiający
 * kontynuowanie odbioru.
 */
bool transmission_controller_has_fault(void);

#endif /* TRANSMISSION_CONTROLLER_H */