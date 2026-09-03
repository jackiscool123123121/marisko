#pragma once

/* Onboard Infineon CYW20706 Bluetooth module (stock TE firmware, Classic
 * A2DP). See bluetooth.c for the protocol note and hardware verification
 * source. */

/* Bring up the UART to the module. Call once at boot. */
void bt_module_init(void);

/* Ask the stock module app to become pairable and start an inquiry (scan)
 * for nearby devices -- the TE user guide's "hold vol+/vol- to scan for
 * nearby bluetooth speakers and headphones" gesture. Fire-and-forget: this
 * sends the standard AIROC/WICED-HCI commands and does not wait for or
 * parse a response. */
void bt_module_scan(void);
