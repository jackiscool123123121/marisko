#include "bluetooth.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>

/* ── Onboard Bluetooth module ─────────────────────────────────────────────
 * The nRF52840 has no antenna wired on the SP-1 -- Bluetooth is a SEPARATE
 * chip, an Infineon CYW20706 (inside the CYBT-353027-02 package), talking
 * to the nRF over a UART. The stock module already runs Teenage
 * Engineering's own Bluetooth-Classic A2DP app; this file does NOT replace
 * or reflash that app (a much riskier operation with its own irrecoverable
 * failure mode) -- it just drives it, over the module's already-working
 * AIROC/WICED-HCI command interface.
 *
 * Wiring (UART1, see sp1.dts / sp1-pinctrl.dtsi) and the 115200 8N1 framing
 * are VERIFIED against community reverse-engineering of this same board
 * (bnjreece/feldd-sp1-firmware, captured on real hardware for a different
 * purpose -- replacing the module's app for BLE). This file only uses the
 * documented, standard AIROC HCI Control Protocol (Infineon doc 002-16618;
 * public header: github.com/Infineon/btsdk-include, hci_control_api.h) --
 * the DEVICE command group (pairing mode, inquiry/scan) is boilerplate the
 * Infineon SDK puts in virtually every hci_control-based app, not something
 * TE would have had to customize, and feldd independently confirmed this
 * exact module responds on the DEVICE group. The SET_PAIRING_MODE/INQUIRY
 * opcodes below are ASSUMED correct by that reasoning, not captured
 * directly off this specific SP-1's UART traffic -- that's the one caveat:
 * verify on real hardware before trusting this fully.
 *
 * Packet format (5-byte header + payload), from hci_control_api.h's own
 * doc comment, cross-checked against feldd's one published example packet
 * (`19 01 01 01 00 01`, a GROUP_LE probe -- op-low=0x01, op-high=0x01,
 * len-lo=0x01, len-hi=0x00, payload=0x01):
 *   [0]     0x19 (HCI_WICED_PKT)
 *   [1]     opcode low byte  (command number within the group)
 *   [2]     opcode high byte (group number)
 *   [3..4]  payload length, little-endian
 *   [5..]   payload
 */

#define HCI_WICED_PKT                       0x19u
#define HCI_CONTROL_GROUP_DEVICE            0x00u
#define HCI_CONTROL_COMMAND_INQUIRY         ((HCI_CONTROL_GROUP_DEVICE << 8) | 0x07u)
#define HCI_CONTROL_COMMAND_SET_PAIRING_MODE ((HCI_CONTROL_GROUP_DEVICE << 8) | 0x09u)

static const struct device *s_bt_uart;

void bt_module_init(void)
{
	s_bt_uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
	/* Not fatal if unready -- bt_module_scan() below no-ops safely, same
	 * pattern as this codebase's other optional-peripheral bring-up. */
}

static void send_hci_wiced(uint16_t opcode, const uint8_t *payload, uint16_t len)
{
	if (!s_bt_uart || !device_is_ready(s_bt_uart)) return;

	uint8_t hdr[5] = {
		HCI_WICED_PKT,
		(uint8_t)(opcode & 0xFFu),
		(uint8_t)(opcode >> 8),
		(uint8_t)(len & 0xFFu),
		(uint8_t)(len >> 8),
	};
	for (int i = 0; i < 5; i++) uart_poll_out(s_bt_uart, hdr[i]);
	for (uint16_t i = 0; i < len; i++) uart_poll_out(s_bt_uart, payload[i]);
}

void bt_module_scan(void)
{
	uint8_t on = 1u;
	/* Pairable first, then start the inquiry/scan -- matches the guide's
	 * "scan for nearby bluetooth speakers and headphones" in one gesture.
	 * Auto-reconnect (the module's own boot-time behavior) is untouched;
	 * this only adds the manual scan trigger. */
	send_hci_wiced(HCI_CONTROL_COMMAND_SET_PAIRING_MODE, &on, 1);
	send_hci_wiced(HCI_CONTROL_COMMAND_INQUIRY, &on, 1);
}
