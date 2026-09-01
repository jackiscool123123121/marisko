#include "battery.h"

#include <soc.h>

#include "saadc.h"

/* ---- BQ24232 battery charger ----
 * Pins verified against community reverse-engineering of this same board
 * (SP-1-dev wiki / the sp1-tape-looper firmware): /CE=P0.21 (active-low
 * charge enable), /CHG=P0.22 (open-drain, LOW=charging now), /PGOOD=P0.24
 * (open-drain, LOW=USB power present). None of these three pins are touched
 * anywhere else in this firmware.
 *
 * Battery voltage is sensed on AIN4 through an on-board divider -- AIN4 is
 * otherwise unused here (button ladders are on AIN0/AIN1, faders on
 * AIN2/AIN3/AIN6/AIN7), using the exact same SAADC path (gain 1/6, 0.6 V
 * internal ref, 12-bit) as everything else that already calls saadc_read().
 *
 * RAW_EMPTY/RAW_FULL are an inherited, UNCALIBRATED starting point (same
 * ones sp1-tape-looper used, on the same divider/gain path): RAW_FULL was
 * actually measured at rest (~4.21 V); RAW_EMPTY is a physics estimate for
 * a ~3.3 V cutoff, not a real measured low reading. Treat battery_percent()
 * as approximate until a real near-empty reading is logged on this unit and
 * these two constants get corrected. */
#define BQ_NCE_PIN     21u
#define BQ_NCHG_PIN    22u
#define BQ_NPGOOD_PIN  24u
#define BATT_RAW_EMPTY 1900
#define BATT_RAW_FULL  2380

void charger_init(void)
{
	NRF_P0->PIN_CNF[BQ_NCHG_PIN] =
		(GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)  |
		(GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos) |
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
	NRF_P0->PIN_CNF[BQ_NPGOOD_PIN] =
		(GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)  |
		(GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos) |
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
	NRF_P0->OUTCLR = (1u << BQ_NCE_PIN);        /* drive low first */
	NRF_P0->PIN_CNF[BQ_NCE_PIN] =
		(GPIO_PIN_CNF_DIR_Output    << GPIO_PIN_CNF_DIR_Pos)  |
		(GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)|
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
	NRF_P0->OUTCLR = (1u << BQ_NCE_PIN);        /* /CE low = charge enabled */
}

bool usb_power_present(void)
{
	return (NRF_P0->IN & (1u << BQ_NPGOOD_PIN)) == 0u;   /* low = USB power good */
}

bool battery_charging(void)
{
	return (NRF_P0->IN & (1u << BQ_NCHG_PIN)) == 0u;     /* low = charging */
}

int battery_percent(void)
{
	int raw = saadc_read(SAADC_CH_PSELP_PSELP_AnalogInput4);
	if (raw < 0) return -1;
	int pct = (raw - BATT_RAW_EMPTY) * 100 / (BATT_RAW_FULL - BATT_RAW_EMPTY);
	if (pct < 0)   pct = 0;
	if (pct > 100) pct = 100;
	return pct;
}

int battery_quarters(void)
{
	int pct = battery_percent();
	if (pct < 0) return 0;
	int q = pct / 25 + 1;
	return (q > 4) ? 4 : q;
}
