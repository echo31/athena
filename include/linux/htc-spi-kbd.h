/* Definitions for the HTC SPI keyboard driver found in
 * drivers/input/keyboard/htc-spi-kbd.c
 */

#define HTC_SPI_KBD_MAX_KEYS 64

struct htc_spi_kbd_keys {
	int id;
	int keycode;
};

struct htc_spi_kbd_platform_data {
	struct htc_spi_kbd_keys keys[HTC_SPI_KBD_MAX_KEYS];
};
