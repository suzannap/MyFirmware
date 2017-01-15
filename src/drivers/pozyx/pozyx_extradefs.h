/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx.h
 *
 * Shared defines for the pozyx driver.
 */
//additions to pozyx provided definitions
#define POZYX_I2C_ADDRESS_ALT				0x4A /* Alternate Pozyx tag address (selected by soldering a 0Ohm resistor on the back of the pozyx tag*/

#define DRV_POS_DEVTYPE_POZYX  			0x40
#define POZYX_WHOAMI_EXPECTED			0x43

#define POZYX_CONVERSION_INTERVAL 		(400000 / 150)
#define BUFFER_LENGTH 					128


/* interface factory */
extern device::Device *POZYX_I2C_interface(int bus, const char *devname, uint8_t devaddr);
typedef device::Device *(*POZYX_constructor)(int, const char*, uint8_t);

