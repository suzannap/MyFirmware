/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx_i2c.h
 *
 * Shared defines for the pozyx driver.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_device.h>

#include "pozyx_extradefs.h"
#include "pozyx_definitions.h"
#include "board_config.h"

device::Device *POZYX_I2C_interface(int bus, const char *devname, uint8_t devaddr);

class POZYX_I2C : public device::I2C
{
public:
	POZYX_I2C(int bus, const char *devname, uint8_t devaddr);
	virtual ~POZYX_I2C();

	virtual int init();
	virtual int read(unsigned address, void *data, unsigned count);
	virtual int write(unsigned address, void *data, unsigned count);

	virtual int ioctl(unsigned operation, unsigned &arg);

protected:
	virtual int probe();

};

