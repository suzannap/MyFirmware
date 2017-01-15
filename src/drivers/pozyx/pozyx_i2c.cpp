//pozyx_i2c.cpp
#include "pozyx_i2c.h"

device::Device *
POZYX_I2C_interface(int bus, const char *devname, uint8_t devaddr)
{
	return new POZYX_I2C(bus, devname, devaddr);
}

POZYX_I2C::POZYX_I2C(int bus, const char *devname, uint8_t devaddr) :
I2C("POZYX_I2C", devname, bus, devaddr, 400000)
{
	_device_id.devid_s.devtype = DRV_POS_DEVTYPE_POZYX;
}

POZYX_I2C::~POZYX_I2C()
{

}

int
POZYX_I2C::init()
{
	/*this will call probe() */
	return I2C::init();
}

int
POZYX_I2C::ioctl(unsigned operation, unsigned &arg)
{
	return 0;
}

int
POZYX_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;

	if (read(POZYX_WHO_AM_I, &data, 1)) {
		DEVICE_DEBUG("reg_read fail");
		PX4_INFO("reg_read fail");
		return -EIO;
	}

	_retries = 2;

	if (data != POZYX_WHOAMI_EXPECTED) {
		DEVICE_DEBUG("ID byte mismatch (%02x)", data);
		PX4_INFO("ID byte mismatch");
		return -EIO;
	}

	return OK;

}

int 
POZYX_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[128];
	if (sizeof(buf)< (count+1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	struct i2c_msg_s writemsg[2];
	unsigned returncount = 0;


	if (IS_FUNCTIONCALL(address)){ //function call with parameters and data in. return value goes to address of parameters
		switch (address) {
			case POZYX_RX_DATA:
				returncount = 100;
			case POZYX_POS_GET_ANCHOR_IDS:
				returncount = 33;
			case POZYX_FLASH_DETAILS:
				returncount = 21;
			case POZYX_DEVICES_GETIDS:
				returncount = 41;
			case POZYX_DEVICE_GETINFO:
				returncount = 24;
			case POZYX_DEVICE_GETCOORDS:
				returncount = 12;
			case POZYX_DEVICE_GETRANGEINFO:
				returncount = 9;
			case POZYX_CIR_DATA:
				returncount = 40;
			default:
				returncount = 1;
		}
		writemsg[0].flags = I2C_M_NORESTART;
		writemsg[0].buffer = &buf[0];
		writemsg[0].length = count + 1;

		writemsg[1].flags = I2C_M_READ;
		writemsg[1].buffer = (uint8_t *)data;
		writemsg[1].length = returncount;
	}
	else { //normal write command

		writemsg[0].flags = I2C_M_NORESTART;
		writemsg[0].buffer = &buf[0];
		writemsg[0].length = 1;

		writemsg[1].flags = 0;
		writemsg[1].buffer = &buf[1];
		writemsg[1].length = count;

	}

	return transfer(writemsg, 2);

}

int
POZYX_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;

	struct i2c_msg_s readmsg[2];


	readmsg[0].flags = I2C_M_NORESTART;
	readmsg[0].buffer = &cmd;
	readmsg[0].length = 1;

	readmsg[1].flags = I2C_M_READ;
	readmsg[1].buffer = (uint8_t *)data;
	readmsg[1].length = count;

	return transfer(readmsg, 2);
}


