/****************************************************************************
 *
 *  Author: Suzanna Paulos
 *	Created: 2016-11-25
 * 	Modified: 2016-11-25
 *
-****************************************************************************/

/**
 * @file pozyx.cpp
 *
 * I2C interface for Pozyx Tag
 */

#include <uORB/uORB.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vehicle_attitude.h>
#include "pozyx.h"

//needed to run daemon
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
//#include <lib/ecl/matrix/matrix/Quaternion.hpp>
//#include <lib/ecl/matrix/matrix/Vector3.hpp>
#include <matrix/math.hpp>


#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int count = 0;

enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL,
	POZYX_BUS_I2C_ALT_INTERNAL,
	POZYX_BUS_I2C_ALT_EXTERNAL
};

int pozyx_pub_main(int argc, char *argv[]);
int pozyx_pub_main_2(int argc, char *argv[]);
extern "C" __EXPORT int pozyx_main(int argc, char *argv[]);

/****************namespace**********************************************/
namespace pozyx
{

	//list of supported bus configurations
	struct pozyx_bus_option {
		enum POZYX_BUS busid;
		const char *devpath;
		POZYX_constructor interface_constructor;
		uint8_t busnum;
		uint8_t devaddr;
		PozyxClass *dev;
		uint8_t index;
	} bus_options[] = {
		{ POZYX_BUS_I2C_INTERNAL, "/dev/pozyx_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, POZYX_I2C_ADDRESS, NULL , 0},
		{ POZYX_BUS_I2C_EXTERNAL, "/dev/pozyx_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, POZYX_I2C_ADDRESS, NULL, 1 },
		{ POZYX_BUS_I2C_ALT_INTERNAL, "/dev/pozyx_alt_int", &POZYX_I2C_interface, PX4_I2C_BUS_ONBOARD, POZYX_I2C_ADDRESS_ALT, NULL, 2 },
		{ POZYX_BUS_I2C_ALT_EXTERNAL, "/dev/pozyx_alt_ext", &POZYX_I2C_interface, PX4_I2C_BUS_EXPANSION, POZYX_I2C_ADDRESS_ALT, NULL, 3 },
	};
		
	const int NUM_BUS_OPTIONS = sizeof(bus_options)/sizeof(bus_options[0]);

	int 	start(enum POZYX_BUS busid);
	bool 	start_bus(struct pozyx_bus_option &bus);
	struct 	pozyx_bus_option &find_bus(enum POZYX_BUS busid, unsigned startid);
	void 	test(enum POZYX_BUS busid, int count);
	void 	config(enum POZYX_BUS busid, int count);
	void	reset(enum POZYX_BUS busid, int count);
	void	getposition(enum POZYX_BUS busid, int count, bool print_result);
	void	addanchor(enum POZYX_BUS busid, int count, uint16_t network_id, int32_t x, int32_t y, int32_t z);
	void	autoanchors(enum POZYX_BUS busid, int count);
	void	getanchors(enum POZYX_BUS busid, int count);
	void	clearanchors(enum POZYX_BUS busid, int count);
	void	getuwb(enum POZYX_BUS busid, int count);
	void	setuwb(enum POZYX_BUS busid, int count, uint8_t bitrate, uint8_t prf, uint8_t plen, float gain_db);
	void	resettofactory(enum POZYX_BUS busid, int count);
	void	clearanchors(enum POZYX_BUS busid, int count);
	void	usage();


	//start driver for specific bus option
	bool 
	start_bus(struct pozyx_bus_option &bus)
	{
		if (bus.dev != nullptr) {
			errx(1, "bus option already started");
		}

		device::Device *interface = bus.interface_constructor(bus.busnum, bus.devpath, bus.devaddr);


		if (interface->init() != OK) {
			delete interface;
			warnx("no device on bus %u", (unsigned)bus.busid);
			return false;
		}

		bus.dev = new PozyxClass(interface);
		usleep(100000);


		int fd = px4_open(bus.devpath, O_RDONLY);
		usleep(100000);

		if (fd < 0) {
			return false;
		}

		close(fd);

		return true;
	}

	//start the driver
	int
	start(enum POZYX_BUS busid)
	{
		int started = 0;
		for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
			if (busid == POZYX_BUS_ALL && bus_options[i].dev != NULL) {
				//this device is already started
				continue;
			}

			if (busid != POZYX_BUS_ALL && bus_options[i].busid != busid) {
				//not the one that is asked for
				continue;
			}

			if (start_bus(bus_options[i])) {
				started ++;
			}
		}
		PX4_INFO("%d Pozyx tags found", started);
		return started;
	}

	//find bus structure for a busid
	struct pozyx_bus_option &find_bus(enum POZYX_BUS busid, unsigned startid)
	{
		for (unsigned i=startid; i < NUM_BUS_OPTIONS; i++) {
			if (((busid == POZYX_BUS_ALL) || (busid == bus_options[i].busid)) && (bus_options[i].dev != NULL)) {
				return bus_options[i];
			}
		}
		errx(1, "No active bus found");
	}


	//basic functional tests
	void
	test(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;
		int testread;
		

		for (int i=0; i<count; i++){	
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;	

			const char *path = bus.devpath;
			int fd = px4_open(path, O_RDONLY);

			if (fd < 0) {
				err(1, "%s open failed (try 'pozyx start')", path);			
			}
			
			uint8_t whoami = 0;
			testread = bus.dev->regRead(POZYX_WHO_AM_I, &whoami, 1);
			PX4_INFO("value of whoami is: 0x%x", whoami);

			if (testread != POZYX_SUCCESS) {
				err(1, "immediate read failed");
			}
			else {
				if (whoami == POZYX_WHOAMI_EXPECTED){
					PX4_INFO("Who Am I Check Successful");
				}
				else{
					PX4_INFO("Who Am I Check Failed: 0x%x",whoami);
				}
			}
			//test a function call by blinking LED3
			uint8_t funcbuf[100];
			funcbuf[0] = 0x44;

			bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
			if (funcbuf[0] != 1) {
				err(1, "Function test failed");
			}
			PX4_INFO("LED3 turned On... ");
			sleep(2);
			funcbuf[0] = 0x40;
			bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
			if (funcbuf[0] != 1) {
				err(1, "Function test failed");
			}
			PX4_INFO("LED3 turned Off");

			PX4_INFO("Tag %d PASS", bus.index);
		}
	}

	void
	reset(enum POZYX_BUS busid, int count)
	{
		exit(0);
	}

	void
	getposition(enum POZYX_BUS busid, int count, bool print_result)
	{

		coordinates_t poz_coordinates[count];
		struct att_pos_mocap_s pos;
		unsigned startid = 0;
		int validcount = 0;
		int totalerror = 0;


		pos.x = 0;
		pos.y = 0;
		pos.z = -5;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_3D)){
				if (print_result) {
					PX4_INFO("Current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
				}
				pos_error_t poz_error;
				if (POZYX_SUCCESS == bus.dev->getPositionError(&poz_error)){
					if (print_result) {
						PX4_INFO("Position covariance: x(%d) y(%d) z(%d) xy(%d) xz(%d) yz(%d)", poz_error.x, poz_error.y, poz_error.z, poz_error.xy, poz_error.xz, poz_error.yz);
					}
					totalerror = abs(poz_error.x) + abs(poz_error.y) + abs(poz_error.z) + abs(poz_error.xy) + abs(poz_error.xz) + abs(poz_error.yz);
					if(totalerror > 800 || totalerror < 10) {
						//position covariance is too big, not a good reading
						poz_coordinates[i].x = 0;
						poz_coordinates[i].y = 0;
					}
					else {
						validcount += 1;
					}
				}
			}
			pos.x += poz_coordinates[i].x;
			pos.y += poz_coordinates[i].y;
			//pos.z += poz_coordinates[i].z;


			//if (count == 1) {
			if (i == 0) {
				quaternion_t poz_orientation;
				if (POZYX_SUCCESS == bus.dev->getQuaternion(&poz_orientation)){
					if (print_result) {
						PX4_INFO("Current orientation: %1.4f  %1.4f  %1.4f  %1.4f", (double)poz_orientation.weight, (double)poz_orientation.x, (double)poz_orientation.y, (double)poz_orientation.z);
					}
					//change orientation from NWU to NED rotate 180 degrees about x
					//[q0, q1, q2, q3] * [0, 1, 0, 0] = [-q1, q0, q3, -q2]
					pos.q[0] = -poz_orientation.x;
					pos.q[1] = poz_orientation.weight;
					pos.q[2] = poz_orientation.z;
					pos.q[3] = -poz_orientation.y;		
					//change orientation from funny vertical to NED rotate -90 degrees about z and +90 about y
					//[q0, q1, q2, q3] >>> [q1, -q2, -q0, -q3]
					pos.q[0] = poz_orientation.x;
					pos.q[1] = -poz_orientation.y;
					pos.q[2] = -poz_orientation.weight;
					pos.q[3] = poz_orientation.z;	
				}			
			}
		}


		//if (validcount > 1) {
		if (false) {
			double yaw = atan2 ((poz_coordinates[1].y - poz_coordinates[0].y),(poz_coordinates[1].x - poz_coordinates[0].x));

			if (print_result) {
				PX4_INFO("Current yaw: %f deg.", (yaw * 180 / 3.14159));
			}
			matrix::Vector3f pozyx_orient(0, 0, yaw);
			matrix::Quatf myq(1.0, 0, 0, 0);
			myq.from_axis_angle(pozyx_orient);
			pos.q[0] = myq(0);
			pos.q[1] = myq(1);
			pos.q[2] = myq(2);
			pos.q[3] = myq(3);
		}	

		pos.timestamp = hrt_absolute_time();

		if (validcount == count) {
			//change position from NWU to NED and from m to mm
			pos.x /= (validcount*1000);
			pos.y /= (-validcount*1000);
			//pos.z /= (-count*1000);
			orb_advert_t pos_pub = orb_advertise(ORB_ID(att_pos_mocap), &pos);
			orb_publish(ORB_ID(att_pos_mocap), pos_pub, &pos);
		}
		else {
			if (print_result) {
				PX4_INFO("No valid RTLS measurements");
			}
		}
	}

	void
	config(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			uint8_t num_anchors =4;

			
			//R&D
			device_coordinates_t anchorlist[num_anchors] = {
				{0x6857, 1, {0, 0, 1902}},
				{0x6827, 1, {-3106, 0, 1963}},
				{0x684E, 1, {877, -6337, 1828}},
				{0x6853, 1, {-3789, -6745, 1635}}
			};
			/*
			//Building 9
			device_coordinates_t anchorlist[num_anchors] = {
				{0x683b, 1, {3846 -213, 2068}},
				{0x685b, 1, {5664, -21178, 2180}},
				{0x6826, 1, {-122, -9874, 1841}},
				{0x6854, 1, {11381, -949, 8242}}
			};
			*/

			if (bus.dev->clearDevices() == POZYX_SUCCESS){
				for (int j = 0; j < num_anchors; j++) {
					if (bus.dev->addDevice(anchorlist[j]) != POZYX_SUCCESS) {
						PX4_INFO("failed to add anchor");
						exit(1);
					}
					PX4_INFO("Anchor 0x%x successfully added at (%d, %d, %d)", anchorlist[j].network_id, anchorlist[j].pos.x, anchorlist[j].pos.y, anchorlist[j].pos.z);
				}
				if (bus.dev->getDeviceListSize(&num_anchors) == POZYX_SUCCESS) {
					PX4_INFO("%d anchors configured", num_anchors);
				}
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("%d anchors saved", num_anchors);
				}
			}
		}
	}

	void
	addanchor(enum POZYX_BUS busid, int count, uint16_t network_id, int32_t x, int32_t y, int32_t z)
	{
		unsigned startid = 0;
		PX4_INFO("Adding anchor 0x%x at coordinates (%d, %d, %d)...", network_id, x, y, z);

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;
			device_coordinates_t anchor = {network_id, 1, {x, -y, -z}};
			if (bus.dev->addDevice(anchor) == POZYX_SUCCESS){
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("Anchor 0x%x added to tag %d", network_id, bus.index);
				}
			}
		}
	}

	void
	clearanchors(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (bus.dev->clearDevices() == POZYX_SUCCESS){
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("All anchors cleared from tag %d", bus.index);
				}
			}
		}
	}

	void
	getanchors(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;
			uint8_t device_list_size;

			if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
				uint16_t anchors[device_list_size];
				PX4_INFO("Found %d anchors configured on tag %d", device_list_size, bus.index);
				if (bus.dev->getAnchorIds(anchors, device_list_size) == POZYX_SUCCESS) {
					coordinates_t coordinates;
					for (int j=0; j<device_list_size; j++){
						if (bus.dev->getDeviceCoordinates(anchors[j], &coordinates) == POZYX_SUCCESS){
							PX4_INFO("   Anchor 0x%x at (%d, %d, %d)", anchors[j], coordinates.x, coordinates.y, coordinates.z);
						}
					}
				}
			}
		}
	}


	void
	setuwb(enum POZYX_BUS busid, int count, uint8_t bitrate, uint8_t prf, uint8_t plen, float gain_db)
	{
		unsigned startid = 0;

		uint8_t bitrates[3] = {0, 1, 2};
		uint8_t prfs[2] = {1, 2};
		uint8_t plens[8] = {0x0C, 0x28, 0x18, 0x08, 0x34, 0x24, 0x14, 0x04};
		UWB_settings_t mysettings;
		mysettings.channel = 5;
		mysettings.bitrate = bitrates[bitrate];
		mysettings.prf = prfs[prf];
		mysettings.plen = plens[plen];
		mysettings.gain_db = gain_db;

		uint8_t device_list_size;


		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
				
				/*
				if (device_list_size > 0){
					uint16_t devices[device_list_size];
					if (bus.dev->getDeviceIds(devices, device_list_size) == POZYX_SUCCESS) {
						for (int j=0; j<device_list_size; j++){
							usleep(100000);
							if (bus.dev->setUWBSettings(&mysettings, devices[j]) == POZYX_SUCCESS){
								PX4_INFO("UWB settings updated on anchor 0x%x", devices[j]);
							}
						}
					}
				}
				*/
				usleep(100000);
				if (bus.dev->setUWBSettings(&mysettings) == POZYX_SUCCESS){
					//if (bus.dev->saveConfiguration(POZYX_FLASH_REGS, changedregs, 3) == POZYX_SUCCESS) {
						PX4_INFO("UWB settings updated on tag %d", bus.index);
					//}
				}
			}
		}
	}

	void
	getuwb(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;
		UWB_settings_t mysettings;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (bus.dev->getUWBSettings(&mysettings) == POZYX_SUCCESS){
				PX4_INFO("UWB settings on tag %d: channel %d, bitrate %d, prf %d, plen 0x%x, gain_db %1.1f", bus.index, mysettings.channel, mysettings.bitrate, mysettings.prf, mysettings.plen, (double)mysettings.gain_db);
			}
		}
	}


	void
	resettofactory(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			bus.dev->resetSystem();
			if (bus.dev->clearConfiguration() == POZYX_SUCCESS) {
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					if (bus.dev->saveConfiguration(POZYX_FLASH_NETWORK) == POZYX_SUCCESS) {
						PX4_INFO("Tag %d reset to factory settings", bus.index);
					}
				}
			}			
		}
	}

	void
	usage()
	{
		warnx("usage: try 'start', 'stop', 'status', 'config', 'test'");
		warnx("Debug functions:");
		warnx("clearanchors");
		warnx("addanchor [anchorID] [x position] [y position] [z position]");
		warnx("getanchors");
		warnx("getposition");
		warnx("getuwb");
		warnx("setuwb [bitrate] [prf] [plen] [gain_db] (see www.pozyx.io/Documentation/Tutorials/uwb_settings for more info)");
		warnx("resettofactory");
	}

} //namespace


int
pozyx_main(int argc, char *argv[])
{
	//int ch;
	enum POZYX_BUS busid = POZYX_BUS_ALL;
	int testnum = 0;

	const char *verb = argv[1];

	//start/load driver and begin cycles
	if (!strcmp(verb, "start")) {
		count = pozyx::start(busid);
		if (count > 0) {
			pozyx::config(busid, count);

			if (thread_running) {
				warnx("pozyx already running\n");
				exit(0);
			}

			thread_should_exit = false;
			if (count == 1) {
				daemon_task = px4_task_spawn_cmd("pozyx_pub", 
												SCHED_DEFAULT, 
												SCHED_PRIORITY_DEFAULT, 
												2000, 
												pozyx_pub_main,
												(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
			}
			if (count == 2) {
				daemon_task = px4_task_spawn_cmd("pozyx_pub_2", 
												SCHED_DEFAULT, 
												SCHED_PRIORITY_DEFAULT, 
												2000, 
												pozyx_pub_main_2,
												(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
			}

		}
		exit(0);
	}

	//stop the daemon
	if (!strcmp(verb, "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(verb, "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	//test driver/device
	if (!strcmp(verb, "test")) {
		pozyx::test(busid, count);
		exit(0);
	}
	//configure pozyx
	if (!strcmp(verb, "config")) {
		pozyx::config(busid, count);
		exit(0);
	}

	//fetch positions
	if (!strcmp(verb, "getposition")) {
		pozyx::getposition(busid, count, true);
		exit(0);
	}

	//debug
	if (!strcmp(verb, "debug")) {
		for (int i = 1; i < argc; i++) {
			if (strcmp(argv[i], "-N") == 0) {
				if (argc > i + 1) {
					testnum = atoi(argv[i + 1]);
				}
			}
		}
		PX4_INFO("testnumber = %d", testnum);
		exit(0);
	}

	//clear anchors	
	if (!strcmp(verb, "clearanchors")) {
		pozyx::clearanchors(busid, count);
		exit(0);
	}

	//add an anchor
	if (!strcmp(verb, "addanchor")) {
		if (argc == 6) {
			uint16_t id = strtol(argv[2], NULL, 16);
			uint32_t x = atoi(argv[3]);
			uint32_t y = atoi(argv[4]);
			uint32_t z = atoi(argv[5]);
			pozyx::addanchor(busid, count, id, x, y, z);
		}
		else {			
			PX4_INFO("wrong number of arguments to add anchor. Requires ID(hex), X, Y, Z (mm)");
			PX4_INFO("example: pozyx addanchor A23D 100 200 300");
			exit(1);
		}
		exit(0);
	}
	//get anchors	
	if (!strcmp(verb, "getanchors")) {
		pozyx::getanchors(busid, count);
		exit(0);
	}

	//set UWB parameters
	if (!strcmp(verb, "setuwb")) {
		if (argc == 6) {
			uint8_t bitrate = atoi(argv[2]);
			uint8_t prf = atoi(argv[3]);
			uint8_t plen = atoi(argv[4]);
			float gain_db = atoi(argv[5])/2.0;
			pozyx::setuwb(busid, count, bitrate, prf, plen, gain_db);
		}
		else {			
			PX4_INFO("wrong number of arguments to configure UWB settings. Requires bitrate, prf, plen, gain_db");
			PX4_INFO("Possible value of bitrate:   0: 110kbits/s    1: 850kbits/s    2: 6.8Mbits/s");
			PX4_INFO("Possible value of prf:   0: 16MHz    1: 64MHz");
			PX4_INFO("Possible value of plen: 0-7: 4096-64 symbols. See Pozyx documentation.");
			PX4_INFO("Possible value of gain_db: integer values 0-67, will be halved to range of 0-33.5dB");
			exit(1);
		}
		exit(0);
	}
	//get UWB parameters	
	if (!strcmp(verb, "getuwb")) {
		pozyx::getuwb(busid, count);
		exit(0);
	}
	//reset to factory settings
	if (!strcmp(verb, "resettofactory")) {
		pozyx::resettofactory(busid, count);
		exit(0);
	}
	
	pozyx::usage();;
	exit(0);
}


int 
pozyx_pub_main(int argc, char *argv[])
{
	warnx("[pozyx_pub] starting\n");
	thread_running = true;

	while (!thread_should_exit) {
		pozyx::getposition(POZYX_BUS_ALL, 1, false);
		usleep(300000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}

int 
pozyx_pub_main_2(int argc, char *argv[])
{
	warnx("[pozyx_pub] starting\n");
	thread_running = true;

	while (!thread_should_exit) {
		pozyx::getposition(POZYX_BUS_ALL, 2, false);
		usleep(600000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}