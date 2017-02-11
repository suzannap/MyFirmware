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

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/pozyx_status.h>
#include <uORB/topics/pozyx_tagstatus.h>
#include <uORB/topics/pozyx_position.h>
#include <uORB/topics/pozyx_anchor.h>
#include <uORB/topics/pozyx_uwb.h>

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
int pozyx_commands(int argc, char *argv[]);
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
	void	testorb(enum POZYX_BUS busid, int count);
	void 	config(enum POZYX_BUS busid, int count);
	void	reset(enum POZYX_BUS busid, int count);
	void	getposition(enum POZYX_BUS busid, int count, bool print_result);
	void	getpositionorb(enum POZYX_BUS busid, int count);
	void	getpositiontest(enum POZYX_BUS busid, int count, bool print_result);
	void	addanchor(enum POZYX_BUS busid, int count, uint16_t network_id, int32_t x, int32_t y, int32_t z);
	void	autoanchors(enum POZYX_BUS busid, int count);
	void	getanchors(enum POZYX_BUS busid, int count);
	void	getanchorsorb(enum POZYX_BUS busid, int count);
	void	clearanchors(enum POZYX_BUS busid, int count);
	void	getuwb(enum POZYX_BUS busid, int count);
	void	getuwborb(enum POZYX_BUS busid, int count);
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

	//Basic Functional tests with result sent over uORB
	void
	testorb(enum POZYX_BUS busid, int count)
	{

		struct pozyx_tagstatus_s status;
		memset(&status, 0, sizeof(status));
		orb_advert_t status_pub_fd = orb_advertise(ORB_ID(pozyx_tagstatus), &status);

		unsigned startid = 0;
		int testread;
		

		for (int i=0; i<count; i++){	
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;	

			const char *path = bus.devpath;
			int fd = px4_open(path, O_RDONLY);
			status.result = 0;


			if (fd < 0) {
				//err(1, "%s open failed (try 'pozyx start')", path);
				status.result += 1;			
			}
		
			//Get Tag Index
			status.id = bus.index;

			//Get Tag ID
			uint16_t devid = 0;
			testread = bus.dev->getNetworkId(&devid);
			status.tag_id = devid;

			//get tag ID/ Whoami
			uint8_t whoami = 0;
			testread = bus.dev->regRead(POZYX_WHO_AM_I, &whoami, 1);
			//PX4_INFO("value of whoami is: 0x%x", whoami);

			if (testread != POZYX_SUCCESS) {
				//err(1, "immediate read failed");
				status.result += 2;
			}
			else {
				if (whoami == POZYX_WHOAMI_EXPECTED){
					//PX4_INFO("Who Am I Check Successful");
				}
				else{
					//PX4_INFO("Who Am I Check Failed: 0x%x",whoami);
					status.result += 4;
				}
			}

			//test a function call by blinking LED3
			uint8_t funcbuf[100];
			funcbuf[0] = 0x44;

			bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
			if (funcbuf[0] != 1) {
				//err(1, "Function test failed");
				status.result += 8;
			}
			//PX4_INFO("LED3 turned On... ");
			sleep(2);
			funcbuf[0] = 0x40;
			bus.dev->regFunction(POZYX_LED_CTRL, (uint8_t *)&funcbuf[0], 1, (uint8_t *)&funcbuf[0], 1);
			if (funcbuf[0] != 1) {
				//err(1, "Function test failed");
				status.result += 16;
			}
			//PX4_INFO("LED3 turned Off");

			//PX4_INFO("Tag %d PASS", bus.index);

			status.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(pozyx_tagstatus),status_pub_fd, &status);


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
		//int totalerror = 0;


		pos.x = 0;
		pos.y = 0;
		pos.z = -5;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			//if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_3D)){
			//if (POZYX_SUCCESS == bus.dev->getCoordinates(&poz_coordinates[i])){
			if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_2_5D, 50)){

				if (print_result) {
					PX4_INFO("Current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
				}
				pos_error_t poz_error;
				if (POZYX_SUCCESS == bus.dev->getPositionError(&poz_error)){
					if (print_result) {
						PX4_INFO("Position covariance: x(%d) y(%d) z(%d) xy(%d) xz(%d) yz(%d)", poz_error.x, poz_error.y, poz_error.z, poz_error.xy, poz_error.xz, poz_error.yz);
					}
					//totalerror = abs(poz_error.x) + abs(poz_error.y) + abs(poz_error.z) + abs(poz_error.xy) + abs(poz_error.xz) + abs(poz_error.yz);
					if(poz_error.xy > 500 ) { //|| totalerror < 7
						poz_coordinates[i].x = 0;
						poz_coordinates[i].y = 0;
						/*
						// not a good reading, try again
						if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_2_5D, 50)){
							if (print_result) {
								PX4_INFO("2nd measure current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
							}
							if (POZYX_SUCCESS == bus.dev->getPositionError(&poz_error)){
								if (print_result) {
									PX4_INFO("2nd measure Position covariance: x(%d) y(%d) z(%d) xy(%d) xz(%d) yz(%d)", poz_error.x, poz_error.y, poz_error.z, poz_error.xy, poz_error.xz, poz_error.yz);
								}								
								totalerror = abs(poz_error.x) + abs(poz_error.y) + abs(poz_error.z) + abs(poz_error.xy) + abs(poz_error.xz) + abs(poz_error.yz);
								if(poz_error.xy > 500 || totalerror < 7) {
									//2nd reading also bad
									poz_coordinates[i].x = 0;
									poz_coordinates[i].y = 0;
									PX4_INFO("Covariance error tag %d: %d", bus.index, totalerror);
								}
								else {
									validcount += 1;
								}
							}
						}
						*/
					}
					else {
						validcount += 1;
					}
				}
			}
			pos.x += poz_coordinates[i].x;
			pos.y += poz_coordinates[i].y;
			//pos.z += poz_coordinates[i].z;

			if (count == 1) {
				quaternion_t poz_orientation;
				if (POZYX_SUCCESS == bus.dev->getQuaternion(&poz_orientation)){
					if (print_result) {
						PX4_INFO("Current orientation: %1.4f  %1.4f  %1.4f  %1.4f", (double)poz_orientation.weight, (double)poz_orientation.x, (double)poz_orientation.y, (double)poz_orientation.z);
					}
					//change orientation from funny vertical to NED rotate 180 degrees about z and -90 about x
					//[q0, q1, q2, q3] >>> [-q0, -q2, -q1, q3]
					pos.q[0] = -poz_orientation.weight;
					pos.q[1] = -poz_orientation.y;
					pos.q[2] = -poz_orientation.x;
					pos.q[3] = poz_orientation.z;	
				}			
			}
		}


		if (validcount > 1) {
		//if (false) {
			double yaw = atan2 ((poz_coordinates[1].y - poz_coordinates[0].y),(poz_coordinates[0].x - poz_coordinates[1].x));

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

			float sensor_distance = sqrt(pow((poz_coordinates[1].x - poz_coordinates[0].x),2) + pow((poz_coordinates[1].y - poz_coordinates[0].y),2));
			if ((sensor_distance > 1400) || (sensor_distance < 600)){
				//sensor measurements are not consistent
				validcount = 0;
			}
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
	getpositiontest(enum POZYX_BUS busid, int count, bool print_result)
	{

		struct att_pos_mocap_s pos;

		pos.q[0] = 1;
		pos.q[1] = 0;
		pos.q[2] = 0;
		pos.q[3] = 0;

		pos.timestamp = hrt_absolute_time();
		pos.x = 0;
		pos.y =0; 
		pos.z =0;
		orb_advert_t pos_pub = orb_advertise(ORB_ID(att_pos_mocap), &pos);
		orb_publish(ORB_ID(att_pos_mocap), pos_pub, &pos);

	}



	void
	getpositionorb(enum POZYX_BUS busid, int count)
	{

		/* Publish Position Topic */
		struct pozyx_position_s position;
		memset(&position, 0, sizeof(position));
		orb_advert_t position_pub_fd = orb_advertise(ORB_ID(pozyx_position), &position);
		
	
		coordinates_t poz_coordinates[count];
		//struct att_pos_mocap_s pos;
		unsigned startid = 0;
		int validcount = 0;
		int totalerror = 0;


		position.x_pos = 0;
		position.y_pos = 0;
		position.z_pos = -5;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_2_5D, -200)){


				//PX4_INFO("Current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
				position.id = bus.index;
				position.x_pos = poz_coordinates[i].x;
				position.y_pos = poz_coordinates[i].y;
				position.z_pos = poz_coordinates[i].z;

				pos_error_t poz_error;
				if (POZYX_SUCCESS == bus.dev->getPositionError(&poz_error)){

					//PX4_INFO("Position covariance: x(%d) y(%d) z(%d) xy(%d) xz(%d) yz(%d)", poz_error.x, poz_error.y, poz_error.z, poz_error.xy, poz_error.xz, poz_error.yz);
					position.x_cov = poz_error.x;
					position.y_cov = poz_error.y;
					position.z_cov = poz_error.z;
					position.xy_cov = poz_error.xy;
					position.xz_cov = poz_error.xz;
					position.yz_cov = poz_error.yz;

					totalerror = abs(poz_error.xy);
					if(totalerror > 300 || totalerror < 10) {
						// not a good reading, try again
						if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[i], POZYX_2_5D, -200)){

							//PX4_INFO("2nd measure current position tag %d: %d   %d   %d", bus.index, poz_coordinates[i].x, poz_coordinates[i].y, poz_coordinates[i].z);
							position.id = bus.index;
							position.x_pos = poz_coordinates[i].x;
							position.y_pos = poz_coordinates[i].y;
							position.z_pos = poz_coordinates[i].z;

							if (POZYX_SUCCESS == bus.dev->getPositionError(&poz_error)){

								//PX4_INFO("2nd measure Position covariance: x(%d) y(%d) z(%d) xy(%d) xz(%d) yz(%d)", poz_error.x, poz_error.y, poz_error.z, poz_error.xy, poz_error.xz, poz_error.yz);
								position.x_cov = poz_error.x;
								position.y_cov = poz_error.y;
								position.z_cov = poz_error.z;
								position.xy_cov = poz_error.xy;
								position.xz_cov = poz_error.xz;
								position.yz_cov = poz_error.yz;

								totalerror = abs(poz_error.xy);
								if(totalerror > 300 || totalerror < 10) {
									//2nd reading also bad
									//poz_coordinates[i].x = 0;
									//poz_coordinates[i].y = 0;
									//PX4_INFO("Covariance error tag %d: %d", bus.index, totalerror);
									position.x_cov = 0;
									position.y_cov = 0;
									position.z_cov = 0;
									position.xy_cov = 0;
									position.xz_cov = 0;
									position.yz_cov = 0;
								}
								else {
									validcount += 1;
								}
							}
						}
					}
					else {
						validcount += 1;
					}
				}
			}
			//pos.x += poz_coordinates[i].x;
			//pos.y += poz_coordinates[i].y;
			//pos.z += poz_coordinates[i].z;

			if (count == 1) {
				quaternion_t poz_orientation;
				if (POZYX_SUCCESS == bus.dev->getQuaternion(&poz_orientation)){

					//PX4_INFO("Current orientation: %1.4f  %1.4f  %1.4f  %1.4f", (double)poz_orientation.weight, (double)poz_orientation.x, (double)poz_orientation.y, (double)poz_orientation.z);
					
					//change orientation from funny vertical to NED rotate 180 degrees about z and -90 about x
					//[q0, q1, q2, q3] >>> [-q0, -q2, -q1, q3]
					//pos.q[0] = -poz_orientation.weight;
					//pos.q[1] = -poz_orientation.y;
					//pos.q[2] = -poz_orientation.x;
					//pos.q[3] = poz_orientation.z;	
				}			
			}
		}


		if (validcount > 1) {
		//if (false) {
			double yaw = atan2 ((poz_coordinates[1].y - poz_coordinates[0].y),(poz_coordinates[0].x - poz_coordinates[1].x));

				//PX4_INFO("Current yaw: %f deg.", (yaw * 180 / 3.14159));
			matrix::Vector3f pozyx_orient(0, 0, yaw);
			matrix::Quatf myq(1.0, 0, 0, 0);
			myq.from_axis_angle(pozyx_orient);
			//pos.q[0] = myq(0);
			//pos.q[1] = myq(1);
			//pos.q[2] = myq(2);
			//pos.q[3] = myq(3);

			float sensor_distance = sqrt(pow((poz_coordinates[1].x - poz_coordinates[0].z),2) + pow((poz_coordinates[1].y - poz_coordinates[0].y),2));
			if ((sensor_distance > 1150) || (sensor_distance < 850)){
				//sensor measurements are not consistent
				validcount = 0;
			}
		}	

		//pos.timestamp = hrt_absolute_time();

		if (validcount == count) {
			//change position from NWU to NED and from m to mm
			position.x_pos /= (validcount*1000);
			position.y_pos /= (-validcount*1000);
			position.z_pos /= (-count*1000);
			position.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(pozyx_position),position_pub_fd, &position);
		}
	}

	void
	config(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			uint8_t num_anchors =12;

			/*
			//Building 9 channel 2/3
			device_coordinates_t anchorlist[num_anchors] = {
				{0x0201, 1, {-313, -7254, 1804}},
				{0x0202, 1, {-314, -13520, 1847}},
				{0x0203, 1, {5686, -21169, 1972}},
				{0x0204, 1, {15509, -1383, 8251}},
				{0x0205, 1, {10117, -347, 2071}},
				{0x0206, 1, {4773, -347, 2071}},
				{0x0301, 1, {-315, -7623, 1811}},
				{0x0302, 1, {-313, -13758, 1865}},
				{0x0303, 1, {5888, -21170, 1970}},
				{0x0304, 1, {15950, -1737, 8236}},
				{0x0305, 1, {10168, -340, 2083}},
				{0x0306, 1, {3834, -35, 2039}}
			};
			*/
			num_anchors =9;	
			/*
			//Pleasant View ch2
			device_coordinates_t anchorlist[num_anchors] = {
				{0x682E, 1, {-12102, -3313, 1017}},
				{0x6011, 1, {-3339, 11420, 1037}},
				{0x6021, 1, {3218, 109, 1046}},
				{0x6030, 1, {-15997, 11421, 1159}},
				{0x6032, 1, {-53, -3305, 1022}},
				{0x6034, 1, {3201, 9689, 1035}},
				{0x6036, 1, {-9634, 11421, 1159}},
				{0x6832, 1, {-5777, -3308, 1012}},
				{0x6852, 1, {-18880, -785, 1000}}
			};
			*/


			//Pleasant View ch3
			device_coordinates_t anchorlist[num_anchors] = {
				{0x603C, 1, {589, -3302, 1026}},
				{0x604C, 1, {3202, 9146, 1042}},
				{0x6038, 1, {-18880, -1340, 1020}},
				{0x6028, 1, {-12770, -3318, 1017}},
				{0x6037, 1, {-8972, 11420, 1157}},
				{0x6824, 1, {3213, 89, 1523}},
				{0x6848, 1, {-2761, 11419, 1044}},
				{0x6853, 1, {-6443, -3309, 1021}},
				{0x6854, 1, {-15296, 11423, 1175}}
			};
			
					

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

			/*
			usleep(100000);
			if (bus.dev->setPositionAlgorithm(POZYX_POS_ALG_UWB_ONLY, POZYX_2_5D) == POZYX_SUCCESS) {
				uint8_t algorithm = 0;
				if (bus.dev->getPositionAlgorithm(&algorithm) == POZYX_SUCCESS){
					PX4_INFO("Algorithm set to: %x", algorithm);
				}
				if (bus.dev->getPositionDimension(&algorithm) == POZYX_SUCCESS){
					PX4_INFO("Dimension set to: %x", algorithm);
				}
			}
			*/
			usleep(100000);
			uint8_t min_anchors = 136; //8 && auto selection bit
			if (bus.dev->regWrite(POZYX_POS_NUM_ANCHORS, &min_anchors, 1) == POZYX_SUCCESS) {
				if (bus.dev->regRead(POZYX_POS_NUM_ANCHORS, &min_anchors, 1) == POZYX_SUCCESS) {
					PX4_INFO("Auto anchor selection set. Minimum %d anchors used.", min_anchors);
				}		
			}
			/*
			usleep(100000);
			int32_t z_value = -300;
			if (bus.dev->regWrite(POZYX_POS_Z, (uint8_t *) &z_value, 4) == POZYX_SUCCESS) {
				if (bus.dev->regRead(POZYX_POS_Z, (uint8_t *) &z_value, 4) == POZYX_SUCCESS) {
					PX4_INFO("Z position fixed to %dmm", z_value);
				}
			}
			usleep(100000);
			if (bus.dev->setUpdateInterval(400) == POZYX_SUCCESS) {				
				uint16_t interval = 0;
				if (bus.dev->getUpdateInterval(&interval) == POZYX_SUCCESS) {
					PX4_INFO("%dms continuous positioning interval set", interval);
				}
			}
			*/
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
	getanchorsorb(enum POZYX_BUS busid, int count)
	{


		/**************************************************************/
		/*  This function crashes if there are no anchors configured  */
		/**************************************************************/

		struct pozyx_anchor_s anchor;
		memset(&anchor, 0, sizeof(anchor));
		orb_advert_t anchor_pub_fd = orb_advertise(ORB_ID(pozyx_anchor), &anchor);

		unsigned startid = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;
			uint8_t device_list_size;

			if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
				uint16_t anchors[device_list_size];
				PX4_INFO("Found %d anchors configured on tag %d", device_list_size, bus.index);
				anchor.id = bus.index;
				anchor.anchor_ct = device_list_size;
				
				if (bus.dev->getAnchorIds(anchors, device_list_size) == POZYX_SUCCESS) {
					coordinates_t coordinates;
					for (int j=0; j<device_list_size; j++){
						if (bus.dev->getDeviceCoordinates(anchors[j], &coordinates) == POZYX_SUCCESS){
							PX4_INFO("   Anchor 0x%x at (%d, %d, %d)", anchors[j], coordinates.x, coordinates.y, coordinates.z);
							anchor.anchor_id = anchors[j];
							anchor.x_pos = coordinates.x;
							anchor.y_pos = coordinates.y;
							anchor.z_pos = coordinates.z;
							
							anchor.timestamp = hrt_absolute_time();
							orb_publish(ORB_ID(pozyx_anchor),anchor_pub_fd, &anchor);							
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
		mysettings.gain_db = 0; //gain_db;

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
	getuwborb(enum POZYX_BUS busid, int count)
	{

		/* Publish UWB Topic */
		struct pozyx_uwb_s uwb;
		memset(&uwb, 0, sizeof(uwb));
		orb_advert_t uwb_pub_fd = orb_advertise(ORB_ID(pozyx_uwb), &uwb);

		unsigned startid = 0;
		UWB_settings_t mysettings;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (bus.dev->getUWBSettings(&mysettings) == POZYX_SUCCESS){
				PX4_INFO("UWB settings on tag %d: channel %d, bitrate %d, prf %d, plen 0x%x, gain_db %1.1f", bus.index, mysettings.channel, mysettings.bitrate, mysettings.prf, mysettings.plen, (double)mysettings.gain_db);
				uwb.id = bus.index;
				//uwb.channel = mysettings.channel;
				uwb.bitrate = mysettings.bitrate;
				uwb.prf = mysettings.prf;
				uwb.plen = mysettings.plen;
				uwb.gain_db = (double)mysettings.gain_db;

				uwb.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(pozyx_uwb),uwb_pub_fd,&uwb);							
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

			daemon_task = px4_task_spawn_cmd("pozyx_commands", 
											SCHED_DEFAULT, 
											(SCHED_PRIORITY_MAX -10), 
											5000, 
											pozyx_commands,
											(argv) ? (char *const *)&argv[2] : (char *const *)NULL);

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
		pozyx::getpositiontest(POZYX_BUS_ALL, 1, false);
		usleep(1000000);
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
		usleep(1000000);
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}

int 
pozyx_commands(int argc, char *argv[])
{
	warnx("[pozyx_pub] starting\n");
	thread_running = true;

	//mostly taken from commander.cpp
	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* Initialize Status Message */
	struct pozyx_status_s status;
	memset(&status, 0, sizeof(status));
	orb_advert_t status_pub_fd = orb_advertise(ORB_ID(pozyx_status), &status);

	/* command ack 
	orb_advert_t command_ack_pub = nullptr;
	struct vehicle_command_ack_s command_ack;
	memset(&command_ack, 0, sizeof(command_ack));
	*/

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];
	

	/* pace output  */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		usleep(100);
		/* wait for up to 2500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 2500);

		/* timed out - periodic check for thread_should_exit, etc. */
		if (pret == 0) {
		} else if (pret < 0) {
		/* this is undesirable but not much we can do - might want to flag unhappy status */
			warn("commander: poll error %d, %d", pret, errno);
			continue;
		} else {

			/* if we reach here, we have a valid command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle relevant commands */

			if (cmd.command == MAV_CMD_POZYX_START) {
				//This command is extraneous since the daemon is already running.
			}
			if (cmd.command == MAV_CMD_POZYX_STOP) {
				thread_should_exit = true;
			}
			if (cmd.command == MAV_CMD_POZYX_GETSTATUS) {
				//This needs redesigned, since the thread must be running for this to happen
				if (thread_running) {
					warnx("\trunning\n");
					status.status = 1;
				} else {
					warnx("\tnot started\n");
					status.status = 0;
				}
				status.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(pozyx_status),status_pub_fd,&status);

			}
			if (cmd.command == MAV_CMD_POZYX_GETTAGSTATUS) {
				pozyx::testorb(POZYX_BUS_ALL, 2);
			}
			if (cmd.command == MAV_CMD_POZYX_GETPOSITION) {
				pozyx::getpositionorb(POZYX_BUS_ALL, 2);
			}
			if (cmd.command == MAV_CMD_POZYX_CLEARANCHORS) {
				pozyx::clearanchors(POZYX_BUS_ALL, 2);
			}
			if (cmd.command == MAV_CMD_POZYX_ADDANCHOR) {
				//uint8_t id = static_cast<int>(cmd.param1);
				uint16_t anchor_id = static_cast<int>(cmd.param2);
				uint32_t x = static_cast<int>(cmd.param3);
				uint32_t y = static_cast<int>(cmd.param4);
				uint32_t z = static_cast<int>(cmd.param5);
				pozyx::addanchor(POZYX_BUS_ALL, 2, anchor_id, x, y, z);
			}
			if (cmd.command == MAV_CMD_POZYX_GETANCHORS) {
				pozyx::getanchorsorb(POZYX_BUS_ALL, 2);
			}
			if (cmd.command == MAV_CMD_POZYX_GETUWB) {
				pozyx::getuwborb(POZYX_BUS_ALL, 2);
			}
			if (cmd.command == MAV_CMD_POZYX_SETUWB) {
				//uint8_t id = static_cast<int>(cmd.param1);
				uint8_t bitrate = static_cast<int>(cmd.param2);
				uint8_t prf = static_cast<int>(cmd.param3);
				uint8_t plen = static_cast<int>(cmd.param4);
				float gain_db = cmd.param5/2.0;
				pozyx::setuwb(POZYX_BUS_ALL, 2, bitrate, prf, plen, gain_db);
			}
			if (cmd.command == MAV_CMD_POZYX_RESETTOFACTORY) {
				pozyx::resettofactory(POZYX_BUS_ALL, 2);
			}
		}
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}
