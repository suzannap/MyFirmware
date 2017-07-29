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
#include <systemlib/param/param.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static int count = 0;
static int tag_height = 0;
static int err_thresholds[3] = {300,1300,700};
static device_coordinates_t stored_anchors[24];
static int stored_anchors_count = 0;
static double actual_yaw = 0.0f;
static double yaw_error = 3.0f;
static int pozyx_err_count = 0;
/* Publish Position Topic */
struct pozyx_position_s position;
//memset(&position, 0, sizeof(position));
orb_advert_t position_pub_fd;// = orb_advertise(ORB_ID(pozyx_position), &position);

struct att_pos_mocap_s pos;
//memset(&pos, 0, sizeof(pos));
orb_advert_t pos_pub;// = orb_advertise(ORB_ID(att_pos_mocap), &pos);

enum POZYX_BUS {
	POZYX_BUS_ALL = 0,
	POZYX_BUS_I2C_INTERNAL,
	POZYX_BUS_I2C_EXTERNAL,
	POZYX_BUS_I2C_ALT_INTERNAL,
	POZYX_BUS_I2C_ALT_EXTERNAL
};

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
	void 	config(enum POZYX_BUS busid, int count);
	void	reset(enum POZYX_BUS busid, int count);
	void	getposition(enum POZYX_BUS busid, int count, bool print_result, uint8_t type);
	void	addanchor(enum POZYX_BUS busid, int count, uint16_t network_id, int32_t x, int32_t y, int32_t z);
	void	setanchors(enum POZYX_BUS busid, int count, uint8_t type);
	void	getanchors(enum POZYX_BUS busid, int count);
	void	clearanchors(enum POZYX_BUS busid, int count);
	void	getuwb(enum POZYX_BUS busid, int count);
	void	setuwb(enum POZYX_BUS busid, int count, uint8_t channel, uint8_t bitrate, uint8_t prf, uint8_t plen, float gain_db, uint16_t target);
	void	resettofactory(enum POZYX_BUS busid, int count);
	void	clearanchors(enum POZYX_BUS busid, int count);
	void 	setthresholds(int cov_thresh, int u_dist_thresh, int l_dist_thresh);
	void 	setheight(int height);
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



	//Basic Functional tests with result sent over uORB
	void
	test(enum POZYX_BUS busid, int count)
	{

		struct pozyx_tagstatus_s status;
		memset(&status, 0, sizeof(status));
		orb_advert_t status_pub_fd = orb_advertise(ORB_ID(pozyx_tagstatus), &status);

		unsigned startid = 0;
		int testread;
		

		for (int i=0; i<count; i++){	
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;	

			/*testing against i2c crashing
			const char *path = bus.devpath;
			int fd = px4_open(path, O_RDONLY);
			if (i==0){	
				status.result = 0;
			}
			else {
				status.result_2 = 0;
			}


			if (fd < 0) {
				err(1, "%s open failed (try 'pozyx start')", path);
				if (i==0){	
					status.result += 1;
				}
				else {
					status.result_2 +=1;
				}
			}
			*/
		
			//Get Tag Index
			if (i==0){
				status.id = bus.index;
			}
			else {
				status.id_2 = bus.index;
			}

			//Get Tag ID
			uint16_t devid = 0;
			testread = bus.dev->getNetworkId(&devid);
			PX4_INFO("Device ID is: 0x%x", devid);
			if (i==0){
				status.tag_id = devid;
			}
			else {
				status.tag_id_2 = devid;
			}

			//check Whoami
			uint8_t whoami = 0;
			testread = bus.dev->regRead(POZYX_WHO_AM_I, &whoami, 1);
			PX4_INFO("value of whoami is: 0x%x", whoami);

			if (testread != POZYX_SUCCESS) {
				//err(1, "immediate read failed");
				if (i==0){	
					status.result += 2;
				}
				else {
					status.result_2 +=2;
				}
			}
			else {
				if (whoami == POZYX_WHOAMI_EXPECTED){
					PX4_INFO("Who Am I Check Successful");
				}
				else{
					PX4_INFO("Who Am I Check Failed: 0x%x",whoami);
				}
				if (i==0){	
					status.result += 4;
				}
				else {
					status.result_2 +=4;
				}
			}

			uint8_t err_code = 0;
			if (bus.dev->getErrorCode(&err_code) == POZYX_SUCCESS) {
				if (i==0){	
					status.error_code = err_code;
				}
				else {
					status.error_code_2 = err_code;
				}
			}

			PX4_INFO("Tag %d PASS", bus.index);

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
	getposition(enum POZYX_BUS busid, int count, bool print_result, uint8_t type)
	{

		coordinates_t poz_coordinates[count];

		unsigned startid = 0;
		int validcount = 0;
		//int totalerror = 0;

		pos.x = 0;
		pos.y = 0;
		pos.z = -5;
		position.x_pos = 0;
		position.y_pos = 0;
		position.z_pos = 0;
		position.x_cov = 0;
		position.y_cov = 0;
		position.z_cov = 0;
		position.xy_cov = 0;
		position.xz_cov = 0;
		position.yz_cov = 0;
		position.x_pos_2 = 0;
		position.y_pos_2 = 0;
		position.z_pos_2 = 0;
		position.x_cov_2 = 0;
		position.y_cov_2 = 0;
		position.z_cov_2 = 0;
		position.xy_cov_2 = 0;
		position.xz_cov_2 = 0;
		position.yz_cov_2 = 0;	


		//measure first tag for every mode
		struct pozyx_bus_option &bus = find_bus(busid, startid);
		startid = bus.index + 1;

		if (POZYX_SUCCESS == bus.dev->doPositioning(&poz_coordinates[0], POZYX_2_5D, tag_height)){
			if (print_result) {
				PX4_INFO("Current position tag %d: %d   %d   %d", bus.index, poz_coordinates[0].x, poz_coordinates[0].y, poz_coordinates[0].z);
			}
			position.id = bus.index;
			position.x_pos = poz_coordinates[0].x;
			position.y_pos = poz_coordinates[0].y;
			position.z_pos = poz_coordinates[0].z;
			
			validcount += 1;
		}
		else {			
			pozyx_err_count += 1;
		}
		//usleep(10000);

		//only measure 2nd tag if in mode to do so and first measurement was successful
		if ((type == 2) && (validcount == 1)) {
			struct pozyx_bus_option &bus2 = find_bus(busid, startid);

			if (POZYX_SUCCESS == bus2.dev->doPositioning(&poz_coordinates[1], POZYX_2_5D, tag_height)){
				if (print_result) {
					PX4_INFO("Current position tag %d: %d   %d   %d", bus2.index, poz_coordinates[1].x, poz_coordinates[1].y, poz_coordinates[1].z);
				}
				position.id_2 = bus2.index;
				position.x_pos_2 = poz_coordinates[1].x;
				position.y_pos_2 = poz_coordinates[1].y;
				position.z_pos_2 = poz_coordinates[1].z;

				validcount += 1;
			}
			else {				
				pozyx_err_count += 1;
			}
			//usleep(10000);

			//if we got 2 successful measurements
			if (validcount == 2) {
				//float paramval = 0.1;
				//param_set(param_find("ATT_W_EXT_HDG"), &paramval);
				double yaw = atan2 ((poz_coordinates[1].y - poz_coordinates[0].y),(poz_coordinates[0].x - poz_coordinates[1].x));
				yaw_error = yaw - actual_yaw;

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
				if ((sensor_distance > err_thresholds[1]) || (sensor_distance < err_thresholds[2])){
					//sensor measurements are not consistent, checking expected distance
					validcount = 0;
				}
				//average positions
				pos.x = 0.0005 * (poz_coordinates[0].x + poz_coordinates[1].x);
				pos.y = -0.0005 * (poz_coordinates[0].y + poz_coordinates[1].y);
			}
			else {
				validcount = 0;
			}
		}
		else if ((type == 1) && (validcount == 1)) { //if just measuring 1 tag, use angle to get center position
			//float paramval = 0.0;
			//param_set(param_find("ATT_W_EXT_HDG"), &paramval);
			/* subscribe to vehicle attitude topic */
			int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
			struct vehicle_attitude_s att;
			memset(&att, 0, sizeof(att));

			/* wakeup source(s) */
			px4_pollfd_struct_t fds[1];
			
			/* pace output  */
			fds[0].fd = att_sub;
			fds[0].events = POLLIN;

			/* wait for up to 2500ms for data */
			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 250);

			/* timed out - periodic check for thread_should_exit, etc. */
			if (pret == 0) {
				/* this means none of our providers is giving us data */
				PX4_INFO("Attitude: Got no data");
			} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
				warn("attitude: poll error %d, %d", pret, errno);
			} else {
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				matrix::Quatf actual_orient(att.q[0], att.q[1], att.q[2], att.q[3]);
				matrix::Vector3f actual_angles(0,0,0);
				actual_angles = actual_orient.to_axis_angle();
				actual_yaw = actual_angles(2);
				//just push current attitude back
				pos.q[0] = att.q[0];
				pos.q[1] = att.q[1];
				pos.q[2] = att.q[2];
				pos.q[3] = att.q[3];
				if (print_result) {
					PX4_INFO("actual yaw is %f", actual_yaw);
				}
			}

			//tags are 0.5m from center, along x axis of vehicle
			pos.x = 0.001 * poz_coordinates[0].x - 0.5 * cos(actual_yaw);
			pos.y = -0.001 * poz_coordinates[0].y - 0.5 * sin(actual_yaw);
			orb_unsubscribe(att_sub);
		}
		
		
		//if we have valid measurements, publish to att-pos-mocap for positioning
		if (validcount > 0) {
			pos.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(att_pos_mocap), pos_pub, &pos);
			pozyx_err_count = 0;
		}
		else {
			if (print_result) {
				PX4_INFO("No valid RTLS measurements");
			}
			pozyx_err_count += 1;
		}
		//publish raw pozyx values regardless of validity
		position.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(pozyx_position),position_pub_fd, &position);

	}

	void
	config(enum POZYX_BUS busid, int count)
	{
		unsigned startid = 0;

		for (int i=0; i<count; i++){
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			/*
			uint8_t max_anchors = 143; //15 && auto selection bit
			if (bus.dev->regWrite(POZYX_POS_NUM_ANCHORS, &max_anchors, 1) == POZYX_SUCCESS) {
				usleep(200);
				if (bus.dev->regRead(POZYX_POS_NUM_ANCHORS, &max_anchors, 1) == POZYX_SUCCESS) {
					PX4_INFO("Auto anchor selection set. Maximum %d anchors used.", max_anchors);
				}		
			}
			*/
			usleep(200);
			if (bus.dev->setRangingProtocol(POZYX_RANGE_PROTOCOL_FAST) == POZYX_SUCCESS) {
				PX4_INFO("Fast ranging protocol set.");

			}
			usleep(200);
			if (bus.dev->setPositionAlgorithm(POZYX_POS_ALG_UWB_ONLY, POZYX_2_5D) == POZYX_SUCCESS) {
				PX4_INFO("UWB-Only, 2.5D algorithm used.");
			}
			usleep(200);

		}
	}

	void
	addanchor(enum POZYX_BUS busid, int count, uint16_t network_id, int32_t x, int32_t y, int32_t z)
	{

		struct pozyx_anchor_s anchor;
		memset(&anchor, 0, sizeof(anchor));
		orb_advert_t anchor_pub_fd = orb_advertise(ORB_ID(pozyx_anchor), &anchor);

		unsigned startid = 0;
		PX4_INFO("Adding anchor 0x%x at coordinates (%d, %d, %d)...", network_id, x, y, z);
		device_coordinates_t poz_anchor = {network_id, 1, {x, y, z}};

		for (int i=0; i<count; i++){	

			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;
			if (bus.dev->addDevice(poz_anchor) == POZYX_SUCCESS){
				/*
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("Anchor 0x%x added to tag %d", network_id, bus.index);
				}
				*/
				usleep(500);
				uint8_t device_list_size;
				if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
					anchor.anchor_ct = device_list_size;
				}
				else{
					anchor.anchor_ct = 0;
				}
				anchor.id = bus.index;
				anchor.anchor_id = network_id;
				anchor.x_pos = x;
				anchor.y_pos = y;
				anchor.z_pos = z;
				anchor.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(pozyx_anchor),anchor_pub_fd, &anchor);
				usleep(300000);	
			}
		}
		stored_anchors_count = anchor.anchor_ct;
		stored_anchors[stored_anchors_count-1] = poz_anchor;

	}

	void
	setanchors(enum POZYX_BUS busid, int count, uint8_t type)
	{
		struct pozyx_anchor_s anchor;
		memset(&anchor, 0, sizeof(anchor));
		orb_advert_t anchor_pub_fd = orb_advertise(ORB_ID(pozyx_anchor), &anchor);

		int set0count = 4;
		int set1count = 5;
		int set2count = 5;
		int set3count = 5;
		int set4count = 5;

		uint16_t anchorlist2_0[set0count] = {0x231, 0x232, 0x233, 0x234};
		uint16_t anchorlist2_1[set1count] = {0x212, 0x233, 0x234, 0x2D2, 0x252};
		uint16_t anchorlist2_2[set2count] = {0x212, 0x2A2, 0x233, 0x2D2, 0x252};
		uint16_t anchorlist2_3[set3count] = {0x212, 0x2A2, 0x233, 0x234, 0x252};
		uint16_t anchorlist2_4[set4count] = {0x212, 0x2A2, 0x234, 0x2D2, 0x252};
		uint16_t anchorlist3_0[set0count] = {0x331, 0x332, 0x333, 0x334};
		uint16_t anchorlist3_1[set1count] = {0x312, 0x333, 0x334, 0x3D2, 0x352};
		uint16_t anchorlist3_2[set2count] = {0x312, 0x3A2, 0x333, 0x3D2, 0x352};
		uint16_t anchorlist3_3[set3count] = {0x312, 0x3A2, 0x333, 0x334, 0x352};
		uint16_t anchorlist3_4[set4count] = {0x312, 0x3A2, 0x334, 0x3D2, 0x352};


		unsigned startid = 0;
		for (int i=0; i<count; i++){	

			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if ((type & 0x30) == 0x10) //channel 2 vom 3 
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist2_0, set0count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set0count);
					anchor.x_pos = 2;
					anchor.y_pos = 0;
				}
			}
			else if ((type & 0x30) == 0x30) //channel 3 vom 3
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist3_0, set0count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set0count);
					anchor.x_pos = 3;
					anchor.y_pos = 0;
				}
			}
			else if ((type & 0x3C) == 0x00) // channel 2 DSL
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist2_1, set1count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set1count);
					anchor.x_pos = 2;
					anchor.y_pos = 1;
				}
			}
			else if ((type & 0x3C) == 0x04) //channel 2 USL
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist2_2, set2count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set2count);
					anchor.x_pos = 2;
					anchor.y_pos = 2;
				}
			}
			else if ((type & 0x3C) == 0x08) // channel 2 DSR
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist2_3, set3count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set3count);
					anchor.x_pos = 2;
					anchor.y_pos = 3;
				}
			}
			else if ((type & 0x3C) == 0x0C) //channel 2 USR
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist2_4, set4count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set4count);
					anchor.x_pos = 2;
					anchor.y_pos = 4;
				}
			}
			else if ((type & 0x3C) == 0x20) //channel 3 DSL
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist3_1, set1count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set1count);
					anchor.x_pos = 3;
					anchor.y_pos = 1;
				}
			}
			else if ((type & 0x3C) == 0x24) //channel 3 USL
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist3_2, set2count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set2count);
					anchor.x_pos = 3;
					anchor.y_pos = 2;
				}
			}
			else if ((type & 0x3C) == 0x28) //channel 3 DSR
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist3_3, set3count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set3count);
					anchor.x_pos = 3;
					anchor.y_pos = 3;
				}
			}
			else if ((type & 0x3C) == 0x2C) //channel 3 USR
			{
				if (bus.dev->setPositioningAnchorIds(anchorlist3_4, set4count) == POZYX_SUCCESS){
					usleep(500);
					bus.dev->setSelectionOfAnchors(POZYX_ANCHOR_SEL_MANUAL, set4count);
					anchor.x_pos = 3;
					anchor.y_pos = 4;
				}
			}
			anchor.id = bus.index;
			anchor.anchor_id = 0xAAAA;
			anchor.z_pos = 0;
			anchor.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(pozyx_anchor),anchor_pub_fd, &anchor);
			usleep(300000);	
		}	
	}


	void
	clearanchors(enum POZYX_BUS busid, int count)
	{

		struct pozyx_anchor_s anchor;
		memset(&anchor, 0, sizeof(anchor));
		orb_advert_t anchor_pub_fd = orb_advertise(ORB_ID(pozyx_anchor), &anchor);

		unsigned startid = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;

			if (bus.dev->clearDevices() == POZYX_SUCCESS){
				/*
				if (bus.dev->saveConfiguration(POZYX_FLASH_ANCHOR_IDS) == POZYX_SUCCESS) {
					PX4_INFO("All anchors cleared from tag %d", bus.index);
				}
				*/
			}
			usleep(500);
			uint8_t device_list_size;
			if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
				anchor.anchor_ct = device_list_size;
			}
			else{
				anchor.anchor_ct = 0;
			}
			anchor.id = bus.index;
			anchor.anchor_id = 0;
			anchor.x_pos = 0;
			anchor.y_pos = 0;
			anchor.z_pos = 0;
			anchor.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(pozyx_anchor),anchor_pub_fd, &anchor);	
		}
		for (int i=0; i<24; i++) {
			stored_anchors[i] = {0, 1, {0, 0, 0}};
			stored_anchors_count = 0;
		}
	}

	void
	setthresholds(int cov_thresh, int u_dist_thresh, int l_dist_thresh)
	{
		err_thresholds[0] = cov_thresh;
		err_thresholds[1] = u_dist_thresh;
		err_thresholds[2] = l_dist_thresh;
	}
	void
	setheight(int height)
	{
		tag_height = height;
	}


	void
	getanchors(enum POZYX_BUS busid, int count)
	{
		struct pozyx_anchor_s anchor;
		memset(&anchor, 0, sizeof(anchor));
		orb_advert_t anchor_pub_fd = orb_advertise(ORB_ID(pozyx_anchor), &anchor);

		unsigned startid = 0;
		anchor.x_pos = 0;
		anchor.y_pos = 0;
		anchor.z_pos = 0;

		for (int i=0; i<count; i++){			
			struct pozyx_bus_option &bus = find_bus(busid, startid);
			startid = bus.index + 1;
			uint8_t device_list_size;

			if (bus.dev->getDeviceListSize(&device_list_size) == POZYX_SUCCESS){
				PX4_INFO("Found %d anchors configured on tag %d", device_list_size, bus.index);
				anchor.id = bus.index;
				anchor.anchor_ct = device_list_size;
				
				if (device_list_size > 0) {
					for (int j=0; j<stored_anchors_count; j++){
						anchor.anchor_id = stored_anchors[j].network_id;
						uint8_t version = 0;
						if (bus.dev->getFirmwareVersion(&version, stored_anchors[j].network_id) == POZYX_SUCCESS) {
							anchor.found = 1;								
						}
						else {
							anchor.found = 0;
						}
						anchor.timestamp = hrt_absolute_time();
						orb_publish(ORB_ID(pozyx_anchor),anchor_pub_fd, &anchor);		
						usleep(1000000);					
					}
					
				}	
			}
		}
	}


	void
	setuwb(enum POZYX_BUS busid, int count, uint8_t channel, uint8_t bitrate, uint8_t prf, uint8_t plen, float gain_db, uint16_t target)
	{
		unsigned startid = 0;

		//uint8_t plens[8] = {0x0C, 0x28, 0x18, 0x08, 0x34, 0x24, 0x14, 0x04};
		UWB_settings_t mysettings;
		mysettings.channel = channel;
		mysettings.bitrate = bitrate;
		mysettings.prf = prf;
		mysettings.plen = plen;
		mysettings.gain_db = gain_db;

		if (target == 0) {
			for (int i=0; i<count; i++){			
				struct pozyx_bus_option &bus = find_bus(busid, startid);
				startid = bus.index + 1;

				if (bus.dev->setUWBSettings(&mysettings) == POZYX_SUCCESS){
					PX4_INFO("UWB settings updated on tag %d", bus.index);
				}
			}
		}
		else {
			struct pozyx_bus_option &bus = find_bus(busid, startid);

			if (bus.dev->setUWBSettings(&mysettings, target) == POZYX_SUCCESS){
				PX4_INFO("UWB settings updated on device %x", target);
			}
			else {
				PX4_INFO("Device %x not found", target);
			}				
		}
	}


	void
	getuwb(enum POZYX_BUS busid, int count)
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
				uwb.channel = mysettings.channel;
				uwb.bitrate = mysettings.bitrate;
				uwb.prf = mysettings.prf;
				uwb.plen = mysettings.plen;
				uwb.gain_db = (double)mysettings.gain_db;

				uwb.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID(pozyx_uwb),uwb_pub_fd,&uwb);	
				usleep(1000000);						
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

	position_pub_fd = orb_advertise(ORB_ID(pozyx_position), &position);

	pos_pub = orb_advertise(ORB_ID(att_pos_mocap), &pos);

	//int ch;
	enum POZYX_BUS busid = POZYX_BUS_ALL;
	int testnum = 0;

	const char *verb = argv[1];

	//start/load driver and begin cycles
	if (!strcmp(verb, "start")) {
		count = pozyx::start(busid);
		if (count >= 0) {
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
		uint8_t type = atoi(argv[2]);
		pozyx::getposition(busid, count, true, type);
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
			uint8_t channel = atoi(argv[1]);
			uint8_t bitrate = atoi(argv[2]);
			uint8_t prf = atoi(argv[3]);
			uint8_t plen = atoi(argv[4]);
			float gain_db = atoi(argv[5])/2.0;
			pozyx::setuwb(busid, count, channel, bitrate, prf, plen, gain_db, 0);
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

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];
	

	/* pace output  */
	fds[0].fd = cmd_sub;
	fds[0].events = POLLIN;


	if (count < 2) {
		while (!thread_should_exit) {
			//send out bad status every 500ms. System will require reboot.
			pozyx::test(POZYX_BUS_ALL, count);
			status.status = 200 + count;
			status.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(pozyx_status),status_pub_fd,&status);
			usleep(500000);
		}
	}
	else if (count == 2){
		while (!thread_should_exit) {
			usleep(100);
			/* wait for up to 2500ms for data */
			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 2500);

			/* timed out - periodic check for thread_should_exit, etc. */
			if (pret == 0) {
				//no comands received. do nothing.
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
						status.status += 1;
					} else {
						warnx("\tnot started\n");
						status.status = 0;
					}
					status.timestamp = hrt_absolute_time();
					orb_publish(ORB_ID(pozyx_status),status_pub_fd,&status);

				}
				if (cmd.command == MAV_CMD_POZYX_GETTAGSTATUS) {
					pozyx::test(POZYX_BUS_ALL, 2);
				}
				if (cmd.command == MAV_CMD_POZYX_GETPOSITION) {
					uint8_t type = static_cast<int>(cmd.param1);
					if ((type & 0x40) > 0){
						pozyx::setanchors(POZYX_BUS_ALL, 2, type);
					}
					else if ((type & 0x03) > 0 ){
						pozyx::getposition(POZYX_BUS_ALL, 2, false, (type & 0x03));
					}
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
					pozyx::getanchors(POZYX_BUS_ALL, 2);
				}
				if (cmd.command == MAV_CMD_POZYX_GETUWB) {
					pozyx::getuwb(POZYX_BUS_ALL, 2);
				}
				if (cmd.command == MAV_CMD_POZYX_SETUWB) {
					uint8_t channel = static_cast<int>(cmd.param1);
					uint8_t bitrate = static_cast<int>(cmd.param2);
					uint8_t prf = static_cast<int>(cmd.param3);
					uint8_t plen = static_cast<int>(cmd.param4);
					float gain_db = cmd.param5/2.0;
					uint16_t target = static_cast<int>(cmd.param6);

					pozyx::config(POZYX_BUS_ALL, 2);
					pozyx::setuwb(POZYX_BUS_ALL, 2, channel, bitrate, prf, plen, gain_db, target);
					pozyx::getuwb(POZYX_BUS_ALL, 2);
				}
				if (cmd.command == MAV_CMD_POZYX_RESETTOFACTORY) {
					pozyx::resettofactory(POZYX_BUS_ALL, 2);
				}
				if (cmd.command == MAV_CMD_POZYX_SETTHRESHOLDS) {
					pozyx::setthresholds(cmd.param1, cmd.param2, cmd.param3);
				}
				if (cmd.command == MAV_CMD_POZYX_SETHEIGHT) {
					pozyx::setheight(cmd.param1);
				}
			}
		}
	}

	warnx("[pozyx_pub] exiting.\n");
	thread_running = false;	
	return 0;
}
