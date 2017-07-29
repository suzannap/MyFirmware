#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>


#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_MOCAP_INIT_COUNT = 1;
static const uint32_t 		MOCAP_TIMEOUT =     10000000;	// 10.0 s
static float last_pozyx_x = 0;
static float last_pozyx_y = 0;
static float last_pozyx_vx = 0;
static float last_pozyx_vy = 0;

void BlockLocalPositionEstimator::mocapInit()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) {
		_mocapStats.reset();
		return;
	}

	// if finished
	if (_mocapStats.getCount() > REQ_MOCAP_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap position init: "
					     "%5.2f, %5.2f, %5.2f m std %5.2f, %5.2f, %5.2f m",
					     double(_mocapStats.getMean()(0)),
					     double(_mocapStats.getMean()(1)),
					     double(_mocapStats.getMean()(2)),
					     double(_mocapStats.getStdDev()(0)),
					     double(_mocapStats.getStdDev()(1)),
					     double(_mocapStats.getStdDev()(2)));
		_mocapInitialized = true;
		_mocapFault = FAULT_NONE;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
		}
	}
}

int BlockLocalPositionEstimator::mocapMeasure(Vector<float, n_y_mocap> &y)
{
	y.setZero();

	//****************************************************************************************
	//waterbee test for not jumping
	
	/* subscribe to vehicle attitude topic */
	int lpos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s lpos;
	memset(&lpos, 0, sizeof(lpos));

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1];
	
	/* pace output  */
	fds[0].fd = lpos_sub;
	fds[0].events = POLLIN;

	/* wait for up to 250ms for data */
	int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 25);

	/* timed out - periodic check for thread_should_exit, etc. */
	if (pret>0) {
		orb_copy(ORB_ID(vehicle_local_position), lpos_sub, &lpos);
	}

	float v;
	float pozyx_x =_sub_mocap.get().x;
	float pozyx_y = _sub_mocap.get().y;
	float pozyx_dt = 0.5;//(lpos.timestamp - _sub_mocap.get().timestamp)*1000000;
	float current_x = _x(X_x);//last_pozyx_x;//lpos.x;
	float current_y = _x(X_y);//last_pozyx_y;//lpos.y;
	//float current_vx = _x(X_vx);//last_pozyx_vx;//lpos.vx;
	//float current_vy = _x(X_vy);//last_pozyx_vy;//lpos.vy;
	float pozyx_vx = (pozyx_x - current_x)/pozyx_dt;
	float pozyx_vy = (pozyx_y - current_y)/pozyx_dt;
	//PX4_INFO("DEBUG::::::::cur_vx = %2.2f, cur_vy = %2.2f, dtime = %2.5f", (double)current_vx, (double)current_vy, (double)pozyx_dt);

	/* no acc
	float pozyx_ax = (pozyx_vx - current_vx)/pozyx_dt;
	float pozyx_ay = (pozyx_vy - current_vy)/pozyx_dt;
	param_get(param_find("MPC_ACC_HOR_MAX"), &v);
	float xy_acc = sqrt(pozyx_ax * pozyx_ax + pozyx_ay * pozyx_ay);
	PX4_INFO("max acc = %2.2f, xy acc = %2.2f, pozyx ax = %2.2f, pozyx ay = %2.2f", (double)v, (double)xy_acc, (double)pozyx_ax, (double)pozyx_ay);
	v = 1;
	if (xy_acc > v) {
		pozyx_ax *= v/xy_acc;
		pozyx_ay *= v/xy_acc; 
		PX4_INFO("acc limited to ax=%2.2f, ay=%2.2f", (double)pozyx_ax, (double)pozyx_ay);
	}
	pozyx_vx = current_vx + pozyx_ax * pozyx_dt;
	pozyx_vy = current_vy + pozyx_ay * pozyx_dt;
	*/ // no acc

	param_get(param_find("MPC_XY_VEL_MAX"), &v);
	float xy_speed = sqrt(pozyx_vx * pozyx_vx + pozyx_vy * pozyx_vy);
	//PX4_INFO("lposx=%2.2f, lposy=%2.2f, pozx=%2.2f, pozy=%2.2f, max vel = %2.2f, xy speed = %2.2f, pozyx vx = %2.2f, pozyx vy = %2.2f", (double)current_x, (double)current_y, (double)pozyx_x, (double)pozyx_y, (double)v, (double)xy_speed, (double)pozyx_vx, (double)pozyx_vy);
	if (xy_speed > v) {
		pozyx_vx *= v/xy_speed;
		pozyx_vy *= v/xy_speed;
		//PX4_INFO("speed limited to vx=%2.2f, vy=%2.2f", (double)pozyx_vx, (double)pozyx_vy);
	}
	//PX4_INFO("final vel of vx=%2.2f, vy=%2.2f", (double)pozyx_vx, (double)pozyx_vy);
	
	pozyx_x = current_x + pozyx_vx * pozyx_dt;
	pozyx_y = current_y + pozyx_vy * pozyx_dt;
	//PX4_INFO("final pos of vx=%2.2f, vy=%2.2f", (double)pozyx_x, (double)pozyx_y);

	last_pozyx_x = pozyx_x;
	last_pozyx_y = pozyx_y;
	last_pozyx_vx = pozyx_vx;
	last_pozyx_vy = pozyx_vy;

	y(Y_mocap_x) = pozyx_x;
	y(Y_mocap_y) = pozyx_y;
	//*******************************************************************************************************

	//y(Y_mocap_x) = _sub_mocap.get().x;
	//y(Y_mocap_y) = _sub_mocap.get().y;
	y(Y_mocap_z) = _sub_mocap.get().z;
	_mocapStats.update(y);
	_time_last_mocap = _sub_mocap.get().timestamp;


	return OK;
}

void BlockLocalPositionEstimator::mocapCorrect()
{
	// measure
	Vector<float, n_y_mocap> y;

	if (mocapMeasure(y) != OK) { return; }

	// mocap measurement matrix, measures position
	Matrix<float, n_y_mocap, n_x> C;
	C.setZero();
	C(Y_mocap_x, X_x) = 1;
	C(Y_mocap_y, X_y) = 1;
	C(Y_mocap_z, X_z) = 1;

	// noise matrix
	Matrix<float, n_y_mocap, n_y_mocap> R;
	R.setZero();
	float mocap_p_var = _mocap_p_stddev.get()* \
			    _mocap_p_stddev.get();
	R(Y_mocap_x, Y_mocap_x) = mocap_p_var;
	R(Y_mocap_y, Y_mocap_y) = mocap_p_var;
	R(Y_mocap_z, Y_mocap_z) = mocap_p_var;

	// residual
	Matrix<float, n_y_mocap, n_y_mocap> S_I = inv<float, n_y_mocap>((C * _P * C.transpose()) + R);
	Matrix<float, n_y_mocap, 1> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);

	if (beta > BETA_TABLE[n_y_mocap]) {
		if (_mocapFault < FAULT_MINOR) {
			//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap fault, beta %5.2f", double(beta));
			_mocapFault = FAULT_MINOR;
		}

	} else if (_mocapFault) {
		_mocapFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap OK");
	}

	// kalman filter correction if no fault
	if (_mocapFault < fault_lvl_disable) {
		Matrix<float, n_x, n_y_mocap> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
	}
}

void BlockLocalPositionEstimator::mocapCheckTimeout()
{
	if (_timeStamp - _time_last_mocap > MOCAP_TIMEOUT) {
		if (_mocapInitialized) {
			_mocapInitialized = false;
			_mocapStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] mocap timeout ");
		}
	}
}
