/*
 * estimator_kf.cpp
 *
 *  Created on: May 15, 2015
 *      Author: Roman Meier
 */

#include <sys/time.h>
#include <sstream>

#include <trackingsetup/estimator_kf.h>
#include <trackingsetup/config.h>

namespace tracking {

EstimatorKF::EstimatorKF() { 	//constructor: initialize parameters at first call of class

		//constant values
		gravity = 9.81;
		var_roll = 0;
		var_phi = pow(var_roll*M_PI/ 180.0,2.0); 		//0.4
		r_a = 0; 										//pow(0.5,2.0);
		r_b = 0; 										//pow(0.3,2.0);
		q = 0; 											//0.0025;
		a_LS = 0; 										//0.05;
		b_LS = 0; 										//0.01;

		// variable values
		xhat 	<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr;

		xhat_old =xhat;

		GPS_pos_vel = xhat;

		GPS_pos_vel_old = xhat;

		P 	<< r_a << 0 << 0 << 0 << 0 << 0 << endr
			<< 0 << r_b << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << r_a << 0 << 0 << 0 << endr
			<< 0 << 0 << 0 << r_b << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << r_a << 0 << endr
			<< 0 << 0 << 0 << 0 << 0 << r_b << endr;

		R 	<< pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << endr
			<< 0.0<< pow(0.7*r_b, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << endr
			<< 0.0 << 0.0 << pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << endr
			<< 0.0 << 0.0 << 0.0 << pow(0.7*r_b, 2.0) << 0.0 << 0.0 << endr
			<< 0.0 << 0.0 << 0.0 << 0.0 << pow(0.15*r_a, 2.0) << 0.0 << endr
			<< 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << pow(0.015*r_b, 2.0) << endr;

		S_init=P+R;

		KFphi=0.0;
		phi_current = 0.0;
		phi_old = 0.0;
		KFvair=0.0;
		vair_old=0.0;

		KFcurrentTimestamp = get_usec(); 	// current Kalman Filter time of execution
		KFlastTimestamp = KFcurrentTimestamp;	// last Kalman Filter time of execution
		dt=0;

		newAttitude = false;
		newPosition = false;
		newVfrHud = false;
		isEstimating=false;
		firstMeasurementCheck=true;
		firstOutputCheck=true;
	}

EstimatorKF::~EstimatorKF() {
	// TODO Auto-generated destructor stub
}

void EstimatorKF::KF_readConfig(Config trackingConfig){ // read config files
	//constant values
	var_roll = trackingConfig.Estimator.var_roll;
	var_phi = pow(var_roll*M_PI/ 180.0,2.0); 			//0.4
	r_a = trackingConfig.Estimator.r_a; 				//pow(0.5,2.0);
	r_b = trackingConfig.Estimator.r_b; 				//pow(0.3,2.0);
	q = trackingConfig.Estimator.q; 					//0.0025;
	a_LS = trackingConfig.Estimator.a_LS; 				//0.05;
	b_LS = trackingConfig.Estimator.b_LS; 				//0.01;

	// TODO: check if necessary

	P 	<< r_a << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << r_b << 0 << 0 << 0 << 0 << endr
		<< 0 << 0 << r_a << 0 << 0 << 0 << endr
		<< 0 << 0 << 0 << r_b << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << r_a << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << r_b << endr;

	R 	<< pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << endr
		<< 0.0<< pow(0.7*r_b, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << pow(0.7*r_b, 2.0) << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << 0.0 << pow(0.15*r_a, 2.0) << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << pow(0.015*r_b, 2.0) << endr;

	S_init=P+R;
}

void EstimatorKF::KF_PredictEstimate() { // predicts position and velocity from given data

// 0.1 calculation of turn rate omega ----------------------------------------------
	double v_total2 = pow(xhat(1),2) + pow(xhat(3),2) + pow(xhat(5),2); //calculate total velocity^2

	if(v_total2<1e-4){		// prevent nan error!!!!!
		v_total2=1e-4;
	}

	double omega = tan(KFphi) * gravity / sqrt(v_total2);

// 0.2 calculation q according to wind ----------------------------------------------

//TODO: check if airspeed is reliable enough to be taken for wind!!!!

if (KFvair<5 && KFvair>0.1){
	KFvair=10;		// in case no airspeed available, set KFvair to assumed airspeed
};

	double v_wind = sqrt(v_total2) - fabs(KFvair);

	double q_add = pow(a_LS * sqrt(fabs(v_wind)) + b_LS * fabs(KFphi),(1.0/9.0));

	double q_z=0.8+0.1*q_add; //0.1

	q = q + q_add;

// 1. compute F matrix  -----------------------------------------------------------

// in continuous time

	mat F_c(6, 6);
		F_c << 0.0 << 1.0 << 0.0 << 0.0 << 0.0 << 0.0 << endr
			<< 0.0 << gravity * xhat(1) * xhat(3) * tan(KFphi)/ pow(v_total2,(3.0 / 2.0)) << 0.0 << gravity * pow(xhat(3),2) * tan(KFphi) / pow(v_total2,(3.0 / 2.0)) - gravity * tan(KFphi) / sqrt(v_total2) << 0.0 << gravity * xhat(3) * xhat(5) * tan(KFphi)/ pow(v_total2,(3.0 / 2.0)) << endr
			<< 0.0 << 0.0 << 0.0 << 1.0 << 0.0 << 0.0 << endr
			<< 0.0 << gravity * tan(KFphi) / sqrt(v_total2) - gravity * pow(xhat(1),2) * tan(KFphi) / pow(v_total2,(3.0 / 2.0)) << 0.0<< -gravity * xhat(1) * xhat(3) * tan(KFphi)/ pow(v_total2,(3.0 / 2.0)) << 0.0 << -gravity * xhat(1) * xhat(5) * tan(KFphi)/ pow(v_total2,(3.0 / 2.0)) << endr
			<< 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 1.0 << endr
			<< 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << endr;

// to discrete time
	mat Id_6(6, 6);
	Id_6.eye();

	//mat F_d = Id_6 + F_c * dt + (F_c *F_c) * pow(dt, 2.0) / 2.0;
	//TODO: check if approximation to first order is enough!!!!!!
	mat F_d = Id_6 + F_c * dt;

// 2. compute measurement covariance matrix Q  -----------------------------------------------------------

	vec b(6);
	b 	<< 0.0 << endr
		<< -4.0 * gravity * xhat(3) * pow(cos(KFphi), 2.0) / (pow((cos(2. * KFphi) + 1.0), 2.0)* sqrt(v_total2)) << endr
		<< 0.0 << endr
		<< 4.0 * gravity * xhat(1) * pow(cos(KFphi),2.0) / (pow((cos(2. * KFphi) + 1.0), 2.0) * sqrt(v_total2)) << endr
		<< 0.0<< endr
		<< 0.0 << endr;

	mat Q_c = b * var_phi * b.t();

	mat Zero_6(6,6);	// 6x6 matrix of zeros
	Zero_6.zeros();

	mat X(6,12);
	X = join_horiz(-F_c, Q_c);

	mat Y(6,12);
	Y = join_horiz(Zero_6,F_c.t());

	mat R_join(12, 12);
	R_join = join_vert(X,Y);

// to discrete time
	mat Id_12(12, 12);
	Id_12.eye();

	//mat G = Id_12 + R_join * dt + (R_join * R_join) * pow(dt,2.0) /2.0;
	//TODO: check if approximation to first order is enough!!!!!!

	mat G = Id_12 + R_join * dt;

	mat F_dTQ_d(6, 6);
	F_dTQ_d << G(0, 6) << G(0, 7) << G(0, 8) << G(0, 9) << G(0, 10) << G(0, 11) << endr
			<< G(1, 6) << G(1, 7) << G(1, 8) << G(1, 9) << G(1, 10) << G(1, 11) << endr
			<< G(2, 6) << G(2, 7) << G(2, 8) << G(2, 9) << G(2, 10) << G(2, 11) << endr
			<< G(3, 6) << G(3, 7) << G(3, 8) << G(3, 9) << G(3, 10) << G(3, 11) << endr
			<< G(4, 6) << G(4, 7) << G(4, 8) << G(4, 9) << G(4, 10) << G(4, 11) << endr
			<< G(5, 6) << G(5, 7) << G(5, 8) << G(5, 9) << G(5, 10) << G(5, 11) << endr;

	mat F_dT(6, 6);
	F_dT 	<< G(6, 6) << G(6, 7) << G(6, 8) << G(6, 9) << G(6, 10) << G(6, 11) << endr
			<< G(7, 6) << G(7, 7) << G(7, 8) << G(7, 9) << G(7, 10) << G(7, 11) << endr
			<< G(8, 6) << G(8, 7) << G(8, 8) << G(8, 9) << G(8, 10) << G(8, 11) << endr
			<< G(9, 6) << G(9, 7) << G(9, 8) << G(9, 9) << G(9, 10) << G(9, 11) << endr
			<< G(10, 6) << G(10, 7) << G(10, 8) << G(10, 9) << G(10, 10) << G(10, 11) << endr
			<< G(11, 6) << G(11, 7) << G(11, 8) << G(11, 9) << G(11, 10) << G(11, 11) << endr;

	mat Q_d(6, 6);
	Q_d = F_dT.t() * F_dTQ_d;

	mat Q_additional(6, 6);
	Q_additional << pow(dt,4.0) / 4.0 *q << pow(dt, 3.0) / 2.0 * q << 0 << 0 << 0 << 0 << endr
			<< pow(dt, 3.0) / 2.0 * q << pow(dt, 2.0) * pow(q,2.0) << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << pow(dt, 4.0) / 4.0 *q << pow(dt , 3.0) / 2.0 * q << 0 << 0 << endr
			<< 0 << 0 << pow(dt, 3.0) / 2.0 * q << pow(dt, 2.0) *pow(q,2.0) << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << pow(dt, 4.0) * q_z<< pow(dt, 4.0)/2.0 * pow(q_z,2)<< endr
			<< 0 << 0 << 0 << 0 << pow(dt, 4.0)/2.0 * pow(q_z,2)<< pow(dt, 4.0) * q_z<< endr;

	mat Q = Q_d + Q_additional;

// 3.1 Predict state estimate (a priori) -----------------------------------------------------------
	vec xhat_new(6);
	xhat_new << xhat(1) * dt << endr
			<< -omega * xhat(3) * dt << endr
			<< xhat(3) * dt << endr
			<< omega * xhat(1) * dt << endr
			<< xhat(5) * dt << endr
			<< 0.0 << endr;

	xhat = xhat + xhat_new;

// 3.2 Predicted (a priori) estimate covariance -----------------------------------------------------------

	P = F_d * P * F_d.t() + Q;

}

void EstimatorKF::KF_UpdateEstimate() { // updates latest position estimate by new ground position, velocity

// TODO: delete if not necessary: 1. Calculate R and H -----------------------------------------------------------
	/*
	mat R(6, 6);
	R 	<< pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << endr
		<< 0.0<< pow(0.7*r_b, 2.0) << 0.0 << 0.0 << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << pow(0.8*r_a, 2.0) << 0.0 << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << pow(0.7*r_b, 2.0) << 0.0 << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << 0.0 << pow(0.15*r_a, 2.0) << 0.0 << endr
		<< 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << pow(0.015*r_b, 2.0) << endr;

	//mat H(6, 6);
	//H.eye();
*/

// 2.1 Measurement residual -----------------------------------------------------------
	vec Y(6);
	//Y = GPS_pos_vel - H * xhat;
	Y = GPS_pos_vel - xhat;

// 2.2 Residual covariance -----------------------------------------------------------
	mat S(6,6);
	//S = (H*P*H.t()) + R;
	S = P + R;

// 2.3 Optimal Kalman gain -----------------------------------------------------------

	//check if S is nearly singular. if true --> take initialization value
	//TODO: check if reasonable approach

	double det_S=1;
	det_S=det(S);		// prevent singularity

	if (det_S<1e-20){
		S=S_init;
	}

	mat S_inv = inv(S);
	//mat K = ((P * (H.t())) * S_inv);

	mat K = P * S_inv;

// 2.4 Update (a posteriori) state estimate ------------------------------------------
	xhat = xhat + K * Y;

// 2.5 Update (a posteriori) estimate covariance --------------------------------------
	mat Id_6(6, 6);
	Id_6.eye();

	//P = (Id_6 - K * H) * P;
	P = (Id_6 - K) * P;
}

void EstimatorKF::KF_Pos_Attitude() {// case new position & attitude available

	KFcurrentTimestamp = get_usec();

	// set new inputs
	GPS_pos_vel << targetPosLocal_.y << endr
			<< targetGlobalPos_.velocity.lat << endr
			<< targetPosLocal_.x << endr
			<< targetGlobalPos_.velocity.lon << endr
			<< targetPosLocal_.z << endr
			<< -targetGlobalPos_.velocity.alt << endr;

	phi_current = targetAttitude_.roll;

	// order timestamps
	if (targetGlobalPos_.localTimestamp > targetAttitude_.localTimestamp) { // check if position or attitude was first
		double timestamp_1 = targetAttitude_.localTimestamp;
		double timestamp_2 = targetGlobalPos_.localTimestamp;

		dt = (timestamp_1 - KFlastTimestamp)/1000000.0; // calculate dt
		KFphi = phi_old;

		// predict with phi_old
		KF_PredictEstimate();

		dt = (timestamp_2 - timestamp_1)/1000000.0; // calculate dt
		KFphi = phi_current;

		// predict with phi_new
		KF_PredictEstimate();

		// udpate position
		KF_UpdateEstimate();

		dt = (KFcurrentTimestamp - timestamp_2)/1000000.0; // calculate dt

		// predict with new position and new phi
		KF_PredictEstimate();

	} else if (targetGlobalPos_.localTimestamp < targetAttitude_.localTimestamp) { // check if GPOS was first
		double timestamp_1 = targetGlobalPos_.localTimestamp;
		double timestamp_2 = targetAttitude_.localTimestamp;

		dt = (timestamp_1 - KFlastTimestamp)/1000000.0; // calculate dt
		KFphi = phi_old;

		// predict
		KF_PredictEstimate();

		// udpate position
		KF_UpdateEstimate();

		dt = (timestamp_2 - timestamp_1)/1000000.0; // calculate dt

		// predict with new position and old phi
		KF_PredictEstimate();

		dt = (KFcurrentTimestamp - timestamp_1)/1000000.0; // calculate dt
		KFphi = phi_current;

		// predict with new position and new phi
		KF_PredictEstimate();

	} else if (targetGlobalPos_.localTimestamp == targetAttitude_.localTimestamp) {
		double timestamp = targetAttitude_.localTimestamp;

		dt = (timestamp - KFlastTimestamp)/1000000.0; // calculate dt
		KFphi = phi_old;

		// predict
		KF_PredictEstimate();

		// udpate position
		KF_UpdateEstimate();

		dt = (KFcurrentTimestamp - timestamp)/1000000.0; // calculate dt
		KFphi = phi_current;

		// predict
		KF_PredictEstimate();

	}

	KF_OutputCheck();

	phi_old = phi_current;
	GPS_pos_vel_old = GPS_pos_vel;
	KFlastTimestamp = KFcurrentTimestamp;
	dt = 0.0;

	// velocity estimate is previous groundspeed measurement
	//transform from NED to ENU
		targetEstimatedVel_.x = xhat(3);
		targetEstimatedVel_.y = xhat(1);
		targetEstimatedVel_.z = xhat(5);

	// update based on ground speed
		targetEstimatedPosLocal_.x = xhat(2);
		targetEstimatedPosLocal_.y = xhat(0);
		targetEstimatedPosLocal_.z = xhat(4);

// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,
			targetEstimatedPosLocal_.y, targetEstimatedPosLocal_.z,
			targetEstimatedPos_.lat, targetEstimatedPos_.lon,
			targetEstimatedPos_.alt);
}

void EstimatorKF::KF_Pos() {	// case ONLY new position available

	KFcurrentTimestamp = get_usec();

	// set new inputs
	GPS_pos_vel << targetPosLocal_.y << endr
			<< targetGlobalPos_.velocity.lat << endr
			<< targetPosLocal_.x << endr
			<< targetGlobalPos_.velocity.lon << endr
			<< targetPosLocal_.z << endr
			<< -targetGlobalPos_.velocity.alt << endr;

	dt = (targetGlobalPos_.localTimestamp - KFlastTimestamp)/1000000.0; // calculate dt

	KFphi = phi_old;
	// predict
	KF_PredictEstimate();

	// udpate position
	KF_UpdateEstimate(); //execution of update function

	dt = (KFcurrentTimestamp - targetGlobalPos_.localTimestamp)/1000000.0; // calculate dt

	// predict
	KF_PredictEstimate();

	KF_OutputCheck();

	KFlastTimestamp = KFcurrentTimestamp;
	GPS_pos_vel_old = GPS_pos_vel;
	dt = 0.0;

	// velocity estimate is previous groundspeed measurement
	//transform from NED to ENU
		targetEstimatedVel_.x = xhat(3);
		targetEstimatedVel_.y = xhat(1);
		targetEstimatedVel_.z = xhat(5);

	// update based on ground speed
		targetEstimatedPosLocal_.x = xhat(2);
		targetEstimatedPosLocal_.y = xhat(0);
		targetEstimatedPosLocal_.z = xhat(4);

// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,
			targetEstimatedPosLocal_.y, targetEstimatedPosLocal_.z,
			targetEstimatedPos_.lat, targetEstimatedPos_.lon,
			targetEstimatedPos_.alt);

}

void EstimatorKF::KF_Attitude() {	// case ONLY new attitude available

	KFcurrentTimestamp = get_usec();

	// set new inputs
	phi_current = targetAttitude_.roll;

	dt = (targetAttitude_.localTimestamp - KFlastTimestamp)/1000000.0; // calculate dt
	KFphi = phi_old;
	// predict with old phi
	KF_PredictEstimate();

	dt = (KFcurrentTimestamp - targetAttitude_.localTimestamp)/1000000.0; // calculate dt
	KFphi = phi_current;

	// predict with new phi
	KF_PredictEstimate();

	KF_OutputCheck();

	phi_old = phi_current;
	KFlastTimestamp = KFcurrentTimestamp;
	dt = 0.0;

	// velocity estimate is previous groundspeed measurement
	//transform from NED to ENU
		targetEstimatedVel_.x = xhat(3);
		targetEstimatedVel_.y = xhat(1);
		targetEstimatedVel_.z = xhat(5);

	// update based on ground speed
		targetEstimatedPosLocal_.x = xhat(2);
		targetEstimatedPosLocal_.y = xhat(0);
		targetEstimatedPosLocal_.z = xhat(4);

// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,
			targetEstimatedPosLocal_.y, targetEstimatedPosLocal_.z,
			targetEstimatedPos_.lat, targetEstimatedPos_.lon,
			targetEstimatedPos_.alt);

}

void EstimatorKF::KF_NoNewInformation() {	// case NO new information available

	KFcurrentTimestamp = get_usec();

	dt = (KFcurrentTimestamp - KFlastTimestamp)/1000000.0; // calculate dt

	KFphi = phi_old;

	KF_PredictEstimate();

	KF_OutputCheck();

	KFlastTimestamp = KFcurrentTimestamp;
	dt = 0.0;

	// velocity estimate is previous groundspeed measurement
	//transform from NED to ENU
		targetEstimatedVel_.x = xhat(3);
		targetEstimatedVel_.y = xhat(1);
		targetEstimatedVel_.z = xhat(5);

	// update based on ground speed
		targetEstimatedPosLocal_.x = xhat(2);
		targetEstimatedPosLocal_.y = xhat(0);
		targetEstimatedPosLocal_.z = xhat(4);

// convert back to WGS84
	antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,
			targetEstimatedPosLocal_.y, targetEstimatedPosLocal_.z,
			targetEstimatedPos_.lat, targetEstimatedPos_.lon,
			targetEstimatedPos_.alt);

}

void EstimatorKF::getEstimate(GlobalPos remoteGlobalPosition,Attitude remoteAtt, VfrHud remoteVHud) {	// main function to get Estimate

double getEstimate_time=get_usec();

// check for last measurement
if (getEstimate_time - remoteGlobalPosition.localTimestamp < 15*1e6){
isEstimating=true;

KF_MeasurementCheck(remoteGlobalPosition, getEstimate_time);

	if(newVfrHud){
		setNewVfrHud(remoteVHud);
		KFvair = targetVfrHud_.airspeed;
	}else {
		KFvair = vair_old;
	}

	if (newPosition) {
		setNewRemoteGPos(remoteGlobalPosition); // new position available
		if (newAttitude) {
			setNewAttitude(remoteAtt); // AND new attitude available

			// UPDATE AND PREDICT
			KF_Pos_Attitude();		// case new position & attitude available

		} else {							// ONLY new position available
			// UPDATE AND PREDICT
			KF_Pos();
		}
	} else if (newAttitude) {				// ONLY new attitude available

		setNewAttitude(remoteAtt);
		// ONLY PREDICT
		KF_Attitude();

	} else {										// NO new information

		// ONLY PREDICT
		KF_NoNewInformation();

	}

	vair_old = KFvair;
	xhat_old = xhat;
	//std::cout<<"xhat: "<<xhat<<std::endl;


}else{

isEstimating=false;

}

}

void EstimatorKF::KF_MeasurementCheck(GlobalPos remoteGlobalPosition, double getEstimate_time) {	// check if new measurement seems reasonable
	// getEstimate_time: time right now
	// remoteGlobalPosition.localTimestamp: time of last measurement

	if (!firstMeasurementCheck && newPosition) {

		double dt_MeasurementCheck = (getEstimate_time - remoteGlobalPosition.localTimestamp)/1000000.0; // in s
		double v_max_limit = 20; // absolute max speed possible (wind included) in m/s

		setNewRemoteGPos(remoteGlobalPosition); // new position available

		// absolute max distance possible to be covered in time between two measurements in m
		double distance_max_limit = v_max_limit*dt_MeasurementCheck; //= 1000

		//check if new measurement is reasonable
		if (fabs(targetPosLocal_.x - GPS_pos_vel_old(0)) > distance_max_limit
				|| fabs(targetPosLocal_.y - GPS_pos_vel_old(2)) > distance_max_limit
				|| fabs(targetPosLocal_.z - GPS_pos_vel_old(4)) > distance_max_limit) {
			//if true: position measurement seems not to be reasonable  --> discard this measurement

			newPosition = false;
		}
	}

	if(xhat(0)-xhat_old(0)==0 && xhat(1)-xhat_old(1)==0 && xhat(2)-xhat_old(2)==0 && xhat(3)-xhat_old(3)==0 && xhat(4)-xhat_old(4)==0 && xhat(5)-xhat_old(5)==0 &&xhat(6)-xhat_old(6)==0){
		firstMeasurementCheck=true;
	} else {
		firstMeasurementCheck=false; // after initialization or new program start
	}
}

void EstimatorKF::KF_OutputCheck(){	// check if estimated position seems reasonable
	//check if first run
	if(!firstOutputCheck){
		double dt_OutputCheck = (KFcurrentTimestamp - KFlastTimestamp)/1000000.0; // in s
		double v_max_limit = 20; // absolute max speed possible (wind included) in m/s

		// absolute max distance possible to be covered in time between two estimator executions in m
		double distance_max_limit = v_max_limit*dt_OutputCheck; //=1000;

		if (fabs(xhat(0) - xhat_old(0)) > distance_max_limit
						|| fabs(xhat(2) - xhat_old(2)) > distance_max_limit
						|| fabs(xhat(4) - xhat_old(4)) > distance_max_limit) {

			// if true: new estimation seems not to be reasonable --> take estimation of forward prediction instead

			// assume constant velocity
			xhat(3)=xhat_old(3);
			xhat(1)=xhat_old(1);
			xhat(5)=xhat_old(5);

			// predict with constant velocity
			xhat(2)=xhat_old(2)+xhat_old(3)*dt_OutputCheck;
			xhat(0)=xhat_old(0)+xhat_old(1)*dt_OutputCheck;
			xhat(4)=xhat_old(4)+xhat_old(5)*dt_OutputCheck;

			//transform from NED to ENU
				targetEstimatedVel_.x = xhat(3);
				targetEstimatedVel_.y = xhat(1);
				targetEstimatedVel_.z = xhat(5);

			// update based on ground speed
				targetEstimatedPosLocal_.x = xhat(2);
				targetEstimatedPosLocal_.y = xhat(0);
				targetEstimatedPosLocal_.z = xhat(4);

		// convert back to WGS84
			antennaLocalCartesian_.Reverse(targetEstimatedPosLocal_.x,
					targetEstimatedPosLocal_.y, targetEstimatedPosLocal_.z,
					targetEstimatedPos_.lat, targetEstimatedPos_.lon,
					targetEstimatedPos_.alt);
				}
		}

	if(xhat(0)-xhat_old(0)==0 && xhat(1)-xhat_old(1)==0 && xhat(2)-xhat_old(2)==0 && xhat(3)-xhat_old(3)==0 && xhat(4)-xhat_old(4)==0 && xhat(5)-xhat_old(5)==0 &&xhat(6)-xhat_old(6)==0){
		firstOutputCheck=true;
	} else {
		firstOutputCheck=false;
	}
}

void EstimatorKF::KF_Reset(){	// reset the important KF parameters to initial values if no measurement for a while

		// variable values
		xhat 	<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr;

		GPS_pos_vel << 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr
				<< 0.0 << endr;

		GPS_pos_vel_old = GPS_pos_vel;

		P 	<< r_a << 0 << 0 << 0 << 0 << 0 << endr
			<< 0 << r_b << 0 << 0 << 0 << 0 << endr
			<< 0 << 0 << r_a << 0 << 0 << 0 << endr
			<< 0 << 0 << 0 << r_b << 0 << 0 << endr
			<< 0 << 0 << 0 << 0 << r_a << 0 << endr
			<< 0 << 0 << 0 << 0 << 0 << r_b << endr;

		KFphi=0.0;
		phi_current = 0.0;
		phi_old = 0.0;
		KFvair=0.0;
		vair_old=0.0;

		KFcurrentTimestamp = get_usec(); 	// current Kalman Filter time of execution
		KFlastTimestamp = KFcurrentTimestamp;	// last Kalman Filter time of execution
		dt=0;

		newAttitude = false;
		newPosition = false;
		newVfrHud = false;
		isEstimating=false;
		firstMeasurementCheck=true;
		firstOutputCheck=true;

	}

} /* namespace tracking */
