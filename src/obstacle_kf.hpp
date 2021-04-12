#ifndef __OBSTACLE_KF_H__
#define __OBSTACLE_KF_H__

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
//#include <Eigen/LU>

#define OBSTACLEKF_POS_VAR 0.1
#define OBSTACLEKF_VEL_NOISE_VAR 0.2  

struct ObstacleKF
{
	// State vector: [x (m), y (m), vx (m/s), vy (m/s)]
	Eigen::MatrixXd x;
	Eigen::MatrixXd P;
	
	// Obstacle ID
	int id;
	std::string name;
	
	// Last time update
	ros::Time tStamp;
	
	// Updated flag
	bool updated;

	// number of detections
	//int seen;
	
	// Default constructor
	ObstacleKF(void) : x(4,1), P(4,4) 
	{
		x.setZero(4, 1);
		P.setIdentity(4, 4);
		id = -1;
		tStamp = ros::Time::now();
		updated = false;
		name = "";
		//seen = 0;
	}
	
	ObstacleKF(const ObstacleKF &data) : x(4,1), P(4,4)
	{
		x = data.x;
		P = data.P;
		id = data.id;
		tStamp = data.tStamp;
		updated = data.updated;
		name = data.name;
		//seen = 0;
	}
	
	ObstacleKF &operator=(const ObstacleKF &data)
	{
		x = data.x;
		P = data.P;
		id = data.id;
		tStamp = data.tStamp;
		updated = data.updated;
		name = data.name;
		//seen = data.seen;
		return *this;
	}
	
	// Filter initialization, position in m and velocity on m/s 
	void init(int _id, double _x, double _y, double _vx, double _vy, std::string _name = "")
	{
		// Get Obstacle ID
		id = _id;

		//increase seen times
		//seen+=1;
		//printf("Initializing obstacle %i, seen: %i\n", id, seen);
		
		// Setup state vector
		x.setZero(4, 1);
		x(0,0) = _x;
		x(1,0) = _y;
		x(2,0) = _vx;
		x(3,0) = _vy;
		
		// Setup cov matrix
		P.setIdentity(4, 4);
		P(0,0) = OBSTACLEKF_POS_VAR;
		P(1,1) = OBSTACLEKF_POS_VAR;
		P(2,2) = 1.0*1.0;
		P(3,3) = 1.0*1.0;
		
		// Update time stamp
		tStamp = ros::Time::now();
		updated = false;
		name = _name;
	}
	
	// State prediction, time in seconds 
	void predict(double _dt)
	{
		// State vector prediction
		x(0,0) += x(2,0)*_dt;
		x(1,0) += x(3,0)*_dt;
		
		// Convariance matrix prediction
		Eigen::Matrix<double, 4, 4> F;
		F.setIdentity(4, 4);
		F(0,2) = _dt;
		F(1,3) = _dt;
		Eigen::Matrix<double, 4, 4> Q;
		Q.setZero(4, 4);
		Q(2,2) = OBSTACLEKF_VEL_NOISE_VAR*_dt*_dt;
		Q(3,3) = OBSTACLEKF_VEL_NOISE_VAR*_dt*_dt;
		P = F*P*F.transpose() + Q;
		
		updated = false;
	}
	
	// State update
	void update(double _x, double _y,  double _posVar = OBSTACLEKF_POS_VAR, ros::Time _t = ros::Time::now())
	{
		// Update time stamp
		tStamp = _t;

		//increase seen times
		//seen+=1;
		//printf("Updating obstacle %i, seen: %i\n", id, seen);
		
		// Compute update jacobian
		Eigen::Matrix<double, 2, 4> H;
		H.setZero(2, 4);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 2, 2> R;
		R.setZero(2, 2);
		R(0,0) = _posVar;
		R(1,1) = _posVar;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 2, 2> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 4, 2> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 2, 1> y;
		y(0,0) = _x - x(0,0);
		y(1,0) = _y - x(1,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 4, 4> I;
		I.setIdentity(4, 4);
		P = (I - K*H)*P;
		
		updated = true;
	}
	
	// State update
	void update(double _x1, double _y1,  double _posVar1, double _x2, double _y2,  double _posVar2, ros::Time _t = ros::Time::now())
	{
		// Update time stamp
		tStamp = _t;

		//seen+=1;
		
		// Compute update jacobian
		Eigen::Matrix<double, 4, 4> H;
		H.setZero(4, 4);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		H(2,0) = 1.0;
		H(3,1) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 4, 4> R;
		R.setZero(4, 4);
		R(0,0) = _posVar1;
		R(1,1) = _posVar1;
		R(2,2) = _posVar2;
		R(3,3) = _posVar2;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 4, 4> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 4, 4> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 4, 1> y;
		y(0,0) = _x1 - x(0,0);
		y(1,0) = _y1 - x(1,0);
		y(2,0) = _x2 - x(0,0);
		y(3,0) = _y2 - x(1,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 4, 4> I;
		I.setIdentity(4, 4);
		P = (I - K*H)*P;
		
		updated = true;
	}
	
	// State update
	void update(double _x1, double _y1,  double _posVar1, 
	            double _x2, double _y2,  double _posVar2, 
	            double _x3, double _y3,  double _posVar3, ros::Time _t = ros::Time::now())
	{
		// Update time stamp
		tStamp = _t;

		//seen+=1;
		
		// Compute update jacobian
		Eigen::Matrix<double, 6, 4> H;
		H.setZero(6, 4);
		H(0,0) = 1.0;
		H(1,1) = 1.0;
		H(2,0) = 1.0;
		H(3,1) = 1.0;
		H(4,0) = 1.0;
		H(5,1) = 1.0;
		
		// Compute update noise matrix
		Eigen::Matrix<double, 6, 6> R;
		R.setZero(4, 4);
		R(0,0) = _posVar1;
		R(1,1) = _posVar1;
		R(2,2) = _posVar2;
		R(3,3) = _posVar2;
		R(4,4) = _posVar3;
		R(5,5) = _posVar3;
		
		// Calculate innovation matrix
		Eigen::Matrix<double, 6, 6> S;
		S = H*P*H.transpose() + R;
		
		// Calculate kalman gain
		Eigen::Matrix<double, 4, 6> K;
		K = P*H.transpose()*S.inverse();
		
		// Calculate innovation vector
		Eigen::Matrix<double, 6, 1> y;
		y(0,0) = _x1 - x(0,0);
		y(1,0) = _y1 - x(1,0);
		y(2,0) = _x2 - x(0,0);
		y(3,0) = _y2 - x(1,0);
		y(4,0) = _x3 - x(0,0);
		y(5,0) = _y3 - x(1,0);
		
		// Calculate new state vector
		x = x + K*y;
		
		// Calculate new cov matrix
		Eigen::Matrix<double, 4, 4> I;
		I.setIdentity(4, 4);
		P = (I - K*H)*P;
		
		updated = true;
	}
};

#endif
