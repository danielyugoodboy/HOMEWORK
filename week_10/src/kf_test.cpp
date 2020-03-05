#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include "week_10/kalman.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kf_test");
    ros::NodeHandle nh;

	int n = 3; // Number of states
	int m = 1; // Number of measurements

	double dt = 1.0 ; // Time step

	Eigen::MatrixXd A(n, n); // System dynamics matrix
	Eigen::MatrixXd C(m, n); // Output matrix
	Eigen::MatrixXd Q(n, n); // Process noise covariance
	Eigen::MatrixXd R(m, m); // Measurement noise covariance
	Eigen::MatrixXd P(n, n); // Estimate error covariance

	// Discrete LTI projectile motion, measuring position only
	A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	C << 1, 0, 0;

	// Reasonable covariance matrices
	Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
	R << 5;
	P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

	std::cout << "A: \n" << A << std::endl;
	std::cout << "C: \n" << C << std::endl;
	std::cout << "Q: \n" << Q << std::endl;
	std::cout << "R: \n" << R << std::endl;
	std::cout << "P: \n" << P << std::endl;

	// Construct the filter
	KalmanFilter kf(dt,A, C, Q, R, P);

	// List of noisy position measurements (y)
	std::vector<double> measurements = {
	21.36207, -18.41971, 143.90836, -52.76939, 81.52611, 67.17912, -5.93449, -0.94632, 111.84432, 38.45614, 141.10963, 196.00361, 110.93513, 157.82011, 163.859, 189.89853, 347.60249, 344.48271, 410.5627, 370.36573, 446.26576, 542.61435, 485.45122, 595.36523, 653.16042, 687.99158, 755.92574, 848.06704, 870.21786, 872.59879, 892.16949, 1048.16843, 1114.30984, 1098.00347, 1273.89832, 1283.50095, 1408.41329, 1356.39875, 1572.20031, 1519.29754, 1695.31637, 1692.42357, 1933.565, 1943.41573, 2008.02941, 2142.3841
	};

	// Best guess of initial states
	Eigen::VectorXd x0(n);
	x0 << measurements[0], 0, -9.81;
	kf.init(dt,x0);

	// Feed measurements into filter, output estimated states
	double t = 0;
	Eigen::VectorXd y(m);
	std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;
	for (int i = 0; i < measurements.size(); i++) {
		t += dt;
		y << measurements[i];
		kf.update(y);
		std::cout <<  kf.state().transpose() << std::endl;
	}
    ros::spin();
	return 0;
}
