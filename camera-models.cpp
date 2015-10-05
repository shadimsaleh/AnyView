/** @file camera-models.cpp
 *	@brief The camera-models.cpp file.
 *
 *	This file contains the implementation of the camera models functions.
 *  Created on: Jan 16, 2014
 *  Author: Ahmed Medhat
 *  email: medhat@cs.umn.edu
 */
#include "camera-models.h"
#include <math.h>		/// To use: sqrt
#include <stdlib.h>

Eigen::Vector2d un_normalize(Eigen::Vector3d norm_point, Eigen::Vector2d fc,
		Eigen::Matrix<double, 5, 1> kc,
		Eigen::Matrix<double, 2, 1> cc) {
	assert(fc.rows() == 2 && fc.cols() == 1);
	assert(cc.rows() == 2 && cc.cols() == 1);
	if (norm_point(2) != 0) {
		norm_point = norm_point/norm_point(2);
	}

	Eigen::Vector2d un_point;
	un_point(0) = fc(0, 0)*norm_point(0) + cc(0, 0);
	un_point(1) = fc(1, 0)*norm_point(1) + cc(1, 0);

	return un_point;
}

Eigen::Vector2d distortTango(const Eigen::Vector2d& point,
		const Eigen::Vector2d& fc,
		const Eigen::Matrix<double, 5, 1>& kc,
		const Eigen::Vector2d& cc) {


	// Convert from pixels to camera center frame
	Eigen::Vector2d point_normalized;
	point_normalized = point - cc;
	point_normalized(0) = point_normalized(0) / fc(0);
	point_normalized(1) = point_normalized(1) / fc(1);

	// compute undistorted length
	double r_u = point_normalized.norm();
	double invw = 1.0/kc(0);
	double wby2 = kc(0)/2.0;

	// compute distorted length
	double r_d = invw * atan2(2*r_u, 1.0/tan(wby2));

	Eigen::Vector2d distorted;
	distorted = point_normalized/r_u;

	// compute distorted in camera center frame
	distorted = r_d*distorted;

	// convert from camera center to pixels
	distorted(0) = distorted(0)*fc(0);
	distorted(1) = distorted(1)*fc(1);
	distorted = distorted + cc;

	return distorted;
}

Eigen::Vector2d distortFisheye(const Eigen::Vector2d& point,
		const Eigen::Vector2d& fc,
		const Eigen::Matrix<double, 5, 1>& kc,
		const Eigen::Vector2d& cc) {


	// Convert from pixels to camera center frame
	Eigen::Vector2d point_normalized;
	point_normalized = point - cc;
	point_normalized(0) = point_normalized(0) / fc(0);
	point_normalized(1) = point_normalized(1) / fc(1);

	double r = point_normalized.norm();

	double theta = atan(r);

	double k1 = kc(0);
	double k2 = kc(1);
	double k3 = kc(2);
	double k4 = kc(3);

	double theta2 = theta * theta;
	double theta4 = theta2 * theta2;
	double theta6 = theta2 * theta4;
	double theta8 = theta4 * theta4;

	double theta_d = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

	double scaling = (r > 1e-8) ? (theta_d / r) : 1;

	Eigen::Vector2d distorted;
	// compute distorted in camera center frame
	distorted = scaling*point_normalized;

	// convert from camera center to pixels
	distorted(0) = distorted(0)*fc(0);
	distorted(1) = distorted(1)*fc(1);
	distorted = distorted + cc;

	return distorted;
}


int normalizeUndistortScalarsTango(float x, float y, float &u, float &v,
		const Eigen::Vector2d& fc,
		const Eigen::VectorXd& kc,
		const Eigen::MatrixXd& cc,
		int iterations) {

	//  *** MATLAB Code ***
	//  function point_out = undistort_tango(point_in,cx, cy, fx, fy,w)
	//
	//  point_normalize(1) = (point_in(1)-cx)/fx;
	//  point_normalize(2) = (point_in(2)-cy)/fy;
	//
	//  mul2tanwby2 = tan(w/2) * 2;
	//  r_d = norm(point_normalize);
	//
	//  if (abs(r_d*w) <= pi*89/180)
	//      r_u = tan(r_d*w)/(r_d*mul2tanwby2);
	//      point_out = r_u * point_normalize;
	//  else
	//      point_out = [-1; -1];
	//  end
	// ***

	x = (x - cc(0, 0)) / fc(0, 0);
	y = (y - cc(1, 0)) / fc(1, 0);
	u = x;
	v = y;
	float omega = kc(0);
	float r_d = sqrt(x * x + y * y);
	float mul2tanwby2 = tan(omega / 2) * 2;
	if (fabs(r_d * mul2tanwby2) < 0.0001)
		return 1;
	if (fabs(r_d * omega) <= 3.1415 * 89.0 / 180.0){
		float r_u = tan(r_d * omega) / (r_d * mul2tanwby2);
		u = r_u * x;
		v = r_u * y;
		return 0;
	} else {
		u = -1;
		v = -1;
		return 0;
	}
}

int normalizeUndistortScalarsTango(float x, float y, double &u, double &v,
		const Eigen::Vector2d& fc,
		const Eigen::VectorXd& kc,
		const Eigen::MatrixXd& cc,
		int iterations) {

	x = (x - cc(0, 0)) / fc(0, 0);
	y = (y - cc(1, 0)) / fc(1, 0);
	u = x;
	v = y;
	double omega = kc(0);
	double r_d = sqrt(x * x + y * y);
	double mul2tanwby2 = tan(omega / 2) * 2;
	if (fabs(r_d * mul2tanwby2) < 1e-6) {
		return 1;
	}
	if (fabs(r_d * omega) <= 3.1415 * 89.0 / 180.0) {
		double r_u = tan(r_d * omega) / (r_d * mul2tanwby2);
		u = r_u * x;
		v = r_u * y;
		return 0;
	} else {
		u = -1;
		v = -1;
		return 0;
	}
}


// @fn normalizeUndistortFisheye, Normalizing and undistort fish eye lenses
Eigen::Vector3d normalizeUndistortFisheye(Eigen::Vector3d point,
		Eigen::Vector2d fc,
		Eigen::VectorXd kc,
		Eigen::MatrixXd cc) {
	Eigen::Vector3d normalized;
	assert(fc.rows() == 2 && fc.cols() == 1);
	assert(kc.rows() == 5 && kc.cols() == 1);
	assert(cc.rows() == 2 && cc.cols() == 1);
	normalized(0) = (point(0) - cc(0, 0)) / fc(0, 0);
	normalized(1) = (point(1) - cc(1, 0)) / fc(1, 0);
	normalized(2) = 1;
	double k1 = kc(0);
	double k2 = kc(1);
	double k3 = kc(2);
	double k4 = kc(3);

	double theta_d = normalized(0) * normalized(0)
    				  + normalized(1) * normalized(1);
	theta_d = sqrt(theta_d);

	double theta = theta_d;
	for (int i = 0; i < 20; i++) {
		double theta2 = theta * theta;
		double theta4 = theta2 * theta2;
		double theta6 = theta2 * theta4;
		double theta8 = theta4 * theta4;
		theta = theta_d
				/ (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
	}
	double scaling = tan(theta) / theta_d;
	normalized = scaling * normalized;
	return normalized;
}

