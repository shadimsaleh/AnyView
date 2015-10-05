#ifndef CAMERA_MODELS_H_
#define CAMERA_MODELS_H_
#include <iostream>
#include <Eigen/Core>


Eigen::Vector2d un_normalize(const Eigen::Vector3d& norm_point,
                             const Eigen::Vector2d& fc,
                             const Eigen::Matrix<double, 5, 1>& kc,
                             const Eigen::Matrix<double, 2, 1>& cc);

Eigen::Vector2d distortFisheye(const Eigen::Vector2d& point,
                               const Eigen::Vector2d& fc,
                               const Eigen::Matrix<double, 5, 1>& kc,
                               const Eigen::Matrix<double, 2, 1>& cc);

Eigen::Vector2d distortTango(const Eigen::Vector2d& point,
		const Eigen::Vector2d& fc,
		const Eigen::Matrix<double, 5, 1>& kc,
		const Eigen::Vector2d& cc);

Eigen::Vector2d distortNormal(const Eigen::Vector2d& point,
		const Eigen::Vector2d& fc,
		const Eigen::Matrix<double, 5, 1>& kc,
		const Eigen::Vector2d& cc);

int normalizeUndistortScalarsTango(float x, float y, float &u, float &v,
                                    const Eigen::Vector2d& fc,
                                    const Eigen::VectorXd& kc,
                                    const Eigen::MatrixXd& cc, int iterations);

int normalizeUndistortScalarsTango(float x, float y, double &u, double &v,
                                    const Eigen::Vector2d& fc,
                                    const Eigen::VectorXd& kc,
                                    const Eigen::MatrixXd& cc, int iterations);


int normalizeUndistortScalars(float x, float y, double &u, double &v,
                               const Eigen::Vector2d& fc,
                               const Eigen::VectorXd& kc,
                               const Eigen::MatrixXd& cc, int iterations);


/// @fn normalizeUndistortFisheye, Normalizing and undistort fish eye lenses
Eigen::Vector3d normalizeUndistortFisheye(Eigen::Vector3d point,
                                          Eigen::Vector2d fc,
                                          Eigen::VectorXd kc,
                                          Eigen::MatrixXd cc);

#endif

