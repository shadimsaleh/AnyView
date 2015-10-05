

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <string.h>
#include <fstream>
#include <Eigen/Core>
#include <iostream>
#include "camera-models.h"
#include <fstream>
#include <iostream>
#include <string>


using namespace cv;
using namespace std;
using namespace Eigen;

Mat rectify_read(string image_name)
{

	int img_init = 0;
	int img_last = 1;
		Mat cv_undistorted_img;
	for(int img_i = img_init; img_i <= img_last; img_i++)
	{

		// sprintf(out_name, "%sundistorted_images/%08d.pgm", folder_name, img_i);


		Mat cv_img = imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
		Vector3d original_point;

		Mat distorted_points_x;
		distorted_points_x.create(cv_img.size(), CV_32FC1);
		Mat distorted_points_y;
		distorted_points_y.create(cv_img.size(), CV_32FC1);

		//peanut
		Vector2d fc(256.418,  256.52);
		Vector2d cc(320.83,   233.784);
		VectorXd kc(5);
		kc << 0.916432, 0, 0, 0, 0;

		for (int i = 0; i < 640; i++) {
			for (int j = 0; j < 480; j++) {
				original_point << i, j, 1;

				Vector2d undistorted_pt;
				undistorted_pt << i, j;
				Vector2d distorted_pt = distortTango(undistorted_pt, fc, kc, cc);
				distorted_points_x.at<float>(j,i) = distorted_pt(0);
				distorted_points_y.at<float>(j,i) = distorted_pt(1);
			}
		}



		remap(cv_img, cv_undistorted_img, distorted_points_x,
				distorted_points_y,
				INTER_LINEAR);


		// imwrite(out_name, cv_undistorted_img);
	}
	return cv_undistorted_img;
}

