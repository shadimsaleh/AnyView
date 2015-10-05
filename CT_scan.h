/*
 * CT.h
 *
 *  Created on: Dec 5, 2014
 *      Author: eric
 */

#include "iostream"
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"

#include <Eigen/Core>

#define WIN_SIZE 1280
// #define PI CV_PI
# define ENABLE 1
# define ERROR 5
# define scale 1
# define DIS_THESHOLD 10
# define step 10
# define X_Y_threshold_ 4
# define MIN_LINE 10
# define DEBUG 0
# define OUTPUT 0

#ifndef READ_DATA_H_
#define READ_DATA_H_
using namespace cv;

std::vector<Mat> slide_set;
int frame_index;
int num_floors;
int y_size;
int resoluation;
vector<Mat> finals;

Size sz;
/*
 * Struct for line segments
 */
struct Line {
	Vec4i lines;
	int weight;
};

float factor;
/*
 * struct for points cloud
 */
struct blue_print {
	std::vector<Point3f> cloud;
	float MAX_X;
	float MAX_Y;
	float MAX_Z;
	float MIN_X;
	float MIN_Y;
	float MIN_Z;
	float center_X;
	float center_Y;
	vector<int> start_p, end_p;
	int NUM;
};

struct camera {
	Eigen::Matrix<float,3,3> rot;
	Eigen::Matrix<float,3,1> translation;
};



inline Eigen::Matrix<float, 3, 3> QuaternionToRotationMatrix(const Eigen::Matrix<float,4,1> &q){
  // EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(float, 4, 1);
  Eigen::Matrix<float, 3, 3> R;
  float two = static_cast<float>(2);
  R(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(0, 1) = two * (q(0) * q(1) + q(2) * q(3));
  R(0, 2) = two * (q(0) * q(2) - q(1) * q(3));
  R(1, 0) = two * (q(0) * q(1) - q(2) * q(3));
  R(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(1, 2) = two * (q(1) * q(2) + q(0) * q(3));
  R(2, 0) = two * (q(0) * q(2) + q(1) * q(3));
  R(2, 1) = two * (q(1) * q(2) - q(0) * q(3));
  R(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  return R;
}

/*
 * struct for each slide
 */
struct slide {
	std::vector<Point3f> cloud;
	float Z_RANGE;
	int NUM;
};

std::vector<camera*> read_poses(std::string file) {
	std::ifstream fin;
	fin.open(file);
	if (!fin) {
		std::cout << "File open error!\n";
	}
	Eigen::Matrix<float,4,1> i_q_c;
	Eigen::Matrix<float,3,1> i_t_c;
	i_q_c << -0.0107418, -0.0190641, -0.708405, 0.705467;	
	i_t_c << 0.0443505, 0.0265335, -0.00447918;

	i_q_c = i_q_c / i_q_c.norm();
	Eigen::Matrix<float,3,3> i_R_c = QuaternionToRotationMatrix(i_q_c);

	std::vector<camera*> camera_set;
	string a[16];


	 std::ofstream outfile;
outfile.open("./results/position.txt", std::fstream::out | std::fstream::app);
    if (!outfile.is_open())
    {
        std::cout<<"ERROR write Homography text!\n";
        exit(-1);
    } 


	while (!fin.eof()) {
		// std::cout<<"while\n";
		camera *cam = new camera;
		for (int i = 0; i < 16; ++i)
		{
			/* code */
			getline(fin, a[i]);
		}

		Eigen::Matrix<float,4,1> i_q_g;
		for (int i = 0; i < 4; ++i)
		{ 
			/* code */
			std::istringstream q(a[i]);
			q >> i_q_g(i, 0);
			// std::cout<<"read q_"<<i<<": "<<i_q_g(i, 0)<<"\n";
			// std::cout<<"hehe";
		}
		// exit(0);
		// cam->rot
		Eigen::MatrixXf ii_q_g = i_q_g / i_q_g.norm();
		Eigen::MatrixXf g_R_ii = QuaternionToRotationMatrix(ii_q_g).transpose();

		cam->rot = g_R_ii*i_R_c;

		Eigen::Matrix<float,3,1> g_p_i;
		for (int i = 13; i < 16; ++i)
		{
			/* code */
			std::istringstream p(a[i]);
			p >> g_p_i(i-13,0);
			// std::cout<<"read p_"<<i<<": "<<g_p_i(i-13, 0)<<"\n";

		}
		  Eigen::Matrix<float,3,1> g_p_ci = g_p_i + g_R_ii*i_t_c;
		  cam->translation = g_p_ci;
		  // std::cout<<"translation: \n"<<cam->translation<<"\n  rotation:\n"<<cam->rot<<"\n";
		 outfile<<cam->translation.transpose()<<"\n";
		camera_set.push_back(cam);
	}
	return camera_set;
}

//////////
/*
  hehehhe
*/
//   parameters.i_q_c << -0.0107418, -0.0190641, -0.708405, 0.705467;
//   parameters.i_p_c << 0.0443505, 0.0265335, -0.00447918;

// i_q_c = [-0.0107418; -0.0190641; -0.708405; 0.705467]; 
// i_q_c = i_q_c / norm(i_q_c);

// i_t_c = [0.0443505; 0.0265335; -0.00447918];
// i_R_c = quat2rot(i_q_c);

// c1_R_i1 = quat2rot(i_q_c)';
// i1_q_g = i_q_g(:,1) / norm(i_q_g(:,1));
// i1_R_g = quat2rot(i1_q_g);
// ii_R_ci = quat2rot(i_q_c);

// for i = 1:length(g_p_i)
// %     ii_q_g = i_q_g(:,i) / norm(i_q_g(:,i));
// %     g_R_ii = quat2rot(ii_q_g)';
// %     g_R_ci = g_R_ii * i_R_c;
// %     g_p_ci = g_p_i(:,i) + g_R_ii * i_t_c;
// %     g_st_ci = [rot2quat(g_R_ci); g_p_ci];
// %     g_ba_ci(:,i) = g_st_ci;
// end

/*
 * Fuction: Read Points cloud file
 * Input: file path
 * Output: blue_print structure
 */
blue_print read_files(std::string file) {
	std::ifstream fin;
	fin.open(file);
	if (!fin) {
		std::cout << "File open error!\n";
	}

		  /*
  rectify feature points
  */

	 std::ofstream outfile;
outfile.open("./results/points_cloud.txt", std::fstream::out | std::fstream::app);
    if (!outfile.is_open())
    {
        std::cout<<"ERROR write Homography text!\n";
        exit(-1);
    } 


	blue_print map;
	map.MAX_X = 0;
	map.MAX_Y = 0;
	map.MAX_Z = 0;
	float sum_X = 0;
	float sum_Y = 0;
	int counter = 0;
	string a[3];
	 Point3f point;
	while (!fin.eof()) {
		getline(fin, a[0]);
		getline(fin, a[1]);
		getline(fin, a[2]);
		std::istringstream x(a[0]);
		x >> point.x;
		
		std::istringstream y(a[1]);
		y >> point.y;
		
		std::istringstream z(a[2]);
		z >> point.z;

		map.cloud.push_back(point);
		counter++;
		outfile<<point.x<<" "<<point.y<<" "<<point.z<<"\n";
	}
	map.NUM = counter;
	map.center_X = sum_X / counter;
	map.center_Y = sum_Y / counter;
	fin.close();

	float x_size = abs(map.MAX_X * 2);
	if (x_size < abs(map.MIN_X) * 2)
		x_size = abs(map.MIN_X) * 2;

	factor = scale * (WIN_SIZE * 1.0) / x_size;

	float y_org = abs(map.MAX_Y);
	if (y_org < abs(map.MIN_Y))
		y_org = abs(map.MIN_Y);

	y_size = (int) (factor * y_org * 2);
	return map;
}

/*
 * Function: divide points cloud into slides
 * Input: Points cloud(blue_print) Starting point of Z axis, ending point of Z axis
 * Output: Slide Structure
 */
slide create_slides(blue_print map, float z_start, float z_end) {
	/*
	 * construct slide
	 */
	slide slide;
	std::vector<Point3f>::iterator iter = map.cloud.begin();
	int cnt = 0;
	while (iter != map.cloud.end()) {
		if (iter->z > z_start && iter->z < z_end) {
			slide.cloud.push_back(*iter);
			cnt++;
		}
		iter++;
	}
	slide.NUM = cnt;
	return slide;
}

/*
 * Function: Creating Slides
 * Input: slides data(slide), points cloud(blue_print)
 * Output: Slide(Mat)
 */
Mat create_slide_mat(slide slide, blue_print map) {
	float x_size = abs(map.MAX_X * 2);
	if (x_size < abs(map.MIN_X) * 2)
		x_size = abs(map.MIN_X) * 2;

	factor = scale * (WIN_SIZE * 1.0) / x_size;

	std::vector<Point3f>::iterator it = slide.cloud.begin();
	Mat slide_map = Mat::zeros(y_size, WIN_SIZE, CV_8UC1);

	while (it != slide.cloud.end()) {
		int x = (int) (((it->x - map.center_X) * factor) + WIN_SIZE / 2);
		int y = (int) (((it->y - map.center_Y) * factor) + y_size / 2);
		if (x < 0 || y < 0 || x > WIN_SIZE || y > y_size) {
			std::cout<<"Points are out of range\n!";
			std::cout << "cord:  [" << x << "]   [" << y << "] " << "\n";
		}
		slide_map.at < uchar > (y, x, 0) = 255;

		it++;
	}
	return slide_map;
}

/*
 * Call-back Function; to display each slide in "Scan" window
 */
static void M(int, void*) {
	imshow("Scan", slide_set.at(frame_index));
}

/*
 * Call-back Function; to display result of each floor with different threshold in "Floor X" window
 */
static void N(int, void*) {
	Mat temp;
	for (int j = 0; j < num_floors; j++) {
		//Apply Threshold function
		cv::threshold(finals[j], temp, resoluation, 200, THRESH_BINARY);
		cvtColor(finals[j], temp, CV_RGB2GRAY);

		/*
		 * Curve pixel value and then apply Threshold function
		 */
		for (int i = 0; i < temp.cols; i++) {
			for (int ii = 0; ii < temp.rows; ii++) {
				if (temp.at < uchar > (ii, i, 0) != 0) {
					uchar x = (pow(2.71828,
							(temp.at < uchar > (ii, i, 0) * 1.0) / 70 * log(2))
							- 1) * 25.5;
					temp.at < uchar > (ii, i, 0) = x;
				}
			}
		}
		cv::threshold(temp, temp, resoluation, 200, THRESH_BINARY);

		/*
		 * Blur Process with kernel:
		 * 	0		0.25	0
		 * 	0.25	0		0.25
		 * 	0		0.25	0
		 */
		cv::Mat kernel(3, 3, CV_32F, cv::Scalar(0));
		kernel.at<float>(2, 2) = 0.0;
		kernel.at<float>(2, 0) = 0.25;
		kernel.at<float>(2, 1) = 0.25;
		kernel.at<float>(0, 2) = 0.25;
		kernel.at<float>(1, 2) = 0.25;
		cv::filter2D(temp, temp, temp.depth(), kernel);
		cv::threshold(temp, temp, resoluation, 200, THRESH_TOZERO);
		char s[10];
		sprintf(s, "floor%d", j);
		imshow(s, temp);
	}
}/**/

typedef std::pair<int, int> PAIR;
/*
 * Inline Compare Function for each structure
 */
inline bool cmp(const PAIR&x, const PAIR&y) {
	return x.second > y.second;
}
inline bool compare_x(const Point2i& A, const Point2i& B) {
	return A.x < B.x;
}
inline bool compare_y(const Point2i& A, const Point2i& B) {
	return A.y < B.y;
}

/*
 * Function: Finding straight lines from slides
 * Input:
 * 		Slide_map: slide
 * 		select_line_X/Y: maps to select rows & cols in Hough Function
 *		slide: slide data (for line segmentation)
 *		blue_print: points cloud data
 *		two: 	Ture: return pure line segmentation results
 *				False: return line segmentation results drawn on slides
 *
 *	Output: Final Result Mat
 */
Mat hough_create(Mat slide_map, std::map<int, int>& select_line_X,
		std::map<int, int>& select_line_Y, slide& slide, blue_print& map,
		bool two) {
	if (two) {

		Mat cdst;
		cvtColor(slide_map, cdst, CV_GRAY2RGB);
#if 0
		/*
		 *  Yep! Nothing happen!
		 *  Beacuse no more Hough Function is used here!
		 */
#else
		vector < Vec2f > lines;
		Mat horizon = Mat::zeros(y_size, 300, CV_8UC3);
		vector<int> Sum_X;
		int threshold__X = 0;
		for (int i = 0; i < slide_map.rows; i++) {
			int Sum = 0;
			for (int j = 0; j < slide_map.cols; j++) {
				if (slide_map.at<int>(i, j, 0) != 0) {
					Sum++;
				}
			}
			threshold__X = threshold__X + Sum;
			Sum_X.push_back(Sum);
		}
		threshold__X = threshold__X / slide_map.cols * (X_Y_threshold_);
		vector<PAIR> pair_vec;
		for (int i = 5; i < slide_map.rows - 5; i++) {
			int value = (Sum_X[i] + Sum_X[i + 1] + Sum_X[i - 1] + Sum_X[i - 2]
					+ Sum_X[i + 2] + Sum_X[i + 3] + Sum_X[i - 3] + Sum_X[i + 4]
					+ Sum_X[i - 4] + Sum_X[i + 5] + Sum_X[i - 5]) / 11;
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec.push_back(pair);
			line(horizon, Point(0, i), Point(value, i), Scalar(0, 255, 0), 1,
					CV_AA);
		}
		line(horizon, Point(threshold__X, 0), Point(threshold__X, y_size - 5),
				Scalar(255, 0, 0), 1, CV_AA);/**/
		vector<PAIR> maximum_X;
		for (int i = 0; i < pair_vec.size(); i++) {

			if (!((pair_vec[i].second > pair_vec[i - 3].second)
					&& (pair_vec[i].second > pair_vec[i + 3].second))
					&& (pair_vec[i].second > pair_vec[i + 5].second)
					&& pair_vec[i].second > pair_vec[i - 5].second) {
				maximum_X.push_back(pair_vec[i]);
				i = i + step;
			}

		}

		for (int i = 0; i < maximum_X.size(); i++) {
			if (maximum_X[i].second > threshold__X) {
				vector<Point3f>::iterator iter = slide.cloud.begin();

				int add_counter = 0;
				vector < Point2i > found_line;
				while (iter != slide.cloud.end()) {
					int x = (int) (((iter->x - map.center_X) * factor)
							+ WIN_SIZE / 2);
					int y = (int) (((iter->y - map.center_Y) * factor)
							+ y_size / 2);
					if ((y <= maximum_X[i].first + ERROR)
							&& (y >= maximum_X[i].first - ERROR)) {
						Point2i point;
						point.x = x;
						point.y = maximum_X[i].first;
						found_line.push_back(point);
					}
					iter++;
				}
				/*
				 * segmentaion process: X Axis
				 */

				Point2i start, end;
				sort(found_line.begin(), found_line.end(), compare_x);
				if (found_line.size() != 0) {
					start = found_line[0];
					end = found_line[0];
					for (int index = 0; index < found_line.size() - 1;
							index++) {
						int distance =
								sqrt(
										((found_line[index].x
												- found_line[index + 1].x)
												* (found_line[index].x
														- found_line[index + 1].x)
												+ (found_line[index].y
														- found_line[index + 1].y)
														* (found_line[index].y
																- found_line[index
																		+ 1].y)));
						if (distance <= DIS_THESHOLD) {
							end = found_line[index + 1];
						} else {
							if ((abs(start.x - end.x) > MIN_LINE)) {
								Line Line_;
								Line_.lines[0] = start.x;
								Line_.lines[1] = start.y;
								Line_.lines[2] = end.x;
								Line_.lines[3] = end.y;
								line(cdst, start, end, Scalar(0, 255, 0), 1,
										CV_AA);
							}
							start = found_line[index + 1];
							end = found_line[index + 1];
						}
					}
					line(cdst, start, end, Scalar(0, 255, 0), 1, CV_AA);
				}

			}
		}

		Mat vertical = Mat::zeros(300, WIN_SIZE, CV_8UC3);
		vector<int> Sum_Y;
		int threshold__Y = 0;

		for (int i = 0; i < slide_map.cols; i++) {
			int Sum = 0;
			for (int j = 0; j < slide_map.rows; j++) {
				if (slide_map.at<int>(j, i, 0) != 0) {
					Sum++;
				}
			}
			Sum_Y.push_back(Sum);
			threshold__Y = threshold__Y + Sum;
		}
		threshold__Y = threshold__Y / slide_map.rows * (X_Y_threshold_);
		vector<PAIR> pair_vec_Y;
		for (int i = 2; i < slide_map.cols-2; i++) {
			int value = (Sum_Y[i] + Sum_Y[i + 1] + Sum_Y[i - 1] + Sum_Y[i - 2]
					+ Sum_Y[i + 2]) / 5;
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec_Y.push_back(pair);
			line(vertical, Point(i, 0), Point(i, value), Scalar(0, 255, 0), 1,
					CV_AA);
		}
		line(vertical, Point(0, threshold__Y), Point(WIN_SIZE, threshold__Y),
				Scalar(255, 0, 0), 1, CV_AA);

		vector<PAIR> maximum_Y;
		for (int i = 0; i < pair_vec_Y.size(); i++) {

			if (!((pair_vec_Y[i].second > pair_vec_Y[i - 3].second)
					&& (pair_vec_Y[i].second > pair_vec_Y[i + 3].second))
					&& (pair_vec_Y[i].second > pair_vec_Y[i + 5].second)
					&& pair_vec_Y[i].second > pair_vec_Y[i - 5].second) {
				maximum_Y.push_back(pair_vec_Y[i]);
				i = i + step;
			}

		}
		for (int i = 0; i < maximum_Y.size(); i++) {
			if (maximum_Y[i].second > threshold__Y) {
				vector<Point3f>::iterator iter = slide.cloud.begin();
				int add_counter = 0;
				vector < Point2i > found_line;
#if DEBUG
				line(cdst, Point(maximum_Y[i].first, 0),
						Point(maximum_Y[i].first, y_size), Scalar(255, 0, 0), 1,
						CV_AA);
#endif
				while (iter != slide.cloud.end()) {
					int x = (int) (((iter->x - map.center_X) * factor)
							+ WIN_SIZE / 2);
					int y = (int) (((iter->y - map.center_Y) * factor)
							+ y_size / 2);
					if ((x <= maximum_Y[i].first + ERROR)
							&& (x >= maximum_Y[i].first - ERROR)) {
						Point2i point;
						point.x = maximum_Y[i].first;
						point.y = y;
						found_line.push_back(point);
					}
					iter++;
				}
				/*
				 * segmentaion process: Y axis
				 */

				Point2i start, end;
				sort(found_line.begin(), found_line.end(), compare_y);
				if (found_line.size() != 0) {
					start = found_line[0];
					end = found_line[0];
					for (int index = 0; index < found_line.size() - 1;
							index++) {
						int distance =
								sqrt(
										((found_line[index].x
												- found_line[index + 1].x)
												* (found_line[index].x
														- found_line[index + 1].x)
												+ (found_line[index].y
														- found_line[index + 1].y)
														* (found_line[index].y
																- found_line[index
																		+ 1].y)));
						if (distance <= DIS_THESHOLD) {
							end = found_line[index + 1];
						} else {
							if ((abs(start.y - end.y) > MIN_LINE)) {
								Line Line_;
								Line_.lines[0] = start.x;
								Line_.lines[1] = start.y;
								Line_.lines[2] = end.x;
								Line_.lines[3] = end.y;
								line(cdst, start, end, Scalar(0, 255, 0), 1,
										CV_AA);
#if DEBUG
								imshow("test", cdst);
								waitKey();/**/
#endif
							}
							start = found_line[index + 1];
							end = found_line[index + 1];
						}
					}
					line(cdst, start, end, Scalar(0, 255, 0), 1, CV_AA);
				}
			}
		}
#if DEBUG
		//DEBUG MODE
		imshow("cdst", cdst);
		imshow("vertical", vertical);
		imshow("horizon", horizon);
		waitKey();/**/
#endif
#endif
		return cdst;
	} else {

		Mat cdst = Mat::zeros(y_size, WIN_SIZE, CV_8UC3);
#if 0
		/*
		 *  Yep! Nothing happen!
		 *  Beacuse no more Hough Function is used here!
		 */

#else
		vector < Vec2f > lines;
		Mat horizon = Mat::zeros(y_size, 300, CV_8UC3);
		vector<int> Sum_X;
		int threshold__X = 0;
		for (int i = 0; i < slide_map.rows; i++) {
			int Sum = 0;
			for (int j = 0; j < slide_map.cols; j++) {
				if (slide_map.at<int>(i, j, 0) != 0) {
					Sum++;
				}
			}
			threshold__X = threshold__X + Sum;
			Sum_X.push_back(Sum);
		}
		threshold__X = threshold__X / slide_map.cols * (X_Y_threshold_);
		vector<PAIR> pair_vec;
		for (int i = 0; i < slide_map.rows; i++) {
			int value = (Sum_X[i] + Sum_X[i + 1] + Sum_X[i - 1] + Sum_X[i - 2]
					+ Sum_X[i + 2] + Sum_X[i + 3] + Sum_X[i - 3] + Sum_X[i + 4]
					+ Sum_X[i - 4] + Sum_X[i + 5] + Sum_X[i - 5]) / 11;
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec.push_back(pair);
			line(horizon, Point(0, i), Point(value, i), Scalar(0, 255, 0), 1,
					CV_AA);
		}
		line(horizon, Point(threshold__X, 0), Point(threshold__X, y_size - 5),
				Scalar(255, 0, 0), 1, CV_AA);
		vector<PAIR> maximum_X;
		for (int i = 0; i < pair_vec.size(); i++) {

			if (!((pair_vec[i].second > pair_vec[i - 3].second)
					&& (pair_vec[i].second > pair_vec[i + 3].second))
					&& (pair_vec[i].second > pair_vec[i + 5].second)
					&& pair_vec[i].second > pair_vec[i - 5].second) {
				maximum_X.push_back(pair_vec[i]);
				i = i + step;
			}

		}
		for (int i = 0; i < maximum_X.size(); i++) {
			if (maximum_X[i].second > threshold__X) {
				vector<Point3f>::iterator iter = slide.cloud.begin();

				int add_counter = 0;
				vector < Point2i > found_line;
				while (iter != slide.cloud.end()) {
					int x = (int) (((iter->x - map.center_X) * factor)
							+ WIN_SIZE / 2);
					int y = (int) (((iter->y - map.center_Y) * factor)
							+ y_size / 2);
					if ((y <= maximum_X[i].first + ERROR)
							&& (y >= maximum_X[i].first - ERROR)) {
						Point2i point;
						point.x = x;
						point.y = maximum_X[i].first;
						found_line.push_back(point);
					}
					iter++;
				}
				/*
				 * segmentaion process: X axis
				 */

				Point2i start, end;
				sort(found_line.begin(), found_line.end(), compare_x);
				if (found_line.size() != 0) {
					start = found_line[0];
					end = found_line[0];
					for (int index = 0; index < found_line.size() - 1;
							index++) {
						int distance =
								sqrt(
										((found_line[index].x
												- found_line[index + 1].x)
												* (found_line[index].x
														- found_line[index + 1].x)
												+ (found_line[index].y
														- found_line[index + 1].y)
														* (found_line[index].y
																- found_line[index
																		+ 1].y)));
						if (distance <= DIS_THESHOLD) {
							end = found_line[index + 1];
						} else {
							if ((abs(start.x - end.x) > MIN_LINE)) {
								Line Line_;
								Line_.lines[0] = start.x;
								Line_.lines[1] = start.y;
								Line_.lines[2] = end.x;
								Line_.lines[3] = end.y;
								line(cdst, start, end, Scalar(0, 255, 0), 1,
										CV_AA);
							}
							start = found_line[index + 1];
							end = found_line[index + 1];
						}
					}
				}
			}
		}

		Mat vertical = Mat::zeros(300, WIN_SIZE, CV_8UC3);
		vector<int> Sum_Y;
		int threshold__Y = 0;

		for (int i = 0; i < slide_map.cols; i++) {
			int Sum = 0;
			for (int j = 0; j < slide_map.rows; j++) {
				if (slide_map.at<int>(j, i, 0) != 0) {
					Sum++;
				}
			}
			Sum_Y.push_back(Sum);
			threshold__Y = threshold__Y + Sum;
		}
		threshold__Y = threshold__Y / slide_map.rows * (X_Y_threshold_);
		vector<PAIR> pair_vec_Y;
		for (int i = 0; i < slide_map.cols; i++) {
			int value = (Sum_Y[i] + Sum_Y[i + 1] + Sum_Y[i - 1] + Sum_Y[i - 2]
					+ Sum_Y[i + 2]) / 5;
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec_Y.push_back(pair);
			line(vertical, Point(i, 0), Point(i, value), Scalar(0, 255, 0), 1,
					CV_AA);
		}
		line(vertical, Point(0, threshold__Y), Point(WIN_SIZE, threshold__Y),
				Scalar(255, 0, 0), 1, CV_AA);

		vector<PAIR> maximum_Y;
		for (int i = 0; i < pair_vec_Y.size(); i++) {

			if (!((pair_vec_Y[i].second > pair_vec_Y[i - 3].second)
					&& (pair_vec_Y[i].second > pair_vec_Y[i + 3].second))
					&& (pair_vec_Y[i].second > pair_vec_Y[i + 5].second)
					&& pair_vec_Y[i].second > pair_vec_Y[i - 5].second) {
				maximum_Y.push_back(pair_vec_Y[i]);
				i = i + step;
			}

		}
		for (int i = 0; i < maximum_Y.size(); i++) {
			if (maximum_Y[i].second > threshold__Y) {
				vector<Point3f>::iterator iter = slide.cloud.begin();
				int add_counter = 0;
				vector < Point2i > found_line;
				while (iter != slide.cloud.end()) {
					int x = (int) (((iter->x - map.center_X) * factor)
							+ WIN_SIZE / 2);
					int y = (int) (((iter->y - map.center_Y) * factor)
							+ y_size / 2);
					if ((x <= maximum_Y[i].first + ERROR)
							&& (x >= maximum_Y[i].first - ERROR)) {
						Point2i point;
						point.x = maximum_Y[i].first;
						point.y = y;
						found_line.push_back(point);
					}
					iter++;
				}
				/*
				 * segmentaion process: Y axis
				 */

				Point2i start, end;
				sort(found_line.begin(), found_line.end(), compare_y);
				if (found_line.size() != 0) {
					start = found_line[0];
					end = found_line[0];
					for (int index = 0; index < found_line.size() - 1;
							index++) {
						int distance =
								sqrt(
										((found_line[index].x
												- found_line[index + 1].x)
												* (found_line[index].x
														- found_line[index + 1].x)
												+ (found_line[index].y
														- found_line[index + 1].y)
														* (found_line[index].y
																- found_line[index
																		+ 1].y)));
						if (distance <= DIS_THESHOLD) {
							end = found_line[index + 1];
						} else {
							if ((abs(start.y - end.y) > MIN_LINE)) {
								Line Line_;
								Line_.lines[0] = start.x;
								Line_.lines[1] = start.y;
								Line_.lines[2] = end.x;
								Line_.lines[3] = end.y;
								line(cdst, start, end, Scalar(0, 255, 0), 1,
										CV_AA);
							}
							start = found_line[index + 1];
							end = found_line[index + 1];
						}
					}
				}
			}
		}

#endif
		return cdst;
	}
}

/*
 * Function Parse floors
 * Input: slides: all the slides
 * 		  num_floors: number of floors
 * Output: vector of <int, int>: the starting point and ending point of each floor
 * 			(Currently:) Only first element is used, which contains the starting point and ending point of whole points cloud
 * 			 the starting point and ending point of other floor are not filled here.
 */
vector<std::pair<int, int> > parse_floor(vector<slide>& slides,
		int num_floors) {
	vector<std::pair<int, int> > floor_info;
	vector<int> diff;
	for (int i = 0; i < slides.size() - 1; i++) {
		diff.push_back(slides[i + 1].NUM - slides[i].NUM);
	}
	int zero_start = 0;
	int zero_end = diff.size() - 1;
	std::pair<int, int> f;
	for (int i = 0; i < diff.size() - 1; i++) {
		if (diff[i] > 0 && slides[i].NUM <= 20) {
			zero_start = i;
		}
		if (diff[i] > 50 && diff[i + 1] < -50) {
			f.first = (i + zero_start) / 2;
			break;
		}
	}
	for (int i = diff.size() - 1; i > 1; i--) {
		if (diff[i] < 0 && slides[i].NUM <= 20) {
			zero_end = i;
		}
		if (diff[i] > 50 && diff[i + 1] < -50) {
			f.second = (i + zero_end) / 2;
			break;
		}
	}
	floor_info.push_back(f);
	for (int i = 0; i < num_floors; i++) {
		floor_info.push_back(std::pair<int, int>(0, 0));
	}
	return floor_info;
}

/*
 * Rotation Function
 */
void rotate(cv::Mat& src, double angle, cv::Mat& dst) {
	int len = std::max(src.cols, src.rows);
	cv::Point2f pt(len / 2., len / 2.);
	cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

	cv::warpAffine(src, dst, r, cv::Size(src.rows, src.cols));
}

/*
 * Function: Calibration Function
 * Input: test: slide of each floor
 * 			map: points cloud data
 * Output: r: how many degrees need to change
 */
float Calibration(slide test, blue_print map) {
	Mat M = create_slide_mat(test, map);
	double angle = 0;
	int MAX = 0;
	int MAX2 = 0;

	int SEC = 0;
	int SEC2 = 0;

	float r = 0;

	while (angle < 90) {
		angle = angle + 1;
		Mat temp;
		rotate(M, angle, temp);
		vector<int> Sum_X, Sum_Y;
		for (int i = 0; i < temp.rows; i++) {
			int Sum = 0;
			for (int j = 0; j < temp.cols; j++) {
				if (temp.at<int>(i, j, 0) != 0) {
					Sum++;
				}
			}
			Sum_X.push_back(Sum);
		}
		for (int i = 0; i < temp.cols; i++) {
			int Sum = 0;
			for (int j = 0; j < temp.rows; j++) {
				if (temp.at<int>(j, i, 0) != 0) {
					Sum++;
				}
			}
			Sum_Y.push_back(Sum);
		}

		vector<PAIR> pair_vec;
		for (int i = 0; i < temp.rows - 10; i=i+5) {
			int value = (Sum_X[i]+Sum_X[i+1]+Sum_X[i+2]+Sum_X[i+3]+Sum_X[i+4]+Sum_X[i+5]+Sum_X[i+6]+Sum_X[i+7]+Sum_X[i+8]+Sum_X[i+9]);
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec.push_back(pair);
		}
		vector<PAIR> pair_vec_Y;
		for (int i = 0; i < temp.cols - 10; i=i+5) {
			int value = (Sum_Y[i] + Sum_Y[i+1]+Sum_Y[i+2]+Sum_Y[i+3]+Sum_Y[i+4]+Sum_Y[i+5]+Sum_Y[i+6]+Sum_Y[i+7]+Sum_Y[i+8]+Sum_Y[i+9]);
			PAIR pair;
			pair.first = i;
			pair.second = value;
			pair_vec_Y.push_back(pair);
		}

		sort(pair_vec.begin(),pair_vec.end(),cmp);
		sort(pair_vec_Y.begin(),pair_vec_Y.end(),cmp);

		if(pair_vec[0].second > MAX && pair_vec_Y[0].second> SEC ){
			MAX = pair_vec[0].second;

			SEC = pair_vec_Y[0].second;

			r = angle;
		}
	}
	std::cout<<"Calibrate one floor: "<<r<<" degree\n";
	return r;

}

#endif /* READ_DATA_H_ */

