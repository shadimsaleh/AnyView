#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <cstdio>
#include "opencv2/stitching/stitcher.hpp"
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>

// #include "undistort.h"

using namespace cv;
// using namespace std;

int NN_threshold = 8;

uchar triangulation_homography(int i, int j, std::vector<KeyPoint> &Keypoints_Object, std::vector<KeyPoint> &Keypoints_Scene, Mat &IMG_object, Mat &IMG_scene, std::vector<int> &indices_left
	,  std::vector<int> &indices_right, match_seg left_matcher, match_seg right_matcher, float sum_dist_left, float sum_dist_right)
{
	// std::cout<<"triangulation_homography\n";
	
	std::vector<Point2f> Fake_Camera_points, objects_points;
	for (int i = 0; i < NN_threshold; ++i)
	{
	 	/* code */
		Fake_Camera_points.push_back(left_matcher.mid[indices_left[i]]);
		objects_points.push_back(left_matcher.obj[indices_left[i]]);
	}

	Mat H_L = findHomography(Fake_Camera_points, objects_points, CV_RANSAC);
	Fake_Camera_points.clear(), objects_points.clear();
	for (int i = 0; i < NN_threshold; ++i)
	{
	 	/* code */
		Fake_Camera_points.push_back(right_matcher.mid[indices_right[i]]);
		objects_points.push_back(right_matcher.obj[indices_right[i]]);

	}
	Mat H_R = findHomography(Fake_Camera_points, objects_points, CV_RANSAC);

	Mat position = Mat::zeros(3,1,CV_64FC1); 
	position.at<double>(0,0) = j; 
	position.at<double>(1,0) = i; 
	position.at<double>(2,0) = 1; 
	Mat p = H_L * position;

// imshow("IMG_scene",IMG_scene); imshow("IMG_object",IMG_object);
// waitKey(0);
    // static Mat H_before = Mat::zeros(3,3,CV_64FC1);

	uchar tmpL =  bi_linear_interpolation( p,  IMG_object);

	Mat q = H_R * position;

	uchar tmpR =  bi_linear_interpolation( q,  IMG_scene);

    // std::cout<<"sum_dist_left "<<sum_dist_left<<"   sum_dist_right"<<sum_dist_right<<"\n";

	// if (tmpL > 0 && tmpR > 0)
	// {
	// 	/* code */
	// 	// std::cout<<"pixel: "<<(uchar)((tmpL + tmpR)*0.5)<<"\n";
 //        // H_before = (H_L + H_R)*0.5;
 //        if (sum_dist_left < sum_dist_right)
 //        {
 //            /* code */
 //            // std::cout<<"right is bigger\n";
 //            return tmpL;
 //        }else{
 //                        // std::cout<<"left is bigger\n";

 //            return tmpR;
 //        }

	// } else if ( tmpL > 0){
 //                // H_before = H_L;
	// 	return tmpL;
	// } else if(tmpR > 0) {
 //                        // H_before = H_R;
	// 	return tmpR;
	// } 

 return tmpR;
 
}

Mat create_view(string obj_dir, string sec_dir, std::vector<KeyPoint> &Keypoints_Object, std::vector<KeyPoint> &Keypoints_Scene, std::vector<KeyPoint> left_fake, std::vector<KeyPoint> right_fake, std::vector<DMatch> left_matches, std::vector<DMatch> right_matches)
{
	Mat IMG_object = rectify_read(obj_dir);
    Mat IMG_scene = rectify_read(sec_dir);  //previous result

    // IMG_object = IMG_object/5;
    // imshow("IMG_scene",IMG_scene);
    // imshow("IMG_object",IMG_object);
    // waitKey(0);

    match_seg left_matcher = find_mid_point(left_matches, 1, Keypoints_Object, left_fake);
    match_seg right_matcher = find_mid_point(right_matches, 1, Keypoints_Scene, right_fake);
    std::cout<<"asdasdasdas"<<left_matcher.mid.size()<<"aasfasfasfasfasfavs"<<right_matcher.mid.size()<<"\n";waitKey(0);
    flann::KDTreeIndexParams indexParams_searcher_left(4);
    flann::KDTreeIndexParams indexParams_searcher_right(4);
    flann::Index kdtree_left(Mat(left_matcher.mid).reshape(1), indexParams_searcher_left);
    flann::Index kdtree_right(Mat(right_matcher.mid).reshape(1), indexParams_searcher_right);
std::cout<<"nonononon\n";

    if (!IMG_object.data || !IMG_scene.data) {
    	std::cout << " --(!) Error reading images " << std::endl;
    	exit(-1);
    }


    std::cout<<"type"<<IMG_scene.type()<<"\n";
    // exit(0);
    Mat output = Mat::zeros(480, 640, CV_8UC1);
    for (int i = 0; i < output.rows; ++i)
    {
    	/* code */
    	for (int j = 0; j < output.cols; ++j)
    	{
    		/* code */
    		std::vector<float> query;
    		query.push_back((float)j); //Insert the 2D point we need to find neighbours to the query
    		query.push_back((float)i); 
    		std::vector<int> indices_left, indices_right;
    		std::vector<float> dists_left, dists_right;
    		kdtree_left.knnSearch(query, indices_left, dists_left, NN_threshold,flann::SearchParams(16));
    		kdtree_right.knnSearch(query, indices_right, dists_right, NN_threshold,flann::SearchParams(16));
                float sum_dist_left = 0;
                 float sum_dist_right = 0;

            for (int i = 0; i < NN_threshold; ++i)
            {
                /* code */
                sum_dist_left = sum_dist_left + dists_left[i];
                sum_dist_right = sum_dist_right + dists_right[i];
                // std::cout<<"left_index "<<indices_left[i]<<"    right_index "<<indices_right[i]<<"\n";
                // output.at<uchar>(left_matcher.mid[indices_left[i]].y, left_matcher.mid[indices_left[i]].x) = 255;
                
            }
    //         std::cout<<"\ni: "<<i<<"  j:"<<j<<"\n\n";
    	    output.at<uchar>(i,j) = triangulation_homography(i, j, Keypoints_Object, Keypoints_Scene, IMG_object, IMG_scene, indices_left, indices_right, left_matcher, right_matcher, sum_dist_left, sum_dist_right);
    // imshow("output",output);
    //     waitKey(25);

    	}
    	imshow("output",output);waitKey(25);
    	  std::cout<<"done: "<<i*1.0/output.rows*100<<"\%\n";


    }
    	imshow("output",output);
    	waitKey(0);
}


Mat create_view_stitching(string obj_dir, string sec_dir, std::vector<KeyPoint> &Keypoints_Object, std::vector<KeyPoint> &Keypoints_Scene, std::vector<KeyPoint> left_fake, std::vector<KeyPoint> right_fake, std::vector<DMatch> left_matches, std::vector<DMatch> right_matches)
{
    Mat IMG_object = rectify_read(obj_dir);
    Mat IMG_scene = rectify_read(sec_dir);  //previous result


    match_seg left_matcher = find_mid_point(left_matches, 1, Keypoints_Object, left_fake);
    match_seg right_matcher = find_mid_point(right_matches, 1, Keypoints_Scene, right_fake);
    std::cout<<"right_matcher size: "<<right_matches.size()<<"left_matcher size: "<<left_matches.size()<<"\n";


     Mat H_L = findHomography(left_matcher.obj, left_matcher.mid, CV_RANSAC);

     Mat H_R = findHomography(right_matcher.obj, right_matcher.mid, CV_RANSAC);

    Mat output(480*2, 640*2, CV_8UC1);


    Mat result_A = stitching_bg( left_matcher, IMG_object,  IMG_scene, output, 10, H_L , 1, left_matches);
    Mat result_B = stitching_bg( right_matcher, IMG_scene,  IMG_object, output, 10, H_R , 1, right_matches);



        imshow("output_A",result_A);
        imshow("output_B",result_B);
        waitKey(0);
}