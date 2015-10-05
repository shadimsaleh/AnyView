#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/stitching/stitcher.hpp"
#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <cstdlib>
// #include "image.h"
// #include "misc.h"
// #include "pnmfile.h"
// #include "segment-image.h"
// #include "uFind.h"
#include "create_mid_points.h"
#include "stitching_two.h"
#include "cutting.h"
// #include "utils.h"
#include "undistort.h"
#include "CT_scan.h"
#include "create_view.h"


//#define CROP
#define EXTEND -1


using namespace cv;
std::vector<DMatch> good_matches;
std::vector<Point2f> obj;
std::vector<Point2f> scene;
std::vector<KeyPoint> keypoints_object, keypoints_scene, keypoints_object_org,
keypoints_scene_org, fake_camera;
Mat img_object;
Mat img_scene;
Mat img_seg;
int y = 500;
cv::Size sze;
void readme();
std::vector<Point2f> middle;
Eigen::Matrix<float,3,3> intrinsic;



int stitching_thread(string obj_dir, string sec_dir, int m_size, int NNN, int frame_counter, int file_name_counter);
/** @function main */


void parse_argv(int argc, char** argv) {
  if(argc < 4) {
    printf("Incorrect usage: ImageD image_directory num_of_stes starting_image \n");
    exit(-1);
  }
}
/*
* Read images and call loop functions to stitching them.
*/
int main(int argc, char** argv) {


  /*
      Set up parameters
  */
      int fake_camera_number = 160/5 -10;
      int real_camera_1 = 160/5 -10;
      int real_camera_2 = 100/5 - 10;


      parse_argv(argc, argv);
      int num_of_sets = atoi(argv[2]);
      int file_name_counter = atoi(argv[3]);

      intrinsic << 256.418, 0,  320.83,
      0,      256.52,  233.784,
      0,      0,       1;   




      std::vector<camera*> camera_set = read_poses("xkk_walter_1.txt");
  // std::cout<<"enenenen......\n";

    // string file_name = "features_in_G1.txt";
      blue_print map = read_files("features_in_G1.txt"); 
// std::cout<<"huhuhu......\n";
      Eigen::MatrixXf point_cloud(4,map.cloud.size());
      for (int i = 0; i < map.cloud.size(); ++i)
      {
        point_cloud(0,i) = map.cloud[i].x;
        point_cloud(1,i) = map.cloud[i].y;
        point_cloud(2,i) = map.cloud[i].z;
        point_cloud(3,i) = 1;
      /* code */
      }


//     Mat test = rectify_read("00000015.pgm");
// imshow("test",test); waitKey(0);
      Eigen::Matrix3f rotation;
      float radians = -15.0/180.0 * CV_PI;
      rotation << cos(radians),    0 ,   sin(radians),
      0 ,              1 ,   0,
      -sin(radians),    0 ,   cos(radians);
      std::cout<<"rotation matrix: "<<rotation;
      waitKey(0);
      Eigen::Matrix<float,3,3> view_rot = (camera_set[fake_camera_number]->rot)*rotation;
      Eigen::Matrix<float,3,1> view_trans = (camera_set[fake_camera_number]->translation);
  // std::cout<<"camera 24: "<<camera_set[24]->translation<<"\n";
  // std::cout<<"camera 25: "<<camera_set[25]->translation;
  // waitKey(0);

      Eigen::Matrix<float,3,1> cameraCenter = view_trans; 

      Eigen::Matrix<float,3,4> projection_matrix;
  view_trans = - view_rot.transpose()*view_trans;//compute camera center

  projection_matrix << view_rot.transpose(),  view_trans;///////////////////////////
  
  /*
    Rotation!!!!
  */
    // projection_matrix = rotation*projection_matrix;

  projection_matrix = intrinsic*projection_matrix;
  std::cout<<"projection_matrix: \n"<<projection_matrix<<"\n";
  std::cout<<"view_trans: \n"<<view_trans<<"\n";
  int width = point_cloud.cols();
  Eigen::MatrixXf projected_points(3, width);
  projected_points = projection_matrix * point_cloud;

  // Mat show_test = Mat::zeros(480,640,CV_8UC1);
  char file_name_1[10];
  char file_name_2[10];
  char target[10];

  std::sprintf(file_name_1,"%08d",5*(real_camera_1 + 10));
  std::sprintf(file_name_2,"%08d",5*(real_camera_2 + 10));
  std::sprintf(target,"%08d",(fake_camera_number+ 10)*5);

  string dd = "/media/eric/DE80F79A80F7777D/eric_linux_data/tracked_images/" ;

  string dir_1 = dd + file_name_1 + ".pgm";
  imshow("target",rectify_read(dd + target + ".pgm"));waitKey(0);
  string dir_2 = dd + file_name_2 + ".pgm";
  Mat show_test = Mat::zeros(480,640,CV_8UC3);//rectify_read(dir_1);

std::cout<<dir_1<<"\n";
std::cout<<dir_2<<"\n";

  Mat back_view = rectify_read(dir_2);
  Mat front_view = rectify_read(dir_1);

imshow("back_view",back_view);
imshow("front_view", front_view);waitKey(0);

  std::vector<Point3f> common_features;
  std::cout<<"start projection!\n";
  for (int i = 0; i < width; ++i)
  {
    float p_x = projected_points(0,i)/projected_points(2,i);
    float p_y = projected_points(1,i)/projected_points(2,i);

    float dist = (point_cloud(0,i) - cameraCenter(0,0))*(point_cloud(0,i)  - cameraCenter(0,0)) 
    + (point_cloud(1,i) - cameraCenter(1,0))*(point_cloud(1,i)  - cameraCenter(1,0))
    + (point_cloud(2,i) - cameraCenter(2,0))*(point_cloud(2,i)  - cameraCenter(2,0));

    if (p_x>-100 && p_x < 740.0 && p_y> -100 && p_y< 580.0 && sqrt(dist) < 10)
    {
              // std::cout<<"p_X: "<<p_x<<"\n p_Y: "<<p_y<<"\n";
      common_features.push_back(map.cloud[i]);
      // show_test.at<char>(floor(p_y),floor(p_x),0) = 255;
            /* code */
    }
  }



/*
  Canditate frames!!! *2 
*/
  std::cout<<"start canadiate projections!\n";
  std::cout<<"initial common_features:"<<common_features.size()<<"\n";

  Eigen::MatrixXf common_f(4,common_features.size());
  for (int i = 0; i < common_features.size(); ++i)
  {
    common_f(0,i) = common_features[i].x;
    common_f(1,i) = common_features[i].y;
    common_f(2,i) = common_features[i].z;
    common_f(3,i) = 1;
      /* code */
    // std::cout<<" x y z "<<common_features[i].x<<" "<<common_features[i].y<<" "<<common_features[i].z<<" \n";
  }
  // waitKey(0);

  Eigen::Matrix<float,3,3> canadiate_view_rot = camera_set[real_camera_1]->rot;
  Eigen::Matrix<float,3,1> canadiate_view_trans = camera_set[real_camera_1]->translation;

  Eigen::Matrix<float,3,4> canadiate_projection_matrix;
  canadiate_projection_matrix << canadiate_view_rot.transpose(), -canadiate_view_rot.transpose() * canadiate_view_trans;////////////////////

  Eigen::Matrix<float,3,3> canadiate_view_rot_2 = camera_set[real_camera_2]->rot;
  Eigen::Matrix<float,3,1> canadiate_view_trans_2 = camera_set[real_camera_2]->translation;

  Eigen::Matrix<float,3,4> canadiate_projection_matrix_2;
  canadiate_projection_matrix_2 << canadiate_view_rot_2.transpose(), -canadiate_view_rot_2.transpose() * canadiate_view_trans_2;///////////////////

  width = common_features.size();
  Eigen::MatrixXf projected_points_view_1(3,width);
  Eigen::MatrixXf projected_points_view_2(3,width);

  projected_points_view_1 = intrinsic*canadiate_projection_matrix*common_f;
  projected_points_view_2 = intrinsic*canadiate_projection_matrix_2*common_f;
  projected_points = projection_matrix * common_f;


  std::ofstream outfile;
  outfile.open("./results/common_f.txt", std::fstream::out);
  if (!outfile.is_open())
  {
    std::cout<<"ERROR write Homography text!\n";
    exit(-1);
  } 

  // outfile<<projected_points_view_1.transpose()<<"\n\n\n";

  // outfile<<projected_points_view_2.transpose()<<"\n\n\n";

  int left_cnt = 0;
  int right_cnt = 0;
  int obj_cnt = 0;
  int sce_cnt = 0;
  std::vector<DMatch> left_match, right_match;
  std::vector<KeyPoint> fake_left, fake_right;
  std::cout<<"here!!!!!\n";
  for (int i = 0; i < width; ++i)
  {
    float p_x_1 = projected_points_view_1(0,i)/projected_points_view_1(2,i);
    float p_y_1 = projected_points_view_1(1,i)/projected_points_view_1(2,i);

    float p_x_2 = projected_points_view_2(0,i)/projected_points_view_2(2,i);
    float p_y_2 = projected_points_view_2(1,i)/projected_points_view_2(2,i);

    float p_x_0 = projected_points(0,i)/projected_points(2,i);
    float p_y_0 = projected_points(1,i)/projected_points(2,i);

      // outfile<<common_f(0,i)<<" "<<common_f(1,i)<<" "<<common_f(2,i)<<"\n";

    if (p_x_1>-10 && p_x_1 < 650.0 && p_y_1>-10 && p_y_1 < 490.0 && projected_points_view_1(2,i)>0 && projected_points(2,i)>0)
    {
      keypoints_object_org.push_back(KeyPoint(Point2f(p_x_1,p_y_1),1.0));
      fake_left.push_back(KeyPoint(Point2f(p_x_0,p_y_0),1.0));

      DMatch a; a.queryIdx = left_cnt; a.trainIdx = obj_cnt;
      left_match.push_back(a);
      if (p_x_1>0 && p_x_1 < 640.0 && p_y_1>0 && p_y_1 < 480.0 && p_x_0>0 && p_x_0 < 640.0 && p_y_0>0 && p_y_0 < 480.0 )
      {
        /* code */
      
      show_test.at<Vec3b>(round(p_y_0),round(p_x_0)).val[1] = 255;
      front_view.at<uchar>(round(p_y_1), round(p_x_1),0) = 255;
      outfile<<common_f(0,i)<<" "<<common_f(1,i)<<" "<<common_f(2,i)<<"\n";


    }


      left_cnt++;obj_cnt++;
    }
    // std::cout<<"hehe: "<<left_cnt<<"  "<<right_cnt<<"\n";
    // waitKey(0);
    if  (p_x_2>-10 && p_x_2 < 650.0 && p_y_2>-10 && p_y_2 < 490.0 && projected_points_view_2(2,i)>0 && projected_points(2,i)>0)
    {
      /* code */
     keypoints_scene_org.push_back(KeyPoint(Point2f(p_x_2,p_y_2),2.0));
     fake_right.push_back(KeyPoint(Point2f(p_x_0,p_y_0),2.0));

     DMatch a; a.queryIdx = right_cnt; a.trainIdx = sce_cnt;
     right_match.push_back(a);
     if (p_x_2>0 && p_x_2 < 640.0 && p_y_2>0 && p_y_2 < 480.0 && p_x_0>0 && p_x_0 < 640.0 && p_y_0>0 && p_y_0 < 480.0 )
     {
       /* code */
     
     // show_test.at<Vec3b>(round(p_y_0),round(p_x_0)).val[0] = 255;
     back_view.at<uchar>(round(p_y_2), round(p_x_2), 0) = 255;
    outfile<<common_f(0,i)<<" "<<common_f(1,i)<<" "<<common_f(2,i)<<"\n";

   }


     right_cnt++;sce_cnt++;
   }
 }
 cvtColor(show_test,show_test, CV_RGB2GRAY);


 show_match(show_test, front_view, fake_left,  keypoints_object_org, left_match, "front_view"+dir_1);
  std::cout<<"asdasdasd\n";
 show_match(show_test, back_view, fake_right,  keypoints_scene_org, right_match, "back_view"+dir_2);

  // waitKey(0);

// outfile<<"\n\n";
 waitKey(1000);outfile<<"\n";
 outfile.close();

 std::cout<<"fake_left features: "<<fake_left.size()<<"\n";
 std::cout<<"fake_right features: "<<fake_right.size()<<"\n";


 std::cout<<"width: "<<width<<"hehe show!\n";
 namedWindow("show_test");
 imshow("show_test",show_test);
 waitKey(0);
 std::cout<<"start stitching!\n";




 // stitching_thread(dir_1, dir_2, 100, 1, 125, 130);

 string obj_dir = dir_1;
 string sec_dir = dir_2;

   Mat result = create_view(obj_dir, sec_dir, keypoints_object_org, keypoints_scene_org, fake_left,fake_right, left_match, right_match);
   // Mat result_2 = create_view_stitching(obj_dir, sec_dir, keypoints_object_org, keypoints_scene_org, fake_left,fake_right, left_match, right_match);


 for (int i = 0; i < camera_set.size(); ++i)
 {
    /* free camera_set memory */
  free(camera_set[i]);
}


return 0;
}

// /*** @ main loop function***/
// //int main(int argc, char** argv) {
// int stitching_thread(string obj_dir, string sec_dir, 
//   std::vector<KeyPoint>& fake_left, std::vector<KeyPoint>& fake_right,
//   std::vector<DMatch> left_match, std::vector<DMatch> right_match,
// /*int m_size, int NNN, int frame_counter, int file_name_counter*/) {

//   std::cout<<"\n************stitching_thread*****************\n\n";
//     // Mat img_object_org = imread(obj_dir, CV_LOAD_IMAGE_COLOR); //new
//   Mat img_object_org = rectify_read(obj_dir);
//     Mat img_scene_org = rectify_read(sec_dir);  //previous result
//     cvtColor(img_object_org, img_object_org, CV_GRAY2RGB);
//     cvtColor(img_scene_org, img_scene_org, CV_GRAY2RGB);

//     if (!img_object_org.data || !img_scene_org.data) {
//       std::cout << " --(!) Error reading images " << std::endl;
//       return -1;
//     }


//     //-- Step 1: Detect the keypoints using SFT Detector
//     //int minHessian = 400; //1400 for m1m2 3&4; 400 for 1&2; 450 for m3 m4

//     // SiftFeatureDetector detector;
//     // Ptr<FeatureDetector> detector = FeatureDetector::create("FAST");

//     // detector->detect(img_object_org, keypoints_object);
//     // detector->detect(img_scene_org, keypoints_scene);

//     //--STEP 1.5   create the boarder
//     // if(img_object_org.rows == img_scene_org.rows && img_object_org.cols == img_scene_org.cols) {
//     keypoints_scene = keypoints_scene_org;
//     keypoints_object = keypoints_object_org;
//     for (int i = 0; i < keypoints_scene_org.size(); i++) {
//       {
//         keypoints_scene[i].pt.x = keypoints_scene_org[i].pt.x
//         + img_object_org.cols * (0.5);
//         keypoints_scene[i].pt.y = keypoints_scene_org[i].pt.y
//         + img_object_org.rows * (0.5);
//            // std::cout<<"choose: "<<keypoints_scene[i].pt <<"\n";
//       }
//     }
//       // waitKey(0);
//     // } 
//     // else {
//     //        Scene does nothing about feature points
//     //      *  Feature points are already in enlarged corredinates

//     //    }

//     for (int i = 0; i < keypoints_object_org.size(); i++) {
//       {
//         keypoints_object[i].pt.x = keypoints_object_org[i].pt.x
//         + img_object_org.cols * (0.5);
//         keypoints_object[i].pt.y = keypoints_object_org[i].pt.y
//         + img_object_org.rows * (0.5);
//       }
//     }


//     for (int i = 0; i < fake_left.size(); i++) {
//       {
//         fake_left[i].pt.x = fake_left[i].pt.x
//         + img_object_org.cols * (0.5);
//         fake_left[i].pt.y = fake_left[i].pt.y
//         + img_object_org.rows * (0.5);
//       }
//     }

//     for (int i = 0; i < fake_right.size(); i++) {
//       {
//         fake_right[i].pt.x = fake_right[i].pt.x
//         + img_object_org.cols * (0.5);
//         fake_right[i].pt.y = fake_right[i].pt.y
//         + img_object_org.rows * (0.5);
//       }
//     }

//     Size sz;
//     img_object = Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2,
//       CV_8UC3);

//       // if(img_object_org.rows == img_scene_org.rows && img_object_org.cols == img_scene_org.cols) {

//     img_scene = Mat::zeros(img_object_org.rows * 2, img_object_org.cols * 2, CV_8UC3);

//       // } else {

//       //   img_scene = Mat::zeros(img_scene_org.rows, img_scene_org.cols, CV_8UC3);

//       // }


//     Mat ROI_obj = img_object(
//       cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
//        img_object_org.cols, img_object_org.rows));

//     Mat ROI_scene = img_scene(
//       cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),
//        img_object_org.cols, img_object_org.rows));

//     img_object_org.copyTo(ROI_obj);
//     if(img_object_org.rows == img_scene_org.rows && img_object_org.cols == img_scene_org.cols) {
//       img_scene_org.copyTo(ROI_scene);
//     } else {

//       img_scene = img_scene_org;

//     }
//     ROI_obj.release();
//     ROI_scene.release();

//     //-- Step 2: Calculate descriptors (feature vectors)
//     //   SiftDescriptorExtractor extractor;

//     //   Mat descriptors_object, descriptors_scene;

//     //   extractor.compute(img_object, keypoints_object, descriptors_object);
//     //   extractor.compute(img_scene, keypoints_scene, descriptors_scene);

//     // //-- Step 3: Matching descriptor vectors using  matcher
//     //   BFMatcher matcher(NORM_L2,false);
//     //   std::vector<DMatch> matches;
//     //   matcher.match(descriptors_object, descriptors_scene, matches);
//     //   std::cout<<"SIFT key points number:"<<matches.size()<<"\n";


//     //   double max_dist = 0;
//     //   double min_dist = 100;

//     // //-- Quick calculation of max and min distances between keypoints
//     //   for (int i = 0; i < descriptors_object.rows; i++) {
//     //     double dist = matches[i].distance;
//     //     if (dist < min_dist)
//     //       min_dist = dist;
//     //     if (dist > max_dist)
//     //       max_dist = dist;
//     //   }

//     //   printf("-- Max dist : %f \n", max_dist);
//     //   printf("-- Min dist : %f \n", min_dist);

//     //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )

//       // for (int i = 0; i < descriptors_object.rows; i++) {
//       //   if ((matches[i].distance < 8 * min_dist)) {
//       //     good_matches.push_back(matches[i]);
//       //   }
//       // }
//     Mat return_result_L = Mat::zeros(img_scene.size, CV_8UC3);
//     Mat return_result_R = Mat::zeros(img_scene.size, CV_8UC3);

//     show_match(return_result_L, img_object, fake_left,  keypoints_object, left_match, "left_match");
//     show_match(return_result_R, img_scene, fake_right,  keypoints_scene, right_match, "left_match");
//       // descriptors_object.release();
//       // descriptors_scene.release();
//     /*
//         K-means and Dual-Homography
//     */
//         good_matches = left_match;

//           Scalar colorTab[] =     
//           {
//             Scalar(0, 0, 255),
//             Scalar(0,255,0),
//           };

//           int K = 2; Mat points(good_matches.size(), 1, CV_32FC3), labels;
//           Mat centers(K, 1, points.type());

//           for (int i = 0; i < good_matches.size(); ++i)
//           {

//             float dx = fake_left.at(good_matches[i].trainIdx).pt.x - keypoints_object.at(good_matches[i].queryIdx).pt.x;
//             float dy = fake_left.at(good_matches[i].trainIdx).pt.y - keypoints_object.at(good_matches[i].queryIdx).pt.y;
//           // std::cout<<"train  query"<<good_matches[i].trainIdx<<" "<<good_matches[i].queryIdx<<" i:"<<i<<" \n";

//           points.at<Vec3f>(i,0)[0] = fake_left.at(good_matches[i].trainIdx).pt.x;//(keypoints_scene_org.at(good_matches[i].trainIdx).pt.x - cc[0])*d[0]/(fc[0]);
//           points.at<Vec3f>(i,0)[1] = fake_left.at(good_matches[i].trainIdx).pt.y;//(keypoints_scene_org.at(good_matches[i].trainIdx).pt.y - cc[1])*d[0]/(fc[1]);
//           points.at<Vec3f>(i,0)[2] = sqrt(dx*dx + dy*dy);
//         }
//         std::vector<DMatch> part_A_matches, part_B_matches;
//         std::cout<<"check its running: "<<"\n";
//         kmeans(points, K, labels,
//           TermCriteria(  TermCriteria::EPS+TermCriteria::COUNT, 10, 3.0),
//           5, KMEANS_RANDOM_CENTERS, centers); 

//         Mat show;
//         img_scene.copyTo(show);

//         for( int i = 0; i < good_matches.size(); i++ )
//         {
//           int clusterIdx = labels.at<int>(i);
//           Point2f ipt;
//             // float depth = points.at<Vec3f>(i)[2];
//             ipt.x =  points.at<Vec3f>(i)[0]; //points.at<Vec3f>(i)[0]*(fc[0]/depth) + cc[0] + img_object_org.cols * (0.5);
//             ipt.y =  points.at<Vec3f>(i)[1]; //points.at<Vec3f>(i)[1]*(fc[1]/depth) + cc[1] + img_object_org.rows * (0.5);
//             std::cout<<"x y "<<ipt<<"\n";
//             circle( show, ipt, 2, colorTab[clusterIdx], CV_FILLED, CV_AA );
//             if (clusterIdx == 0)
//             {
//                /* code */
//               part_A_matches.push_back(good_matches[i]);
//             } else{
//               part_B_matches.push_back(good_matches[i]);
//             }
//           }
//           std::cout<<"k-means done!\n part_A size:"<<part_A_matches.size()<<"\n part_B size: "<<part_B_matches.size()<<"\n";
        
//         for (int i = 0; i < 2; ++i)
//         {
//     /* code */
//           Point2f ipt;
//             ipt.x = centers.at<Vec3f>(i)[0]; //points.at<Vec3f>(i)[0]*(fc[0]/depth) + cc[0] + img_object_org.cols * (0.5);
//             ipt.y = centers.at<Vec3f>(i)[1]; //points.at<Vec3f>(i)[1]*(fc[1]/depth) + cc[1] + img_object_org.rows * (0.5);
//             circle( show, ipt, 5, Scalar(255,0,0), CV_FILLED, CV_AA );
//           }
//           imshow("keypoints distrubtion",show);
//           waitKey(0);
//           int cnt = 0;
//           float ratio = 1;
//           int num = 1;
//           match_seg mid, part_A, part_B;
//           mid = find_mid_point_stitch(good_matches, ratio, fake_left, keypoints_object, fake_camera);
//           part_A = find_mid_point_stitch(part_A_matches, ratio, keypoints_object, keypoints_scene, fake_camera);
//           part_B = find_mid_point_stitch(part_B_matches, ratio, keypoints_object, keypoints_scene, fake_camera);
//     // Mat Hl = findHomography(mid.obj, mid.mid, CV_RANSAC);
//     // Mat Hr = findHomography(mid.scene, mid.mid, CV_RANSAC);
//      // assert(good_matches.size() > 4);
//           if (good_matches.size()<4)
//           {
//        /* code */
//             std::cout<<"less than 4 points! ERROR!\n";
//             return 0;
//           }

//     // Mat total[6];
//           std::vector<Mat> total(6);
//           try{
//            total[0] = findHomography(mid.obj, mid.mid, CV_RANSAC);
//            total[1] = findHomography(mid.scene, mid.mid, CV_RANSAC);
//            total[2] = findHomography(part_A.obj, part_A.mid, CV_RANSAC);
//            total[3] = findHomography(part_A.scene, part_A.mid, CV_RANSAC);
//            total[4] = findHomography(part_B.obj, part_B.mid, CV_RANSAC);
//            total[5] = findHomography(part_B.scene, part_B.mid, CV_RANSAC);
//          }catch (int e){
//           std::cout<<"missing a frame due to lack of feature points < 4\n";
//           good_matches.clear();
//           keypoints_object.clear();
//           keypoints_object_org.clear();
//           keypoints_scene.clear();
//           keypoints_scene_org.clear();
//           img_object.release();
//           img_scene.release();
//           return 0;
//         }

//  //  Mat sqrtm[3];
//  //  for (int i = 0; i < 3; ++i)
//  //  {
//  //  /* code */
//  //    Eigen::MatrixXd X = Eigen::MatrixXd(total[i].rows,total[i].cols);
//  //    //OpenCV -> Eigen 
//  //    try{
//  //      cv2eigen(total[i],X);
//  //      Eigen::MatrixXd C = X.sqrt();
//  //      Eigen::MatrixXd C2 = C.sqrt();
//  //    //Eigen -> OpenCV
//  //      eigen2cv(C2, sqrtm[i]);
//  //    }catch (int i){
//  //     std::cout<<"cannot solve square root of Homography matrix\n";

//  //     good_matches.clear();
//  //     keypoints_object.clear();
//  //     keypoints_object_org.clear();
//  //     keypoints_scene.clear();
//  //     keypoints_scene_org.clear();
//  //     img_object.release();
//  //     img_scene.release();

//  //     return 0;
//  //   }
//  // }

//         while(cnt < num) {
//          Mat result;
//         //show_match(result_A, result_B, keypoints_object,  keypoints_scene, good_matches);
//         //result = Mat::zeros(result_A.rows,result_A.cols,CV_8UC3);
//          if (part_A_matches.size() < 4 || part_B_matches.size() < 4 )
//          {
//            /* code */
//           std::cout<<"part_B or part_A < 4! ERROR\n";exit(0);
//     // result = stitching_bg( mid, img_object,  img_scene, img_object, file_name_counter + cnt, total , cnt, good_matches);

//         }else{

//           result = dual_stitching_bg(mid, part_A, part_B, img_object,  img_scene, img_object, file_name_counter + cnt, total , cnt, good_matches, centers);
//         }
//         /*
//          * start stitching!--> Object!
//          */
//          std::cout<<"start stitching!--> Object!\n";
//          int seg_index = 0;
//          char s[10];
//          char d[50];
//          Mat cropped_result;
//          if(img_object_org.rows == img_scene_org.rows && img_object_org.cols == img_scene_org.cols) {
//           std::sprintf(s,"./results/result: %d.ppm",cnt);
//           std::sprintf(d,"./results/render: %d.ppm",file_name_counter  + cnt);
//           cropped_result = result(cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),img_object_org.cols, img_object_org.rows));
//            imwrite(d, cropped_result);//        waitKey(300);


//          } else {
//           std::sprintf(s,"./results/result: %d.ppm",frame_counter);
//           imwrite(s, result);
// #ifdef CROP
//           std::sprintf(d,"./results/cropped_result: %d.ppm",frame_counter);
//           cropped_result = result(cv::Rect(img_object_org.cols * (0.5), img_object_org.rows * (0.5),img_object_org.cols, img_object_org.rows));
// #endif
//         }
// #ifdef CROP
//         imwrite(d, cropped_result);//        waitKey(300);
// #endif
//         std::cout<<"--------->"<<file_name_counter<<"ratio: "<<ratio<<"\n\n";
//         ratio = ratio - 1.0/(num-1);
//         cnt++;
//       }


//     //exit(0);
//     //free memeory:
//       good_matches.clear();
//       keypoints_object.clear();
//       keypoints_object_org.clear();
//       keypoints_scene.clear();
//       keypoints_scene_org.clear();
//       img_object.release();
//       img_scene.release();

//       return 0;

//     }

/** @function readme */
    void readme() {

      std::cout << " Usage: ./ImageD <img1>(large) <img2> num_of_frames min_size_of_seg" << std::endl;
    }
