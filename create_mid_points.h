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
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"
using namespace cv;

bool my_f(DMatch a, DMatch b) {
    return (a.distance < b.distance);
}

struct match_seg {
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    std::vector<Point2f> mid;
};

match_seg find_mid_point(std::vector<DMatch> match, float x,std::vector<KeyPoint> keypoints_object, std::vector<KeyPoint> fake_camera)
{
    match_seg match_seg;
    // sort(match.begin(), match.end(), my_f);

    for (int i = 0; i < match.size(); i++) {
        //-- Get the keypoints from the good matches
        match_seg.obj.push_back(keypoints_object[match[i].queryIdx].pt);
        //std::cout << match_seg.obj[i] << "==" << match_seg.scene[i] << "\n";
        match_seg.mid.push_back(fake_camera[match[i].queryIdx].pt);
       // std::cout << match_seg.mid[i] << "\n";
    }
    return match_seg;
}

match_seg find_mid_point_stitch(std::vector<DMatch> match, float x,std::vector<KeyPoint> keypoints_object,std::vector<KeyPoint> keypoints_scene, std::vector<KeyPoint> fake_camera)
{
    match_seg match_seg;
    // sort(match.begin(), match.end(), my_f);

    for (int i = 0; i < match.size(); i++) {
        //-- Get the keypoints from the good matches
        match_seg.obj.push_back(keypoints_object[match[i].queryIdx].pt);
        match_seg.scene.push_back(keypoints_scene[match[i].trainIdx].pt);
        //std::cout << match_seg.obj[i] << "==" << match_seg.scene[i] << "\n";
        match_seg.mid.push_back(fake_camera[match[i].queryIdx].pt);
       // std::cout << match_seg.mid[i] << "\n";
    }
    return match_seg;
}

match_seg resort_mid_point(match_seg mid)
{
    std::cout<<"\n\n\nWe are in the error detection!!!\n\n";
   for (int i = 0; i < 1; ++i){
     cv::Point2f temp_mid =  mid.mid[0];
     cv::Point2f temp_scene =  mid.scene[0];
     cv::Point2f temp_obj =  mid.obj[0];
     mid.mid.erase(mid.mid.begin());
     mid.scene.erase(mid.scene.begin());
     mid.obj.erase(mid.obj.begin());
     //mid.mid.push_back(temp_mid);
     //mid.scene.push_back(temp_scene);
     //mid.obj.push_back(temp_obj);
   }
   if (mid.mid.size()<4){
       std::cout<<"no enough points for stithcing!"<<std::endl;
   }
    
    return mid;
}

void show_match(Mat imgA, Mat imgB, std::vector<KeyPoint>& keypoints_object, std::vector<KeyPoint>& keypoints_scene, std::vector<DMatch>& match, string name) {
    Mat img_matches;
    drawMatches(imgA, keypoints_object, imgB, keypoints_scene,
                match, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	// Size sz;
    //pyrDown(img_matches, img_matches, sz, BORDER_DEFAULT);
   imshow(name, img_matches);
       // std::cout<<"error befoer\n";

   waitKey(0);
       // std::cout<<"error here\n";

   // char key = cvWaitKey(300);
}
