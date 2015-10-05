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
#include <iostream>
#include <iomanip>
#include <fstream>
#include "opencv2/video/tracking.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>

#define WRITE_H
#define PI 3.14159265
using namespace cv;


void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    //Mat p1 = H.col(0);       // Pointer to first column of H
    // Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

    cv::normalize( H.col(0),  pose.col(0));   // Normalize the rotation, and copies the column to pose

    // p1 = H.col(1);           // Pointer to second column of H
    // p2 = pose.col(1);        // Pointer to second column of pose (empty)

    cv::normalize(H.col(1), pose.col(1));   // Normalize the rotation and copies the column to pose

    //p1 = pose.col(0);
    //p2 = pose.col(1);

    Mat p3 = pose.col(0).cross(pose.col(1));   // Computes the cross-product of p1 and p2
    //Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(pose.col(2));       // Third column is the crossproduct of columns one and two

    Mat temp = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
    temp.copyTo(pose.col(3));
}





struct seg
{
    image<rgb>* IMG;
    vector<DMatch> match;
    long ID;
    long match_ID;
    Mat img;
};

double calculating_hist(Mat &test_image, Mat &Standard_image) {

    int bins = 256;
    int histSize[] = {bins};

    float lranges[] = {0, 256};

    const float* ranges[] = { lranges };

    int channels[] = { 0 };

    MatND hist_test_image, hist_standard_image;
    calcHist( &test_image, 1, channels, Mat(), hist_test_image, 1, histSize, ranges, true, false );
    normalize( hist_test_image, hist_test_image, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &Standard_image, 1, channels, Mat(), hist_standard_image, 1, histSize, ranges, true, false );
    normalize( hist_standard_image, hist_standard_image, 0, 1, NORM_MINMAX, -1, Mat() );

    double test_standard = compareHist( hist_standard_image, hist_test_image, 0 );
    return test_standard;
}

void blending(Mat &temp, Mat &temp2, Mat &result, int counter_) {
    Vec3b zeros;
    zeros[0] = 0;
    zeros[1] = 0;
    zeros[2] = 0;
    Vec3b x;
    float weight = (counter_+1)*1.0/5;
    for (int i = 0; i < temp.cols; i++) {
        for (int ii = 0; ii < temp.rows; ii++) {
            if (temp.at < Vec3b > (ii, i) != zeros && temp2.at < Vec3b > (ii, i) != zeros) {
                x =  temp.at < Vec3b > (ii, i);//*weight + temp2.at < Vec3b > (ii, i)*(1-weight);

            } else if(temp.at < Vec3b > (ii, i) != zeros) {
                x = temp.at<Vec3b>(ii,i);
            } else if(temp2.at<Vec3b>(ii,i)!=zeros) {
                x = temp2.at<Vec3b>(ii,i);
            } else {
                Vec3b a;
                a[0] = 0;
                a[1]=255;
                a[2]=0;

                x = zeros;
            }
            result.at < Vec3b > (ii, i) = x;
        }
    }
}

void pyrDown_(Mat input, Mat& output, Size size){
   GaussianBlur( input, input, Size( 31, 31), 0, 0 );
   resize(input, output, size, 0, 0, INTER_LINEAR);
}

void pyrUp_(Mat input, Mat& output, Size size){
   GaussianBlur( input, input, Size( 31, 31), 0, 0 );
   resize(input, output, size, 0, 0, INTER_LINEAR);
}

std::vector<Mat> gaussian_pyramid(Mat input_, int level){
    std::vector<Mat> prymaid;
    Mat input, pre_input;
    resize(input_, pre_input, Size( input_.cols*2, input_.rows*2 ), 0, 0, INTER_LINEAR);
    pre_input.convertTo(input,CV_32FC1);

    for (int i = 0; i < level; ++i)
    {
        /* code */
        Mat temp = Mat::zeros(input.rows/2,input.cols/2,CV_32FC1);
        pyrDown_( input, temp, Size( input.cols/2, input.rows/2));
        prymaid.push_back(temp);
                        // std::cout<<"Size of gaussian_pyramid: "<<temp.size()<<"\n";

        input = temp;
    }
    return prymaid;
}

std::vector<Mat> laplacian_pyramid(Mat input_, int level){
    Mat input;
    // Where we can only handle gray image then!
    cvtColor(input_, input_, CV_RGB2GRAY);

    input_.convertTo(input,CV_32FC1,1.0/255.0);

    std::cout<<"size of laplacian_pyramid input: "<<input.channels()<<"\n";
    std::vector<Mat> G_pyramid = gaussian_pyramid(input, level);

    
    std::vector<Mat> prymaid;
    for (int i = 0; i < level - 1; ++i)
    {
        /* code */
        Mat L;// = input - G_pyramid[i];
        Mat upsampled;
        pyrUp_( G_pyramid[i+1], upsampled, Size( G_pyramid[i+1].cols*2, G_pyramid[i+1].rows*2));
        addWeighted(input, 1, upsampled, -1, 0, L);
        prymaid.push_back(L);

        Mat temp;
        resize(G_pyramid[i], input, Size( input.cols/2, input.rows/2 ), 0, 0, INTER_LINEAR);
    }
    prymaid.push_back(input);
    return prymaid;
}
Mat laplacian_pyramid_reconstruct(std::vector<Mat> prymaid){
    int level = prymaid.size();
    std::cout<<"size of laplacian_pyramid_reconstruct prymaid size"<<prymaid.size()<<"\n";

    Mat output = prymaid[level -1];

    for (int i = level - 2; i > -1; --i)
    {
        /* code */
        Mat temp;
        pyrUp_( output, temp, Size( output.cols*2, output.rows*2));
        addWeighted(temp, 1, prymaid[i], 1, 0, output);
        // imshow("***",prymaid[i]);waitKey(0);
    }
    std::cout<<"size of laplacian_pyramid_reconstruct output"<<output.size()<<"\n";
    return output;
}

void laplacian_blending(Mat temp, Mat temp2, Mat &result, Mat Hr, Mat Hl, int counter) {

    std::cout<<"Start laplacian_blending!!!\n";

    Mat temp_mask;
    temp.convertTo(temp_mask,CV_8UC1);
    temp_mask = temp_mask.mul(255);
    Mat temp_mask_inv;    Mat intersects_;

    Mat temp2_mask;
    temp2.convertTo(temp2_mask,CV_8UC1);
    temp2_mask = temp2_mask.mul(255);

    // Difference
    Mat differ_mask, temp_differ_region, temp_differ_region2;
    bitwise_and(temp_mask, temp2_mask, differ_mask);
    bitwise_and(differ_mask, temp, temp_differ_region);
    bitwise_and(differ_mask, temp2, temp_differ_region2);
    Mat real_difference;
    // addWeighted(temp_differ_region, 1.0, temp_differ_region2, -1.0, 0, real_difference);
    // cvtColor(temp_differ_region, temp_differ_region, CV_RGB2GRAY); 
    // cvtColor(temp_differ_region2, temp_differ_region2, CV_RGB2GRAY); 

    absdiff(temp_differ_region, temp_differ_region2, real_difference);
    threshold(real_difference, real_difference, 30, 255, THRESH_TOZERO);

        // imshow("real_difference",real_difference);waitKey(0);

            // cvtColor(real_difference, real_difference, CV_RGB2GRAY); 
            // Mat one(temp.rows,temp.cols,CV_8UC3,CV_RGB(1,1,1));
            // // one.copyTo(one,differ_mask);
            //  bitwise_and(one, differ_mask, one);
            // cvtColor(one, one, CV_RGB2GRAY); 
            // one = one.mul(255);
    // cvtColor(real_difference, real_difference, CV_RGB2GRAY); 
    // std::cout<<"type "<<real_difference.type()<<" "<<one.type()<<"\n";


     // cvtColor(real_difference, real_difference, CV_RGB2GRAY); 

        // imshow("one",one);

        // waitKey(0);
    // real_difference.convertTo(real_difference,CV_32FC1,1.0/255);
    // real_difference = real_difference.mul(1/255);
    // imshow("asas",real_difference);waitKey(0);

    //
    threshold(temp_mask, temp_mask_inv, 100, 255, THRESH_BINARY_INV);

    bitwise_and(temp_mask_inv, temp2_mask, intersects_);
    // imshow("now",intersects_);wa

    // blur( real_difference, real_difference, Size( /*(counter+1)*20 +*/ 5, /*(counter+1)*20 +*/ 5));
        imshow("real_difference",real_difference);waitKey(0);

    blur( intersects_, intersects_, Size( /*(5-counter)*30 +*/ 31, /*(5-counter)*30 +*/ 31));

          addWeighted(intersects_, 1.0, real_difference, 0.0, 0, intersects_);



    threshold(intersects_, intersects_, 10, 255, THRESH_BINARY_INV);

    // imshow("intersects_",intersects_);//waitKey(0);

    cvtColor(intersects_, intersects_, CV_BGR2GRAY);       // 1. change the number of channels
    intersects_.convertTo(intersects_, CV_32FC1, 1.0/255.0);

    // imshow("intersects_",intersects_);

    // waitKey(0);

    std::cout<<"binary_mask Start !\n";

    // imshow("binary_mask",binary_mask);
    //Create the gaussian prymaid of binary mask
    std::cout<<"Start binary_mask_G_pyramid!!!\n";

                // std::cout<<"intersects_ type "<<intersects_.type()<<"\n";


    std::vector<Mat> binary_mask_G_pyramid = gaussian_pyramid(intersects_,6);
    //Create the lapician prymaid of temp mask
    std::cout<<"Start laplacian_blending!!!\n";

    std::vector<Mat> temp_L_prymaid = laplacian_pyramid(temp,6);
    //Create the lapician prymaid of temp2 mask
    std::vector<Mat> temp2_L_pryamid = laplacian_pyramid(temp2,6);
    //blend each level of pyramid
    std::cout<<"Start blending dot-product!!!\n";

    std::vector<Mat> blend;
    float weight = (counter+1)*1.0/5;
    for (int i = 0; i < 6; ++i)
    {
        /* code */
        Mat one = Mat::ones(binary_mask_G_pyramid[i].rows, binary_mask_G_pyramid[i].cols,CV_32FC1);
        Mat diff_G;
            // std::cout<<"Start A\n";
            // std::cout<<"type "<<binary_mask_G_pyramid[i].type()<<"  "<<one.type()<<"\n";

        addWeighted(one, 1.0, binary_mask_G_pyramid[i], -1.0, 0, diff_G);
                    // imshow("diff_G",diff_G);waitKey(0);
            // std::cout<<"Start B\n";

        Mat tmp = temp_L_prymaid[i].mul(binary_mask_G_pyramid[i]);
        Mat tmp2 =  temp2_L_pryamid[i].mul(diff_G);
        Mat out;
        addWeighted(tmp, 1, tmp2, 1, 0, out);
        blend.push_back(out);
    }
    //reconsturct.
    std::cout<<"Start laplacian_pyramid_reconstruct!!!\n";
    Mat pre_result;
    pre_result = laplacian_pyramid_reconstruct(blend);

//Draw seeam lines
    // Point p1,p2;
    // p1.x = seam_point.at<double>(0,0);
    // p1.y = seam_point.at<double>(1,0);
    // p2.x = seam_point.at<double>(0,1);
    // p2.y = seam_point.at<double>(1,1);

    // line(pre_result, p1, p2, Scalar( 255) );
    // imshow("current",pre_result);
    pre_result.convertTo(result,CV_8UC1,255);

            //test
    // Mat o = laplacian_pyramid_reconstruct(temp2_L_pryamid); imshow("XXX",o);imshow("tenp2",temp2); waitKey(0);
}


Mat stitching_two(match_seg mid, std::vector<seg> seg_A, std::vector<seg>  seg_B, Mat img_object, int seg_index, Mat result) {
    Mat Hl = findHomography(mid.obj, mid.mid, CV_RANSAC);
    Mat Hr = findHomography(mid.scene, mid.mid, CV_RANSAC);

    // Find the Homography Matrix
    // Use the Homography Matrix to warp the images

    //std::cout << Hl;
    ///std::cout << "\n\n";
    //std::cout << Hr;
    // std::cout << "hehe\n\n";
    /*warpPerspective(seg_A[seg_index].img, result, Hl,
    		cv::Size(img_object.cols, img_object.rows));*/
    cv::Mat temp, temp2;
    warpPerspective(seg_B[seg_index].img, temp, Hr,
        cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);
    int threshold_value = 0;
    int max_BINARY_value = 256;
    Mat mask;
    Mat result2;
    // temp.copyTo(result2, mask);

    warpPerspective(seg_A[seg_index].img, temp2, Hl,
        cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);
    // temp.copyTo(result2, mask);
    Mat haha = Mat::zeros(result.rows,result.cols,CV_8UC3);
    //temp.copyTo(haha);
    //std::cout<<"NEWNEWNEWEW\n";

    // blending(temp,temp2,haha);
    //threshold(haha, mask, threshold_value, max_BINARY_value, THRESH_BINARY);
    //haha.copyTo(result,mask);
    Mat x = Mat::zeros(result.rows,result.cols,CV_8UC3);
    //imshow("haha",temp);waitKey(0);


    // blending(haha,result,x);
    //result = x;
    return x;
}


Mat stitching_bg(match_seg mid, Mat A, Mat B, Mat img_object, int number, Mat &sqrtm, int counter_, std::vector<DMatch> & good_matches) {

    // Mat Hl = Mat::eye(3,3,CV_64FC1);
    // for (int i = 0; i < counter_  ; ++i)
    // {
    //     Hl = Hl*sqrtm;
    // }
    // Mat Hr = Mat::eye(3,3,CV_64FC1);
    // for (int i = 0; i < 4 - counter_; ++i)
    // {
    //     Hr = Hr*sqrtm.inv();
    // }
    // std::cout<<"****: "<<counter_<<std::endl;
    Mat Hl = sqrtm;

    // for (int i = 0; i < good_matches.size(); ++i)
    // {
    //        // circle(A, mid.obj[i], 1.5, Scalar( 0, 0, 255 ));
    //        // circle(B, mid.scene[i], 1, Scalar( 0, 255, 0 ));
    // }

#ifdef WRITE_H
    std::ofstream outfile;
    Mat dummy_point_temp;
    outfile.open("./results/Homography_matrix.txt", std::fstream::out | std::fstream::app);
    if (!outfile.is_open())
    {
        std::cout<<"ERROR write Homography text!\n";
        exit(-1);
    } else {

        // Mat dummy_point = Mat::zeros(3,1,CV_64FC1);

        // dummy_point.at<double>(0,0) = 200;
        // dummy_point.at<double>(1,0) = 100;
        // dummy_point.at<double>(2,0) = 1;

        // //dummy_point_temp = Hr*dummy_point;
        // //multiply(Hr,dummy_point,dummy_point_temp);
        // dummy_point_temp = Hr*dummy_point;
        // dummy_point_temp.at<float>(0,0) = round(dummy_point_temp.at<float>(1,0));
        // dummy_point_temp.at<float>(1,0) = round(dummy_point_temp.at<float>(1,0));
        //  dummy_point_temp.at<float>(2,0) = round(dummy_point_temp.at<float>(2,0));
        // outfile<<"****************************frame: "<< number <<std::endl;
        //outfile<<"dummy_point x: "<< dummy_point_temp<<std::endl;


        //   outfile<<"\n img:"<<number<<"\n";
        //outfile<<"Left H:\n";
        // outfile<<Hl;
        // outfile<<"\nRight H:\n";
        // outfile<<Hr;
        ////    outfile<<"\nProduct:\n";
        //                outfile<<Hl*Hr.inv();

        //  outfile<<"\nTotoal:\n";
        //  outfile<<total;
        //outfile<<"\nHl Norms: \n";
        // outfile<<cv::norm(previous_Hl,Hl);
        // outfile<<"\nHr Norms: \n";
        //outfile<<cv::norm(previous_Hr,Hr)<<"\n";
    }
#endif
    /*warpPerspective(seg_A[seg_index].img, result, Hl,
    		cv::Size(img_object.cols, img_object.rows));*/
    Mat temp, temp2;
    if(1) {
        // warpPerspective(B, temp, Hr,
        //     cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);

        warpPerspective(A, temp2, Hl,
            cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);
    }
    if(0) {
        Mat Ar = estimateRigidTransform(mid.obj,mid.mid,false);
        Mat Al = estimateRigidTransform(mid.mid,mid.scene,false);
        warpAffine(B,temp,Ar,cv::Size(temp.cols,temp.rows),INTER_NEAREST);
        warpAffine(A,temp2,Al,cv::Size(temp2.cols,temp2.rows),INTER_NEAREST);
    }
    //std::cout<<A.type()<<"  "<<A.channels()<<"\n";
    Mat result = Mat::zeros(temp.rows, temp.cols,CV_8UC3);
     // laplacian_blending(temp,temp2,result, Hr, Hl);
        // blending(temp,temp2,result);



    Mat pose = Mat::zeros(3,3, CV_32FC1);

    cameraPoseFromHomography(sqrtm*sqrtm*sqrtm*sqrtm, pose);
    std::cout<<"\n angel_x: "<<atan2(pose.at<float>(2,1),pose.at<float>(2,2)) * 180 / PI<<std::endl;
    std::cout<<"\n angel_z: "<<atan2(pose.at<float>(1,0),pose.at<float>(0,0)) * 180 / PI<<std::endl;
    std::cout<<"\n angel_y: "<<atan2((-1*pose.at<float>(2,0)),sqrt((pose.at<float>(2,1)*pose.at<float>(2,1)+pose.at<float>(2,2)*pose.at<float>(2,2)))) * 180 / PI<<std::endl;

    //outfile<< atan2(pose.at<float>(1,0),pose.at<float>(0,0)) * 180 / PI<<std::endl;
    //outfile<< pose.at<float>(0,3) <<std::endl;
    //outfile<< pose.at<float>(1,3) <<std::endl;
    outfile<< pose.at<float>(2,3) <<std::endl<<std::endl<<std::endl<<std::endl;

    // Mat checking = result(cv::Rect(img_object.cols * (0.25), img_object.rows * (0.25),img_object.cols*(0.5), img_object.rows*(0.5)));
    // static int sum_y = 0;

    // static Mat temp_previous = checking;
    // sum_y = sum_y + atan2(pose.at<float>(1,0),pose.at<float>(0,0)) * 180 / PI;
    // std::cout<<"SUM: "<<sum_y<<std::endl;
    // double correlation = calculating_hist(checking, temp_previous);
    // //outfile<<correlation<<"\n";
    // std::cout<<"correlation: "<<correlation<<"\n";

    // // imshow("ERROR",result);
    // imshow("temp_previous",temp_previous);
    // //waitKey(0);

    // if (correlation < 0)
    // {
    //     std::cout<<"What should we do?? hwo to rebuild?"<<std::endl; waitKey(0);
    //     // return stitching_bg( resort_mid_point(mid), A, B, img_object, number, sqrtm, counter_);
    // } else {
    //     temp_previous = checking;
    // }

    return temp2;
}

/*
    Dual Homography
*/
uchar bi_linear_interpolation(Mat p, Mat img){
    // if (p.at<double>(2,0) < 0)
    // {
    //      code 
    //     return 0;
    // }
    p = p*(1.0/p.at<double>(2,0));
    double x_floor = (int)floor(p.at<double>(0,0));
    double y_floor = (int)floor(p.at<double>(1,0));
    double x = (p.at<double>(0,0));
    double y = (p.at<double>(1,0));
    uchar pixel = 0;
    if (x > 0 && y > 0 && y < img.rows && x < img.cols)
    {
        /* code */
     pixel = (x - x_floor)*(y - y_floor)*img.at<uchar>(y_floor+1,x_floor+1) + (x_floor + 1 - x)*(y_floor + 1 - y)*img.at<uchar>(y_floor,x_floor) + (x - x_floor)*(y_floor + 1 - y)*img.at<uchar>(y_floor,x_floor+1) + (x_floor + 1 - x)*(y - y_floor)*img.at<uchar>(y_floor+1,x_floor);
         // std::cout<<"pixel value:" << pixel<<"\n";
    }
    return pixel;

}



void dual_warp(Mat img, Mat& output, Mat H_A, Mat H_B, std::vector<Point2f> part_A, std::vector<Point2f> part_B, Mat & centers){

flann::KDTreeIndexParams indexParams_A, indexParams_B;
flann::Index kdtree_A(Mat(part_A).reshape(1), indexParams_A);
flann::Index kdtree_B(Mat(part_B).reshape(1), indexParams_B);

output = Mat::zeros(img.rows,img.cols,CV_8UC3);
std::cout<<"start dual Homography! May take a while\n";
for (int col = 0; col < output.cols; ++col)
{
    for (int row = 0; row < output.rows; ++row)
    {
        std::vector<float> query;
        // query.push_back(col); //Insert the 2D point we need to find neighbours to the query
        // query.push_back(row); //Insert the 2D point we need to find neighbours to the query
        std::vector<int> indices;
        std::vector<float> dists_A, dists_B;
        float cx_1 = centers.at<Vec3f>(0)[0];
        float cy_1 = centers.at<Vec3f>(0)[1];
        float cx_2 = centers.at<Vec3f>(1)[0];
        float cy_2 = centers.at<Vec3f>(1)[1];
        dists_A.push_back((col - cx_1)*(col - cx_1) + (row - cy_1)*(row - cy_1));
        dists_B.push_back((col - cx_2)*(col - cx_2) + (row - cy_2)*(row - cy_2));
        // kdtree_A.knnSearch(query, indices, dists_A, 1,flann::SearchParams(16));
        // kdtree_B.knnSearch(query, indices, dists_B, 1),flann::SearchParams(16);
// std::cout<<"dists_A: "<<dists_A[0]<<"  dists_B"<<dists_B[0]<<"\n";
        float dist_all = dists_A[0] + dists_B[0];
        if (dist_all == 0)
        {
            /* code */
            dists_A[0] = 1;
            dists_B[0] = 1;
            dist_all = 2;
        }

        Mat H = H_A * (dists_B[0]/dist_all) + H_B * (dists_A[0]/dist_all);
            // std::cout<<"all distance"<<dist_all<<"\n"<<"\n";
        Mat position = Mat::zeros(3,1,CV_64FC1); 
        position.at<double>(0,0) = col; 
        position.at<double>(1,0) = row; 
        position.at<double>(2,0) = 1; 

        Mat p = H.inv() * position;
                        // std::cout<<"new position"<<p<<"\n";

        output.at<Vec3b>(row, col) =  bi_linear_interpolation(p, img);
    }
}
// imshow("Dual wrap",output);//waitKey(0);
}

Mat dual_stitching_bg(match_seg mid, match_seg part_A, match_seg part_B, Mat A, Mat B, Mat img_object, int number, std::vector<Mat> sqrtm, int counter_, std::vector<DMatch> & good_matches, Mat& centers) {

    Mat Hl_A = sqrtm[2];
    Mat Hl_B = sqrtm[4];

    Mat Hr_A = sqrtm[3];
    Mat Hr_B = sqrtm[5];

//     std::cout<<"counter: "<<counter_<<"\n";
 // std::cout<<"H_A"<<Hl_A<<Hl_B<<"\n"; waitKey(0);

    Mat temp, temp2;
    if(0) {
        warpPerspective(B, temp, Hr_A,

            cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);

        warpPerspective(A, temp2, Hl_A,
            cv::Size(img_object.cols, img_object.rows),INTER_NEAREST);
    }else{
        dual_warp(B,temp, Hr_A,Hr_B, part_A.obj, part_B.obj, centers);
        dual_warp(A,temp2, Hl_A,Hl_B, part_B.obj, part_B.obj, centers);
    }
    Mat result = Mat::zeros(temp.rows, temp.cols,CV_8UC3);
    imshow("one",temp);imshow("the other one",temp2);waitKey(0);
    laplacian_blending(temp,temp2,result, Hr_A, Hl_A, counter_);
    // blending(temp,temp2,result, counter_);


    Mat checking = result(cv::Rect(img_object.cols * (0.25), img_object.rows * (0.25),img_object.cols*(0.5), img_object.rows*(0.5)));

    static Mat temp_previous = checking;
 
    double correlation = calculating_hist(checking, temp_previous);
    std::cout<<"correlation: "<<correlation<<"\n";

    // imshow("ERROR",result);
    imshow("Eric's thread is working! Please don't shut me done...",temp_previous);
    waitKey(10);

    if (correlation < -10)
    {
        std::cout<<"What should we do?? hwo to rebuild?"<<std::endl; waitKey(0);
        // return stitching_bg( resort_mid_point(mid), A, B, img_object, number, sqrtm_A, counter_);
    } else {
        temp_previous = checking;
    }

    return result;
}