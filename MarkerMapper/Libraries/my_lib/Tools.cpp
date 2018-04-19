//
//  Tools.cpp
//  ARTest
//
//  Created by 郭子乐 on 2017/12/28.
//  Copyright © 2017年 郭子乐. All rights reserved.
//

#include "Tools.hpp"

void getViewMatrixFromRvecTvec(float* rvec, float* tvec, Mat& viewMatrix){
    Mat rotation;
    Mat rvector = Mat(Size(1,3), CV_32F, rvec);
    Mat tvector = Mat(Size(1,3), CV_32F, tvec);
    viewMatrix = Mat(4, 4, CV_32F);
    cv::Rodrigues(rvector, rotation);
    
    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<float>(row, col) = rotation.at<float>(row, col);
        }
        viewMatrix.at<float>(row, 3) = tvector.at<float>(row, 0);
    }
    viewMatrix.at<float>(3, 3) = 1.0f;
}

void getViewMatrixFromRvecTvec(InputArray& rvec, InputArray& tvec, Mat& viewMatrix) {
    assert(rvec.type()==CV_32FC1);
    assert(tvec.type()==CV_32FC1);
    
    Mat rotation;
    Mat rvector = rvec.getMat();
    Mat tvector = tvec.getMat();
    viewMatrix = Mat::zeros(4, 4, CV_32F);
    cv::Rodrigues(rvector, rotation);
    
    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<float>(row, col) = rotation.at<float>(row, col);
        }
        viewMatrix.at<float>(row, 3) = tvector.at<float>(row, 0);
    }
    viewMatrix.at<float>(3, 3) = 1.0f;
}

void getRvecTvecFromViewMatrix(InputArray& viewMatrix, Mat& rvec, Mat& tvec){
    assert(viewMatrix.type()==CV_32FC1);
    
    Mat vm = viewMatrix.getMat();
    rvec = Mat(3, 1, CV_32F);
    tvec = Mat(3, 1, CV_32F);
    Mat rotation = vm(Range(0, 3), Range(0, 3));
    cv::Rodrigues(rotation, rvec);
    tvec = vm(Range(0, 3), Range(3, 4));
}

void cvToGl(const Mat& cv, Mat& gl){
    cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis
    cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis
    cvToGl.at<double>(3, 3) = 1.0f;
    gl = cvToGl * cv;
}

void glToCv(const Mat& gl, Mat& cv){
    cv::Mat glToCv = cv::Mat::zeros(4, 4, CV_64F);
    glToCv.at<double>(0, 0) = 1.0f;
    glToCv.at<double>(1, 1) = -1.0f; // Invert the y axis
    glToCv.at<double>(2, 2) = -1.0f; // invert the z axis
    glToCv.at<double>(3, 3) = 1.0f;
    cv = glToCv * gl;
}

void cvToGlm(const Mat& cv, glm::mat4& glm){

}
void glmToCv(const glm::mat4& glm, Mat& cv){
    
}
