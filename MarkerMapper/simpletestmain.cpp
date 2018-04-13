//
//  simpletestmain.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/4/12.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "stdafx.h"

#include "Tools.hpp"

int main(int argc, const char * argv[]) {
    float rvec[3] = {0, 0, 0};
    float tvec[3] = {0, 0, 0};
    
    Mat mat;
    getViewMatrixFromRvecTvec(rvec, tvec, mat);
    
    cout << mat << endl;
    cout << mat.type() << endl;
    cout << cv::Mat::eye(4, 4, CV_32F) << endl;
    cout << CV_32F << endl;
    
    return 0;
}


