//
//  dodtracmain.cpp
//  MarkerMapper
//
//  Created by 郭子乐 on 2018/4/9.
//  Copyright © 2018年 郭子乐. All rights reserved.
//

#include "stdafx.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "sglviewer.h"

#include "DodecaTracker.hpp"
#include "Calibration.hpp"

#include "Tools.hpp"

int main(int argc, const char * argv[]) {
    string camera_parameters_file_path = "Calibration/output/high_fps_camera_parameters.yml";
    
    string marker_map_path = "Calibration/output/dodecahedron_marker_map.yml";
    
    string case_path = "Tracking/high_fps_camera/case4.mov";
    
    DodecaTracker dt;
    dt.initCamera(camera_parameters_file_path, case_path);
    dt.initPen(marker_map_path);
    dt.initPenDetector();
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width    = 1000;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);
    int pcd_id = 0;
    
    while (dt.grab()) {
        if(dt.detect()){
            Mat m = dt.getPose();
            Mat rvec, tvec;
            getRvecTvecFromViewMatrix(m, rvec, tvec);
            cout << tvec << endl;
            
            cloud.points[pcd_id].x = tvec.at<float>(0, 0)*100.0f;
            cloud.points[pcd_id].y = tvec.at<float>(1, 0)*100.0f;
            cloud.points[pcd_id].z = tvec.at<float>(2, 0)*100.0f;
            pcd_id++;
        }
//        Mat img;
//        pd.camera->current_frame.copyTo(img);
//        pd.camera->drawDetectedMarkers(img);
//        pd.camera->draw3DAxis(img, dodeca_marker_size*2);
//
//        imshow("in", img);
//        waitKey();
//        while (char(cv::waitKey(0)) != 27)
//            ;  // wait for esc to be pressed
    }
    
    string pcd_path = "Tracking/high_fps_camera/case4_track.pcd";
    pcl::io::savePCDFileASCII (pcd_path, cloud);
    
    return 0;
}

