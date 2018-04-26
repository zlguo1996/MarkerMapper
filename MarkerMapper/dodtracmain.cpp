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
    string camera_parameters_file_path = "Calibration/output/camera/logitech_brio_camera_calibration_720p.yml";
    
    string marker_map_path = "Calibration/output/dodeca/dodecahedron_marker_map.yml";
    
    string case_path = "Tracking/brio_camera/case1.mp4";
    
    string pentip_parameters_file_path = "Calibration/output/dodeca/pentip_calibration.yml";
    string dodeca_center_parameters_file_path = "Calibration/output/dodeca/dodeca_center_calibration.yml";
    
    
    DodecaTracker dt;
    dt.initCamera(camera_parameters_file_path, case_path);
    dt.initPen(marker_map_path);
    dt.setPenTip(pentip_parameters_file_path);
    dt.setDodecaCenter(dodeca_center_parameters_file_path);
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
            
            Mat ptvec = dt.getPenTipPosition();
            cout << ptvec << endl;
            
            cloud.points[pcd_id].x = ptvec.at<float>(0, 0)*100.0f;
            cloud.points[pcd_id].y = ptvec.at<float>(1, 0)*100.0f;
            cloud.points[pcd_id].z = ptvec.at<float>(2, 0)*100.0f;
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
    
    string pcd_path = "Tracking/brio_camera/case1_track.pcd";
    pcl::io::savePCDFileASCII (pcd_path, cloud);
    
    return 0;
}

