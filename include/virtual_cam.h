//
// Created by qzj on 23-4-6.
//

#ifndef CMAKE_TEMPLATE_VIRTUAL_CAM_H
#define CMAKE_TEMPLATE_VIRTUAL_CAM_H

#include "eigen_utils.h"

class VirtualCam {

public:

    VirtualCam(){
        width = 640;
        height = 480;
        fx = 525.0;
        fy = 525.0;
        cx = 319.5;
        cy = 239.5;
        K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        D = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    }

    void RandomSampleForPnP(std::vector<cv::Point3f> &points_3d, std::vector<cv::Point2f> &points_2d,
                            Eigen::Matrix4d &T_wc, int num_points, double outlier_ratio = 0.0){
        points_3d.clear();
        points_2d.clear();
        for (int i = 0; i < num_points; ++i) {
            cv::Point2f p2d;
            p2d.x = random_uniform(0, width);
            p2d.y = random_uniform(0, height);
            cv::Point3f p3d;
            double depth = random_uniform(0.5, 50.0);
            p3d.x = (p2d.x - cx) / fx * depth;
            p3d.y = (p2d.y - cy) / fy * depth;
            p3d.z = depth;

            p3d = transform_point(p3d, T_wc);

            if (outlier_ratio > 0.0 && random_uniform(0.0, 1.0) < outlier_ratio) {
                cv::Point2f old_p2d = p2d;
                while (norm((p2d - old_p2d)) < 50.0) {
                    p2d.x = random_uniform(0, width);
                    p2d.y = random_uniform(0, height);
                }
            }

            // add noise
            p2d.x += random_normal(1.0);
            p2d.y += random_normal(1.0);

            p3d.x += random_normal(0.1);
            p3d.y += random_normal(0.1);
            p3d.z += random_normal(0.1);

            points_2d.push_back(p2d);
            points_3d.push_back(p3d);
        }
    }

    cv::Mat GetK(){
        return K;
    }

    cv::Mat GetD(){
        return D;
    }

private:
    int width, height;
    double fx, fy, cx, cy;
    cv::Mat K, D;

};

#endif //CMAKE_TEMPLATE_VIRTUAL_CAM_H
