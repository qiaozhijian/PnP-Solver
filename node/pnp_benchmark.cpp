#include <iostream>
#include <string>
#include "file_manager.hpp"
#include <eigen3/Eigen/Dense>
#include "global_defination/global_defination.h"
#include "pnp_solver.h"
#include "eigen_utils.h"
#include "virtual_cam.h"
#include "algorithm"
using namespace std;

void PnPExp(std::string method, std::vector<std::vector<cv::Point3f>>& points_3d_all,
            std::vector<std::vector<cv::Point2f>>& points_2d_all,
            std::vector<Eigen::Matrix4d>& T_wc_gts);
void Dataset(std::vector<std::vector<cv::Point3f>>& points_3d_all,
             std::vector<std::vector<cv::Point2f>>& points_2d_all,
             std::vector<Eigen::Matrix4d>& T_wc_gts,
             double outlier_ratio);

///////主函数
int main(int argc, char **argv) {
    std::vector<std::vector<cv::Point3f>> points_3d_all;
    std::vector<std::vector<cv::Point2f>> points_2d_all;
    std::vector<Eigen::Matrix4d> T_wc_gts;

    std::cout << "PnP Benchmark with " << 0.0 << " outlier ratio" << std::endl;
    Dataset(points_3d_all, points_2d_all, T_wc_gts, 0.0);
    PnPExp("OPENCV", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("OPENCV_RANSAC", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("DLT", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("DLT_RANSAC", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("GAUSS_NEWTON", points_3d_all, points_2d_all, T_wc_gts);
    std::cout << std::endl;

    std::cout << "PnP Benchmark with " << 0.2 << " outlier ratio" << std::endl;
    Dataset(points_3d_all, points_2d_all, T_wc_gts, 0.2);
    PnPExp("OPENCV", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("OPENCV_RANSAC", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("DLT", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("DLT_RANSAC", points_3d_all, points_2d_all, T_wc_gts);
    PnPExp("GAUSS_NEWTON", points_3d_all, points_2d_all, T_wc_gts);
    std::cout << std::endl;

    return 0;
}

// generate dataset
void Dataset(std::vector<std::vector<cv::Point3f>>& points_3d_all,
             std::vector<std::vector<cv::Point2f>>& points_2d_all,
             std::vector<Eigen::Matrix4d>& T_wc_gts,
             double outlier_ratio){
    VirtualCam cam;
    points_2d_all.clear();
    points_3d_all.clear();
    T_wc_gts.clear();
    for (int i = 0; i < 100; ++i) {
        // generate random transformation
        Eigen::Matrix4d T_wc_gt = random_transformation_matrix(5);
        std::vector<cv::Point3f> points_3d;
        std::vector<cv::Point2f> points_2d;
        // generate random 2d points and corresponding 3d points
        cam.RandomSampleForPnP(points_3d, points_2d, T_wc_gt, 100, outlier_ratio);
        points_3d_all.push_back(points_3d);
        points_2d_all.push_back(points_2d);
        T_wc_gts.push_back(T_wc_gt);
    }
}

void PnPExp(std::string method, std::vector<std::vector<cv::Point3f>>& points_3d_all,
            std::vector<std::vector<cv::Point2f>>& points_2d_all,
            std::vector<Eigen::Matrix4d>& T_wc_gts){

    VirtualCam cam;
    std::vector<double> rot_errs;
    std::vector<double> trans_errs;
    for (int monte_carlo = 0; monte_carlo < points_2d_all.size(); ++monte_carlo) {
        std::vector<cv::Point3f>& points_3d = points_3d_all[monte_carlo];
        std::vector<cv::Point2f>& points_2d = points_2d_all[monte_carlo];
        Eigen::Matrix4d T_wc_gt = T_wc_gts[monte_carlo];

        cv::Mat rvec, tvec;
        if (method == "OPENCV"){
            cv::solvePnP(points_3d, points_2d, cam.GetK(), cam.GetD(), rvec, tvec);
        }
        else if (method == "OPENCV_RANSAC"){
            cv::solvePnPRansac(points_3d, points_2d, cam.GetK(), cam.GetD(), rvec, tvec);
        }
        else if (method == "DLT"){
            custom::solvePnP(points_3d, points_2d, cam.GetK(), cam.GetD(), rvec, tvec, PNP_SOLVER_DLT);
        }
        else if (method == "DLT_RANSAC"){
            custom::solvePnP(points_3d, points_2d, cam.GetK(), cam.GetD(), rvec, tvec, PNP_SOLVER_RANSAC);
        }
        else if (method == "GAUSS_NEWTON"){
            custom::solvePnP(points_3d, points_2d, cam.GetK(), cam.GetD(), rvec, tvec, PNP_SOLVER_GAUSS_NEWTON);
        }
        else{
            std::cerr << method << " is not supported." << std::endl;
        }

        cv::Mat R_cw_est;
        cv::Rodrigues(rvec, R_cw_est);
        Eigen::Matrix3d R_cw_est_eigen = cv2eigen(R_cw_est);

        Eigen::Matrix3d R_cw_gt = T_wc_gt.block<3, 3>(0, 0).transpose();
        Eigen::Vector3d t_cw_gt = -R_cw_gt * T_wc_gt.block<3, 1>(0, 3);

        double rot_err = RotationDistance(R_cw_est_eigen, R_cw_gt);
        Eigen::Vector3d t_cw_est;
        t_cw_est << tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0);
        double trans_err = (t_cw_gt - t_cw_est).norm();

        rot_errs.push_back(rot_err);
        trans_errs.push_back(trans_err);
    }

    double rot_err_avg = std::accumulate(rot_errs.begin(), rot_errs.end(), 0.0) / rot_errs.size();
    double trans_err_avg = std::accumulate(trans_errs.begin(), trans_errs.end(), 0.0) / trans_errs.size();
    std::cout << "Method: " << method << ", rotation error " << rot_err_avg << " deg, " << "translation error " << trans_err_avg << std::endl;
}
