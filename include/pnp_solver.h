//
// Created by qzj on 23-3-24.
//

#ifndef SRC_PnPSolver_H
#define SRC_PnPSolver_H

#include <iostream>
#include "eigen_utils.h"
#include "file_manager.hpp"
#include <opencv2/opencv.hpp>

#define PNP_SOLVER_DLT 0
#define PNP_SOLVER_RANSAC 1
#define PNP_SOLVER_GAUSS_NEWTON 2

class PnPSolver {
private:
    int method_;
    cv::Mat K, D;

public:
    
    PnPSolver(const int method, cv::Mat K, cv::Mat D) : method_(method), K(K), D(D) {
    }

    void drawReprojectedPoints(const std::vector<cv::Point3f> &pts_3, const Eigen::Matrix4d& T, cv::Mat& img) {
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Vector3d t = T.block<3, 1>(0, 3);
        //convert mono8 to rgb
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
        for (int i = 0; i < pts_3.size(); ++i) {
            Eigen::Vector3d pt_3(pts_3[i].x, pts_3[i].y, pts_3[i].z);
            Eigen::Vector3d pt_2 = cv2eigen(K) * (R * pt_3 + t);
            pt_2 /= pt_2[2];
            cv::circle(img, cv::Point2f(pt_2[0], pt_2[1]), 2, cv::Scalar(255, 0, 0), 2);
        }
    }

    //Gaussian-Newton method to solve PnP
    void solvePnPbyGaussianNewton(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2,
                                             Eigen::Matrix3d &R_cw, Eigen::Vector3d &t_cw, std::vector<int> inliers = std::vector<int>()){
        assert(pts_3.size() == pts_2.size());
        std::vector<cv::Point2f> un_pts_2; // normalized points in camera frame
        cv::undistortPoints(pts_2, un_pts_2, K, D);

        if (inliers.empty()) {
            inliers.resize(pts_3.size());
            for (int i = 0; i < pts_3.size(); ++i) {
                inliers[i] = i;
            }
        }

        for (int iter = 0; iter < 10; ++iter) {
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
            for (auto i : inliers) {
                Eigen::Vector3d p_w(pts_3[i].x, pts_3[i].y, pts_3[i].z);
                Eigen::Vector2d p_norm_c(un_pts_2[i].x, un_pts_2[i].y);
                Eigen::Vector3d p_c = R_cw * p_w + t_cw;
                if (abs(p_c(2)) < 1e-2) {
                    p_c(2) = 1e-2;
                }
                Eigen::Vector2d p_norm_c_est(p_c(0) / p_c(2), p_c(1) / p_c(2));
                Eigen::Vector2d e = p_norm_c - p_norm_c_est;
                Eigen::Matrix<double, 2, 3> J_p_norm_c_est;
                J_p_norm_c_est << -1 / p_c(2), 0, p_c(0) / (p_c(2) * p_c(2)),
                        0, -1 / p_c(2), p_c(1) / (p_c(2) * p_c(2));
                Eigen::Matrix<double, 3, 6> J_se3;
                J_se3.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                J_se3.block<3, 3>(0, 3) = -skew(p_c);
                Eigen::Matrix<double, 2, 6> J = J_p_norm_c_est * J_se3;
                H += J.transpose() * J;
                b += -J.transpose() * e;
            }
            Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(b);
            //std::cout << "dx: " << dx.transpose() << std::endl;
            if (dx.norm() < 1e-3 || isnan(dx.norm()) || isinf(dx.norm()) || dx.block<3, 1>(0, 0).norm() > 0.5) {
                break;
            }
            Eigen::AngleAxisd angle_axis(dx.block<3, 1>(3, 0).norm(), dx.block<3, 1>(3, 0).normalized());
            R_cw = angle_axis.toRotationMatrix() * R_cw;
            t_cw += dx.block<3, 1>(0, 0);
            //std::cout << "R_cw: " << std::endl << R_cw << std::endl;
        }
    }

    void solvePnPbyRANSAC(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, 
                          Eigen::Matrix3d &R_cw, Eigen::Vector3d &t_cw, 
                          std::vector<int> &inliers_best) {
        assert(pts_3.size() == pts_2.size());
        std::vector<cv::Point2f> un_pts_2; // normalized points in camera frame
        cv::undistortPoints(pts_2, un_pts_2, K, D);

        int total_pairs = pts_3.size();
        std::vector<int> idx;
        for (int i = 0; i < pts_3.size(); ++i) {
            idx.push_back(i);
        }
        //    randomly select 4 pairs
        int max_inliers = 0;
        for (int iter = 0; iter < 1000; ++iter) {
            random_shuffle(idx.begin(), idx.end());
            std::vector<cv::Point3f> pts_3_sample;
            std::vector<cv::Point2f> pts_2_sample;
            for (int i = 0; i < 6; ++i) {
                pts_3_sample.push_back(pts_3[idx[i]]);
                pts_2_sample.push_back(pts_2[idx[i]]);
            }
            solvePnPbyDLT(pts_3_sample, pts_2_sample, R_cw, t_cw);

            std::vector<double> errors;
            std::vector<int> inliers_idx;
            for (int i = 0; i < total_pairs; ++i) {
                Eigen::Vector3d p_w(pts_3[i].x, pts_3[i].y, pts_3[i].z);
                Eigen::Vector3d p_c = R_cw * p_w + t_cw;
                Eigen::Vector2d p_norm_c_est(p_c(0) / p_c(2), p_c(1) / p_c(2));
                Eigen::Vector2d p_norm_c(un_pts_2[i].x, un_pts_2[i].y);
                double error = (p_norm_c - p_norm_c_est).norm() * K.at<double>(0, 0);
                if (error < 5.0) {
                    inliers_idx.push_back(i);
                }
            }
            if (inliers_idx.size() > max_inliers) {
                max_inliers = inliers_idx.size();
                inliers_best = inliers_idx;
            }
            if (max_inliers > total_pairs * 0.5) {
                break;
            }
        }
    //    use all inliers to re-estimate the pose
        std::vector<cv::Point3f> pts_3_inliers;
        std::vector<cv::Point2f> pts_2_inliers;
        for (int i = 0; i < inliers_best.size(); ++i) {
            pts_3_inliers.push_back(pts_3[inliers_best[i]]);
            pts_2_inliers.push_back(pts_2[inliers_best[i]]);
        }
        solvePnPbyDLT(pts_3_inliers, pts_2_inliers, R_cw, t_cw);
    }

    void solvePnPbyDLT(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, Eigen::Matrix3d &R_cw, Eigen::Vector3d &t_cw)
    {
        assert(pts_3.size() == pts_2.size());
        std::vector<cv::Point2f> un_pts_2; // normalized points in camera frame
        cv::undistortPoints(pts_2, un_pts_2, K, D);
        //DLT method to solve PnP problem. Note that if all points are on a plane (e.g. calibration board), the solution is not unique.
        Eigen::MatrixXd A(un_pts_2.size() * 2, 12);
        for (int i = 0; i < un_pts_2.size(); i++) {
            A.row(i * 2) << pts_3[i].x, pts_3[i].y, pts_3[i].z, 1, 0, 0, 0, 0, -un_pts_2[i].x * pts_3[i].x, -un_pts_2[i].x * pts_3[i].y, -un_pts_2[i].x * pts_3[i].z, -un_pts_2[i].x;
            A.row(i * 2 + 1) << 0, 0, 0, 0, pts_3[i].x, pts_3[i].y, pts_3[i].z, 1, -un_pts_2[i].y * pts_3[i].x, -un_pts_2[i].y * pts_3[i].y, -un_pts_2[i].y * pts_3[i].z, -un_pts_2[i].y;
        }
        // compute the rank of A
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::VectorXd h = svd.matrixV().col(11);
        Eigen::Matrix<double, 3, 4> H_hat;
        H_hat << h(0), h(1), h(2), h(3),
                h(4), h(5), h(6), h(7),
                h(8), h(9), h(10), h(11);
        double s = H_hat(2, 0) * pts_3[0].x + H_hat(2, 1) * pts_3[0].y + H_hat(2, 2) * pts_3[0].z + H_hat(2, 3);
        if (s < 0) { // make sure the depth is positive
            H_hat = -H_hat;
        }
        // if this value is too big, the solution is not reliable (since the residual equals to the smallest singular value)
        double smallest_singular_value = svd.singularValues()(svd.singularValues().size() - 1);
        //std::cout << "smallest_singular_value: " << smallest_singular_value << std::endl;
        Eigen::Matrix3d R_hat = H_hat.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(R_hat, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd_R.matrixU();
        Eigen::Matrix3d V = svd_R.matrixV();
        R_cw = U * V.transpose();
        if (R_cw.determinant() < 0) { // make sure the rotation matrix is right-handed
            V.col(2) = -V.col(2);
            R_cw = U * V.transpose();
        }
        t_cw = H_hat.col(3) / H_hat.col(0).norm();
    }

    // test function, can be used to verify your estimation
    void calculateReprojectionError(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        std::vector<cv::Point2f> un_pts_2;
        cv::undistortPoints(pts_2, un_pts_2, K, D);
        double err_mean = 0.0;
        double err_max = 0.0;
        for (unsigned int i = 0; i < pts_3.size(); i++)
        {
            Eigen::Vector3d p_mat(pts_3[i].x, pts_3[i].y, pts_3[i].z);
            Eigen::Vector3d p = (R * p_mat + t);
            double err = sqrt((p(0) / p(2) - un_pts_2[i].x) * (p(0) / p(2) - un_pts_2[i].x) +
                              (p(1) / p(2) - un_pts_2[i].y) * (p(1) / p(2) - un_pts_2[i].y));
            printf("(%f, %f, %f) -> (%f, %f) and (%f, %f) -> error: %f\n",
                   pts_3[i].x, pts_3[i].y, pts_3[i].z,
                   un_pts_2[i].x, un_pts_2[i].y,
                   p(0) / p(2), p(1) / p(2), err);
            err_mean += err;
            if (err > err_max)
                err_max = err;
        }
        err_mean = err_mean / pts_3.size();
        //if (err_mean > 0.05)
        //    printf("mean error: %f, max error: %f\n", err_mean, err_max);
    }
};

namespace custom{

    inline void solvePnP(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2,
                  const cv::Mat &K, const cv::Mat &D,
                  cv::Mat &rvec, cv::Mat &t_est, int method_=PNP_SOLVER_DLT){
        PnPSolver solver(method_, K, D);
        Eigen::Matrix3d R_cw;
        Eigen::Vector3d t_cw;
        if (method_==PNP_SOLVER_DLT){
            solver.solvePnPbyDLT(pts_3, pts_2, R_cw, t_cw);
        } else if (method_==PNP_SOLVER_RANSAC){
            std::vector<int> inliers_best;
            solver.solvePnPbyRANSAC(pts_3, pts_2, R_cw, t_cw, inliers_best);
        } else if (method_==PNP_SOLVER_GAUSS_NEWTON){
            std::vector<int> inliers_best;
            solver.solvePnPbyRANSAC(pts_3, pts_2, R_cw, t_cw, inliers_best);
            solver.solvePnPbyGaussianNewton(pts_3, pts_2, R_cw, t_cw, inliers_best);
        } else {
            std::cerr << "method not supported!" << std::endl;
        }

        Eigen::Vector3d rvec_ = RotationVector(R_cw);
        rvec = (cv::Mat_<double>(3, 1) << rvec_(0), rvec_(1), rvec_(2));
        t_est = (cv::Mat_<double>(3, 1) << t_cw(0), t_cw(1), t_cw(2));
    }
}

#endif //SRC_PnPSolver_H
