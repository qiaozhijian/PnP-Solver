//
// Created by qzj on 2021/12/24.
//

#include "icp.h"
#include "glog/logging.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>

using namespace std;

ICP::ICP(int _max_iteration) {
    max_iteration = _max_iteration;
//    一些系统初始化操作，比如初始化变量，新建线程
    LOG(INFO) << "ICP init!";
}

Eigen::Matrix4d ICP::Registration(CloudPtr cloudA, CloudPtr cloudB) {

//    专门用来最近邻查询的点云
    CloudPtr cloudA_copy(new Cloud());
    pcl::copyPointCloud(*cloudA, *cloudA_copy);
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloudB);

    Eigen::Vector3d source, target, xyz_trans;
    PointT searchPoint, resultPoint, sourcePoint;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    Sophus::SE3d SE3_x = Sophus::SE3d::exp(Vector6d::Zero());
    Eigen::Matrix<double, 3, 6> _jacobian;
    Eigen::Matrix<double, 6, 6> information;
    Vector6d residual, _residual, x_plus;

    for (int iter = 0; iter < max_iteration; iter++) {
        information = Eigen::Matrix<double, 6, 6>::Zero();
        residual = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i = 0; i < cloudA->size(); i++) {
            searchPoint = cloudA_copy->points[i];
            sourcePoint = cloudA->points[i];
            kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
            resultPoint = cloudB->points[pointIdxNKNSearch[0]];

            source << sourcePoint.x, sourcePoint.y, sourcePoint.z;
            target << resultPoint.x, resultPoint.y, resultPoint.z;
            xyz_trans = SE3_x * source;

            _jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
            _jacobian.block<3, 3>(0, 3) = -Sophus::SO3d::hat(xyz_trans);

            information = information + _jacobian.transpose() * _jacobian;
            residual = residual - _jacobian.transpose() * (xyz_trans - target);
        }
//        求解增量
        x_plus = information.inverse() * residual;
        x_plus = information.ldlt().solve(residual);
//        估计量更新
        SE3_x = Sophus::SE3d::exp(x_plus) * SE3_x;
//        点云更新，方便查询
        pcl::transformPointCloud(*cloudA_copy, *cloudA_copy, Sophus::SE3d::exp(x_plus).matrix());

//        LOG(INFO) << "information: \n" << information;
        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(information);
//        判断退化
//        LOG(INFO) << "information logdet: \n" << log(information.determinant());
//        LOG(INFO) << "information values = \n" << es.eigenvalues().real().transpose();
//        列是特征向量
//        LOG(INFO) << "information values = \n" << es.eigenvectors().real();

        if(x_plus.norm() < 1e-2){
//            LOG(INFO)<<"finished earlier."<<endl;
            break;
        }
//        LOG(INFO) << "residual: \n" << residual;
//        LOG(INFO) << "SE3_x: \n" << SE3_x.matrix();
    }
    LOG(INFO) << "information logdet: \n" << log(information.determinant());
    Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> es(information);
    LOG(INFO) << "information min eigenvalue: \n" << es.eigenvalues().real().minCoeff();
    return SE3_x.matrix();
}
