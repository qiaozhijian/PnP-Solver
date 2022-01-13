//
// Created by qzj on 2021/12/24.
//

#ifndef CMAKE_TEMPLATE_ICP_H
#define CMAKE_TEMPLATE_ICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include<pcl/common/transforms.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr CloudPtr;
typedef pcl::PointCloud<PointT> Cloud;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class ICP {

public:

    ICP(int _max_iteration);

    Eigen::Matrix4d Registration(CloudPtr cloudA, CloudPtr cloudB);

private:
    int max_iteration;
};


#endif //CMAKE_TEMPLATE_ICP_H
