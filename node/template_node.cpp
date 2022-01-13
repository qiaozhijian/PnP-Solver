#include <iostream>
#include <string>
#include "glog/logging.h"
#include "file_manager.hpp"
#include <Eigen/Core>
#include "global_defination/global_defination.h"
#include "icp.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
void CreatePlanePCL(CloudPtr cloud, double height);
void CreateCylinderPCL(CloudPtr cloud, double replace);
void CreateRabbitPCL(CloudPtr cloud, double replace);
void visualize(CloudPtr cloudA, CloudPtr cloudB, Eigen::Matrix4d transformation);
///////主函数
int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = cmake_template::WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = false;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);

    ICP icp(100);
    CloudPtr cloudA(new Cloud());
    CloudPtr cloudB(new Cloud());
    Eigen::Matrix4d transformation;

    CreatePlanePCL(cloudA, 0.0);
    CreatePlanePCL(cloudB, 1.0);
    LOG(INFO) << "register plane.";
    transformation = icp.Registration(cloudA, cloudB);
    visualize(cloudA, cloudB, transformation);

    CreateCylinderPCL(cloudA, 0.0);
    CreateCylinderPCL(cloudB, 1.0);
    LOG(INFO) << "register cylinder.";

    transformation = icp.Registration(cloudA, cloudB);
    visualize(cloudA, cloudB, transformation);

    CreateRabbitPCL(cloudA, 0.0);
    CreateRabbitPCL(cloudB, 0.1);
    LOG(INFO) << "register rabbit.";

    transformation = icp.Registration(cloudA, cloudB);
    visualize(cloudA, cloudB, transformation);

    return 0;
}

void CreatePlanePCL(CloudPtr cloud, double height){
    cloud->clear();
    cloud->resize(1000);
    cloud->width = 1000;
    cloud->height = 1;
    for (int i = 0; i < cloud->points.size (); ++i){
        cloud->points[i].x = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 10 * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = height;
//        cloud->points[i].z = 10 * rand() / (RAND_MAX + 1.0f);
//        cout<< cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    }
}

void CreateCylinderPCL(CloudPtr cloud, double replace){
    cloud->clear();
    cloud->resize(1000);
    cloud->width = 1000;
    cloud->height = 1;
    for (int i = 0; i < cloud->points.size (); ++i){
        double theta = 3.14159259 * 2.0 * rand() / (RAND_MAX + 1.0f);
        double r = 1.0;
        cloud->points[i].x = r * sin(theta) + replace;
        cloud->points[i].y = r * cos(theta) + replace;
        cloud->points[i].z = rand() / (RAND_MAX + 1.0f) + replace;
    }
}

void CreateRabbitPCL(CloudPtr cloud, double replace){
    cloud->clear();
    pcl::io::loadPCDFile<pcl::PointXYZ>(cmake_template::WORK_SPACE_PATH + "/rabbit/rabbit.pcd", *cloud);
    for (int i = 0; i < cloud->points.size (); ++i){
        cloud->points[i].x = cloud->points[i].x + replace;
        cloud->points[i].y = cloud->points[i].y + replace;
        cloud->points[i].z = cloud->points[i].z + replace;
    }
}

void visualize(CloudPtr cloudA, CloudPtr cloudB, Eigen::Matrix4d transformation){

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0);
    int v2(1);
    viewer->createViewPort(0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (255, 255, 255, v1);
    viewer->setBackgroundColor (255, 255, 255, v2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> fildColorA(cloudA, 255, 180, 0);
    viewer->addPointCloud<PointT>(cloudA, fildColorA, "A", v1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> fildColorB(cloudA, 0, 166, 237);
    viewer->addPointCloud<PointT>(cloudB, fildColorB, "B", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A"); // 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "B"); // 设置点云大小
    ;
    pcl::transformPointCloud(*cloudA, *cloudA, transformation);
    viewer->addPointCloud<PointT>(cloudA, fildColorA, "A_", v2);
    viewer->addPointCloud<PointT>(cloudB, fildColorB, "B_", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "A_"); // 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "B_"); // 设置点云大小

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}
