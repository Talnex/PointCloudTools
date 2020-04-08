#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv4/opencv2/opencv.hpp>

#include <string>
#include <vector>

double fx = 579.826;
double fy = 579.826;
double cx = 322.725;
double cy = 256.047;
double s = 1000;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem(0.5);
    viewer->initCameraParameters();
    return (viewer);
}

int main() {
    cv::Mat srcframe = cv::imread("/Users/talnex/Downloads/depth_qian.pgm", -1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < srcframe.rows; ++i) {
        ushort *tempPoint = ((ushort *) (srcframe.data + srcframe.step.p[0] * i));
        for (int j = 0; j < srcframe.cols; ++j) {
            ushort *tempData = tempPoint + j;
            if (*tempData != 0) {
                pcl::PointXYZ point;
                double z = *tempData / s;
                point.x = (i - cx) * z / fx;
                point.y = (j - cy) * z / fy;
                point.z = z;
                point_cloud_ptr->points.push_back(point);
            }
        }
    }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    std::cout << "读取完毕" << point_cloud_ptr->points.size() << "个点" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(point_cloud_ptr);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
