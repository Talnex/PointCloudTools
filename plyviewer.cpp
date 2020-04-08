#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>

using namespace std;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/ply/2.ply", *raw_point_cloud);
    cout<<"raw points:"<<raw_point_cloud->points.size()<<endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(raw_point_cloud);
    filter.setRadiusSearch(0.01f);
    filter.filter(*point_cloud);
    cout << "downsample point_cloud has :" << point_cloud->points.size() << " points." << endl;

    viewer = simpleVis(point_cloud);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}