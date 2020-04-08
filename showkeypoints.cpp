#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2.ply", *point_cloud1);
    cout<<"cloud1 raw points:"<<point_cloud1->points.size()<<endl;
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/4.ply", *point_cloud2);
    cout<<"cloud2 raw points:"<<point_cloud2->points.size()<<endl;

    ifstream keypoints1("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2_kps.txt",ios::in);
    vector<int> indexs;
    int _num = 0;
    while(keypoints1>>_num){
        indexs.push_back(_num);
    }
    cout<<"keypoints num:"<<indexs.size()<<endl;

    boost::shared_ptr<std::vector<int>> indexs_ptr1 = boost::make_shared<std::vector<int>>(indexs);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (point_cloud1);
    extract.setIndices (indexs_ptr1);
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*point_cloud1);


    ifstream keypoints2("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/4_kps.txt",ios::in);
    indexs.clear();
    _num = 0;
    while(keypoints2>>_num){
        indexs.push_back(_num);
    }
    cout<<"keypoints num:"<<indexs.size()<<endl;

    boost::shared_ptr<std::vector<int>> indexs_ptr2 = boost::make_shared<std::vector<int>>(indexs);
    extract.setInputCloud (point_cloud2);
    extract.setIndices (indexs_ptr2);
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*point_cloud2);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(point_cloud1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud2_color_hander(point_cloud2,
                                                                                               230, 20, 20); // Red
    viewer->addPointCloud(point_cloud2, cloud2_color_hander, "point_cloud2");

    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2_k.ply",*point_cloud1);
    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/4_k.ply",*point_cloud2);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}