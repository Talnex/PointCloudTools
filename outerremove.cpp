#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
        exit(0);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPLYFile("/Volumes/Data/datasets/2.000000.ply",*cloud);
    cout<<"num before filter:"<<cloud->points.size()<<endl;

    if (strcmp(argv[1], "-r") == 0){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // 创建滤波器
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(4);
        outrem.setMinNeighborsInRadius(1000);
        // 应用滤波器
        outrem.filter(*cloud_filtered);
    }
//    else if (strcmp(argv[1], "-c") == 0){
//        // 创建环境
//        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
//        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
//        pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));//此方法只保留z在0.8-2之间的点云
//        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
//        pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
//        // 创建滤波器
//        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
//        condrem.setCondition(range_cond);
//        condrem.setInputCloud(cloud);
//        condrem.setKeepOrganized(true);
//        // 应用滤波器
//        condrem.filter(*cloud_filtered);
//    }
    else if (strcmp(argv[1], "-s") == 0){
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);//50个临近点
        sor.setStddevMulThresh(1.0);//距离大于1倍标准方差

        sor.filter(*cloud_filtered);
    }
    else{
        std::cerr << "please specify command line arg '-r' or '-s'" << std::endl;
        exit(0);
    }

    cout<<"num after filter:"<<cloud_filtered->points.size()<<endl;

    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/final.ply",*cloud_filtered);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud_filtered);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}