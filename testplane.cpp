#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <iostream>
#include <math.h>

using namespace std;
double offset = 1;

class planenum {
public:
    double A = 0;
    double B = 0;
    double C = 0;
    double D = 0;
    int num = 0;
};

double countdis(pcl::PointXYZ point, planenum plane) {
    return abs(point.x * plane.A + point.y * plane.B + point.z * plane.C + plane.D) /
           sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C);
}

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

int main(int argc, char **argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/ply/4.ply", *raw_point_cloud);
    cout << "raw points:" << raw_point_cloud->points.size() << endl;

    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(raw_point_cloud);
    filter.setRadiusSearch(0.01f);
    filter.filter(*raw_point_cloud);
    cout << "after filter points:" << raw_point_cloud->points.size() << endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    planenum best = planenum();

    //2.ply
//    best.A = 1.89375;
//    best.B = 2.06288;
//    best.C = 68.0267;
//    best.D = -212.305;

    //4.ply
    best.A = -0.117987;
    best.B = 0.669613;
    best.C = -24.6887;
    best.D = -71.0585;

    std::vector<int> indexs;
    for (int k = 0; k < raw_point_cloud->points.size(); k++) {
        if (countdis(raw_point_cloud->points[k], best) <= offset) {
            indexs.push_back(k);
        }
    }
    cout<<"found:"<<indexs.size()<<endl;

    boost::shared_ptr<std::vector<int>> indexs_ptr = boost::make_shared<std::vector<int>>(indexs);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (raw_point_cloud);
    extract.setIndices (indexs_ptr);
    extract.setNegative (true);//如果设为true,可以提取指定index之外的点云
    extract.filter (*raw_point_cloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // 创建滤波器
    outrem.setInputCloud(raw_point_cloud);
    outrem.setRadiusSearch(2);
    outrem.setMinNeighborsInRadius(500);
    // 应用滤波器
    outrem.filter(*raw_point_cloud);

    cout<<"new point cloud has:"<<raw_point_cloud->points.size()<<endl;

    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/" + to_string(offset) + ".ply", *raw_point_cloud);

    viewer = simpleVis(raw_point_cloud);


    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}