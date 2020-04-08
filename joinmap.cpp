#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main() {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPLYFile("/Volumes/Data/datasets/zagreb_cathedral/cloud_bin_0.ply", *raw_point_cloud);
//    cout<<"raw points:"<<raw_point_cloud->points.size()<<endl;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//
//    pcl::UniformSampling<pcl::PointXYZ> filter;
//    filter.setInputCloud(raw_point_cloud);
//    filter.setRadiusSearch(1.0f);
//    filter.filter(*point_cloud);
//    cout << "downsample point_cloud has :" << point_cloud->points.size() << " points." << endl;
//
//    viewer = simpleVis(raw_point_cloud);

//    while (!viewer->wasStopped()) {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
//    vector<Eigen::Vector3d> angles;
//    vector<Eigen::Vector3d> trans;
//    for (int j = 0; j < 8; ++j) {
//        ifstream fin("/Volumes/Data/datasets/zagreb_cathedral/pose/scan00" + to_string(j) + ".pose");
//
//        string line;
//        vector<string> fields; //声明一个字符串向量
//
//        Eigen::Vector3d angle;//欧拉角
//        Eigen::Vector3d tran;//位移
//
//        for (int k = 0; k < 3; ++k) {
//            fin>>tran[k];
//        }
//
//        for (int k = 0; k < 3; ++k) {
//            fin>>angle[k];
//        }
//
//        angles.push_back(angle);
//        trans.push_back(tran);
//
//        fin.close();
//    }
//
//    for (int k = 0; k < angles.size(); ++k) {
//        cout<<angles.at(k)<<endl;
//        cout<<trans.at(k)<<endl;
//        cout<<endl;
//    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudadd(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::UniformSampling<pcl::PointXYZRGB> filter;

    for (int j = 1; j < 8; ++j) {
        if(j==7||j == 1||j==3 ||j==4||j==5){
            continue;
        }
        pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/2-AlignedPointCloud/ply/"+to_string(j)+".ply",*cloud);
        filter.setInputCloud(cloud);
        filter.setRadiusSearch(0.1f);
        filter.filter(*cloud);

        int r = rand()%255;
        int g = rand()%255;
        int b = rand()%255;
//        switch (j){
//            case 1:{
//                r = 100;
//                g = 0;
//                b = 0;
//                break;
//            }
//            case 2:{
//                r = 255;
//                g = 0;
//                b = 0;
//                break;
//            }
//            case 3:{
//                r = 0;
//                g = 100;
//                b = 0;
//                break;
//            }
//            case 4:{
//                r = 0;
//                g = 255;
//                b = 0;
//                break;
//            }
//            case 5:{
//                r = 0;
//                g = 0;
//                b = 100;
//                break;
//            }
//            case 6:{
//                r = 0;
//                g = 0;
//                b = 255;
//                break;
//            }
//            case 7:{
//                r = 255;
//                g = 255;
//                b = 255;
//                break;
//            }
//        }

        for (int k = 0; k < cloud->points.size(); ++k) {
            cloud->points.at(k).r = r;
            cloud->points.at(k).g = g;
            cloud->points.at(k).b = b;
        }
        *cloudadd = *cloudadd + *cloud;
        cloud->clear();

    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudadd);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudadd, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}