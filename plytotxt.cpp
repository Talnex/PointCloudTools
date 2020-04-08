#include <pcl/io/ply_io.h>
#include <iostream>

using namespace std ;

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//点云对象指针
    pcl::PointCloud<pcl::PointXYZ> &point_cloud = *point_cloud_ptr;//引用　上面点云的别名　常亮指针

//    pcl::io::loadPLYFile("/Users/talnex/PycharmProjects/TDSmoothNet/data/demo/cloud_bin_0.ply",point_cloud);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/zagreb_cathedral/2.ply",point_cloud);

    for (int j = 0; j < point_cloud.points.size(); ++j) {
        point_cloud.points.at(j).x = point_cloud.points.at(j).x/100;
        point_cloud.points.at(j).y = point_cloud.points.at(j).y/100;
        point_cloud.points.at(j).z = point_cloud.points.at(j).z/100;
    }

//    double max = 0;
//    double min = 0;
//    for (int j = 0; j < point_cloud.points.size(); ++j) {
//        if(point_cloud.points.at(j).x>max) {
//            max = point_cloud.points.at(j).x;
//        }
//        if(point_cloud.points.at(j).y>max) {
//            max = point_cloud.points.at(j).y;
//        }
//        if(point_cloud.points.at(j).z>max) {
//            max = point_cloud.points.at(j).z;
//        }
//
//        if(point_cloud.points.at(j).x<min) {
//            min = point_cloud.points.at(j).x;
//        }
//        if(point_cloud.points.at(j).y<min) {
//            min = point_cloud.points.at(j).y;
//        }
//        if(point_cloud.points.at(j).z<min) {
//            min = point_cloud.points.at(j).z;
//        }
//    }
//    cout<< max<<endl;
//    cout<< min<<endl;
      pcl::io::savePLYFileBinary("/Volumes/Data/datasets/zagreb_cathedral/21.ply",point_cloud);
//    ofstream file;
//    file.open("/Users/talnex/PycharmProjects/TDSmoothNet/data/demo/0.txt");
//    for (int j = 0; j < point_cloud.points.size(); ++j) {
//        file<<point_cloud.points.at(j).x<<" "<<point_cloud.points.at(j).y<<" "<<point_cloud.points.at(j).z<<endl;
//    }
//    file.close();

    return 0;
}