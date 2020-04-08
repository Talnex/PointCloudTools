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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>

using namespace std;
double offset = 0.5;

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


void deletpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> indexs);

bool comp(const planenum &a, const planenum &b) {
    return a.num > b.num;
}

pcl::visualization::PCLVisualizer::Ptr simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}

int main(int argc, char **argv) {
//    ifstream fin("/Users/talnex/Projects/mysmoothnet/data/demo/cloud_bin_0_keypoints.txt");
//    string line;
//    vector<int> indexs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/ply/4.ply", *raw_point_cloud);

    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(raw_point_cloud);
    filter.setRadiusSearch(0.01f);
    filter.filter(*point_cloud_ptr);

    cout << "downsample point_cloud has :" << point_cloud_ptr->points.size() << " points." << endl;
//    while(fin){
//        int data;
//        fin>>data;
//        indexs.push_back(data);
//    }
//    pcl::copyPointCloud(*raw_point_cloud,indexs,*point_cloud_ptr);

    //点云绘制
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    vector<planenum> res_vec;
    vector<int> deleteindex;
    planenum best = planenum();

    viewer = simpleVis(point_cloud_ptr);

    bool found = false;
    //找5个拟合平面
    for (int i = 0; i < 5; ++i) {
        int size = point_cloud_ptr->points.size();
        cerr << "正在拟合点数目：" << size << endl;
        //每次随机取10000个平面 找到拟合点数目最多的平面
        for (int j = 0; j < 1000; ++j) {
            //取三个随机点
            int r1 = rand() % (size - 0 + 1) + 0;
            int r2 = rand() % (size - 0 + 1) + 0;
            int r3 = rand() % (size - 0 + 1) + 0;
            planenum planenum1 = planenum();
            planenum1.A = (point_cloud_ptr->points[r3].y - point_cloud_ptr->points[r1].y) *
                          (point_cloud_ptr->points[r3].z - point_cloud_ptr->points[r1].z) -
                          (point_cloud_ptr->points[r2].z - point_cloud_ptr->points[r1].z) *
                          (point_cloud_ptr->points[r3].y - point_cloud_ptr->points[r1].y);

            planenum1.B = (point_cloud_ptr->points[r3].x - point_cloud_ptr->points[r1].x) *
                          (point_cloud_ptr->points[r2].z - point_cloud_ptr->points[r1].z) -
                          (point_cloud_ptr->points[r2].x - point_cloud_ptr->points[r1].x) *
                          (point_cloud_ptr->points[r3].z - point_cloud_ptr->points[r1].z);

            planenum1.C = (point_cloud_ptr->points[r2].x - point_cloud_ptr->points[r1].x) *
                          (point_cloud_ptr->points[r3].y - point_cloud_ptr->points[r1].y) -
                          (point_cloud_ptr->points[r3].x - point_cloud_ptr->points[r1].x) *
                          (point_cloud_ptr->points[r2].y - point_cloud_ptr->points[r1].y);

            planenum1.D = -(planenum1.A * point_cloud_ptr->points[r1].x
                            + planenum1.B * point_cloud_ptr->points[r1].y
                            + planenum1.C * point_cloud_ptr->points[r1].z);

            int num = 0;
            for (int k = 0; k < size; ++k) {
                if (countdis(point_cloud_ptr->points[k], planenum1) <= offset) {
                    num++;
                    deleteindex.push_back(k);
                }
            }
            planenum1.num = num;

            if(j%100 == 0)cerr<<"loop: "<<j<<endl;
            if (num > 10000) {
                if (best.num < num) {
                    best = planenum1;
                    found = true;
                }
            }else{
                deleteindex.clear();
            }
        }
        if(found){
            cerr<<best.A<<" "<<best.B<<" "<<best.C<<" "<<best.D<<" "<<best.num<<endl;
            found = false;
            best.num = 0;
            pcl::ModelCoefficients coeffs;
            coeffs.values.resize(4);
            coeffs.values[0] = (best.A);
            coeffs.values[1] = (best.B);
            coeffs.values[2] = (best.C);
            coeffs.values[3] = (best.D);
            viewer->addPlane(coeffs, to_string(best.A));

            res_vec.push_back(best);
            //删除已经拟合的点
            deletpoint(point_cloud_ptr, deleteindex);
        }else{
            break;
        }
    }
    for (int l = 0; l < res_vec.size(); ++l) {
        cerr<<res_vec.at(l).A<<" "<<res_vec.at(l).B<<" "<<res_vec.at(l).C<<" "<<res_vec.at(l).D<<" "<<res_vec.at(l).num<<endl;
    }
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

void deletpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int> indexs) {
    boost::shared_ptr<std::vector<int>> indexs_ptr = boost::make_shared<std::vector<int>>(indexs);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indexs_ptr);
    extract.setNegative (true);//如果设为true,可以提取指定index之外的点云
    extract.filter (*cloud);
}
