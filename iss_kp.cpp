#include <iostream>//标准输入输出流
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/iss_3d.h>
#include<time.h>

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2.ply", *model);

    cout<<"raw points num:"<<model->points.size()<<endl;
    double model_resolution = 0.06;
    //4869 0.08
    //9527 0.06
    clock_t startTime,endTime;
    startTime = clock();

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(6 * model_resolution);
    iss_detector.setNonMaxRadius(4 * model_resolution);

    iss_detector.setNormalRadius (4 * model_resolution);
    iss_detector.setBorderRadius (4 * model_resolution);

    iss_detector.setThreshold21(0.975);
    iss_detector.setThreshold32(0.975);
    iss_detector.setMinNeighbors(5);
    iss_detector.setNumberOfThreads(8);
    iss_detector.setInputCloud(model);
    iss_detector.compute(*model_keypoints);
    boost::shared_ptr<const pcl::PointIndices> keypoints = iss_detector.getKeypointsIndices();
    ofstream fout;
    fout.open("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2_kps.txt");
    for (int j = 0; j < keypoints->indices.size(); ++j) {
        fout<<keypoints->indices.at(j)<<endl;
    }
    cout << model_keypoints->points.size() << endl;

    endTime = clock();
    cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
    viewer->addPointCloud(model, "raw");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> iss_color_handler(model_keypoints, 0, 255,
                                                                                      0);//第一个参数类型为　指针
    viewer->addPointCloud(model_keypoints, iss_color_handler, "iss");//第一个参数类型为　指针
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "iss");

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        pcl_sleep(0.1);
    }
}
