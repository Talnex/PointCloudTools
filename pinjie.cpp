#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <vector>

using namespace std;

//simpleVis函数实现最基本的点云可视化操作，
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    //创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    //用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
/*******************************************************************************************************
  查看复杂的点云，经常让人感到没有方向感，为了保持正确的坐标判断，需要显示坐标系统方向，可以通过使用X（红色）
  Y（绿色 ）Z （蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小可以通过scale参数来控制，本例中scale设置为1.0

 ******************************************************************************************************/
    viewer->addCoordinateSystem(0.5);
    //通过设置照相机参数使得从默认的角度和方向观察点云
    viewer->initCameraParameters();
    return (viewer);
}

int main(){
    ifstream fin("/Users/talnex/Downloads/zagreb_cathedral/scan000.3d");
    string line;
    vector<string> fields; //声明一个字符串向量
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    int num = 1000000;
    while (getline(fin, line)) {
        if(num--==0)break;
        fields.clear();
        //cout << "原始字符串：" << line << endl; //整行输出
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        string field;
        while (getline(sin, field, ' ')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        pcl::PointXYZ point;

        sin.clear();
        sin.str(fields[0]);
        sin.seekg(0, ios::beg);
        sin >> point.x;

        sin.clear();
        sin.str(fields[1]);
        sin.seekg(0, ios::beg);
        sin >> point.y;

        sin.clear();
        sin.str(fields[2]);
        sin.seekg(0, ios::beg);
        sin >> point.z;

        point_cloud_ptr->points.push_back(point);

    }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    cout<<"读取完毕"<<point_cloud_ptr->points.size()<<"个点"<<endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(point_cloud_ptr);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}
