#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    //创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    //设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
    viewer->setBackgroundColor(0, 0, 0);
    /*这是最重要的一行，我们将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能
      标志引用该点云，多次调用addPointCloud可以实现多个点云的添加，，每调用一次就会创建一个新的ID号，如果想更新一个
      已经显示的点云，必须先调用removePointCloud（），并提供需要更新的点云ID 号，
     *******************************************************************************************/
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

int main (int argc, char ** argv)
{
    //filename1 = "/Users/talnex/Downloads/datasets/room_scan1.pcd";
    //filename2 = "/Users/talnex/Downloads/datasets/room_scan2.pcd";
    cout << "Loading clouds...\n";

    // open the clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPLYFile("/Users/talnex/Downloads/cloud_bin_0.ply", *cloud1);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud1);

    pcl::io::savePLYFileBinary("res.ply",*cloud1);

  return (0);
}