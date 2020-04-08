#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>                  //allows us to use pcl::transformPointCloud function
#include <pcl/visualization/pcl_visualizer.h>


// This is the main function
int main(int argc, char **argv) {

    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Load ply file
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/2_k.ply", *source_cloud);
    pcl::io::loadPLYFile("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/4_k.ply", *target_cloud);

    /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
    */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1(0, 0) = 0.998274;
    transform_1(0, 1) = 0.058718;
    transform_1(0, 2) = -0.000805;
    transform_1(0, 3) = 20.296595;

    transform_1(1, 0) = -0.058718;
    transform_1(1, 1) = 0.998275;
    transform_1(1, 2) = -0.000419;
    transform_1(1, 3) = 18.899187;

    transform_1(2, 0) = 0.000779;
    transform_1(2, 1) = 0.000465;
    transform_1(2, 2) = 1.000000;
    transform_1(2, 3) = -0.456440;

    transform_1(3, 0) = 0.0;
    transform_1(3, 1) = 0.0;
    transform_1(3, 2) = 0.0;
    transform_1(3, 3) = 1.0;

    // Print the transformation
    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;


    Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

    transform_2(0, 0) = 0.984901;
    transform_2(0, 1) = -0.173108;
    transform_2(0, 2) = 0.001876;
    transform_2(0, 3) = 26.900753;

    transform_2(1, 0) = 0.173104;
    transform_2(1, 1) = 0.984901;
    transform_2(1, 2) = 0.002052;
    transform_2(1, 3) = -20.744003;

    transform_2(2, 0) = -0.002203;
    transform_2(2, 1) = -0.001696;
    transform_2(2, 2) = 0.999996;
    transform_2(2, 3) = 0.088777;

    transform_2(3, 0) = 0.0;
    transform_2(3, 1) = 0.0;
    transform_2(3, 2) = 0.0;
    transform_2(3, 3) = 1.0;

    // Print the transformation
    printf("Method #2: using a Matrix4f\n");
    std::cout << transform_2 << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Apply an affine transform defined by an Eigen Transform.
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);

    // Visualization
    printf("\nPoint cloud colors :  white  = source point cloud\n"
           "                        red  = target point cloud\n");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(transformed_cloud, 255, 255,
                                                                                               255);
    // We add the point cloud to the viewer and pass the color handler
    viewer->addPointCloud(transformed_cloud, source_cloud_color_handler, "source_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler(target_cloud,
                                                                                                    230, 20, 20); // Red
    viewer->addPointCloud(target_cloud, target_cloud_color_handler, "target_cloud");

    //viewer.addCoordinateSystem(1.0, 0);  //Adds 3D axes describing a coordinate system to screen at 0,0,0.
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    cout<<"source point num:"<<source_cloud->points.size()<<endl;
    cout<<"target point num:"<<target_cloud->points.size()<<endl;
    *target_cloud = *target_cloud + *source_cloud;
    cout<<"saved point "<<target_cloud->points.size()<<endl;
    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/final/joined_k.ply",*target_cloud);

    while (!viewer->wasStopped()) { // Display the visualiser until 'q' key is pressed
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}