#include <iostream>//标准输入输出流
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>// RangeImage 深度图像
#include <pcl/io/pcd_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>//关键点检测
#include <pcl/console/parse.h>//解析 命令行 参数

//定义别名
typedef pcl::PointXYZ PointType;

float angular_resolution = 0.1f;//角坐标分辨率
float support_size = 0.002f;//感兴趣点的尺寸（球面的直径）
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//坐标框架：相机框架（而不是激光框架）
bool setUnseenToMaxRange = false;//是否将所有不可见的点 看作 最大距离


int main (int argc, char** argv)
{

    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;//将所有不可见的点 看作 最大距离
        cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    int tmp_coordinate_frame;//坐标框架：相机框架（而不是激光框架）
    if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
        cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
    }
    // 感兴趣点的尺寸（球面的直径）
    if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
        cout << "Setting support size to "<<support_size<<".\n";
    // 角坐标分辨率
    if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
        cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
    angular_resolution = pcl::deg2rad (angular_resolution);

    //读取pcd文件；如果没有指定文件，就创建样本点
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);//点云对象指针
    pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;//引用　上面点云的别名　常亮指针
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;//带视角的点云
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());//仿射变换
    //检查参数中是否有pcd格式文件名，返回参数向量中的索引号
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile (filename, point_cloud) == -1)//如果指定了pcd文件，读取pcd文件
        {
            std::cerr << "Was not able to open file \""<<filename<<"\".\n";
            return 0;
        }
        //设置传感器的姿势
        scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                                   point_cloud.sensor_origin_[1],
                                                                   point_cloud.sensor_origin_[2])) *
                            Eigen::Affine3f (point_cloud.sensor_orientation_);
    }
    else//没有指定pcd文件，生成点云，并填充它
    {
        //检查参数中是否有ply格式文件名，返回参数向量中的索引号
        std::vector<int> ply_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "ply");
        if (!ply_filename_indices.empty())
        {
            std::string filename = argv[ply_filename_indices[0]];
            if (pcl::io::loadPLYFile (filename, point_cloud) == -1)//如果指定了ply文件，读取ply文件
            {
                std::cerr << "Was not able to open file \""<<filename<<"\".\n";
                return 0;
            }
            scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                                       point_cloud.sensor_origin_[1],
                                                                       point_cloud.sensor_origin_[2])) *
                                Eigen::Affine3f (point_cloud.sensor_orientation_);
        }else {
            return 1;
        }
    }

    point_cloud.width = (int) point_cloud.points.size();
    point_cloud.height = 1;
    // 从点云数据，创建深度图像
    // 直接把三维的点云投射成二维的图像
    float noise_level = 0.0;
//noise level表示的是容差率，因为1°X1°的空间内很可能不止一个点，
//noise level = 0则表示去最近点的距离作为像素值，如果=0.05则表示在最近点及其后5cm范围内求个平均距离
//minRange表示深度最小值，如果=0则表示取1°X1°的空间内最远点，近的都忽略
    float min_range = 0.0f;
//bordersieze表示图像周边点
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);//创建RangeImage对象（智能指针）
    pcl::RangeImage& range_image = *range_image_ptr; //RangeImage的引用
    //从点云创建深度图像
    range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);//整合远距离点云
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();


    // 3D点云显示
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);//背景颜色　白色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");//添加点云
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    viewer.initCameraParameters ();


    //显示深度图像（平面图）
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

    // 提取NARF关键点
    pcl::RangeImageBorderExtractor range_image_border_extractor;//创建深度图像的边界提取器，用于提取NARF关键点
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);//创建NARF对象
    narf_keypoint_detector.setRangeImage (&range_image);//设置点云对应的深度图
    narf_keypoint_detector.getParameters ().support_size = support_size;// 感兴趣点的尺寸（球面的直径）

    pcl::PointCloud<int> keypoint_indices;//用于存储关键点的索引
    narf_keypoint_detector.compute (keypoint_indices);//计算NARF关键
    std::cout << "Found找到关键点： "<<keypoint_indices.points.size ()<<" key points.\n";


//    ofstream file;
//    file.open("/Users/talnex/Projects/mysmoothnet/data/demo/cloud_bin_0_keypoints.txt");
//    for (int j = 0; j < keypoint_indices.size(); ++j) {
//        file << keypoint_indices.points.at(j) << endl;
//    }
//    file.close();
//    cout<<"created file"<<endl;

    //在3D图形窗口中显示关键点
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);//创建关键点指针
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;//引用
    keypoints.points.resize (keypoint_indices.points.size ());//初始化大小
    for (size_t i=0; i<keypoint_indices.points.size (); ++i)//按照索引获得　关键点
        keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");//添加显示关键点
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();  // process GUI events　　 处理 GUI事件
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }
}