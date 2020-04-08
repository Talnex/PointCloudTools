#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>


using namespace std;

bool Plane(false), Sphere(false);
double offset = 0.1;

class planenum {
public:
    double A = 0;
    double B = 0;
    double C = 0;
    double D = 0;
    int num = 0;
};

void deletpoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sharedPtr, planenum planenum);

pcl::visualization::PCLVisualizer::Ptr
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    return (viewer);
}


void
printUsage(const char *progName) {
    std::cout << "\n\nUsage: " << progName << " [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-h           this help\n"
              << "-r           RGBD colour visualisation \n"
              << "-f           filter visualisation \n"
              << "-m           matching visualisation \n"
              << "\n\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbdVis
        (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr colorpoint(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ralpoint(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j < cloud->points.size(); ++j) {
        pcl::PointXYZ color;
        color.z = cloud->points[j].rgb;
        colorpoint->points.push_back(color);

        pcl::PointXYZ point;
        point.x = cloud->points[j].x;
        point.y = cloud->points[j].y;
        point.z = cloud->points[j].z;
        ralpoint->points.push_back(point);
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(colorpoint, "z"); // 按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZ>(ralpoint, fildColor, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                             "sample cloud1"); // 设置点云大小
    return (viewer);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr fltcloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    //滤波处理
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(300);
    sor.setStddevMulThresh(0.4);
    sor.filter(*cloud);
    return cloud;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> fltVis
        (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr orignal_cloud) {

    //复制一份原来的点云用来滤波
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*orignal_cloud, *filted_cloud);
    //滤波处理
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(filted_cloud);
    sor.setMeanK(300);
    sor.setStddevMulThresh(0.4);
    sor.filter(*filted_cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();
    int v1(0);  //创建新的视口
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("orignal points:" + to_string(orignal_cloud->points.size()), 10, 10, "v1 text", v1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr colorpoint(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ralpoint(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j < orignal_cloud->points.size(); ++j) {
        pcl::PointXYZ color;
        color.z = orignal_cloud->points[j].rgb;
        colorpoint->points.push_back(color);

        pcl::PointXYZ point;
        point.x = orignal_cloud->points[j].x;
        point.y = orignal_cloud->points[j].y;
        point.z = orignal_cloud->points[j].z;
        ralpoint->points.push_back(point);
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(colorpoint, "z"); // 按照z字段进行渲染
    viewer->addPointCloud<pcl::PointXYZ>(ralpoint, fildColor, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("filted points:" + to_string(filted_cloud->points.size()), 10, 10, "v2 text", v2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr colorpoint1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ralpoint1(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j < filted_cloud->points.size(); ++j) {
        pcl::PointXYZ color;
        color.z = filted_cloud->points[j].rgb;
        colorpoint1->points.push_back(color);

        pcl::PointXYZ point;
        point.x = filted_cloud->points[j].x;
        point.y = filted_cloud->points[j].y;
        point.z = filted_cloud->points[j].z;
        ralpoint1->points.push_back(point);
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor1(colorpoint1, "z"); // 按照z字段进行渲染
    viewer->addPointCloud<pcl::PointXYZ>(ralpoint1, fildColor1, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");

    return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> matchVis
        (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(30);
    n.compute(
            *normals); //* normals should not contain the point normals + surface curvatures // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals,
                           *cloud_with_normals); //* cloud_with_normals = cloud + normals // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles; // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025); // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false); // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles); // Additional vertex information
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(triangles, "triangles");
    viewer->addPolylineFromPolygonMesh(triangles);
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    /*
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    if(Plane){
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            exit(-1);
        }
        cerr<<inliers->indices.size()<<endl;
    }

*/
/*
    std::vector<int> inliers;
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
            model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    if(Plane)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }
    else if (Sphere)
    {
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*cloud, inliers, *final);
    viewer = simpleVis(final);
 */
    return (viewer);
}

double countdis(pcl::PointXYZRGB point, planenum plane) {
    return abs(point.x * plane.A + point.y * plane.B + point.z * plane.C + plane.D) /
           sqrt(plane.A * plane.A + plane.B * plane.B + plane.C * plane.C);
}

bool comp(const planenum &a, const planenum &b) {
    return a.num > b.num;
}

int main(int argc, char **argv) {

    bool rgbd(false), filter(false), matching(false);
    if (pcl::console::find_argument(argc, argv, "-r") >= 0) {
        rgbd = true;
        std::cout << "RGBD visualisation\n";
    } else if (pcl::console::find_argument(argc, argv, "-f") >= 0) {
        filter = true;
        std::cout << "filter visualisation\n";
    } else if (pcl::console::find_argument(argc, argv, "-m") >= 0) {
        matching = true;
        std::cout << "matching visualisation\n";
        if (pcl::console::find_argument(argc, argv, "-s") >= 0) {
            Sphere = true;
        } else if (pcl::console::find_argument(argc, argv, "-p") >= 0) {
            Plane = true;
        } else {
            cout << "use -s ModelSphere\n"
                    "    -p ModelPlane\n" << endl;
            return 0;
        }

    } else {
        //printUsage(argv[0]);
        filter = true;
        //return 0;
    }

    ifstream fin("all_point.csv");
    string line;
    vector<string> fields; //声明一个字符串向量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    getline(fin, line);//第一行去掉

    while (getline(fin, line)) {
        fields.clear();
        //cout << "原始字符串：" << line << endl; //整行输出
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        pcl::PointXYZRGB point;

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

        sin.clear();
        sin.str(fields[3]);
        sin.seekg(0, ios::beg);
        sin >> point.rgb;

        point_cloud_ptr->points.push_back(point);

    }

    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    //点云绘制
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (rgbd) {
        viewer = rgbdVis(point_cloud_ptr);
    } else if (filter) {
        //viewer = fltVis(point_cloud_ptr);
    } else if (matching) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr origal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*point_cloud_ptr, *origal_cloud);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(origal_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.7);
        //sor.setKeepOrganized(true);
        sor.filter(*origal_cloud);
        viewer = matchVis(origal_cloud);

    }

    //滤波
    point_cloud_ptr = fltcloud(point_cloud_ptr);
    vector<planenum> res_vec;
    planenum best = planenum();

    viewer = rgbdVis(point_cloud_ptr);

    bool found = false;
    //找5个拟合平面
    for (int i = 0; i < 20; ++i) {
        int size = point_cloud_ptr->points.size();
        cerr << "正在拟合点数目：" << size << endl;
        //每次随机取10000个平面 找到拟合点数目最多的平面
        for (int j = 0; j < 5000; ++j) {
            //取三个随机点
            int r1 = rand() % (size - 0 + 1) + 0;
            int r2 = rand() % (size - 0 + 1) + 0;
            int r3 = rand() % (size - 0 + 1) + 0;
            planenum planenum1 = planenum();
            /*
             * A = (y3 - y1)*(z3 - z1) - (z2 -z1)*(y3 - y1);
               B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
               C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
               D = -(A * x1 + B * y1 + C * z1)
             */
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
                if (countdis(point_cloud_ptr->points[k], planenum1) <= offset) num++;
            }
            planenum1.num = num;

            if (num > 100) {
                if (best.num < num) {
                    best = planenum1;
                    found = true;
                }
            }

        }
        if(found){
            cerr<<best.num<<endl;
            found = false;
            best.num = 0;
            pcl::ModelCoefficients coeffs;
            coeffs.values.resize(4);
            coeffs.values[0] = (best.A);
            coeffs.values[1] = (best.B);
            coeffs.values[2] = (best.C);
            coeffs.values[3] = (best.D);
            viewer->addPlane(coeffs, to_string(best.A));

            //删除已经拟合的点
            deletpoint(point_cloud_ptr, best);
        }else{
            break;
        }

    }
//            if (res_vec.size() == 0) res_vec.push_back(planenum1);
//            else {
//                int min_no = 0;
//                double min_error = 10;
//                for (int j = 0; j < res_vec.size(); ++j) {
//                    double n1 = res_vec[j].A / planenum1.A;
//                    double n2 = res_vec[j].B / planenum1.B;
//                    double n3 = res_vec[j].C / planenum1.C;
//                    double n4 = res_vec[j].D / planenum1.D;
//                    double mean = (n1 + n2 + n3 + n4) / 4;
//                    double error = sqrt((pow(n1 - mean, 2) + pow(n2 - mean, 2)
//                                         + pow(n3 - mean, 2))/ 2);
//
//                    if (error < min_error) {
//                        min_error = error;
//                        min_no = j;
//                    }
//                }
//
//                if (min_error > error_offset) {
//                    res_vec.push_back(planenum1);
////                    cerr << "add: A:" << planenum1.A << " B:" << planenum1.B << " C:" << planenum1.C << " D:"
////                         << planenum1.D
////                         << " num:" << planenum1.num << endl;
//                } else if (planenum1.num > res_vec[min_no].num) {
//                    res_vec[min_no].A = planenum1.A;
//                    res_vec[min_no].B = planenum1.B;
//                    res_vec[min_no].C = planenum1.C;
//                    res_vec[min_no].D = planenum1.D;
//                    res_vec[min_no].num = planenum1.num;
////                    cerr << "update: A:" << planenum1.A << " B:" << planenum1.B << " C:" << planenum1.C << " D:"
////                         << planenum1.D
////                         << " num:" << planenum1.num << endl;
//                }
//            }
//        }

    pcl::ModelCoefficients coeffs;
    coeffs.values.resize(4);
    coeffs.values[0] = (0);
    coeffs.values[1] = (0);
    coeffs.values[2] = (1);
    coeffs.values[3] = (-1.3);
    viewer->addPlane(coeffs, "z1");

    pcl::ModelCoefficients coeffs1;
    coeffs1.values.resize(4);
    coeffs1.values[0] = (0);
    coeffs1.values[1] = (0);
    coeffs1.values[2] = (1);
    coeffs1.values[3] = (0.8);
    viewer->addPlane(coeffs1,2,2,3, "z2");


    viewer = rgbdVis(point_cloud_ptr);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

void deletpoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, planenum plane) {
    for (int i = 0; i < cloud->points.size(); ++i) {
        if (countdis(cloud->points[i], plane) <= 0.5) {
            cloud->points.erase(begin(cloud->points) + i);
        } else {
            i++;
        }
    }
}
