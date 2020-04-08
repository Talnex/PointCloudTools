#include <iostream>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

float angular_resolution = 0.3f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

int main(int argc, char **argv) {
    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity ());

    pcl::io::loadPLYFile("/Users/talnex/Projects/1.ply",point_cloud);
    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f(point_cloud.sensor_orientation_);

    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &range_image = *range_image_ptr;
    range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges(far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange();

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0f, "global");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0);
    viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");

    pcl::RangeImageBorderExtractor border_extractor(&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute(border_descriptions);

    pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
    pcl::PointCloud<pcl::PointWithRange> &border_points = *border_points_ptr,
            &veil_points = *veil_points_ptr,
            &shadow_points = *shadow_points_ptr;
    for (int y = 0; y < (int) range_image.height; ++y) {
        for (int x = 0; x < (int) range_image.width; ++x) {
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
                border_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
                veil_points.points.push_back(range_image.points[y * range_image.width + x]);
            if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
                shadow_points.points.push_back(range_image.points[y * range_image.width + x]);
        }
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr,
                                                                                                      0, 255, 0);
    viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr,
                                                                                                    255, 0, 0);
    viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr,
                                                                                                      0, 255, 255);
    viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

    pcl::visualization::RangeImageVisualizer *range_image_borders_widget = NULL;
    range_image_borders_widget =
            pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image,
                                                                                 -std::numeric_limits<float>::infinity(),
                                                                                 std::numeric_limits<float>::infinity(),
                                                                                 false,
                                                                                 border_descriptions,
                                                                                 "Range image with borders");

    while (!viewer.wasStopped()) {
        range_image_borders_widget->spinOnce();
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}
