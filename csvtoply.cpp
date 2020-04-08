#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/uniform_sampling.h>

using namespace std;

int main() {
    for (int j = 1; j < 8; ++j) {
        ifstream fin("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/txt/"+to_string(j)+".txt");
        string line;
        vector<string> fields; //声明一个字符串向量
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        //int num = 0;
        while (getline(fin, line)) {
            fields.clear();
            istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
            string field;
            while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
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

//            point.x = point.x/100.0;
//            point.y = point.y/100.0;
//            point.z = point.z/100.0;

            point_cloud_ptr->points.push_back(point);
            //num++;
            //if (num % 100000 == 0) cout << "line " << num << "  " << line << endl;
        }

        point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
        point_cloud_ptr->height = 1;

        cout << "read finish start downsample:" << endl;
        cout<< "before has "<<point_cloud_ptr->points.size()<<endl;

//        pcl::UniformSampling<pcl::PointXYZ> filter;
//        filter.setInputCloud(point_cloud_ptr);
//        filter.setRadiusSearch(0.01f);
//        filter.filter(*point_cloud_ptr);
//        cout<<"after has "<<point_cloud_ptr->points.size()<<endl;

        pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/1-RawPointCloud/ply/"+to_string(j)+".ply", *point_cloud_ptr);

        cout << "saved  " <<j<< endl;
        fin.close();
    }
    cout<<"save finish"<<endl;
    return 0;
}
