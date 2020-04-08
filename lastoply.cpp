#include <liblas/liblas.hpp>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <time.h>
using namespace std;

int main() {
    clock_t sT, tT, eT;
//打开文件
    ifstream fp;
    fp.open("/Volumes/Data/datasets/WH-ZZ/2-AlignedPointCloud/F01_reg.las", ios::in | ios::binary);
    sT = clock();
//读取
    liblas::ReaderFactory readerFactory;
    liblas::Reader reader = readerFactory.CreateWithStream(fp);
    tT = clock();
    cout << "读取" << reader.GetHeader().GetPointRecordsCount() << "个点花费" << (double) (tT - sT) / CLOCKS_PER_SEC << "s"<< endl;
//点云类型
    pcl::PointCloud<pcl::PointXYZ> cloudOutput;
    cloudOutput.clear();
//转换xyz
    while (reader.ReadNextPoint()) {
        double x = reader.GetPoint().GetX();
        double y = reader.GetPoint().GetY();
        double z = reader.GetPoint().GetZ();
        pcl::PointXYZ thePt(x, y, z);
        cloudOutput.push_back(thePt);
    }
    cloudOutput.width = cloudOutput.size();
    cloudOutput.height = 1;
    cloudOutput.is_dense = false;
    cloudOutput.resize(cloudOutput.width * cloudOutput.height);

    pcl::io::savePLYFileBinary("/Volumes/Data/datasets/WH-ZZ/2-AlignedPointCloud/ply/1_reg.ply",cloudOutput);
    cloudOutput.clear();
    eT = clock();
    cout << "存入" << reader.GetHeader().GetPointRecordsCount() << "个点花费" << (double) (eT - tT) / CLOCKS_PER_SEC << "s"<< endl;
    return 0;
}