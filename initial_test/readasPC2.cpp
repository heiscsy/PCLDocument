#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char **argv){
    pcl::PCLPointCloud2 input;
    pcl::io::loadPCDFile("dog0.pcd",input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2 (input,*cloud);
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("outputdog0.pcd",*cloud,false);
    std::cout<<cloud;
    

}
