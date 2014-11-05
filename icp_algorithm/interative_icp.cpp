#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ICP_Class {
public:    
       
    PointCloudT::Ptr cloud_in;
    PointCloudT::Ptr cloud_icp;
    PointCloudT::Ptr cloud_tr;
    Eigen::Matrix4d transformation_matrix;
    pcl::visualization::PCLVisualizer viewer;
   
    void transformPointCloud(){
    transformation_matrix = Eigen::Matrix4d::Identity();
    double theta=M_PI / 8;
    transformation_matrix(0,0) = cos(theta);
    transformation_matrix(0,1) = -sin(theta);
    transformation_matrix(1,0) = sin(theta);
    transformation_matrix(1,1) = cos(theta);
    

    transformation_matrix(2,3) = 0.4;
    
    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
    
    *cloud_tr = *cloud_icp;
    }

    void readPointCloud(){

    if (pcl::io::loadPLYFile("4by4.ply", *cloud_in)<0){
        PCL_ERROR("Error loading cloud 4by4.pyl");
        }
    }

    void icp_calc(){
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(5);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);

    if (icp.hasConverged()){
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else{
        PCL_ERROR("not converged\n");
    }
    }
    void visualize(){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_in, 255,0,0);
     //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_tr, 0,255,0);
     //   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (cloud_icp, 0,0,255);
        viewer.addPointCloud(cloud_in,red,"in");
     //   viewer.addPointCloud(cloud_tr,green,"mid");
     //   viewer.addPointCloud(cloud_icp,blue,"tar");
    }


    void print4x4Matrix (const Eigen::Matrix4d & matrix){
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
        }


    
};


int main(int argc, char **argv){
    ICP_Class *icp = new ICP_Class;
    icp->readPointCloud();
    icp->transformPointCloud();
    icp->icp_calc();
    icp->visualize();
}
