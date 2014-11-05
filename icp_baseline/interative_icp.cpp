#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix (const Eigen::Matrix4d & matrix){
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

     
    

int main(int argc, char **argv){

    PointCloudT::Ptr cloud_model(new PointCloudT);
    PointCloudT::Ptr cloud_object(new PointCloudT);
    PointCloudT::Ptr cloud_objectalign(new PointCloudT);
    PointCloudT::Ptr cloud_mid(new PointCloudT);
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    pcl::visualization::PCLVisualizer viewer ("viewer");
   
    Eigen::Vector4f centroid_model;
    Eigen::Vector4f centroid_object;
    Eigen::Vector4f distance_centroid;

    if (pcl::io::loadPLYFile("4by4_3faces_pointcloud.ply", *cloud_model)<0){
        PCL_ERROR("Error loading cloud 4by4.pyl");
    }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_model, 255,0,0);

        viewer.addPointCloud(cloud_model,red,"1",0);
    if (pcl::io::loadPLYFile("output.ply", *cloud_objectalign)<0){
        PCL_ERROR("Error loading cloud 4by4.pyl");
    }
    
    pcl::compute3DCentroid (*cloud_model, centroid_model);
    pcl::compute3DCentroid (*cloud_objectalign, centroid_object);
    distance_centroid = centroid_object - centroid_model;

//    double theta=M_PI / 2;
//    transformation_matrix(0,0) = cos(theta);
//    transformation_matrix(0,1) = -sin(theta);
//    transformation_matrix(1,0) = sin(theta);
//    transformation_matrix(1,1) = cos(theta);
////    
//   pcl::transformPointCloud (*cloud_objectalign, *cloud_object, transformation_matrix);
//    
//
     *cloud_object = *cloud_objectalign; 
     transformation_matrix = Eigen::Matrix4d::Identity();
      std::cout<<"transformation vector: ("<<distance_centroid(0)<<","<<distance_centroid(1)<<","<<distance_centroid(2)<<")"<<std::endl;
      transformation_matrix(0,3) = distance_centroid(0);
      transformation_matrix(1,3) = distance_centroid(1);
      transformation_matrix(2,3) = distance_centroid(2);
//    
   pcl::transformPointCloud (*cloud_model, *cloud_mid, transformation_matrix);
    
//    *cloud_icp = *cloud_tr; 
//    *cloud_mid = *cloud_model;
    *cloud_model = *cloud_mid;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(100);
    icp.setInputSource (cloud_model);
    icp.setInputTarget (cloud_object);
    icp.align(*cloud_mid);
    icp.setMaximumIterations(1);

 

    transformation_matrix = Eigen::Matrix4d::Identity();
   if (icp.hasConverged()){
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else{
        PCL_ERROR("not converged\n");
    }



  

    pcl::io::savePLYFileASCII ("/home/yuhan/catkin_ws/rosbag/mid.ply", *cloud_mid);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_mid, 0,255,0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (cloud_object, 0,0,255);
        viewer.addPointCloud(cloud_mid,green,"2",0);
        viewer.addPointCloud(cloud_object,blue,"3",0);
        viewer.addCoordinateSystem (1.0);
        viewer.spin ();





}
