#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <string.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255,100,100);
  pcl::visualization::PCLVisualizer viewer("Point Cloud Filtering");
  if (argc!= 2) std::cout<<"please select the point cloud for visualzation (1,2,3)" << std::endl;
  // Fill in the cloud data
 // pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  //reader.io::<pcl::PointXYZ> ("table_scene_mug_stereo_textured.pcd", *cloud);
  pcl::PCLPointCloud2 inputCloud;
  pcl::io::loadPCDFile("table_scene_lms400.pcd",inputCloud);
  pcl::fromPCLPointCloud2(inputCloud,*cloud);


  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  if(strcmp(argv[1],"1")==0) 
        viewer.addPointCloud(cloud,source_cloud_color_handler,"voxelized cloud");
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  if(strcmp(argv[1],"2")==0) 
        viewer.addPointCloud(cloud_filtered,source_cloud_color_handler,"voxelized cloud");
  //pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255,255,255);
  //viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud" );
  //viewer.addPointCloud(cloud, "original_cloud" );
  //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"table_scene_lms400_inliers.pcd");
  //while(!viewer.wasStopped()){
  //    viewer.spinOnce();
  //}
 // writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
  pcl::VoxelGrid<pcl::PointXYZ> sor2;
  sor2.setInputCloud(cloud_filtered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZ>);
  sor2.setLeafSize(0.01f,0.01f,0.01f);
  sor2.filter(*cloud_voxelized);

  std::cerr<<"Point cloud after voxelizing: "<<cloud_voxelized->width * cloud_voxelized->height << std::endl;
    
 
  if(strcmp(argv[1],"3")==0) 
        viewer.addPointCloud(cloud_voxelized,source_cloud_color_handler,"voxelized cloud");
  
  while(!viewer.wasStopped()){
      viewer.spinOnce();
  } 
  return (0);


}
