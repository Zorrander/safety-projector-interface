#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix : \n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

int main (int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  PointCloudT::Ptr cloud_final_icp (new PointCloudT);  // ICP output point cloud
  PointCloudT::Ptr cloud_tmp (new PointCloudT);  // ICP output point cloud

  int iterations = 50;
  pcl::console::TicToc time;
  time.tic ();
  //pcl::io::loadPCDFile<pcl::PointXYZ>("/home/altair/catkin_abb/rosbags/high_filter/sub/1.pcd",*cloud_in);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/altair/catkin_abb/rosbags/low_filter/sub/2.pcd", *cloud_in) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read PCD file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_in->width * cloud_in->height
            << " data points from sub_pcd.pcd with the following fields: "
            << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/altair/catkin_abb/rosbags/low_filter/master/2.pcd", *cloud_tmp) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read PCD file \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud_tmp->width * cloud_tmp->height
            << " data points from master_pcd.pcd with the following fields: "
            << std::endl;


  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // A translation on Z axis (0.4 meters)
  //transformation_matrix (2, 3) = 0.4;
  Eigen::Matrix4d reach_icp_matrix = Eigen::Matrix4d::Identity ();
  /**reach_icp_matrix (0, 0) = 0.0;
  reach_icp_matrix (0, 1) = 0.0;
  reach_icp_matrix (0, 2) = 0.0;
  reach_icp_matrix (1, 0) = 0.0;
  reach_icp_matrix (1, 1) = 0.0;
  reach_icp_matrix (1, 2) = 0.0;
  reach_icp_matrix (2, 0) = 0.0;
  reach_icp_matrix (2, 1) = 0.0;
  reach_icp_matrix (2, 2) = 0.0;
  reach_icp_matrix (0, 3) = -0.35;
  reach_icp_matrix (1, 3) = -0.55;
  reach_icp_matrix (2, 3) = 0.0;**/

/**Rotation matrix : 
    |  0.995 -0.019  0.096 | 
R = |  0.002  0.986  0.167 | 
    | -0.097 -0.166  0.981 | 
Translation vector :
t = <  0.010,  0.057,  0.183 >**/


  Eigen::Matrix4d icp_matrix = Eigen::Matrix4d::Identity ();
  /*icp_matrix (0, 0) = 0.995;
  icp_matrix (0, 1) = -0.019;
  icp_matrix (0, 2) = 0.096;
  icp_matrix (1, 0) = 0.002;
  icp_matrix (1, 1) = 0.986;
  icp_matrix (1, 2) = 0.167;
  icp_matrix (2, 0) = -0.097;
  icp_matrix (2, 1) = -0.166;
  icp_matrix (2, 2) = 0.981;
  icp_matrix (0, 3) = -0.340;
  icp_matrix (1, 3) = -0.493;
  icp_matrix (2, 3) = 0.293;*/

  Eigen::Matrix4d sum = reach_icp_matrix * icp_matrix; 

  std::cout<<"FINAL MATRIX TRANSFORM \n";
  print4x4Matrix (icp_matrix);
  std::cout<<"------------------- \n";


  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (icp_matrix);

  // Executing the transformation
  //pcl::transformPointCloud (*cloud_tmp, *cloud_icp, icp_matrix);
  *cloud_icp = *cloud_tmp;
  *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

  //pcl::transformPointCloud (*cloud_final_icp, *cloud_icp, reach_icp_matrix);

  // The Iterative Closest Point algorithm

  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1);
  icp.setMaxCorrespondenceDistance(0.03); // 50cm
  icp.setRANSACOutlierRejectionThreshold(0.03);
  icp.setMaximumIterations (iterations);
  icp.align (*cloud_icp);
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    //return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two vertically separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);;
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();

    // The user pressed "space" :
    if (next_iteration)
    {
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
        reach_icp_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (reach_icp_matrix);  // Print the transformation between original pose and current pose

        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
        viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    }
    next_iteration = false;
  }
  return (0);
}