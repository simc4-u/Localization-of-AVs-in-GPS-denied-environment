	

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
bool next_file=false;
std::string path = "dataset3/mid";

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
  else if(event.getKeySym () == "n" && event.keyDown ())
    next_file=true;
}

int
main (int argc,
      char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  PointCloudT::Ptr cloud_final (new PointCloudT);
  pcl::VoxelGrid<PointT> sor;
  
  sor.setLeafSize (0.1f, 0.1f, 0.1f);

  // Checking program arguments
  if (argc < 3)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one ply file.\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 3)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[3]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPCDFile (argv[2], *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  //sor.setInputCloud (cloud_in);
  //sor.filter (*cloud_in);
  
  std::string str = path + std::to_string(2) + ".pcd";
  if (pcl::io::loadPCDFile (str, *cloud_icp) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  //sor.setInputCloud (cloud_icp);
  //sor.filter (*cloud_icp);
  std::cout << "\nLoaded file " << argv[2] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
  std::cout << "\nLoaded file " << " (" << cloud_icp->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d global_tr_matrix = Eigen::Matrix4d::Identity ();

  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  /*double theta = M_PI / 8;  // The angle of rotation in radians
  transformation_matrix (0, 0) = std::cos (theta);
  transformation_matrix (0, 1) = -sin (theta);
  transformation_matrix (1, 0) = sin (theta);
  transformation_matrix (1, 1) = std::cos (theta);

  // A translation on Z axis (0.4 meters)
  transformation_matrix (2, 3) = 0.4;

  // Display in terminal the transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix (transformation_matrix);

  // Executing the transformation
  pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
*/
  *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
  *cloud_final= *cloud_in; 
  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  icp.align (*cloud_icp);
  //icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
    global_tr_matrix=transformation_matrix;
    *cloud_final += *cloud_icp;
      sor.setInputCloud (cloud_final);
      sor.filter (*cloud_final);
    std::cout << " (" << cloud_in->size () << " points of in)" << std::endl;
    std::cout << " (" << cloud_icp->size () << " points of icp )" << std::endl;
    std::cout << " (" << cloud_final->size () << " points of final )" << std::endl;
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
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
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_final_color_h (cloud_final, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_final, cloud_final_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
  viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  //viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

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
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Register keyboard callback :
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
  int i=1;
  
  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();

/*    if(next_file)
    {
      std::string str = path + std::to_string(i+2) + ".pcd";
      if (pcl::io::loadPCDFile (str, *cloud_icp) < 0)
	  {
	    PCL_ERROR ("Error loading cloud %d.\n", i );
	    return (-1);
	  }
      std::cout << "NEW FILE LOADED" << std::endl;
      
      //sor.setInputCloud (cloud_final);
      //sor.filter (*cloud_final);
      
      iterations=atoi(argv[3]);
      next_iteration=true;
      i++;
    }
*/
    // The user pressed "space" :
    if (true)
    {
      std::string str = path + std::to_string(i+2) + ".pcd";
      if (pcl::io::loadPCDFile (str, *cloud_icp) < 0 || i+2>atoi(argv[1]))
	  {
	    PCL_ERROR ("Error loading cloud %d.\n", i );
            pcl::io::savePCDFile ("final.pcd", *cloud_final, true);
	    return (-1);
	  }
      std::cout << "NEW FILE LOADED" << i+2 << std::endl;
      i+=2;
      //sor.setInputCloud (cloud_icp);
      //sor.filter (*cloud_icp);
      *cloud_in=*cloud_tr;
      *cloud_tr=*cloud_icp;
      std::cout << "new cloud size" << cloud_icp->size () << std::endl; 
      // The Iterative Closest Point algorithm
      time.tic ();
      icp.setMaximumIterations (iterations);
      icp.setInputSource (cloud_icp);
      icp.setInputTarget (cloud_in);
      icp.align (*cloud_icp);
      std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

      if (icp.hasConverged ())
      {
        printf ("\033[11A");  // Go up 11 lines in terminal output.
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
        pcl::transformPointCloud (*cloud_icp, *cloud_icp, global_tr_matrix);
        *cloud_final += *cloud_icp;
      //sor.setInputCloud (cloud_final);
      //sor.filter (*cloud_final);
	global_tr_matrix *= transformation_matrix;
        ss.str ("");
        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str ();
        viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
       // viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
        viewer.updatePointCloud (cloud_final, cloud_final_color_h, "cloud_in_v2");
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        pcl::io::savePCDFile ("final.pcd", *cloud_final, true); 
        return (-1);
      }
    }
    next_iteration = false;
    next_file = false;
  }
  pcl::io::savePCDFile ("final.pcd", *cloud_final, true);
  return (0);
}

