#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <iostream>
#include <string>
#include <queue>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

queue<PointCloudT> q;
float_t velo=0.00;
void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input, const boost::shared_ptr<const carla_msgs::CarlaEgoVehicleStatus>& status_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    if(status_msg->velocity>0.1)
    q.push(*temp_cloud);
}

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



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "icp");
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
  PointCloudT::Ptr cloud_final (new PointCloudT);
  pcl::VoxelGrid<PointT> sor;
  bool flag=true;
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  int iterations=100;
  pcl::console::TicToc time;
  time.tic ();
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d global_tr_matrix = Eigen::Matrix4d::Identity ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
 

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::Publisher output_pub = n.advertise<sensor_msgs::PointCloud2>("icp/output_points", 1000);
  //ros::Subscriber pointcloud_sub = n.subscribe("/carla/hero/lidar/front/point_cloud", 1000, pointcloudCallback);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(n, "/carla/hero/lidar/front/point_cloud", 100);
  message_filters::Subscriber<carla_msgs::CarlaEgoVehicleStatus> velo_sub(n, "/carla/hero/vehicle_status", 100);
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, carla_msgs::CarlaEgoVehicleStatus> sync(pointcloud_sub, velo_sub, 100);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  //ros::Subscriber velocity_sub = n.subscribe("/carla/hero/vehicle_status", 1, velocityCallback);
  ros::Rate loop_rate(2);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    if(flag && q.size()>4)
    {
       *cloud_in = q.front();
       q.pop();
       *cloud_in += q.front();
       q.pop();
       sor.setInputCloud (cloud_in);
       sor.filter (*cloud_in);
       *cloud_icp= q.front();
       q.pop();
       *cloud_icp += q.front();
       q.pop();
       sor.setInputCloud (cloud_icp);
       sor.filter (*cloud_icp);
       *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
       *cloud_final= *cloud_in; 
       // The Iterative Closest Point algorithm
       time.tic ();
       
       icp.setMaximumIterations (iterations);
       icp.setInputSource (cloud_icp);
       icp.setInputTarget (cloud_in);
       icp.align (*cloud_icp);
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
        pcl::io::savePCDFile ("final1.pcd", *cloud_final, true);
		return (-1);
	  }
       
       flag=false;
   
    }
    else if(q.size()>4)
     {
      *cloud_icp = q.front();
       q.pop();
      *cloud_icp += q.front();
       q.pop();
       sor.setInputCloud (cloud_icp);
       sor.filter (*cloud_icp);
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
        sor.setInputCloud (cloud_final);
        sor.filter (*cloud_final);
	    global_tr_matrix *= transformation_matrix;
      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
        pcl::io::savePCDFile ("final1.pcd", *cloud_final, true); 
        return (-1);
      }     

     }

   
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_final, output);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
output.header.frame_id = "map";
    output_pub.publish(output);

    ros::spinOnce();

    loop_rate.sleep();
  }

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  pcl::io::savePCDFile ("final1.pcd", *cloud_final, true);
  return 0;
}
