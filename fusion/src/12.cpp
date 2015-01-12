#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h> // not pcl/point_cloud.h --- gives error with xyz publish
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void cloud_cb (const sensor_msgs::PointCloud2& msg)
{
PointCloud x;
pcl::fromROSMsg(msg,x);
BOOST_FOREACH (const pcl::PointXYZ& pt, x.points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int
main (int argc, char** argv)
{
int a;
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, cloud_cb);
  // Spin
  while(ROS::ok())
  {cout<<"a"<<endl;
  cin>>a;
  ros::spinOnce ();}
}
