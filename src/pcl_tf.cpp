#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//#include <vector>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/pfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/cvfh.h>
#include <pcl/segmentation/region_growing.h>
#include <Eigen/StdVector>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
//#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/console/parse.h>
#include <iostream>

#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
using namespace std;

std::string ns;
int id;
//float r;
//float g;
//float b;
float p1;
float p2;
float p3;
float p4;
float p5;
float p6;
float p7;
float p8;

ros::Publisher pubt;
ros::Publisher pub;
ros::Publisher pub0;
ros::Publisher pubx;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;
ros::Publisher pub7;
ros::Publisher pub8;
ros::Publisher pubr;
ros::Publisher pubo;
ros::Publisher marker_pub;
ros::Publisher marker_pub1;
ros::Publisher marker_pub2;
std::vector <float> moment_of_inertia;
std::vector <float> eccentricity;
pcl::PointXYZRGB min_point_AABB;
pcl::PointXYZRGB max_point_AABB;
pcl::PointXYZRGB min_point_OBB;
pcl::PointXYZRGB max_point_OBB;
pcl::PointXYZRGB position_OBB;
Eigen::Matrix3f rotational_matrix_OBB;
float major_value, middle_value, minor_value;
Eigen::Vector3f major_vector, middle_vector, minor_vector;
Eigen::Vector3f mass_center;
Eigen::Vector4f centroidObject;



void 
cbt (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Transformation matrix object, initialized to the identity matrix
	// (a null transformation).
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// Set a rotation around the Z axis (right hand rule).
    float theta1 = 30.0f * (M_PI / 180.0f); // 90 degrees.
	transform_1(1, 1) = cos(theta1);
	transform_1(1, 2) = sin(theta1);
	transform_1(2, 1) = -sin(theta1);
	transform_1(2, 2) = cos(theta1);

	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
	// Set a rotation around the Z axis (right hand rule).
	float theta2 = 90.0f * (M_PI / 180.0f); // 90 degrees.
	transform_2(1, 1) = cos(theta2);
	transform_2(1, 2) = sin(theta2);
	transform_2(2, 1) = -sin(theta2);
	transform_2(2, 2) = cos(theta2);
	
	Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();
	// Set a rotation around the Z axis (right hand rule).
	float theta3 = 90.0f * (M_PI / 180.0f); // 90 degrees.
	transform_3(0, 0) = cos(theta3);
	transform_3(0, 1) = sin(theta3);
	transform_3(1, 0) = -sin(theta3);
	transform_3(1, 1) = cos(theta3);
	
        Eigen::Matrix4f transform_4 = Eigen::Matrix4f::Identity();
	// Set a translation on the XYZ axis.
    transform_4(0, 3) = 0.18f;
    transform_4(1, 3) = 0.0236f;
    transform_4(2, 3) = 0.4095f;
 	pcl::transformPointCloud(*passCloud, *transformed, transform_1);
	pcl::transformPointCloud(*transformed, *transformed, transform_2);
	pcl::transformPointCloud(*transformed, *transformed, transform_3);
	pcl::transformPointCloud(*transformed, *transformed, transform_4);
	sensor_msgs::PointCloud2 tfPointcloud;
	pcl::toROSMsg(*transformed, tfPointcloud);

	//Publish
	pubt.publish(tfPointcloud);
	

}


void 
cb0 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with Z values not in the [0-2] range.
	filter.setFilterFieldName("z");
	filter.setFilterLimits(-0.5, 0.8);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutput;
	pcl::toROSMsg(passFiltered, passOutput);

	//Publish
	pub0.publish(passOutput);
	

}

void 
cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with y values not in the [0-2] range.
	filter.setFilterFieldName("y");
	filter.setFilterLimits(-0.3, 0.2);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutputY;
	pcl::toROSMsg(passFiltered, passOutputY);

	//Publish
	pub.publish(passOutputY);
	

}

void 
cbx (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with x values not in the [0-2] range.
	filter.setFilterFieldName("x");
	filter.setFilterLimits(-0.6, 0.4);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutputX;
	pcl::toROSMsg(passFiltered, passOutputX);

	//Publish
	pubx.publish(passOutputX);
	

}


void 
cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;

	pcl::fromROSMsg (*cloud, *inputCloud);
  
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
	voxelFilter.setInputCloud(inputCloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	voxelFilter.setLeafSize(0.008f, 0.008f, 0.008f);

	voxelFilter.filter(filteredCloud);
	
	sensor_msgs::PointCloud2 voxelOutput;
	pcl::toROSMsg(filteredCloud, voxelOutput);

	//Publish
	pub1.publish(voxelOutput);
	

}


void 
cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal> normals;
	
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
 
	// Object for normal estimation.
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;

	pcl::fromROSMsg (*cloud, *voxCloud);

	normalEstimation.setInputCloud(voxCloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(0.03);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.

	kdtree->setInputCloud(voxCloud);
	
	normalEstimation.setSearchMethod(kdtree);
 
	// Calculate the normals.
	normalEstimation.compute(normals);

	sensor_msgs::PointCloud2 normalOutput;
	pcl::toROSMsg (normals, normalOutput);

	
	
	pub2.publish(normalOutput);

}



void 
cb3 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 
	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	pcl::fromROSMsg (*cloud, *voxCloud);

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGB> segmentation1;
	segmentation1.setInputCloud(voxCloud);
	// Configure the object to look for a plane.
	segmentation1.setModelType(pcl::SACMODEL_PLANE);
	// Use RANSAC method.
	segmentation1.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation1.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional).
	segmentation1.setOptimizeCoefficients(true);
 
	//pcl::PointIndices inlierIndices;
	//segmentation.segment(inlierIndices, *coefficients);
	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);

	segmentation1.segment(*pointIndices, *coefficients);
 
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(voxCloud);
	extract.setIndices(pointIndices);
	// We will extract the points that are NOT indexed (the ones that are not in a plane).
	extract.setNegative(true);

	extract.filter(*segCloud);

	sensor_msgs::PointCloud2 segOutput;
	pcl::toROSMsg (*segCloud, segOutput);
	
	pub3.publish(segOutput);



}

void 
cbr (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;

	pcl::fromROSMsg (*cloud, *inputCloud);
  
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> filter;
	filter.setInputCloud(inputCloud);
	// Every point must have 10 neighbors within 15cm, or it will be removed.
	filter.setRadiusSearch(0.10);

	filter.setMinNeighborsInRadius(10);
	filter.filter(filteredCloud);

	sensor_msgs::PointCloud2 Outlier;
	pcl::toROSMsg(filteredCloud, Outlier);

	//Publish
	pubr.publish(Outlier);
}
void
cb8 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//Object for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;
	pcl::fromROSMsg (*cloud, *inputCloud);
	
	// Filter object.
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
	filter.setInputCloud(inputCloud);
	// Set number of neighbors to consider to 50.
	filter.setMeanK(50);
	// Set standard deviation multiplier to 1.
	// Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
	filter.setStddevMulThresh(1.0);
 
	filter.filter(filteredCloud);
 	sensor_msgs::PointCloud2 OutlierRemoval;
	pcl::toROSMsg(filteredCloud, OutlierRemoval);
	//Publish
	pub8.publish(OutlierRemoval);
        
	Eigen::Vector4f centroid;
 
		pcl::compute3DCentroid(filteredCloud, centroid);
		
                double centroid_X = centroid[0];//centroid[2]*1.2+0.036;//(centroid[2]+0.13);//centroid[0];
                double centroid_Y = centroid[1];//-centroid[0]+0.075;//(-centroid[0]*0.1+0.12);//(-centroid[0]*0.829+0.12);//centroid[1];
                double centroid_Z = centroid[2];//-centroid[1]/7; //(-centroid[1]*0.829-0.05);//centroid[2];

                geometry_msgs::Point centroid_msg;

                centroid_msg.x = centroid_X;
                centroid_msg.y = centroid_Y;
                centroid_msg.z = centroid_Z;

                pub6.publish(centroid_msg);
                
                std::cout << "The XYZ coordinates of the centroid are: ("
		  //<< centroid[0] << ", "
		  //<< centroid[1] << ", "
		  //<< centroid[2] << ")." << std::endl;
		  << centroid_X << ", "
		  << centroid_Y << ", "
		  << centroid_Z << ")." << std::endl;


}
int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_line_test");

	
        ros::NodeHandle nh1;


	// Create a ROS subscriber for the input point cloud from the tf
	ros::Subscriber sub0 = nh1.subscribe ("/camera/depth_registered/points", 1, cb0);

	//Ros publisher for voxel output
	pub0 = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutput", 1);

	// Create a ROS subscriber for the input point cloud from the passOutput
	ros::Subscriber sub = nh1.subscribe ("passOutput", 1, cb);

	//Ros publisher for voxel output
	pub = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutputY", 1);

	// Create a ROS subscriber for the input point cloud from the passOutputY
	ros::Subscriber subx = nh1.subscribe ("passOutputY", 1, cbx);

	//Ros publisher for voxel output
	pubx = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutputX", 1);


	// Create a ROS subscriber for the input point cloud from the passOutputX
	ros::Subscriber sub1 = nh1.subscribe ("passOutputX", 1, cb1);

	//Ros publisher for voxel output
	pub1 = nh1.advertise<sensor_msgs::PointCloud2>  ("voxelOutput", 1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub2 = nh1.subscribe ("voxelOutput", 1, cb2);

	//Ros publisher for normal output
	pub2 = nh1.advertise<sensor_msgs::PointCloud2>  ("normalOutput", 1);

	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber subt = nh1.subscribe ("voxelOutput", 1, cbt);

	//Ros publisher for voxel output
	pubt = nh1.advertise<sensor_msgs::PointCloud2>  ("tfPointcloud", 1);
	
        // Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub3 = nh1.subscribe ("tfPointcloud", 1, cb3); 

        //Ros publisher for normal output
	pub3 = nh1.advertise<sensor_msgs::PointCloud2>  ("segOutput", 1);
 
	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber subr = nh1.subscribe ("segOutput", 1, cbr); 

        //Ros publisher for normal output
	pubr = nh1.advertise<sensor_msgs::PointCloud2>  ("Outlier", 1);
 
        // ROS publisher for centroid of the segmented object
        pub6 = nh1.advertise<geometry_msgs::Point>  ("centroid",1);
	// ROS publisher for the pose of the object 
	pubo = nh1.advertise<geometry_msgs::Quaternion> ("orientation", 1);
	
       
        //Ros Publisher for Visualization Message	
	marker_pub = nh1.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	//Ros Publisher for Marker2	
	//marker_pub1 = nh1.advertise<visualization_msgs::Marker>("visualization_marker2", 100);

	//Ros Publisher for arrow	
	//marker_pub2 = nh1.advertise<visualization_msgs::Marker>("visualization_marker2", 100);


	// Create a ROS subscriber for the cloud from the outlier
	ros::Subscriber sub8 = nh1.subscribe ("Outlier", 1, cb8);

	//Ros publisher for normal output
	pub8 = nh1.advertise<sensor_msgs::PointCloud2>  ("OutlierRemoval", 1);
	

	// Spin
	ros::spin ();
}
