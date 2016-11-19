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

#include <pcl/console/parse.h>
#include <iostream>


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
ros::Publisher pubm;
void 
cbt (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Transformation matrix object, initialized to the identity matrix
	// (a null transformation).
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	// Set a rotation around the Z axis (right hand rule).
	float theta1 = 34.9f * (M_PI / 180.0f); // 90 degrees.
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
	transform_4(0, 3) = 0.21f; 
	transform_4(1, 3) = -0.02f; 
	transform_4(2, 3) = 0.23f; 
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with Z values not in the [0-2] range.
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0.5, 1.0);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutput;
	pcl::toROSMsg(passFiltered, passOutput);

	//Publish
	pub0.publish(passOutput);
	

}

void 
cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with y values not in the [0-2] range.
	filter.setFilterFieldName("y");
	filter.setFilterLimits(-1, 0.1);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutputY;
	pcl::toROSMsg(passFiltered, passOutputY);

	//Publish
	pub.publish(passOutputY);
	

}

void 
cbx (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(passCloud);
	// Filter out all points with x values not in the [0-2] range.
	filter.setFilterFieldName("x");
	filter.setFilterLimits(-0.3, 0.3);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutputX;
	pcl::toROSMsg(passFiltered, passOutputX);

	//Publish
	pubx.publish(passOutputX);
	

}


void 
cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;

	pcl::fromROSMsg (*cloud, *inputCloud);
  
	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
	voxelFilter.setInputCloud(inputCloud);
	// We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);

	voxelFilter.filter(filteredCloud);
	
	sensor_msgs::PointCloud2 voxelOutput;
	pcl::toROSMsg(filteredCloud, voxelOutput);

	//Publish
	pub1.publish(voxelOutput);
	

}


void 
cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal> normals;
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
 
	// Object for normal estimation.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZ>);
 
	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	pcl::fromROSMsg (*cloud, *voxCloud);

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZ> segmentation1;
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
 
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(voxCloud);
	extract.setIndices(pointIndices);
	// We will extract the points that are NOT indexed (the ones that are not in a plane).
	extract.setNegative(true);

	extract.filter(*segCloud);

	sensor_msgs::PointCloud2 segOutput;
	pcl::toROSMsg (*segCloud, segOutput);
	
	pub3.publish(segOutput);



}

/*void 
cb8 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;

	pcl::fromROSMsg (*cloud, *inputCloud);
  
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(inputCloud);
	// Every point must have 10 neighbors within 15cm, or it will be removed.
	filter.setRadiusSearch(0.10);

	filter.setMinNeighborsInRadius(10);
	filter.filter(filteredCloud);

	sensor_msgs::PointCloud2 OutlierRemoval;
	pcl::toROSMsg(filteredCloud, OutlierRemoval);

	//Publish
	pub8.publish(OutlierRemoval);*/
void
cb8 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	//Object for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*cloud, *inputCloud);
	
	// Filter object.
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
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
	Eigen::Vector4f min;
	Eigen::Vector4f max;
	pcl::getMinMax3D(filteredCloud, min, max);
 	uint32_t shape = visualization_msgs::Marker::SPHERE;
  	visualization_msgs::Marker marker;
 	marker.header.frame_id ="/camera_depth_frame";
  	marker.header.stamp = ros::Time::now(); 

        marker.ns = ns;
        marker.id = 1;
        marker.type = shape;
   	marker.action = visualization_msgs::Marker::ADD; 
                 

	marker.pose.position.x = centroid[2];
  	marker.pose.position.y = -centroid[0];
  	marker.pose.position.z = -centroid[1];
  	marker.pose.orientation.x = 0.0;
  	marker.pose.orientation.y = 0.3755;
 	marker.pose.orientation.z = 0.0;
  	marker.pose.orientation.w = 0.926798; 
	
 	marker.scale.x = (max[2]-min[2]);
  	marker.scale.y = (min[0]-max[0]);
  	marker.scale.z = (min[1]-max[1]); 
	//if (marker.scale.x ==0)
      		//marker.scale.x=0.1;

  	//if (marker.scale.y ==0)
    		//marker.scale.y=0.1;

  	//if (marker.scale.z ==0)
    	        //marker.scale.z=0.1;
   
  	marker.color.r = 255;
  	marker.color.g = 0;
  	marker.color.b = 0;
  	marker.color.a = 1.0;

  	marker.lifetime = ros::Duration();

  	pubm.publish(marker);
	
	visualization_msgs::Marker points, line_strip, line_list;
    	points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/camera_depth_frame";
    	points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    	points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    	points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        //points.pose.orientation.x = line_strip.pose.orientation.x = line_list.pose.orientation.x = 0.0;
       //points.pose.orientation.y = line_strip.pose.orientation.y = line_list.pose.orientation.y = 0.0; 
        //points.pose.orientation.z = line_strip.pose.orientation.z = line_list.pose.orientation.z = 0.0;
        //points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 0.78;
	points.id = 0;
    	line_strip.id = 1;
    	line_list.id = 2;
        points.type = visualization_msgs::Marker::POINTS;
    	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    	line_list.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.01;
        points.scale.y = 0.01;
        line_list.scale.x = 0.005;
        points.color.g = 1.0f;
        points.color.a = 1.0;
       // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = centroid[2];
	p.y = -centroid[0];
	p.z = -centroid[1];
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	geometry_msgs::Point p3;
	geometry_msgs::Point p4;
	geometry_msgs::Point p5;
	geometry_msgs::Point p6;
	geometry_msgs::Point p7;
	geometry_msgs::Point p8;
	p1.x = 	(centroid[2]+((max[1]-min[1]/2)-(max[2]-min[2])/1.4)*0.573);
	p1.y = 	(-centroid[0]+(max[0]-min[0])/2);
	p1.z = 	(-centroid[1]-(((max[1]-min[1]/2)-(max[2]-min[2])/1.4)*0.819+(max[2]-min[2])/1.146));
	points.points.push_back(p);
        line_list.points.push_back(p1);
        p5.x = 	p1.x-(max[1]-min[1])*0.573;//(centroid[2]-(max[2]-min[2])/2);
	p5.y = 	(-centroid[0]+(max[0]-min[0])/2);
	p5.z = 	p1.z+(max[1]-min[1])*0.819;//(-centroid[1]+(max[1]-min[1])/2);        
        line_list.points.push_back(p5);
        p2.x = p1.x;//(centroid[2]+(max[2]-min[2])/2);
	p2.y = 	(-centroid[0]-(max[0]-min[0])/2);
	p2.z = 	p1.z;//(-centroid[1]-(max[1]-min[1])/2);
        line_list.points.push_back(p2);
        p6.x = 	p5.x;//(centroid[2]-(max[2]-min[2])/2);
	p6.y = 	(-centroid[0]-(max[0]-min[0])/2);
	p6.z = 	p5.z;(-centroid[1]+(max[1]-min[1])/2);        
        line_list.points.push_back(p6);
        p4.x = p1.x+(max[2]-min[2])*0.879;//(centroid[2]+(max[2]-min[2]));
	p4.y = 	(-centroid[0]+(max[0]-min[0])/2);
	p4.z = 	p1.z+(max[2]-min[2])*0.573;//(-centroid[1]-(max[1]-min[1])/2);
        line_list.points.push_back(p4);
        p8.x = 	p5.x+(max[2]-min[2])*0.819;//centroid[2]+0);
	p8.y = 	(-centroid[0]+(max[0]-min[0])/2);
	p8.z = 	p5.z+(max[2]-min[2])*0.573;//(-centroid[1]+(max[1]-min[1])/2);        
        line_list.points.push_back(p8);
        p3.x = p4.x;//(centroid[2]+(max[2]-min[2]));
	p3.y = 	(-centroid[0]-(max[0]-min[0])/2);
	p3.z = 	p4.z;//(-centroid[1]-(max[1]-min[1])/2);
        line_list.points.push_back(p3);
        p7.x = 	p8.x;//(centroid[2]+0);
	p7.y = 	(-centroid[0]-(max[0]-min[0])/2);
	p7.z = 	p8.z;//(-centroid[1]+(max[1]-min[1])/2);        
        line_list.points.push_back(p7);
	line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        line_list.points.push_back(p2);
        line_list.points.push_back(p3);
	line_list.points.push_back(p3);
        line_list.points.push_back(p4);
        line_list.points.push_back(p4);
        line_list.points.push_back(p1);
        line_list.points.push_back(p5);
	line_list.points.push_back(p6);
        line_list.points.push_back(p6);
        line_list.points.push_back(p7);
        line_list.points.push_back(p7);
	line_list.points.push_back(p8);
        line_list.points.push_back(p8);
        line_list.points.push_back(p5);
	points.lifetime = ros::Duration();
 	pub7.publish(points);
 	pub7.publish(line_list);
	
	

}

/*void 
cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*cloud, *segCloud);
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(segCloud);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZ>);

	// Euclidean clustering object.
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
	// Set cluster tolerance to 2cm (small values may cause objects to be divided
	// in several clusters, whereas big values may join objects in a same cluster).
	clustering.setClusterTolerance(0.02);
	// Set the minimum and maximum number of points that a cluster can have.
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(10000);
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(segCloud);

	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);
	
	// For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(segCloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		sensor_msgs::PointCloud2 euclideanObject;
		pcl::toROSMsg (*cluster, euclideanObject);
		
		euclideanObject.header.frame_id = "/camera_link";
		euclideanObject.header.stamp = ros::Time::now();
	
		pub4.publish(euclideanObject);

		Eigen::Vector4f centroid;
 
		pcl::compute3DCentroid(*cluster, centroid);
		std::cout << "The XYZ coordinates of the centroid are: ("
		  << centroid[0] << ", "
		  << centroid[1] << ", "
		  << centroid[2] << ")." << std::endl;


		currentClusterNum++;
			
	}

}*/



int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_line_test");

	
        ros::NodeHandle nh1;


	// Create a ROS subscriber for the input point cloud from the tf
	ros::Subscriber sub0 = nh1.subscribe ("input", 1, cb0);

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

	/*// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber subt = nh1.subscribe ("voxelOutput", 1, cbt);

	//Ros publisher for voxel output
	pubt = nh1.advertise<sensor_msgs::PointCloud2>  ("tfPointcloud", 1);*/
	
        // Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub3 = nh1.subscribe ("voxelOutput", 1, cb3); 

        //Ros publisher for normal output
	pub3 = nh1.advertise<sensor_msgs::PointCloud2>  ("segOutput", 1);

        pub6 = nh1.advertise<geometry_msgs::Point>  ("centroid",1);
	
	pub7 = nh1.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	pubm = nh1.advertise<visualization_msgs::Marker>("visualization_marker3", 1);


	// Create a ROS subscriber for the cloud from the outlier
	ros::Subscriber sub8 = nh1.subscribe ("segOutput", 1, cb8);

	//Ros publisher for normal output
	pub8 = nh1.advertise<sensor_msgs::PointCloud2>  ("OutlierRemoval", 1);
	

       


        //ros::Subscriber sub3 = nh1.subscribe ("centroid", 1, cb4);

	// Create a ROS subscriber for the cloud from the Voxel output
	//ros::Subscriber sub4 = nh1.subscribe ("segOutput", 1, cb4);

	//Ros publisher for normal output
	//pub4 = nh1.advertise<sensor_msgs::PointCloud2>  ("euclideanObject", 1);

	//Ros publisher for normal output
	//pub5 = nh1.advertise<sensor_msgs::PointCloud2>  ("objectSurface", 1);



	// Spin
	ros::spin ();
}
