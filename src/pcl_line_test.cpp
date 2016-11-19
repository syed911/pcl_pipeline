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

#include <pcl/console/parse.h>
#include <iostream>

ros::Publisher pub;
ros::Publisher pub0;
ros::Publisher pubx;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;



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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> passFiltered;

	pcl::fromROSMsg (*cloud, *passCloud);
  
	// Filter object.
	pcl::PassThrough<pcl::PointXYZRGB> filter;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> filteredCloud;

	pcl::fromROSMsg (*cloud, *inputCloud);
  
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
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

	Eigen::Vector4f centroid;
 
		pcl::compute3DCentroid(*segCloud, centroid);
		
                double centroid_X = (centroid[2]+0.13);
                double centroid_Y = (-centroid[0]*0.829+0.12);
                double centroid_Z = (-centroid[1]*0.829-0.05);

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

/*void 
cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg (*cloud, *segCloud);
	
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	kdtree->setInputCloud(segCloud);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Euclidean clustering object.
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
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

	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber sub0 = nh1.subscribe ("input", 1, cb0);

	//Ros publisher for voxel output
	pub0 = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutput", 1);

	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber sub = nh1.subscribe ("passOutput", 1, cb);

	//Ros publisher for voxel output
	pub = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutputY", 1);

	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber subx = nh1.subscribe ("passOutputY", 1, cbx);

	//Ros publisher for voxel output
	pubx = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutputX", 1);


	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber sub1 = nh1.subscribe ("passOutputX", 1, cb1);

	//Ros publisher for voxel output
	pub1 = nh1.advertise<sensor_msgs::PointCloud2>  ("voxelOutput", 1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub2 = nh1.subscribe ("voxelOutput", 1, cb2);

	//Ros publisher for normal output
	pub2 = nh1.advertise<sensor_msgs::PointCloud2>  ("normalOutput", 1);
	
        //Ros publisher for normal output
	pub3 = nh1.advertise<sensor_msgs::PointCloud2>  ("segOutput", 1);

        pub6 = nh1.advertise<geometry_msgs::Point>  ("centroid",1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub3 = nh1.subscribe ("voxelOutput", 1, cb3);        


        //ros::Subscriber sub3 = nh1.subscribe ("centroid", 1, cb4);

	// Create a ROS subscriber for the cloud from the Voxel output
	//ros::Subscriber sub4 = nh1.subscribe ("segOutput", 1, cb4);

	//Ros publisher for normal output
	//pub4 = nh1.advertise<sensor_msgs::PointCloud2>  ("euclideanObject", 1);

	//Ros publisher for normal output
	//pub5 = nh1.advertise<sensor_msgs::PointCloud2>  ("objectSurface", 1);



	// Spin
        ros::Rate loop_rate(1);
	ros::spin ();
}
