#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

#include <visualization_msgs/Marker.h>
#include <pcl/common/transforms.h>

#include <pcl/features/moment_of_inertia_estimation.h>


#include <pcl/console/parse.h>
#include <iostream>

ros::Publisher pub0;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher marker_pub;
ros::Publisher marker_pub1;
ros::Publisher marker_pub2;
ros::Publisher pub7;
ros::Publisher pubt;

////////////////////////GLOBAL VARIABLES////////////////////////////

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;
	/////////////////////////////////
	Eigen::Vector4f centroidObject;
	



	//pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	//pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	//pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	//pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));




////////////////////////////////////////////////////////////////////


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
	filter.setFilterLimits(0.625, 1.0);
 
	filter.filter(passFiltered);

	sensor_msgs::PointCloud2 passOutput;
	pcl::toROSMsg(passFiltered, passOutput);

	//Publish
	pub0.publish(passOutput);
	

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


void 
cb4 (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*cloud, *segCloud);
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(segCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr totalCloud (new pcl::PointCloud<pcl::PointXYZ>);

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
		
		euclideanObject.header.frame_id = "/camera_depth_optical_frame";
		euclideanObject.header.stamp = ros::Time::now();
	
		pub4.publish(euclideanObject);

		////////////////////////////////CENTROID DEFINITION//////////////////////

		int rows=0;
		Eigen::MatrixXf cn(1,3);
		cn<<0,0,0;

		////////////////////////////////REGION GROWTH/////////////////////////////

		// Object for storing the normals.
		pcl::PointCloud<pcl::Normal>::Ptr normalsR(new pcl::PointCloud<pcl::Normal>);
 
		// kd-tree object for searches.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeR(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtreeR->setInputCloud(cluster);
 
		// Estimate the normals.
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimationR;
		normalEstimationR.setInputCloud(cluster);
		normalEstimationR.setRadiusSearch(0.03);
		normalEstimationR.setSearchMethod(kdtreeR);
		normalEstimationR.compute(*normalsR);
 
		// Region growing clustering object.
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clusteringR;
		clusteringR.setMinClusterSize(50);
		clusteringR.setMaxClusterSize(10000);
		clusteringR.setSearchMethod(kdtreeR);
		clusteringR.setNumberOfNeighbours(30);
		clusteringR.setInputCloud(cluster);
		clusteringR.setInputNormals(normalsR);
		// Set the angle in radians that will be the smoothness threshold
		// (the maximum allowable deviation of the normals).
		clusteringR.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
		// Set the curvature threshold. The disparity between curvatures will be
		// tested after the normal deviation check has passed.
		clusteringR.setCurvatureThreshold(1.0);
 
		std::vector <pcl::PointIndices> clustersR;
		clusteringR.extract(clustersR);

		// For every cluster...
		int currentClusterNumR = 1;
		for (std::vector<pcl::PointIndices>::const_iterator j = clustersR.begin(); j != clustersR.end(); ++j)
		{
			// ...add all its points to a new cloud...
			pcl::PointCloud<pcl::PointXYZ>::Ptr clusterR(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pointR = j->indices.begin(); pointR != j->indices.end(); pointR++)
				clusterR->points.push_back(cluster->points[*pointR]);
			clusterR->width = clusterR->points.size();
			clusterR->height = 1;
			clusterR->is_dense = true;

			//CENTROID FOR INDIVIDUAL SURFACES
			Eigen::Vector4f centroid;
 
			pcl::compute3DCentroid(*clusterR, centroid);
			
			cn(rows,0)=centroid[0];
			cn(rows,1)=centroid[1];
			cn(rows,2)=centroid[2];
			//cn(rows,3)=centroid[3];

			rows=rows+1;
			cn.conservativeResize ((rows+1),3);


			//NORMAL CETNROID FOR INDIVIDUAL SURFACES
			// Estimate the normals.
			pcl::PointCloud<pcl::Normal>::Ptr normalsR2(new pcl::PointCloud<pcl::Normal>);

			 
			// kd-tree object for searches.

			// kd-tree object.
			pcl::search::KdTree<pcl::PointXYZ> kdtreeR2;
			kdtreeR2.setInputCloud(clusterR);
			//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtreeR2(new pcl::search::KdTree<pcl::PointXYZ>);
			pcl::PointXYZ point;
			point.x = centroid[0];
			point.y = centroid[1];
			point.z = centroid[2];

			std::vector<int> pointIndicesR(15); //store output neighbours
			std::vector<float> squaredDistances(15); //for distances

			kdtreeR2.nearestKSearch(point, 15, pointIndicesR, squaredDistances);
			
				
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimationR2;
			normalEstimationR2.setInputCloud(clusterR);
			normalEstimationR2.setRadiusSearch(0.03);
			//normalEstimationR2.setSearchMethod(kdtreeR2);
			normalEstimationR2.compute(*normalsR2);
		
			Eigen::Vector4f plane_parameters;
			float curvature;

		
			normalEstimationR2.computePointNormal (*clusterR, pointIndicesR , plane_parameters, curvature);
			//std::cout<<plane_parameters<<std::endl;
			//std::cout<<curvature<<std::endl;
			//storing normals in  matrix
			cn(rows,0)=plane_parameters[0];
			cn(rows,1)=plane_parameters[1];
			cn(rows,2)=plane_parameters[2];
			//cn(rows,3)=plane_parameters[3];
			rows++;
			cn.conservativeResize((rows+1),3);

			currentClusterNumR++;
		}
		
		if(rows==4)
		{
			Eigen::Vector4f centroidDifference;   //p2-p1
			//Eigen::Vector4f centroidObject;
			float dotProduct;
			Eigen::Vector4f transition;
			//p2-p1

			centroidDifference[0]= cn(2,0)-cn(0,0);
			centroidDifference[1]= cn(2,1)-cn(0,1);
			centroidDifference[2]= cn(2,2)-cn(0,2);
			//centroidDifference[3]= cn(2,3)-cn(0,3);
			
			//dot product

			dotProduct=centroidDifference[0]*cn(1,0)+centroidDifference[1]*cn(1,1)+centroidDifference[2]*cn(1,2);

			transition[0]= dotProduct*cn(1,0);
			transition[1]= dotProduct*cn(1,1);
			transition[2]= dotProduct*cn(1,2);

			//object Centroid

			centroidObject[0] = cn(0,0)+transition[0];
			centroidObject[1] = cn(0,1)+transition[1];
			centroidObject[2] = cn(0,2)+transition[2];

			//std::cout<<centroidObject<<std::endl;


			////////////////////////////MARKER//////////////////////////////

			// Set our initial shape type to be a cube
			uint32_t shape = visualization_msgs::Marker::CUBE;

			int count =0;		

			  while (count<2)
			  {
			    visualization_msgs::Marker marker;
			    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
			    marker.header.frame_id = "/camera_depth_optical_frame";
			    marker.header.stamp = ros::Time::now();

			    // Set the namespace and id for this marker.  This serves to create a unique ID
			    // Any marker sent with the same namespace and id will overwrite the old one
			    marker.ns = "basic_shapes";
			    marker.id = 0;

			    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			    marker.type = shape;

			    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
			    marker.action = visualization_msgs::Marker::ADD;

			    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
			    marker.pose.position.x = centroidObject[0];
			    marker.pose.position.y = centroidObject[1];
			    marker.pose.position.z = centroidObject[2];
			    marker.pose.orientation.x = 0.0;
			    marker.pose.orientation.y = 0.0;
			    marker.pose.orientation.z = 0.0;
			    marker.pose.orientation.w = 1.0;

			    // Set the scale of the marker -- 1x1x1 here means 1m on a side
			    marker.scale.x = 0.01;
			    marker.scale.y = 0.01;
			    marker.scale.z = 0.01;

			    // Set the color -- be sure to set alpha to something non-zero!
			    marker.color.r = 0.0f;
			    marker.color.g = 0.0f;
			    marker.color.b = 255.0f;
			    marker.color.a = 1.0;

			    marker.lifetime = ros::Duration();

			    marker_pub.publish(marker);
			    count++;
			  }

		}

	

		/////////////////////////////BOUNDING BOX CALCULATION////////////////////////////

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud (cluster);
		feature_extractor.compute ();
		
		feature_extractor.getMomentOfInertia (moment_of_inertia);
		feature_extractor.getEccentricity (eccentricity);
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
		feature_extractor.getEigenValues (major_value, middle_value, minor_value);
		feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
		feature_extractor.getMassCenter (mass_center);



		/////////////////////////////////BOUNDING BOX VISUALIZATION////////////////////////

		Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
		Eigen::Quaternionf quat (rotational_matrix_OBB);
		cout << "major_vector is" << major_vector << endl;
		cout << "middle_vector is" << middle_vector << endl;
		cout << "minor_vector is" << minor_vector << endl;
		cout << "rotational_matrix_OBB" << rotational_matrix_OBB << endl;
		cout << "quat" << quat.x() << ","<< quat.y() << ","<< quat.z() << ","<< quat.w()<< endl;

		// Set our initial shape type to be a cube
		uint32_t shape = visualization_msgs::Marker::CUBE;
		uint32_t shape1 = visualization_msgs::Marker::ARROW;

		int count1 =0;		

		  while (count1<2)
		  {
		    visualization_msgs::Marker marker1;
		    visualization_msgs::Marker marker2;
		    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		    marker1.header.frame_id = "/camera_depth_optical_frame";
		    marker1.header.stamp = ros::Time::now();
		    marker2.header.frame_id = "/camera_depth_optical_frame";
		    marker2.header.stamp = ros::Time::now();

	visualization_msgs::Marker points, line_strip, line_list;
    	points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/camera_depth_optical_frame";
    	points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    	points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    	points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
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
        /*line_list.color.r = 1.0;
        line_list.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = position[0];
	p.y = position[1];
	p.z = position[2];
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	geometry_msgs::Point p3;
	p1.x = major_vector[0];
	p1.y = major_vector[1];
	p1.z = major_vector[2];
	p2.x = middle_vector[0];
	p2.y = middle_vector[1];
	p2.z = middle_vector[2];
	p3.x = minor_vector[0];
	p3.y = minor_vector[1];
	p3.z = minor_vector[2];
        line_list.points.push_back(p);
	line_list.points.push_back(p1);
        line_list.points.push_back(p);
        line_list.points.push_back(p2);
        line_list.points.push_back(p);
	line_list.points.push_back(p3);*/

		    // Set the namespace and id for this marker.  This serves to create a unique ID
		    // Any marker sent with the same namespace and id will overwrite the old one
		    marker1.ns = "basic_shapes2";
		    marker1.id = 1;
		    marker2.ns = "arrow";
		    marker2.id = 0;

		    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		    marker1.type = shape;
		    marker2.type = shape1;
		    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		    marker1.action = visualization_msgs::Marker::ADD;
		    marker2.action = visualization_msgs::Marker::ADD;
		    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		    marker1.pose.position.x = position[0];
		    marker1.pose.position.y = position[1];
		    marker1.pose.position.z = position[2];
		    marker1.pose.orientation.x = quat.x();
		    marker1.pose.orientation.y = quat.y();
		    marker1.pose.orientation.z = quat.z();
		    marker1.pose.orientation.w = quat.w();
		    marker2.pose.position.x = position[0];
		    marker2.pose.position.y = position[1];
		    marker2.pose.position.z = position[2];
		    marker2.pose.orientation.x = quat.x();
		    marker2.pose.orientation.y = quat.y();
		    marker2.pose.orientation.z = quat.z();
		    marker2.pose.orientation.w = quat.w();
		    // Set the scale of the marker -- 1x1x1 here means 1m on a side
		    marker1.scale.x = max_point_OBB.x - min_point_OBB.x;
		    marker1.scale.y = max_point_OBB.y - min_point_OBB.y;
		    marker1.scale.z = max_point_OBB.z - min_point_OBB.z;
		    marker2.scale.x = 0.1;
		    marker2.scale.y = 0.005;
		    marker2.scale.z = 0.005;


		    // Set the color -- be sure to set alpha to something non-zero!
		    marker1.color.r = 255.0f;
		    marker1.color.g = 0.0f;
		    marker1.color.b = 0.0f;
		    marker1.color.a = 0.4;
		    marker2.color.r = 0.0f;
		    marker2.color.g = 255.0f;
		    marker2.color.b = 0.0f;
		    marker2.color.a = 1.2;

		    marker1.lifetime = ros::Duration();
		    marker2.lifetime = ros::Duration();
		    marker_pub1.publish(marker1);
		    marker_pub2.publish(marker2);
	points.lifetime = ros::Duration();
 	pub7.publish(points);
 	pub7.publish(line_list);
		    count1++;
		  }


		currentClusterNum++;
			
	}

}




int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pcl_line");
	ros::NodeHandle nh1;
	


	
	// Create a ROS subscriber for the cloud from the Voxel output

	
		
	
	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber sub0 = nh1.subscribe ("/camera/depth/points", 1, cb0);

	//Ros publisher for voxel output
	pub0 = nh1.advertise<sensor_msgs::PointCloud2>  ("passOutput", 1);

	// Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber sub1 = nh1.subscribe ("passOutput", 1, cb1);

	//Ros publisher for voxel output
	pub1 = nh1.advertise<sensor_msgs::PointCloud2>  ("voxelOutput", 1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub2 = nh1.subscribe ("voxelOutput", 1, cb2);

	//Ros publisher for normal output
	pub2 = nh1.advertise<sensor_msgs::PointCloud2>  ("normalOutput", 1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub3 = nh1.subscribe ("voxelOutput", 1, cb3);
	
        //Ros publisher for planar segmentation
	pub3 = nh1.advertise<sensor_msgs::PointCloud2>  ("segOutput", 1);
	
        // Create a ROS subscriber for the input point cloud from the camera
	ros::Subscriber subt = nh1.subscribe ("segOutput", 1, cbt);

	//Ros publisher for voxel output
	pubt = nh1.advertise<sensor_msgs::PointCloud2>  ("tfPointcloud", 1);
	
        // Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub4 = nh1.subscribe ("tfPointcloud", 1, cb4);

	//Ros publisher for euclidean object
	pub4 = nh1.advertise<sensor_msgs::PointCloud2>  ("euclideanObject", 1);

	// Create a ROS subscriber for the cloud from the Voxel output
	ros::Subscriber sub5 = nh1.subscribe ("euclideanObject", 1, cb4);

	//Ros publisher for region growth
	//pub5 = nh1.advertise<sensor_msgs::PointCloud2>  ("objectSurface", 1);

	
	//Ros Publisher for Visualization Message	
	marker_pub = nh1.advertise<visualization_msgs::Marker>("visualization_marker", 100);

	//Ros Publisher for Marker2	
	marker_pub1 = nh1.advertise<visualization_msgs::Marker>("visualization_marker2", 100);

	//Ros Publisher for arrow	
	marker_pub2 = nh1.advertise<visualization_msgs::Marker>("visualization_marker2", 100);

	pub7 = nh1.advertise<visualization_msgs::Marker>("visualization_marker", 1);



   	// Spin
 	ros::spin ();

	
}
