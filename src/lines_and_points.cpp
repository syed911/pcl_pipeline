#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/camera_depth_optical_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    //points.pose.position.x = 0;
    //points.pose.position.y = 0;   
    //points.pose.position.z = 0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // Create the vertices for the points and lines
    /*for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }*/
      for (int i = 0; i < 4; ++i)
    {
      int x = 0;
      int y = 0;	
      int z = 0;

      geometry_msgs::Point p;
      switch (i){
      case 0:
      p.x += 2;
      case 1:
      p.y += 2;
      case 2:
      p.x += -2;
      case 3:
      p.y += -2;
      }

      points.points.push_back(p);
      line_strip.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }
      /*geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      float x1 = 0;
      float y1 = 0;
      float z1 = 0;
      float x2 = 0;
      float y2 = 1;
      float z2 = 0;
      p1.x = x1;
      p1.y = y1;
      p1.z = z1;
      p2.x = x2;
      p2.y = y2;
      p2.z = z2;
      points.points.push_back(p1);
      line_strip.points.push_back(p1);
      line_list.points.push_back(p1);
      p1.z += 5.0;
      p1.x += 2.5;
      p1.y += 3.0;
      line_list.points.push_back(p1);
      points.points.push_back(p2);
      line_strip.points.push_back(p2);
      line_list.points.push_back(p2);
      p2.z += 5.0;
      p2.x += 2.5;
      p2.y += 3.0;
      line_list.points.push_back(p2);*/

      /*// The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);*/

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}
