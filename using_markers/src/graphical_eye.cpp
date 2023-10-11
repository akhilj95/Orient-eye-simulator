#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"
#include <std_msgs/ColorRGBA.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>


namespace rvt = rviz_visual_tools;
class SubscribeAndPublish
{

private:
  ros::NodeHandle n; 
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle n1; 
  ros::Publisher pub1;
  ros::Subscriber sub1;
  visualization_msgs::Marker marker;
  visualization_msgs::Marker arrow;
  visualization_msgs::Marker arrow2;
  visualization_msgs::Marker sphere;
  visualization_msgs::Marker circle;
  visualization_msgs::Marker sphere2;
  visualization_msgs::Marker goal;
  visualization_msgs::Marker opt_time_marker;
  visualization_msgs::Marker eq_end_marker;
  nav_msgs::Path path;
  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  std::string name_;


public:
  SubscribeAndPublish()
  {
    path.header.frame_id = "my_frame";

    marker.header.frame_id = "my_frame";
    marker.ns = "eye_rotation";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.scale.x = 0.01;
    opt_time_marker.header.frame_id = "my_frame";
    opt_time_marker.ns = "eye_rotation";
    opt_time_marker.id = 7;
    opt_time_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    opt_time_marker.color.r = 0.0f;
    opt_time_marker.color.g = 0.0f;
    opt_time_marker.color.b = 1.0f;
    opt_time_marker.color.a = 1.0;
    opt_time_marker.scale.x = 0.015;
    opt_time_marker.scale.y = 0.015;
    opt_time_marker.scale.z = 0.015;
    eq_end_marker.header.frame_id = "my_frame";
    eq_end_marker.ns = "eye_rotation";
    eq_end_marker.id = 8;
    eq_end_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    eq_end_marker.color.r = 0.0f;
    eq_end_marker.color.g = 1.0f;
    eq_end_marker.color.b = 0.0f;
    eq_end_marker.color.a = 1.0;
    eq_end_marker.scale.x = 0.015;
    eq_end_marker.scale.y = 0.015;
    eq_end_marker.scale.z = 0.015;
    circle.header.frame_id = "my_frame";
    circle.ns = "eye_rotation";
    circle.id = 3;
    circle.type = visualization_msgs::Marker::SPHERE_LIST;
    circle.color.r = 0.0f;
    circle.color.g = 0.0f;
    circle.color.b = 1.0f;
    circle.color.a = 1.0;
    circle.scale.x = 0.03;
    circle.scale.y = 0.03;
    circle.scale.z = 0.03;
    arrow.header.frame_id = "my_frame";
    arrow.ns = "eye_rotation";
    arrow.id = 1;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0;
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.1;
    arrow2.header.frame_id = "my_frame";
    arrow2.ns = "eye_rotation";
    arrow2.id = 6;
    arrow2.type = visualization_msgs::Marker::ARROW;
    arrow2.color.r = 0.0f;
    arrow2.color.g = 1.0f;
    arrow2.color.b = 0.0f;
    arrow2.color.a = 1.0;
    arrow2.scale.x = 0.015;
    arrow2.scale.y = 0.015;
    arrow2.scale.z = 0.015;
    goal.header.frame_id = "my_frame";
    goal.ns = "eye_rotation";
    goal.id = 5;
    goal.type = visualization_msgs::Marker::ARROW;
    goal.color.r = 1.0f;
    goal.color.g = 0.0f;
    goal.color.b = 0.0f;
    goal.color.a = 1.0;
    goal.scale.x = 0.01;
    goal.scale.y = 0.01;
    goal.scale.z = 0.01;
    sphere.header.frame_id = "my_frame";
    sphere.ns = "eye_rotation";
    sphere.id = 2;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.color.r = 226.0f;
    sphere.color.g = 227.0f;
    sphere.color.b = 222.0f;
    sphere.color.a = 1.0;
    sphere.scale.x = 0.79;
    sphere.scale.y = 0.79;
    sphere.scale.z = 0.785;
    sphere2.header.frame_id = "my_frame";
    sphere2.ns = "eye_rotation";
    sphere2.id = 4;
    sphere2.type = visualization_msgs::Marker::SPHERE_LIST;
    sphere2.color.r = 0.0f;
    sphere2.color.g = 1.0f;
    sphere2.color.b = 1.0f;
    sphere2.color.a = 1.0;
    sphere2.scale.x = 0.05;
    sphere2.scale.y = 0.05;
    sphere2.scale.z = 0.05;
 
    
    //Topic you want to publish
    pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    pub1 = n1.advertise<nav_msgs::Path>("quat", 10);

    //Topic you want to subscribe
    sub = n.subscribe("/insertion_points", 10, &SubscribeAndPublish::callback, this);
    sub1 = n1.subscribe("/trajectory2", 10, &SubscribeAndPublish::callback1, this);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("my_frame","/visualization_marker1"));
    
  }

 void callback1(const geometry_msgs::Pose::ConstPtr& input1)
  {
   
      geometry_msgs::PoseStamped ps;

      ps.pose.orientation = input1->orientation;	
      ps.pose.position = input1->position;   
      ps.pose.position = ps.pose.position;   		
      path.poses.push_back(ps);
      pub1.publish(path);
}
 void callback(const std_msgs::Float64MultiArray::ConstPtr& input)
  {
    //std::vector<double> data(6);
    
    //marker.action = visualization_msgs::Marker::MODIFY;
    marker.header.stamp = ros::Time::now();
    arrow.header.stamp = ros::Time::now();
    sphere.header.stamp = ros::Time::now();
    circle.header.stamp = ros::Time::now();
    sphere2.header.stamp = ros::Time::now();
    arrow2.header.stamp = ros::Time::now();
    goal.header.stamp = ros::Time::now();
    opt_time_marker.header.stamp = ros::Time::now();
    eq_end_marker.header.stamp = ros::Time::now();
    geometry_msgs::Point p_arrow_base;
    geometry_msgs::Point p_arrow_end;
    geometry_msgs::Point p_goal_base;
    geometry_msgs::Point p_goal_end;
    p_goal_base.x = 0;
    p_goal_base.y = 0;
    p_goal_base.z = 0;
    p_arrow_base.x = 0;
    p_arrow_base.y = 0;
    p_arrow_base.z = 0;
    arrow.points.push_back(p_arrow_base);
    arrow2.points.push_back(p_arrow_base);
    goal.points.push_back(p_goal_base);
    p_goal_end.x = input->data[63];
    p_goal_end.y = input->data[64];
    p_goal_end.z = input->data[65];
    goal.points.push_back(p_goal_end);
    p_arrow_end.x = input->data[36];
    p_arrow_end.y = input->data[37];
    p_arrow_end.z = input->data[38];
    arrow.points.push_back(p_arrow_end);
    p_arrow_end.x = 1.3*input->data[36];
    p_arrow_end.y = 1.3*input->data[37];
    p_arrow_end.z = 1.3*input->data[38];
    arrow2.points.push_back(p_arrow_end);
    geometry_msgs::Point sphere_center;
    sphere_center.x = 0;
    sphere_center.y = 0;
    sphere_center.z = 0;
    sphere.points.push_back(sphere_center);

    geometry_msgs::Point eq_end_center;
    eq_end_center.x = 1.3*input->data[67];
    eq_end_center.y = 1.3*input->data[68];
    eq_end_center.z = 1.3*input->data[69];
    eq_end_marker.points.push_back(eq_end_center);
      
    if (input->data[66]==2) 
{	
    geometry_msgs::Point opt_time_center;
    opt_time_center.x = 1.3*input->data[36];
    opt_time_center.y = 1.3*input->data[37];
    opt_time_center.z = 1.3*input->data[38];
    opt_time_marker.points.push_back(opt_time_center);

}     
    
    for(int i = 0; i<12; i++)
    {
      geometry_msgs::Point p;
      p.x = 10*input->data[i*3];
      p.y = 10*input->data[i*3+1];
      p.z = 10*input->data[i*3+2];
      std_msgs::ColorRGBA c;
     float nr = floor(i/2);
     if( input->data[39+nr] == 0){
        c.r = 0.0;
        c.g = 1.0;
        c.b = 0.0;
        c.a = 1.0;
     }else{ 
        c.r = 1.0;
        c.g = 0.0;
        c.b = 0.0;
        c.a = 1.0;

     }
      // Here, the field colors is populated with a specific color per point.
      marker.colors.push_back(c);	
      marker.points.push_back(p);
      circle.points.push_back(p);
     
    }
     for(int i = 0; i<6; i++)
    {

      geometry_msgs::Point p;
      
	
      p.x = 10*input->data[45 +i*3];
      
     
      p.y = 10*input->data[45 +i*3+1];
      

      p.z = 10*input->data[45 +i*3+2];
      sphere2.points.push_back(p);
}
    // Here, the field colors is populated with a specific color per point.
    
	
	  
    pub.publish(goal);	
    pub.publish(arrow);
    pub.publish(marker);
    pub.publish(sphere);
    pub.publish(circle);
    pub.publish(sphere2);
    pub.publish(arrow2);
    pub.publish(opt_time_marker);
    pub.publish(eq_end_marker);
    arrow.points.clear();
    marker.points.clear();
    sphere.points.clear();
    circle.points.clear();
    marker.colors.clear();
    sphere2.points.clear();
    eq_end_marker.points.clear();

      if (input->data[66]==1) 
{
        path.poses.clear();
opt_time_marker.points.clear();
 
       
}    
    goal.points.clear();
    arrow2.points.clear();
    //marker.action = visualization_msgs::Marker::DELETE;
  }


};//End of class SubscribeAndPublish
int main( int argc, char** argv )
{
  ros::init(argc, argv, "graphical_eye");
 /* rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  std::string name_;
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  double space_between_rows = 0.2;
  double y = 0;
  double step;
     ROS_INFO_STREAM_NAMED(name_, "Displaying Graph");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    step = 0.1;
    graph_msgs::GeometryGraph graph;
    ROS_INFO_STREAM_NAMED(name_, "Displaying 1");
    for (double i = 0; i <= 1.0; i += step)
    {
      graph.nodes.push_back(visual_tools_->convertPose(pose1).position);
      graph_msgs::Edges edges;
      ROS_INFO_STREAM_NAMED(name_, "Displaying 2");
      if (i > 0)
      {
        edges.node_ids.push_back(0);
	ROS_INFO_STREAM_NAMED(name_, "Displaying 4");
      }
      graph.edges.push_back(edges);

      if (i == 0.0)
      {
	
        Eigen::Isometry3d pose_copy = pose1;

    	pose_copy.translation().x() -= 0.2;

    	visual_tools_->publishText(pose_copy, "Graph", rvt::WHITE, rvt::XXLARGE, false);

      }

      pose1.translation().x() += step;
	ROS_INFO_STREAM_NAMED(name_, "Displaying 6");
      pose1.translation().z() += visual_tools_->dRand(-0.1, 0.1);
	ROS_INFO_STREAM_NAMED(name_, "Displaying 5");

    }
    visual_tools_->publishGraph(graph, rvt::ORANGE, 0.005);
    visual_tools_->trigger(); */
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}



