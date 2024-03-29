// RVIZ Drawing (Just for debugging)

#include <visualization_msgs/Marker.h>
#include "Navigator.h"

void Navigator::initDrawing()
{
        target_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/target", 1);
        obstacles_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/obstacles", 1);
        collision_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/collision", 1);
        desired_velocity_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker/desired_velocity", 1);
}

void Navigator::drawObstacleVelocity(geometry_msgs::PointStamped &p,geometry_msgs::Vector3 velocity, bool relevant)
{
        visualization_msgs::Marker marker;
        marker.header.frame_id = p.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id=obstacleIndex++;
        marker.ns = ns+"/obstacles";
        marker.type =visualization_msgs::Marker::ARROW;

        geometry_msgs::Point p1=p.point;
        p1.z+=0.05;
        geometry_msgs::Point p2=p1;
        p2.x+=velocity.x;
        p2.y+=velocity.y;
        p2.z+=velocity.z;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.scale.x = 0.1;
        marker.scale.y = 0.2;

        // Set the color -- be sure to set alpha to something non-zero!

        marker.color.r = 0.7f;
        marker.color.g = 0.7f;
        marker.color.b = 0.0f;
        marker.color.a = 0.4;

        if(relevant)
        {
                marker.color.a = 1.0;
        }

        marker.lifetime = ros::Duration(updatePeriod*1.5);
        // Publish the marker
        obstacles_marker_pub.publish(marker);
}

void Navigator::drawObstacle(geometry_msgs::PointStamped &p,double height,double radius,double socialMargin,bool relevant)
{
        visualization_msgs::Marker marker;
        marker.header.frame_id = p.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id=obstacleIndex++;
        marker.ns = ns+"/obstacles";
        marker.type =visualization_msgs::Marker::CYLINDER;
        marker.pose.position=p.point;
        marker.pose.position.z = p.point.z-height*0.5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = radius*2;
        marker.scale.y = radius*2;
        marker.scale.z = fmax(0.1,height);

        // Set the color -- be sure to set alpha to something non-zero!

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.4;

        if(relevant)
        {
                marker.color.a = 1.0;
        }

        marker.lifetime = ros::Duration(updatePeriod*1.5);
        // Publish the marker
        obstacles_marker_pub.publish(marker);
}


void Navigator::drawDesiredVelocity(double desiredSpeed,double desiredAngle)
{
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link"; //ns;//ns+"/base_link";
        marker.header.stamp = ros::Time::now();
        marker.id=0;
        marker.ns = ns+"/desired_velocity";
        marker.type =visualization_msgs::Marker::ARROW;


        //ROS_INFO("%.4f %.4f\n",desiredSpeed,desiredAngle);

        marker.scale.x = fmax(0.01,desiredSpeed);
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,desiredAngle);

        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        //marker.lifetime = ros::Duration(dt);
        // Publish the marker
        desired_velocity_marker_pub.publish(marker);
}

void Navigator::drawCollisionMap()
{
        visualization_msgs::Marker lines;
        lines.header.frame_id = "base_link"; //ns;//ns+"/base_link";
        lines.header.stamp = ros::Time::now();
        lines.id=0;
        lines.ns = ns+"/collisions";
        lines.type =visualization_msgs::Marker::LINE_STRIP;
        lines.pose.orientation.w = 1.0;
        lines.scale.x = 0.05;
        lines.color.r =0.0;
        lines.color.g =1.0;
        lines.color.b =0.0;
        lines.color.a=1.0f;
        lines.lifetime = ros::Duration();

        std::vector<Real> d=hlAgent.getDistances();

        double da=hlAgent.angleResolution().GetValue();
        double a=-hlAgent.aperture.GetValue();
        double r=hlAgent.radius;
        for(uint i=0; i<d.size(); i++)
        {
                if(d[i]<0) continue;
                geometry_msgs::Point p;
                p.y = (r+d[i]) * sin(a);
                p.x = (r+d[i]) * cos(a);
                lines.points.push_back(p);
                a+=da;
        }

        // Publish the marker
        collision_marker_pub.publish(lines);
}

void Navigator::drawTarget(const geometry_msgs::Point &msg, std::string frame_id)
{
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.id=0;
        marker.ns = ns+"/target";
        marker.type =visualization_msgs::Marker::CYLINDER;
        marker.pose.position=msg;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 2.0 * minDeltaDistance;
        marker.scale.y = 2.0 * minDeltaDistance;
        marker.scale.z = 0.1;

        marker.color.r =1.0;
        marker.color.g =0.0;
        marker.color.b =0.0;
        marker.color.a=1.0f;

        marker.lifetime = ros::Duration();

        // Publish the marker
        target_marker_pub.publish(marker);
}
