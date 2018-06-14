#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "phasespace_ros/owl.hpp"

#include "phasespace_ros/Camera.h"
#include "phasespace_ros/Cameras.h"
#include "phasespace_ros/Marker.h"
#include "phasespace_ros/Markers.h"

//using namespace std;

int main(int argc, char **argv)
{
    // get the owl server address through the command line
    // 'rosrun phasespace phasespace_node 192.168.1.123'
    //string address = argc > 1 ? argv[1] : "localhost";

    // create the context
    OWL::Context owl;
    OWL::Markers markers;
    OWL::Cameras cameras;

    // initialize ROS
    ros::init(argc, argv, "phasespace_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // declare publishers
    ros::Publisher errors_pub = nh.advertise<std_msgs::String>("phasespace_errors", 1000, true);
    //ros::Publisher camerasPub = nh.advertise<phasespace_ros::Cameras>("phasespace_cameras", 1000, true);
    ros::Publisher markers_pub = nh.advertise<phasespace_ros::Markers>("phasespace_markers", 1000);
    ros::Publisher vis_marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    tf::TransformBroadcaster br;
    std::string base_frame = "phasespace_base";
    std::vector<std::pair<int, tf::Transform>> transforms;

    // load ip address from ros param
    std::string address;
    if (pnh.getParam("server_ip", address))
    {
        ROS_INFO("Opening port at: %s", address.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get ip address!!");
        return 0;
    }

    // simple example
    if (owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0)
    {
        ROS_ERROR("Connection failed!!");
        return 0;
    }

    // start streaming
    ROS_INFO("Starting streaming...");
    owl.streaming(1);
    
    ros::Rate r(1000);
    // main loop
    while (ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
    {
        // std::cout << owl.isOpen() << owl.property<int>("initialized") << std::endl;
        const OWL::Event *event = owl.nextEvent(1000);
        // publish camera frames if available 
        if (transforms.size() != 0)
        {
            for (auto & t: transforms)
            {
                br.sendTransform(tf::StampedTransform(t.second, ros::Time::now(), base_frame, std::string("camera_") + std::to_string(t.first)));
            }
        }

        if (!event)
            continue;

        if (event->type_id() == OWL::Type::ERROR)
        {
            std::cerr << event->name() << ": " << event->str() << std::endl;
            std_msgs::String str;
            str.data = event->str();
            ROS_ERROR(event->str());
            errors_pub.publish(str);
        }
        else if (event->type_id() == OWL::Type::CAMERA)
        {
            if (event->name() == std::string("cameras") && event->get(cameras) > 0)
            {
                transforms.clear();
                for (OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++)
                {
                    // TODO: check cond or flag
                    // publish tf frames
                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(c->pose[0]/1000, c->pose[1]/1000, c->pose[2]/1000) );
                    transform.setRotation( tf::Quaternion(c->pose[4], c->pose[5], c->pose[6], c->pose[3]) );
                    transforms.push_back( std::make_pair(c->id, transform) );
                }
            }
        }
        else if (event->type_id() == OWL::Type::FRAME)
        {
            if (event->find("markers", markers) > 0)
            {
                phasespace_ros::Markers out;
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                    phasespace_ros::Marker mout;
                    mout.id = m->id;
                    mout.time = m->time;
                    mout.flags = m->flags;
                    mout.cond = m->cond;
                    mout.point.x = m->x;
                    mout.point.y = m->y;
                    mout.point.z = m->z;
                    out.markers.push_back(mout);
                }

                markers_pub.publish(out);

                visualization_msgs::Marker points;
                points.header.frame_id = base_frame;
                points.header.stamp = ros::Time::now();
                points.ns = "vis_markers";
                points.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = 1.0;
                points.id = 0;
                points.type = visualization_msgs::Marker::SPHERE_LIST;
                // POINTS markers use x and y scale for width/height respectively
                points.scale.x = 0.1;
                points.scale.y = 0.1;
                points.scale.z = 0.1;

                // Points are green
                points.color.r = 1.0f;
                points.color.a = 1.0f;
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                    geometry_msgs::Point p;
                    p.x = m->x / 1000;
                    p.y = m->y / 1000;
                    p.z = m->z / 1000;
                    points.points.push_back(p);

                }
                vis_marker_pub.publish(points);
            }
        }

        r.sleep();
    } // while

    owl.done();
    owl.close();

    return 0;
}
