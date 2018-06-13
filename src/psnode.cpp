#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

#include "phasespace/owl.hpp"

#include "phasespace/Camera.h"
#include "phasespace/Cameras.h"
#include "phasespace/Marker.h"
#include "phasespace/Markers.h"

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

    std::string address;

    ros::Publisher errorsPub = nh.advertise<std_msgs::String>("phasespace_errors", 1000, true);
    ros::Publisher camerasPub = nh.advertise<phasespace::Cameras>("phasespace_cameras", 1000, true);
    ros::Publisher markersPub = nh.advertise<phasespace::Markers>("phasespace_markers", 1000);

    static tf::TransformBroadcaster br;
    std::string base_frame = "phasespace_base";

    // simple example
    if (!nh.getParam("/server_ip", address) || owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0)
        ROS_INFO("Connection failed!!");
        return 0;

    // start streaming
    ROS_INFO("Starting streaming...");
    owl.streaming(1);

    // main loop
    while (ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
    {
        const OWL::Event *event = owl.nextEvent(1000);
        if (!event)
            continue;

        if (event->type_id() == OWL::Type::ERROR)
        {
            std::cerr << event->name() << ": " << event->str() << std::endl;
            std_msgs::String str;
            str.data = event->str();
            errorsPub.publish(str);
        }
        else if (event->type_id() == OWL::Type::CAMERA)
        {
            if (event->name() == std::string("cameras") && event->get(cameras) > 0)
            {
                phasespace::Cameras out;
                for (OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++)
                {
                    phasespace::Camera cam;
                    //
                    cam.id = c->id;
                    cam.flags = c->flags;
                    cam.x = c->pose[0];
                    cam.y = c->pose[1];
                    cam.z = c->pose[2];
                    cam.qw = c->pose[3];
                    cam.qx = c->pose[4];
                    cam.qy = c->pose[5];
                    cam.qz = c->pose[6];
                    cam.cond = c->cond;
                    //
                    out.cameras.push_back(cam);

                    tf::Transform transform;
                    transform.setOrigin( tf::Vector3(c->pose[0], c->pose[1], c->pose[2]) );
                    transform.setRotation( tf::Quaternion(c->pose[4], c->pose[5], c->pose[6], c->pose[3]) );
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame, std::string("camera_") + std::to_string(c->id)));
                }
                camerasPub.publish(out);
            }
        }
        else if (event->type_id() == OWL::Type::FRAME)
        {
            if (event->find("markers", markers) > 0)
            {
                phasespace::Markers out;
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                    phasespace::Marker mout;
                    mout.id = m->id;
                    mout.time = m->time;
                    mout.flags = m->flags;
                    mout.cond = m->cond;
                    mout.x = m->x;
                    mout.y = m->y;
                    mout.z = m->z;
                    out.markers.push_back(mout);
                }

                markersPub.publish(out);
            }
        }
    } // while

    owl.done();
    owl.close();

    return 0;
}
