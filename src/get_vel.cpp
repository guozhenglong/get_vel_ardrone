#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

geometry_msgs::PointStamped vel_msg;
double vx, vy, vz;
double dt, h, v, h_, v_, z, p1, p2, p3, p4, p1_, p2_, p3_, p4_, k1, k2, q, r;

void PositionKalmanFilter(double &z_m)
{
    h_ = h + v * dt;
    v_ = v;

    p1_ = p1 + p2 * dt + p3 * dt + p4 * dt * dt;
    p2_ = p2 + p4 * dt;
    p3_ = p3 + p4 * dt;
    p4_ = p4 + q;

    k1 = p1_ / (p1_ + r);
    k2 = p2_ / (p1_ + r);

    h = h_ + k1 * (z_m - h_);
    v = v_ + k2 * (z_m - h_);

    p1 = p1_ * (1 - k1);
    p2 = p2_ * (1 - k1);
    p3 = -k2 * p1_ + p3_;
    p4 = -k2 * p2_ + p4_;

}

void GetVelCallback(const ardrone_autonomy::Navdata& msg)
{
    vel_msg.header.frame_id = "vel_from_ardrone";
    vel_msg.header.stamp = msg.header.stamp;
    vx = (msg.vx) / 1000.0;
    vy = -(msg.vy) / 1000.0;
    vz = (msg.altd) / 1000.0;
    PositionKalmanFilter(vz);
    vz = v;
    vel_msg.point.x = vx;
    vel_msg.point.y = vy;
    vel_msg.point.z = vz;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_vel_from_ardrone");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    double hz;
    pnh.param("hz", hz, 200.0);
    pnh.param("p1", p1, 1.0);
    pnh.param("p2", p2, 0.0);
    pnh.param("p3", p3, 0.0);
    pnh.param("p4", p4, 1.0);
    pnh.param("q", q, 0.1);
    pnh.param("r", r, 10.0);

    dt = 1.0 / hz;
    h = 0.0;
    v = 0.0;

    ros::Subscriber get_vel_drone = nh.subscribe("/ardrone1/ardrone/navdata", 1, GetVelCallback);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::PointStamped>("/vel_from_drone", 1);
    ros::Rate loopRate(hz);

    while (ros::ok())
    {
        pub_vel.publish(vel_msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}