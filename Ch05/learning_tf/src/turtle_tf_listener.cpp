#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");  // 初始化节点

  ros::NodeHandle node; // 创建节点句柄

  ros::service::waitForService("spawn");  // 等待名为spawn的服务
  // 创建一个service client，尖括号中是服务的类型
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  // 创建该类型的srv
  turtlesim::Spawn srv;
  // 调用服务，存到srv
  add_turtle.call(srv);

  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  // 创建一个TransformListener对象,一旦监听器被创建，它开始接收tf转换，并缓冲它们长达10秒
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    // 从/turtle2坐标系开始变换到/turtle1坐标系,提供ros::Time(0)即会给出最近的可用的变换,结果存放的变换对象transform
    try{
      listener.lookupTransform("/turtle2", "/turtle1",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // 变换用于计算小海龟的新的线性和角速度，基于它与龟的距离和角度
    // 新的速度发布在话题"turtle2/cmd_vel"中，turtlesim将使用它来更新turtle2的运动
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};







