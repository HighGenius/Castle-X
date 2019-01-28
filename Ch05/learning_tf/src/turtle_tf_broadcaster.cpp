#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;  

void poseCallback(const turtlesim::PoseConstPtr& msg){
  // 创建一个TransformBroadcaster对象，将使用它通过线路发送转换
  static tf::TransformBroadcaster br; 
  // 创建一个Transform对象，并将信息从2D乌龟姿势复制到3D变换中
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q; 
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q); // 设置旋转
  // 使用TransformBroadcaster发送转换需要四个参数；
  // 使用当前时间戳ros::Time::now()；
  // 传递创建的链接父框架的名称world；
  // 传递正在创建链接的子框架的名称turtle_name
  // sendTransform和StampedTransform具有父对象和子对象的相反顺序
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster"); // 初始化节点
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

