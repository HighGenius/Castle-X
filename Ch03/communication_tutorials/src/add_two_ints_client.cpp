#include "ros/ros.h"
#include "communication_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");	//初始化节点
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;	//创建ROS句柄
  ros::ServiceClient client = n.serviceClient<communication_tutorials::AddTwoInts>("add_two_ints"); //从add_two_ints服务端接收数据
  communication_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);	//从终端获取累加起始值
  srv.request.b = atoll(argv[2]);	//从终端获取累加到的目标值
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
