#include "ros/ros.h"
#include "communication_tutorials/AddTwoInts.h"

//这个函数为int值求和的服务，int值从request里面获取，而返回数据装入response内，函数返回一个boolean值。
bool cum(communication_tutorials::AddTwoInts::Request  &req,
         communication_tutorials::AddTwoInts::Response &res)
{
  //输出在终端输入的两个累加值
  ROS_INFO("request: the starting value:%ld, the target value:%ld", (long int)req.a, (long int)req.b);

  //if条件判断，如果输入第一个比第二个数小才执行累加函数
  if(req.a < req.b)
  {
    // 累加函数
    for( int i = req.a; req.b > i; req.b = req.b - 1 )

      {
        req.a = req.a + req.b;
      }
    res.sum = req.a;
    //输出累加之后的结果
    ROS_INFO("Cumulative results: %ld", (long int)res.sum);
  }
  else
  {
    ROS_INFO("The number is wrong,please enter again!\n");
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server"); //初始化节点
  ros::NodeHandle n;  //创建节点句柄

  ros::ServiceServer service = n.advertiseService("add_two_ints", cum); //向ROS Master注册add_two_ints节点，使该节点具有给定名称的RPC服务
  ROS_INFO("Ready to sum ints.");
  ros::spin();  //回调函数

  return 0;
}

