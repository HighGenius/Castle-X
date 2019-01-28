#include <ros/ros.h>

// actionlib/client/simple_action_client.h 是行为库，从简单行为客户端实现
#include <actionlib/client/simple_action_client.h>
// actionlib/client/terminal_state.h 定义了目标可能的状态
#include <actionlib/client/terminal_state.h>
// 从Fibonacci.action中生成的行为消息
#include <actionlib_tutorials/FibonacciAction.h>


int main (int argc, char **argv)
{
  int a;

  ros::init(argc, argv, "test_fibonacci");
  // 检查终端调用的指令是否正确
  if (argc != 2)
  {
    ROS_INFO("usage: Enter a target number after calling the command");
    return 1;
  }

  // 行为客户端的定义模板，指定和行为服务器通信的消息类型
  // 行为客户端的构造函数需要两个参数，包括用于连接的行为服务器名称和一个布尔类型的选项用于自动循环线程
  // 成功会开启客户端创建自己的线程
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // 等待行为服务器开启
  // 如果行为服务端没有启动或运行，那么行为客户端会在进行下一步之前等待行为服务器
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // 设置目标值并且发送给行为服务端
  actionlib_tutorials::FibonacciGoal goal;

  a = atoll(argv[1]);	//从终端获取goal的值
  goal.order = a;
  ac.sendGoal(goal);

  // 等待行为返回
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return 0;
}
