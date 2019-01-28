#include <ros/ros.h>

// 使用行为库actionlib/server/simple_action_server.h
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // 必须在此行之前创建NodeHandle实例,否则会发生错误
  std::string action_name_;
  // 创建用于发布反馈/结果的消息
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:

  FibonacciAction(std::string name) :
    // 行为服务器会得到一个节点句柄（nh）、行为名称和选择一个运行回调函数（executeCB）参数
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  // 回调函数executeCB会传递一个指向目标消息的boost共享指针，在目标消息类型最后附加一个给定的"ConstPtr"
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // # 附加种子用于fibonacci序列
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // 发布信息到控制台，用于用户查看
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // 开始运行行为
    for(int i=1; i < goal->order; i++)
    {
      // 检查客户端是否未请求抢占
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // 将动作状态设置为“抢占”
        as_.setPreempted();
        success = false;
        break;
      }

      // 计算fibonacci序列并反馈回去
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // 发布反馈(feedback)
      as_.publishFeedback(feedback_);
      // 这个步骤不是必要的，用于演示效果所以序列会以1Hz完成。
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // 将动作状态设置为“成功”
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  // 初始化节点
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();	// 回调函数

  return 0;
}
