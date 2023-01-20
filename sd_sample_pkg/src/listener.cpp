#include "ros/ros.h"
#include "std_msgs/String.h"

// メッセージをサブスクライブしたときに呼ばれるコールバック関数
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // サブスクライバの定義. トピック名, キューのサイズ, コールバック関数
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  // ctrl+Cが押される, ros::ok()がfalseになるまでコールバック関数の処理を待ち続ける
  ros::spin();

  return 0;
}
