// rosの基本ヘッダファイルと, ここで使うメッセージの型を定義するファイルをインクルード
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  // rosノードの起動. ノード名"talker"
  ros::init(argc, argv, "talker");

  // rosシステムのとの通信を行うノードハンドラの定義
  ros::NodeHandle n;

  // rosのパブリッシャの定義. メッセージ型, トピック名, キューのサイズ
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // ノードの処理周期
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    // メッセージ型の変数の定義
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    // 端末に出力
    ROS_INFO("%s", msg.data.c_str());

    // メッセージをパブリッシュ
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
