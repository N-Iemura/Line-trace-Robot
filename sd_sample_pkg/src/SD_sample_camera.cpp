//  https://wiki.ros.org/image_transport/Tutorials/SubscribingToImages をほぼそのまま使用
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

// 画像トピックを受けたときに実行されるコールバック関数
void camera_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
  try
  {
    // 画像をbgrの8bit形式に変換を試み、それを表示
    cv::imshow("view", cv_bridge::toCvShare(img_msg, "bgr8")->image);

    // 30ms表示
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    // 変換に失敗すると、メッセージを出す
    // 例:トピックがモノクロ画像だったときなど
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_sample_node");
  ros::NodeHandle nh;

  // 画像は直接pub/subするとデータ量が大きいので、image_transportを介して行うのが普通
  image_transport::ImageTransport it(nh);

  // image_transportを介して、画像のトピックのサブスクライバの定義
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, camera_callback);

  // おまじない
  ros::spin();

    return 0;
}