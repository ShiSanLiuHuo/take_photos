#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include "mindvision.hpp"
#include "inner_shot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/videoio.hpp>
#include <mutex>

namespace sensor {

struct ROI{
    int x = 0;
    int y = 0;
    int w = 720;
    int h = 720;
};

class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);

private:
    /**
     * @brief 发布图像
     */
    void LoopForPublish();

    /**
     * @brief 调整曝光
     */
    void TuneExposure();

    /**
     * @brief 通过trackbar设置曝光时间
     */
    void SetExposureTimeWithTrack();

    /**
     * @brief 保存曝光参数
     */
    // void CameraNode::ExposureCallback();

    /**
     * @brief 获取图像保存到 frame，从相机或者视频流
     */
    void GetImg();

    /**
     * @brief 开启内录节点
     */
    void InnerShot();

    /**
     * @brief 显示图像并按键保存
     */
    void SaveImg();

    /**
     * @brief 发布相机内参
     */

    void PublicCameraInfo();


    void timer();
    std::string generateFrameName() ;
    

    // 保存从摄像头获取的图像
    std::shared_ptr<cv::Mat> frame_;

    std::string enemy_color_flag_;
//TODO:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_for_radar_;

    // 是否外部输入视频流标志位
    bool videoflag;
    std::string video_path;
    std::shared_ptr<MindVision> mindvision_;
    cv::VideoCapture capture;
    std::thread thread_for_publish_;    //获取图像的线程
    std::thread thread_for_save_;       //显示与保存线程
    std::thread thread_for_inner_shot_; //获取图像的线程
    bool inner_shot_flag;
    int exposure_time;
    int gain;
    bool rosbag_flag;

    bool debug_exposure = false;
    int failed_count;
    std::atomic<int> frame_count_{0};
    int count = 1;
    bool preview_enable_{true};
    bool tune_on_startup_{false};
    std::string save_dir_;
    std::mutex frame_mutex_;
};

} // namespace sensor

#endif // CAMERA_NODE_HPP
