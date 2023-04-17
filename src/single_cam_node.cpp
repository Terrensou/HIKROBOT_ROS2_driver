#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdio>
#include <cstring>
#include "HIKRobot_MVS_SDK/MvCameraControl.h"
#include "hkcamera.hpp"

using namespace std;

void __stdcall ImageCallBackEx(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pub_) {

    auto *pub = (rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr *) pub_;
    if (pFrameInfo) {
        // printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
        //     pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
#if DEBUG_LOG
        cout << "start copy to Opencv" << endl;
#endif
        cv::Mat frame = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData).clone();
        // cout << pFrameInfo->nDevTimeStampHigh << '\n' << endl;
        //     cout << pFrameInfo->nDevTimeStampLow << '\n' << endl;
        //     cout << pFrameInfo->nHostTimeStamp << '\n' << endl;
        //     cout << pFrameInfo->nSecondCount << '\n' << endl;


        if (!frame.empty()) {
            std_msgs::msg::Header _header;

            _header.stamp = rclcpp::Clock().now();
            // const std::time_t t_c = std::chrono::system_clock::to_time_t(_header.stamp);
            // std::cout << "The system clock is currently at " << std::ctime(&t_c);

            _header.frame_id = "hikrobot_camera";
#if DEBUG_LOG
            cout << "transform to sersor img" << endl;
#endif
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::RGB8,
                                                                            frame).toImageMsg();

            (*pub)->publish(*img_msg.get());
            // sleep(10000000000000000);


        }
    }
}

class HIKRobotSingleCameraPublisher : public rclcpp::Node {
public:
    explicit HIKRobotSingleCameraPublisher(const string &name) : Node(name) {
        devices = new HKCamera_set;
        devices->FindDevices();
        devices->PrintDevicesInfo();
        cam = devices->SelectDevice();

        bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
        auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                       : rmw_qos_profile_default;
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

#if DEBUG_LOG
        cout << "create publisher" << endl;
#endif
    }

    ~HIKRobotSingleCameraPublisher() override {
        delete cam;
        delete devices;
    }

    void publish_flow() {
        printf("Start Grabbing!\n");
        cam->RegisterFlowCallBackForRGB(ImageCallBackEx, &publisher_);

    }


private:
    int nRet = MV_OK;
    HKCamera_set *devices = nullptr;
    HKCamera *cam = nullptr;

    image_transport::CameraPublisher pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};


};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<HIKRobotSingleCameraPublisher>("single_HIKRobot_camera_node");
    RCLCPP_INFO(node->get_logger(), "HIKRobot_Camera_Driver_Node is running...");
    node->publish_flow();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}