#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "HIKRobot_MVS_SDK/MvCameraControl.h"

using namespace std;

#define SET_TriggerMode_OFF false
#define DEBUG_LOG false

void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pub_)
{

    auto* pub = (rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr *)pub_;
    if (pFrameInfo)
    {
        printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
            pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);
        #if DEBUG_LOG
        cout << "start copy to Opencv" << endl;
        #endif    
        cv::Mat frame = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData).clone();


        if (!frame.empty())
        {
            std_msgs::msg::Header _header;
            _header.stamp = rclcpp::Clock().now();
            _header.frame_id = "hikrobot_camera";
            #if DEBUG_LOG
            cout << "transform to sersor img" << endl;
            #endif    
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
              
            // cout << "publish image" << endl;
            (*pub)->publish(*img_msg.get());

            
        }
    }
}

class HIKRobotSingleCameraPublisher : public rclcpp::Node
{
public:
    explicit HIKRobotSingleCameraPublisher(const string& name) : Node(name)
    {
        find_and_select_camera();

        bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
        auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                        : rmw_qos_profile_default;
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

        #if DEBUG_LOG
        cout << "create publisher" << endl;
        #endif
    }

    ~HIKRobotSingleCameraPublisher() override
    {
        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(camHandle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            return ;
        }

        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(camHandle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            return ;
        }    
    }

    void publish_flow()
    {

        nRet = MV_OK;

        // 注册抓图回调
        // register image callback
        #if DEBUG_LOG
        cout << "register image callback" << endl;
        #endif
        nRet = MV_CC_RegisterImageCallBackForRGB(camHandle, ImageCallBackEx, &publisher_);
        if (MV_OK != nRet)
        {
            printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
            abort(); 
        }

        // ch:开始取流 | en:Start grab image
        #if DEBUG_LOG
        cout << "start grab image" << endl;
        #endif
        nRet = MV_CC_StartGrabbing(camHandle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            abort();
        }

        PressEnterToExit();


        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(camHandle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            abort();
        }

    }



private:
    int nRet = MV_OK;
    void* camHandle = nullptr;
    unsigned int g_nPayloadSize = 0;
    bool g_bExit = false;

    image_transport::CameraPublisher pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;


    
    void find_and_select_camera()
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);


        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            abort();
        }

            // ch:打印设备信息 | en:Print device information
        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (nullptr == pDeviceInfo)
                {
                    abort();
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            abort();
        }

        // ch:选择设备,        

        // #if DEBUG_LOG
        // cout << "select device" << endl;
        // #endif

        unsigned int nIndex = 0;
        
        if (stDeviceList.nDeviceNum > 1)
        {
            printf("Please Intput camera index:");
            scanf("%d", &nIndex);
        }
        else if (stDeviceList.nDeviceNum == 1)
        {
            printf("Only one camera, select it automatically\n");
        }
        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            abort();
        }

        // ch:选择设备并创建句柄 | en:Select device and create handle
        #if DEBUG_LOG
        cout << "create handle" << endl;
        #endif
        nRet = MV_CC_CreateHandle(&camHandle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            abort();
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(camHandle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            abort();
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(camHandle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(camHandle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        #if SET_TriggerMode_OFF
        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(camHandle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }
        #endif

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(camHandle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            abort();
        }
        g_nPayloadSize = stParam.nCurValue;

    }

    

    // 等待用户输入enter键来结束取流或结束程序
    // wait for user to input enter to stop grabbing or end the sample program
    void PressEnterToExit()
    {
        int c;
        while ( (c = getchar()) != '\n' && c != EOF );
        fprintf( stderr, "\nPress enter to exit.\n");
        while( getchar() != '\n');
        g_bExit = true;
        sleep(1);
    }

    static bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (nullptr == pstMVDevInfo)
        {
            printf("The Pointer of pstMVDevInfo is NULL!\n");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        }
        else
        {
            printf("Not support.\n");
        }

        return true;
    }


};




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<HIKRobotSingleCameraPublisher>("single_HIKRobot_camera_node");
    RCLCPP_INFO(node->get_logger(), "HIKRobot_Camera_Driver_Node is running...");
    node->publish_flow();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}