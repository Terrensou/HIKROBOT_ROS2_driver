#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "HIKRobot_MVS_SDK/MvCameraControl.h"

using namespace std;

void __PressEnterToExitGrabFlow(void);

class HKCamera {
protected:
    friend void *__CallBackThread(void *pCam_t);

public:
    HKCamera(MV_CC_DEVICE_INFO *pDeviceInfo) {
        camInfo = pDeviceInfo;
        __CreateHandle(pDeviceInfo);
        SetOptimalPacketSize();
        __OpenCamera();

    }

    ~HKCamera() {
        if (NULL == camHandle) {
            return;
        }

        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(camHandle);
        if (MV_OK != nRet) {
            printf("Close Device fail! nRet [0x%x]\n", nRet);
            return;
        }

        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(camHandle);
        if (MV_OK != nRet) {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
        }
    }

    bool GetHandle(void *&handle) {
        if (camHandle == NULL)
            return false;
        handle = camHandle;
        return true;
    }

    unsigned int GetPayloadSize() {
        return g_nPayloadSize;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    void SetOptimalPacketSize() {
        if (camInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(camHandle);
            if (nPacketSize > 0) {
                nRet = MV_CC_SetIntValue(camHandle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK) {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            } else {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
    }

    void RegisterFlowCallBackForRGB(
            void(__stdcall *FlowCallBackEx)(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser),
            void *pUser) {

        // pthread_t nThreadID;
        // nRet = pthread_create(&nThreadID, NULL, __CallBackThread ,(void *)this);
        // if (nRet != 0)
        // {
        //     printf("thread create failed.ret = %d\n",nRet);
        //     abort();
        // }


        int nRet = MV_OK;
        nRet = MV_CC_RegisterImageCallBackForRGB(camHandle, FlowCallBackEx, pUser);
        if (MV_OK != nRet) {
            printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
            return;
        }

        // ch:开始取流 | en:Start grab image
#if DEBUG_LOG
        cout << "start grab image" << endl;
#endif
        nRet = MV_CC_StartGrabbing(camHandle);
        if (MV_OK != nRet) {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            abort();
        }

        __PressEnterToExitGrabFlow();


        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(camHandle);
        if (MV_OK != nRet) {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        }

    }

    void SaveDeviceInfo(char *filename) {
        // ch:将相机属性导出到文件中 | en:Export the camera properties to the file
        nRet = MV_CC_FeatureSave(camHandle, filename);
        if (MV_OK != nRet) {
            printf("Save Feature fail! nRet [0x%x]\n", nRet);
            return;
        }
        printf("Finish export the camera properties to the file\n\n");

    }

    void LoadDeviceInfo(const char *filename) {
        // ch:从文件中导入相机属性 | en:Import the camera properties from the file
        printf("Start import the camera properties from the file\n");
        nRet = MV_CC_FeatureLoad(camHandle, filename);
        if (MV_OK != nRet) {
            printf("Load Feature fail! nRet [0x%x]\n", nRet);
            return;
        }
        printf("Finish import the camera properties from the file\n");
    }

private:
    int nRet = MV_OK;
    void *camHandle = NULL;
    unsigned int g_nPayloadSize = 0;
    bool g_bExit = false;
    MV_CC_DEVICE_INFO *camInfo = NULL;


    // ch:获取数据包大小 | en:Get payload size
    bool __DetectPayloadSize() {
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(camHandle, "PayloadSize", &stParam);
        if (MV_OK != nRet) {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return false;
        }
        g_nPayloadSize = stParam.nCurValue;
        return true;
    }

    bool __OpenCamera() {
        if (NULL == camHandle) {
            printf("The Pointer of Camera is NULL!\n");
            return false;
        }

        nRet = MV_CC_OpenDevice(camHandle);
        if (MV_OK != nRet) {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            return false;
        }

        // // ch:设置触发模式为off | en:Set trigger mode as off
        // nRet = MV_CC_SetEnumValue(camHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        // if (MV_OK != nRet)
        // {
        //     printf("Set TriggerMode fail! nRet [0x%x]\n", nRet);
        //     return false;
        // }

        // ch:获取数据包大小 | en:Get payload size
        if (!__DetectPayloadSize()) {
            return false;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        SetOptimalPacketSize();

        return true;


    }

    bool __CreateHandle(MV_CC_DEVICE_INFO *pstMVDevInfo) {
        if (NULL == pstMVDevInfo) {
            printf("Device Information is NULL!\n");
            return false;
        }
        nRet = MV_CC_CreateHandle(&camHandle, pstMVDevInfo);
        if (MV_OK != nRet) {
            printf("Create Camera Handle fail! nRet [0x%x]\n", nRet);
            return false;
        }
        return true;
    }


    // 等待用户输入enter键来结束取流或结束程序
    // wait for user to input enter to stop grabbing or end the sample program


};


class HKCamera_set {
public:
    HKCamera_set() {
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // cout << "find devices()" << endl;
        if (!FindDevices()) abort();
        PrintDevicesInfo();

    }

    ~HKCamera_set() {
        if (stDeviceList.nDeviceNum > 0) {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                free(stDeviceList.pDeviceInfo[i]);
            }
        }
        free(stDeviceList.pDeviceInfo);
    }


    bool FindDevices() {
        // ch:枚举设备 | en:Enum device 
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        // cout << "nRet = " << nRet << endl;
        if (MV_OK != nRet) {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            return false;
        }
        return true;
    }


    // ch:打印设备信息 | en:Print device information
    bool PrintDevicesInfo() {
        if (__CheckEmpty()) return false;
        if (stDeviceList.nDeviceNum > 0) {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo) {
                    printf("No Device found.\n");
                    return false;
                }
                printf("[device %d]:\n", i);
                if (__PrintCameraInfo(pDeviceInfo)) {
                    return false;
                }
            }
        } else {
            printf("Find No Devices!\n");
            return false;
        }

        return true;
    }

    // ch:选择设备, 
    HKCamera *SelectDevice(int nIndex = -1) {
        if (__CheckEmpty()) return NULL;
        if (nIndex < 0) {
            if (stDeviceList.nDeviceNum > 1) {
                printf("Please Intput camera index:");
                scanf("%d", &nIndex);
            } else if (stDeviceList.nDeviceNum == 1) {
                printf("Only one camera, select it automatically\n");
                nIndex = 0;
            }
        }
        printf("Device = %d \n", nIndex);
        if (nIndex >= stDeviceList.nDeviceNum) {
            printf("Intput error!\n");
            return NULL;
        }
        return new HKCamera(stDeviceList.pDeviceInfo[nIndex]);
    }

private:
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList; // ch:设备列表 | en:Device list

    // ch:判断设备列表是否为空 | en:Check whether the device list is empty
    bool __CheckEmpty() {
        if (stDeviceList.nDeviceNum == 0) {
            printf("No Device found!\n");
            return true;
        }
        return false;
    }

    bool __PrintCameraInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
            printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        } else {
            printf("Not support.\n");
        }

        return true;
    }

};


void __PressEnterToExitGrabFlow(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n');
}

void *__CallBackThread(void *pCam_t) {
    HKCamera *pCam = (HKCamera *) pCam_t;

    // 注册抓图回调
    // register image callback
#if DEBUG_LOG
    cout << "register image callback" << endl;
#endif
    int nRet = MV_OK;
    // nRet = MV_CC_RegisterImageCallBackForRGB(pCam->camHandle, FlowCallBackEx, &pUser);
    if (MV_OK != nRet) {
        printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
        abort();
    }

    // ch:开始取流 | en:Start grab image
#if DEBUG_LOG
    cout << "start grab image" << endl;
#endif
    nRet = MV_CC_StartGrabbing(pCam->camHandle);
    if (MV_OK != nRet) {
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
        abort();
    }
    __PressEnterToExitGrabFlow();


    // end grab image
    nRet = MV_CC_StopGrabbing(pCam->camHandle);
    if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    }

    return 0;
}