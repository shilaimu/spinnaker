/**
 *  @ WJH Acquisition.cpp
 *  # Mulity thread
 *  # Mulity camera
 *  # Soft trigger
 *  # Image parameter set
 */
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include <fstream>
#include <iomanip>


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef _WIN32
#include <pthread.h>
#endif

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

int err=0;

// 相机参数
double exposureTimeToSet;
double framerateToSet;
struct ImageSettingConfig
{
public:
	int64_t Offset_X;
	int64_t Offset_Y;
	int64_t Height;
	int64_t Weight;
}imagesettingconfig;
int bingningToSet = 4;

// 设置times.txt
string addr = "/home/wjh/Projects/spinnaker/data/";
string timeStampDir = addr + "/times.txt";
ofstream fileTime(timeStampDir, ofstream::out);
//fileTime << setiosflags(ios::fixed);

// 定义全局变量，是否使用软件触发以及硬件触发的标志位
enum triggerType
{
    SOFTWARE,
    HARDWARE
};
const triggerType chosenTrigger = SOFTWARE;

// 打印设备信息
int PrintDeviceInfo(INodeMap& nodeMap, std::string camSerial)
{
    int result = 0;

    cout << "[" << camSerial << "] Printing device information ..." << endl << endl;

    FeatureList_t features;
    CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category))
    {
        category->GetFeatures(features);

        FeatureList_t::const_iterator it;
        for (it = features.begin(); it != features.end(); ++it)
        {
            CNodePtr pfeatureNode = *it;
            CValuePtr pValue = (CValuePtr)pfeatureNode;
            cout << "[" << camSerial << "] " << pfeatureNode->GetName() << " : "
                 << (IsReadable(pValue) ? pValue->ToString() : "Node not readable") << endl;
        }
    }
    else
    {
        cout << "[" << camSerial << "] "
             << "Device control information not available." << endl;
    }

    cout << endl;

    return result;
}

// 设置触发函数,触发方式设为off以便在调用时进行设置，设置后生效，触发一次采集一张图片
int ConfigureTrigger(INodeMap& nodeMap)
{
    int result = 0;

    cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;

    if (chosenTrigger == SOFTWARE)
    {
        cout << "Software trigger chosen..." << endl;
    }
    else if (chosenTrigger == HARDWARE)
    {
        cout << "Hardware trigger chosen..." << endl;
    }

    try
    {
        // *** NOTES ***
        // 1、*******trigger mode必须先禁用才能设置为software和hardware*******//
        CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
        {
            cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
        if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
        {
            cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

        cout << "Trigger mode disabled..." << endl;


        // 2、选择触发方式
        CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
        if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
        {
            cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
            return -1;
        }

        if (chosenTrigger == SOFTWARE)
        {
            // 设置为软件触发方式
            CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
            if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
            {
                cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
                return -1;
            }

            ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());

            cout << "Trigger source set to software..." << endl;
        }
        else if (chosenTrigger == HARDWARE)
        {
            // Set trigger mode to hardware ('Line0')
            CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
            if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
            {
                cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
                return -1;
            }

            ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());

            cout << "Trigger source set to hardware..." << endl;
        }

        // *** NOTES ***
        // 3、触发模式设置为“on“
        CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
        if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
        {
            cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

        // TODO: Blackfly and Flea3 GEV cameras need 1 second delay after trigger mode is turned on

        cout << "Trigger mode turned back on..." << endl << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// 关闭触发模式
int ResetTrigger(INodeMap& nodeMap)
{
    int result = 0;

    try
    {
        //
        // 将触发模式设置为”OFF“
        //
        // *** NOTES ***
        // 使用后需要将触发模式复位
        CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
        {
            cout << "Unable to disable trigger mode (node retrieval). Non-fatal error..." << endl;
            return -1;
        }

        CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
        if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
        {
            cout << "Unable to disable trigger mode (enum entry retrieval). Non-fatal error..." << endl;
            return -1;
        }

        ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

        cout << "Trigger mode disabled..." << endl << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// 使用触发模式采集一张图像
int GrabNextImageByTrigger(INodeMap& nodeMap, CameraPtr pCam)
{
    int result = 0;

    try
    {
        //使用触发模式采集一张图像
        // *** NOTES ***
        // 设置软件触发方式（原方式为“enter”键）
        if (chosenTrigger == SOFTWARE)
        {
            // Get user input
            cout << "software trigger..." << endl;
            //getchar();

            // Execute software trigger
            CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
            if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand))
            {
                cout << "Unable to execute trigger. Aborting..." << endl;
                return -1;
            }

            ptrSoftwareTriggerCommand->Execute();

            // TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger
        }
        else if (chosenTrigger == HARDWARE)
        {
            // Execute hardware trigger
            cout << "Use the hardware to trigger image acquisition." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    cout<<"GrabNextImageByTrigger_end"<<endl;
    return result;
}

// 首先设置“ExposureAuto = Off”，然后自定义曝光时间（单位：ms）
int ConfigureExposure(INodeMap& nodeMap, double exposureTimeToSet)
{
    int result = 0;

    cout << endl << endl << "*** CONFIGURING EXPOSURE ***" << endl << endl;

    try
    {
        // 关闭自动曝光
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
        {
            cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
        if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
        {
            cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
            return -1;
        }

        ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());

        cout << "Automatic exposure disabled..." << endl;

        // Set exposure time manually; exposure time recorded in microseconds
        CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
        if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
        {
            cout << "Unable to set exposure time. Aborting..." << endl << endl;
            return -1;
        }

        // Ensure desired exposure time does not exceed the maximum
        const double exposureTimeMax = ptrExposureTime->GetMax();
        // 手动设置曝光值
        //double exposureTimeToSet = 2000000.0;

        if (exposureTimeToSet > exposureTimeMax)
        {
            exposureTimeToSet = exposureTimeMax;
        }

        ptrExposureTime->SetValue(exposureTimeToSet);

        cout << std::fixed << "Exposure time set to " << exposureTimeToSet << " us..." << endl << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// 重置为”Outo Exposure“
int ResetExposure(INodeMap& nodeMap)
{
    int result = 0;

    try
    {
       	// 重置为”Outo Exposure“
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
        {
            cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
            return -1;
        }

        CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
        if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
        {
            cout << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
            return -1;
        }

        ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());

        cout << "Automatic exposure enabled..." << endl << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}
 
// 设置帧率AcquisitionFrameRate，首先设置AcquisitionFrameRateEnable = enabled
int ConfigureFrameRate(INodeMap& nodeMap, double framerateToSet)
{
	int result = 0;

    cout << endl << endl << "*** CONFIGURING FRAMERATE ***" << endl << endl;
	try
	{
		// 关闭自动帧率
	    CBooleanPtr ptrFrameRateEnable = nodeMap.GetNode("AcquisitionFrameRateEnable");
	    if (!IsAvailable(ptrFrameRateEnable) || !IsWritable(ptrFrameRateEnable))
	    {
	        cout << "Unable to enable AcquisitionFrameRate. Aborting..." << endl << endl;
	        return -1;
	    }

	    ptrFrameRateEnable->SetValue(true);
	    cout  << "ptrFrameRateEnable set to " << "enable"  << endl << endl;

		CFloatPtr ptrFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
	    // Ensure desired fps does not exceed the maximum
	    const double framerateMax = ptrFrameRate->GetMax();
	    
	    // 手动设置fps    
	    if (framerateToSet > framerateMax)
	    {
	        framerateToSet = framerateMax;
	    }

	    ptrFrameRate->SetValue(framerateToSet);

	    cout  << "AcquisitionFrameRate set to " << framerateToSet << " Hz..." << endl << endl;
	}
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}


// 设置自定义图像格式 offsets X and Y, width, height
// 需要在BeginAcquisition()前执行，执行立即生效 
int ConfigureCustomImageSettings(INodeMap& nodeMap, ImageSettingConfig value, int bingning_value)
{
    int result = 0;

    cout << endl << endl << "*** 设置自定义图像格式 ***" << endl << endl;

    try
    {
        // 应用 RGB8Packed 像素格式
        // 获取枚举节点
        CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
        if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
        {
            // 从枚举节点中获取想要的入口点
            CEnumEntryPtr ptrPixelFormatBayerRGB = ptrPixelFormat->GetEntryByName("RGB8Packed");	//Mono8,Mono16，YUV422Packed,BayerRG8...
            if (IsAvailable(ptrPixelFormatBayerRGB) && IsReadable(ptrPixelFormatBayerRGB))
            {
                // Retrieve the integer value from the entry node
                int64_t pixelFormatBayerRGB = ptrPixelFormatBayerRGB->GetValue();

                // Set integer as new value for enumeration node
                ptrPixelFormat->SetIntValue(pixelFormatBayerRGB);

                cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
                cout << endl;
            }
            else
            {
                cout << "Pixel format BayerRGB not available..." << endl;
            }
        }
        else
        {
            cout << "Pixel format not available..." << endl;
        }

        //设置binning格式=4;相当于resize
        CIntegerPtr ptrBinNingV = nodeMap.GetNode("BinningVertical");
        if (IsAvailable(ptrBinNingV) && IsWritable(ptrBinNingV))
        {
        	ptrBinNingV->SetValue(bingning_value);
        	cout << "BinningVertical set to " << ptrBinNingV->GetValue() << "..." << endl;
        	cout << endl;
        }
        else
        {
            cout << "BinningVertical not available..." << endl;
        }
        CIntegerPtr ptrBinNingH = nodeMap.GetNode("BinningHorizontal");
        if (IsAvailable(ptrBinNingH) && IsWritable(ptrBinNingH))
        {
        	ptrBinNingH->SetValue(bingning_value);
        	cout << "BinningVertical set to " << ptrBinNingH->GetValue() << "..." << endl;
        	cout << endl;
        }
        else
        {
            cout << "BinningVertical not available..." << endl;
        }


        // 设置offset X
        // *** NOTES ***
        // 有些参数有增量限制，意味着并不是任何值都适用，可以用getInc()获取增量值
        CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
        if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX))
        {
        	cout << "Offset X 增量为 " << ptrOffsetX->GetInc() <<  endl;
        	cout << "Offset X max为 " << ptrOffsetX->GetMax() <<  endl;
        	cout << "Offset X min为 " << ptrOffsetX->GetMin() <<  endl;
        	
            ptrOffsetX->SetValue(value.Offset_X);
            //ptrOffsetX->SetValue(ptrOffsetX->GetMin());
            cout << "Offset X set to " << ptrOffsetX->GetValue() << "..." << endl;
            cout << endl;
        }
        else
        {
            cout << "Offset X not available..." << endl;
        }

        // 设置offset Y
        CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
        if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY))
        {
        	cout << "Offset Y 增量为 " << ptrOffsetY->GetInc() <<  endl;
        	cout << "Offset Y max为 " << ptrOffsetY->GetMax() <<  endl;
        	cout << "Offset Y min为 " << ptrOffsetY->GetMin() <<  endl;
        	
            ptrOffsetY->SetValue(value.Offset_Y);
            //ptrOffsetY->SetValue(ptrOffsetY->GetMax());
            cout << "Offset Y set to " << ptrOffsetY->GetValue() << "..." << endl;
            cout << endl;
        }
        else
        {
            cout << "Offset Y not available..." << endl;
        }


        // 设置width(注意增量设置)
        CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
        if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
        {
        	cout << "Width  增量为 " << ptrWidth->GetInc() <<  endl;
        	cout << "Width  max为 " << ptrWidth->GetMax() <<  endl;
        	cout << "Width  min为 " << ptrWidth->GetMin() <<  endl;
        	
            int64_t widthToSet = value.Weight;
            //int64_t widthToSet = ptrWidth->GetMax();

            ptrWidth->SetValue(widthToSet);

            cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
            cout << endl;
        }
        else
        {
            cout << "Width not available..." << endl;
        }

        // 设置 height(注意增量设置)
        CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
        if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
        {
        	cout << "Height  增量为 " << ptrHeight->GetInc() <<  endl;
        	cout << "Height  max为 " << ptrHeight->GetMax() <<  endl;
        	cout << "Height  min为 " << ptrHeight->GetMin() <<  endl;

            int64_t heightToSet = value.Height;
            //int64_t heightToSet = ptrHeight->GetMax();

            ptrHeight->SetValue(heightToSet);

            cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
        	cout << endl;
        }
        else
        {
            cout << "Height not available..." << endl << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

#ifdef _DEBUG
// 网口相机调试需关闭heartbeat
int DisableHeartbeat(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;

    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) && !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                     << endl
                     << endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
                cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
            }
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif

// 采集图像
#if defined(_WIN32)
DWORD WINAPI AcquireImages(LPVOID lpParam)
{
    CameraPtr pCam = *((CameraPtr*)lpParam);
#else
void* AcquireImages(void* arg)
{   //4、选择相机
    CameraPtr pCam = *((CameraPtr*)arg);
#endif
    try
    {
        // 5、获取TL设备 nodemap
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // 6、获取相机SN号
        CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

        std::string serialNumber = "";

        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            serialNumber = ptrStringSerial->GetValue();
        }

        cout << endl
             << "[" << serialNumber << "] "
             << "*** IMAGE ACQUISITION THREAD STARTING"
             << " ***" << endl
             << endl;

        // 6、打印设备信息 7、获取设备特征
        PrintDeviceInfo(nodeMapTLDevice, serialNumber);

        // 8、初始化相机
        pCam->Init();
        
        // wjh-设置trigger
		ConfigureTrigger(pCam->GetNodeMap());
       	cout <<"Configure trigger"<<endl;

	    // 设置图像参数
	    if(serialNumber=="18475880")//右侧
	    {
	    	exposureTimeToSet = 999;
	    	bingningToSet = 4; //resize 1/4
	    	framerateToSet = 30;
	    	imagesettingconfig.Offset_X=0;
		    imagesettingconfig.Offset_Y=0;
		    imagesettingconfig.Weight=1024;
		    imagesettingconfig.Height=540;
	    }
	    if(serialNumber=="11111111")//左侧
	    {
	    	exposureTimeToSet = 2000;
	    	bingningToSet = 4;
	    	framerateToSet = 15;
	    	imagesettingconfig.Offset_X=0;
		    imagesettingconfig.Offset_Y=0;
		    imagesettingconfig.Weight=1024;
		    imagesettingconfig.Height=540;
	    }
	    if(serialNumber=="11111111")//前左侧
	    {	    	
	    	exposureTimeToSet = 2000;
	    	bingningToSet = 1;
	    	framerateToSet = 15;
	    	imagesettingconfig.Offset_X=480;
		    imagesettingconfig.Offset_Y=700;
		    imagesettingconfig.Weight=1080;
		    imagesettingconfig.Height=720;
	    }
	    if(serialNumber=="11111111")//前右侧
	    {
	    	exposureTimeToSet = 2000;
	    	bingningToSet = 1;
	    	framerateToSet = 15;
	    	imagesettingconfig.Offset_X=480;
		    imagesettingconfig.Offset_Y=700;
		    imagesettingconfig.Weight=1080;
		    imagesettingconfig.Height=720;
	    }

	     //设置曝光
    	err = ConfigureExposure(pCam->GetNodeMap(),exposureTimeToSet);
	    if (err < 0)
	    {
	        return (void*)err;
	    }
	    err = ConfigureFrameRate(pCam->GetNodeMap(),framerateToSet);
	    if (err < 0)
	    {
	        return (void*)err;
	    }
	    
	    // 设置图像格式
        err = ConfigureCustomImageSettings(pCam->GetNodeMap(), imagesettingconfig , bingningToSet);
        if (err < 0)
        {
            return (void*)err;
        }


#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0)
        {
#if defined(_WIN32)
            return 0;
#else
            return (void*)0;
#endif
        }

        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        // 9、设置相机采集模式为“连续采集”
        CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
#if defined(_WIN32)
            return 0;
#else
            return (void*)0;
#endif
        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
#if defined(_WIN32)
            return 0;
#else
            return (void*)0;
#endif
        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "[" << serialNumber << "] "
             << "Acquisition mode set to continuous..." << endl;

        // 10、开始采集
        pCam->BeginAcquisition();

        cout << "[" << serialNumber << "] "
             << "Started acquiring images..." << endl;

        //
        // 10、循环采集图像并保存

       	// 时间戳及实际帧率计算
	    auto t_firstGrabbing = std::chrono::system_clock::now();
	    auto t_start = std::chrono::system_clock::now();
	    auto t_now = std::chrono::system_clock::now();

        /*const unsigned int k_numImages = 10;
        cout << endl;
        for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)*/

        unsigned int imageCnt = 0;
        while(1)//wjh-连续采集
        {
            try
            {
            	// wjh-从触发方式获取一张图像
            	// ***NOTICE**
            	// 注释掉为从视频流中抓取一张图像
            	GrabNextImageByTrigger(pCam->GetNodeMap(), pCam);
        
                // 11、获取完整图像
                ImagePtr pResultImage = pCam->GetNextImage();
                // 11、图像不完整
                if (pResultImage->IsIncomplete())
                {
                    cout << "[" << serialNumber << "] "
                         << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl
                         << endl;
                }
                // 11、获取完整图像后
                else
                {
                    // 12、图像格式转换
                    ImagePtr convertedImage = pResultImage->Convert(PixelFormat_BGR8, HQ_LINEAR);

                    // 12、设置图像名
                    ostringstream filename;
					filename << addr;
                    filename << "R-";
                    filename << setfill('0') << setw(6) << imageCnt << ".jpg";
                    
                    // 13、保存图像
                    //convertedImage->Save(filename.str().c_str());
					unsigned int XPadding = convertedImage->GetXPadding();
					unsigned int YPadding = convertedImage->GetYPadding();
					unsigned int rowsize = convertedImage->GetWidth();
					unsigned int colsize = convertedImage->GetHeight();

					//image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding. 
					Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
					namedWindow("current Image", CV_WINDOW_AUTOSIZE);
					imshow("current Image", cvimg);
					resizeWindow("current Image", rowsize / 2, colsize / 2);
					imwrite(filename.str().c_str(), cvimg);
					waitKey(1);//otherwise the image will not display...

                    // 13、打印图像尺寸
                    cout << "[" << serialNumber << "] "
                         << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth()
                         << ", height = " << pResultImage->GetHeight() << ". Image saved at " << filename.str() << endl;
                }

                // 14、释放图像
                pResultImage->Release();
                cout << endl;
            }
            catch (Spinnaker::Exception& e)
            {
                cout << "[" << serialNumber << "] "
                     << "Error: " << e.what() << endl;
           	}
           	if(imageCnt==0) //第一张图像、设置初始时间
            {
                t_firstGrabbing = std::chrono::system_clock::now();
            } 
            else
            {
                t_now   = std::chrono::system_clock::now();
                auto durationFromStart = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_firstGrabbing);
                auto durationDeltaT = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start);
                double grabTimeStamp = double(durationFromStart.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
                double deltaT = double(durationDeltaT.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;              
                std::cout << "FPS = " << 1/deltaT << "\tgrabTimeStamp = " << grabTimeStamp  <<std::endl;
                fileTime << setprecision(6) << grabTimeStamp << endl;
            }
            t_start = std::chrono::system_clock::now();  
           	imageCnt++;
        }

        // 15、停止采集
        pCam->EndAcquisition();

        // Reset trigger
        ResetTrigger(pCam->GetNodeMap());
        
        // 16、去除初始化
        pCam->DeInit();

#if defined(_WIN32)
        return 1;
#else
        return (void*)1;
#endif
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
#if defined(_WIN32)
        return 0;
#else
        return (void*)0;
#endif
    }
}

// This function acts as the body of the example
int RunMultipleCameras(CameraList camList)
{
    int result = 0;
    unsigned int camListSize = 0;
    int err = 0;

    try
    {
        // 获取相机数量
        camListSize = camList.GetSize();

        // wjh-多线程创建
        CameraPtr* pCamList = new CameraPtr[camListSize];
#if defined(_WIN32)
        HANDLE* grabThreads = new HANDLE[camListSize];
#else
        pthread_t* grabThreads = new pthread_t[camListSize];
#endif


        //××××××××××××××设置每个相机的参数×××××××××××××××//wjh
 

        //为每个相机开线程并采集保存图像
        for (unsigned int i = 0; i < camListSize; i++)
        {
            // 选择相机
            pCamList[i] = camList.GetByIndex(i);
  			// 创建采集线程，AcquireImages为采集函数
#if defined(_WIN32)
            grabThreads[i] = CreateThread(nullptr, 0, AcquireImages, &pCamList[i], 0, nullptr);
            assert(grabThreads[i] != nullptr);
#else
            int err = pthread_create(&(grabThreads[i]), nullptr, &AcquireImages, &pCamList[i]);
            assert(err == 0);
#endif
        }

#if defined(_WIN32)
        // Wait for all threads to finish
        WaitForMultipleObjects(
            camListSize, // number of threads to wait for
            grabThreads, // handles for threads to wait for
            TRUE,        // wait for all of the threads
            INFINITE     // wait forever
        );

        // Check thread return code for each camera
        for (unsigned int i = 0; i < camListSize; i++)
        {
            DWORD exitcode;

            BOOL rc = GetExitCodeThread(grabThreads[i], &exitcode);
            if (!rc)
            {
                cout << "Handle error from GetExitCodeThread() returned for camera at index " << i << endl;
            }
            else if (!exitcode)
            {
                cout << "Grab thread for camera at index " << i
                     << " exited with errors."
                        "Please check onscreen print outs for error details"
                     << endl;
            }
        }

#else   //线程结束后处理错误
        for (unsigned int i = 0; i < camListSize; i++)
        {
            // Wait for all threads to finish
            void* exitcode;
            int rc = pthread_join(grabThreads[i], &exitcode);
            if (rc != 0)
            {
                cout << "Handle error from pthread_join returned for camera at index " << i << endl;
            }
            else if ((int)(intptr_t)exitcode == 0) // check thread return code for each camera
            {
                cout << "Grab thread for camera at index " << i
                     << " exited with errors."
                        "Please check onscreen print outs for error details"
                     << endl;
            }
        }
#endif

        // wjh-多线程清除
        for (unsigned int i = 0; i < camListSize; i++)
        {
            pCamList[i] = 0;
#if defined(_WIN32)
            CloseHandle(grabThreads[i]);
#endif
        }

        // Delete array pointer
        delete[] pCamList;

        // Delete array pointer
        delete[] grabThreads;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
    // 0.测试路径权限
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
        cout << "Failed to create file in current folder.  Please check permissions." << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }

    fclose(tempFile);
    remove("test.txt");

    int result = 0;

    // 0.打印编译时间
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // 1、获取系统总线
    SystemPtr system = System::GetInstance();

    // 1、打印spin SDK版本
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
         << endl;

    // 2、从系统中获取相机列表
    CameraList camList = system->GetCameras();
    // 3、获取相机数量
    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // 没有连接相机
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << "没有检测到相机!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    // 采集图像
    cout << endl << "Running acquisition for all cameras..." << endl;

    result = RunMultipleCameras(camList);

    cout << "Acquisition complete..." << endl << endl;

    // 17、清除相机列表
    camList.Clear();

    // 18、释放系统
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return result;
}
