//--------------------------------------------------------------------------------
/**
\date     2022-12-6
\author   yjq

*/
//----------------------------------------------------------------------------------

#include "GxCamera.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace galaxy_camera
{

GxCamera::GxCamera(const rclcpp::NodeOptions& options) : Node("camera_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting DahengCamera Node!");
    nh_ = rclcpp::Node::make_shared("camera_node");

    loadCameraInfo();
    // init camrea lib
    initAll();
    //   open device      SN号
    openDevice("1");
    // Attention:   (Width-64)%2=0; (Height-64)%2=0; X%16=0; Y%2=0;
    // Daheng frame max size: [1280(Width)*1024(Height)]   offset center:[320(X),192(Y)]
    //   ROI       Width           Height       X       Y max:544    544-272=272      272中间
    setRoiParam(image_width_, image_height_, image_offset_x_, image_offset_y_);

    // ! ExposureTime比赛时一般设置为4500-5500  实际具体要根据场地亮度进行调节
    //                    autoExposure  autoGain  ExposureTime  AutoExposureMin  AutoExposureMax  Gain(<=16)  AutoGainMin  AutoGainMax  GrayValue
    setExposureGainParam(    false,     false,      exposure_time_,      3000,         11000,         gain_,         4,            8,        127);
    //   WhiteBalance         Applied?       light source type
    setWhiteBalanceParam(true, GX_AWB_LAMP_HOUSE_ADAPTIVE);

    //   Acquisition Start!
    acquisitionStart();

}

void GxCamera::loadCameraInfo()
{
    frame_id_ = nh_->declare_parameter("frame_id", std::string("camera_optical_frame"));
    exposure_time_ = nh_->declare_parameter("exposure_time", 3500.0);
    gain_ = nh_->declare_parameter("gain", 4);
    fps_ = nh_->declare_parameter("fps", 100);
    image_width_ = nh_->declare_parameter("image_width",640);
    image_height_ = nh_->declare_parameter("image_height",640);
    image_offset_x_ = nh_->declare_parameter("image_offset_x",320);
    image_offset_y_ = nh_->declare_parameter("image_offset_y",192);
    camera_name_ = nh_->declare_parameter("camera_name", std::string("narrow_stereo"));

    bool use_sensor_data_qos = nh_->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
        nh_->declare_parameter("camera_info_url", std::string("package://galaxy_camera/config/camera_info.yaml"));

    if (camera_info_manager_->validateURL(camera_info_url))
    {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    image_msg_.header.frame_id = frame_id_;
    image_msg_.height = image_height_;
    image_msg_.width = image_width_;
    image_msg_.step = image_width_ * 3;
    image_msg_.encoding = "bgr8";
    image_msg_.data.resize(image_msg_.height * image_msg_.step);
    img_ = new char[image_msg_.height * image_msg_.step];
}

/// Initialize libary
GX_STATUS GxCamera::initAll()
{
    g_hDevice = NULL;                         ///< Device handle
    g_pDeviceSN = (char*)"1";                 ///<"KE0210020112";
    g_bColorFilter = false;                   ///< Color filter support flag
    g_i64ColorFilter = GX_COLOR_FILTER_NONE;  ///< Color filter of device
    // g_bAcquisitionFlag = true;                ///< Thread running flag
    // g_nAcquisitonThreadID = 0;                ///< Thread ID of Acquisition thread
    // g_nPayloadSize = 0;                       ///< Payload size
    enable_trigger_mode_ = 0;                 ///< 0为不使用触发模式

    GX_STATUS status = GX_STATUS_SUCCESS;
    RCLCPP_INFO(
        this->get_logger(),
        "You've entered camera function which create a thread to acquire color image continuously "
        "and save it into extern frame.");
    RCLCPP_INFO(this->get_logger(), "version: 1.0.1901.9311");
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "Initializing......");
    // Initialize libary
    status = GXInitLib();
    GX_VERIFY(status);
    return status;
}

/// Open device and display device information
GX_STATUS GxCamera::openDevice(const char* CameraSN)
{
    int count = 0;
    // return GX_STATUS_SUCCESS;
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ui32DeviceNum = 0;
    g_pDeviceSN = const_cast<char*>(CameraSN);

    // Get device enumerated number
    status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    GX_VERIFY_EXIT(status);

    // If no device found, app exit<char*>(const char*);
    while (ui32DeviceNum <= 0)
    {
        count++;
        status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
        if (count > 400)
        {
            count = 0;
            RCLCPP_FATAL(this->get_logger(), "<No device found>");
        }
    }
    count = 0;
    RCLCPP_INFO(this->get_logger(), "<Connected...>");
    // Init OpenParam , Open device in exclusive mode by SN
    GX_OPEN_PARAM stOpenParam;
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;  // GX_OPEN_SN;
    stOpenParam.pszContent = g_pDeviceSN;

    // Open device
    status = GXOpenDevice(&stOpenParam, &g_hDevice);
    GX_VERIFY_EXIT(status);

    status = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    GX_VERIFY_EXIT(status);

    // This app only support color cameras
    if (!g_bColorFilter)
    {
        RCLCPP_WARN(this->get_logger(), "<This app only support color cameras! App Exit!>");
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        exit(0);
    }
    else
    {
        status = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        GX_VERIFY_EXIT(status);
    }
    return GX_STATUS_SUCCESS;
}

/// For client
/// Set camera exposure and gain params
void GxCamera::setExposureGainParam(
    bool AutoExposure, bool AutoGain, double ExposureTime, double AutoExposureTimeMin, double AutoExposureTimeMax,
    double Gain, double AutoGainMin, double AutoGainMax, int64_t GrayValue)
{
    exposure_gain.m_bAutoExposure = AutoExposure;
    exposure_gain.m_bAutoGain = AutoGain;
    exposure_gain.m_dExposureTime = ExposureTime;
    exposure_gain.m_dAutoExposureTimeMin = AutoExposureTimeMin;
    exposure_gain.m_dAutoExposureTimeMax = AutoExposureTimeMax;
    exposure_gain.m_dGain = Gain;
    exposure_gain.m_dAutoGainMin = AutoGainMin;
    exposure_gain.m_dAutoGainMax = AutoGainMax;
    exposure_gain.m_i64GrayValue = GrayValue;
}
/// Set camera exposure and gain
GX_STATUS GxCamera::setExposureGain()
{
    GX_STATUS status;

    // Set Exposure
    if (exposure_gain.m_bAutoExposure)
    {
        status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_gain.m_dAutoExposureTimeMax);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_gain.m_dAutoExposureTimeMin);
        GX_VERIFY(status);
    }
    else
    {
        status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        GX_VERIFY(status);
        status = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_gain.m_dExposureTime);
        GX_VERIFY(status);
    }

    // Set Gain
    if (exposure_gain.m_bAutoGain)
    {
        status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_GAIN_MAX, exposure_gain.m_dAutoGainMax);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_AUTO_GAIN_MIN, exposure_gain.m_dAutoGainMin);
        GX_VERIFY(status);
    }
    else
    {
        status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
        GX_VERIFY(status);
        status = GXSetEnum(g_hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        GX_VERIFY(status);
        status = GXSetFloat(g_hDevice, GX_FLOAT_GAIN, exposure_gain.m_dGain);
        GX_VERIFY(status);
    }

    // Set Expected Gray Value
    status = GXSetInt(g_hDevice, GX_INT_GRAY_VALUE, exposure_gain.m_i64GrayValue);
    GX_VERIFY(status);

    return GX_STATUS_SUCCESS;
}

/// For client
/// Set camera roi params
void GxCamera::setRoiParam(int64_t Width, int64_t Height, int64_t OffsetX, int64_t OffsetY)
{
    roi.m_i64Width = Width;
    roi.m_i64Height = Height;
    roi.m_i64OffsetX = OffsetX;
    roi.m_i64OffsetY = OffsetY;
}
/// Set camera roi
GX_STATUS GxCamera::setRoi()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //设 置 一 个 offset 偏 移 为 (X,Y) ,WidthXHeight 尺 寸 的 区 域
    status = GXSetInt(g_hDevice, GX_INT_WIDTH, 64);
    GX_VERIFY(status);
    status = GXSetInt(g_hDevice, GX_INT_HEIGHT, 64);
    GX_VERIFY(status);
    status = GXSetInt(g_hDevice, GX_INT_OFFSET_X, roi.m_i64OffsetX);
    GX_VERIFY(status);
    status = GXSetInt(g_hDevice, GX_INT_OFFSET_Y, roi.m_i64OffsetY);
    GX_VERIFY(status);
    status = GXSetInt(g_hDevice, GX_INT_WIDTH, roi.m_i64Width);
    GX_VERIFY(status);
    status = GXSetInt(g_hDevice, GX_INT_HEIGHT, roi.m_i64Height);
    GX_VERIFY(status);
    return status;
}

/// For client
/// Set camera white balance params
void GxCamera::setWhiteBalanceParam(bool WhiteBalanceOn, GX_AWB_LAMP_HOUSE_ENTRY lightSource)
{
    //自动白平衡光照环境
    // GX_AWB_LAMP_HOUSE_ADAPTIVE 自适应
    // GX_AWB_LAMP_HOUSE_FLUORESCENCE 荧光灯
    // GX_AWB_LAMP_HOUSE_INCANDESCENT 白炽灯
    // GX_AWB_LAMP_HOUSE_U30 光源温度3000k
    // GX_AWB_LAMP_HOUSE_D50 光源温度5000k
    // GX_AWB_LAMP_HOUSE_D65 光源温度6500k
    // GX_AWB_LAMP_HOUSE_D70 光源温度7000k
    white_balance.m_bWhiteBalance = WhiteBalanceOn;
    white_balance.lightSource = lightSource;
}
/// Set camera WhiteBalance
GX_STATUS GxCamera::setWhiteBalance()
{
    //选择白平衡通道
    GX_STATUS status;

    if (white_balance.m_bWhiteBalance)
    {
        // 红色通道
        status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
        //设置自动白平衡感兴趣区域(整个roi)
        status = GXSetInt(g_hDevice, GX_INT_AWBROI_WIDTH, roi.m_i64Width);
        status = GXSetInt(g_hDevice, GX_INT_AWBROI_HEIGHT, roi.m_i64Height);
        status = GXSetInt(g_hDevice, GX_INT_AWBROI_OFFSETX, 0);
        status = GXSetInt(g_hDevice, GX_INT_AWBROI_OFFSETY, 0);
        GX_VERIFY(status);

        //自动白平衡设置
        status = GXSetEnum(g_hDevice, GX_ENUM_AWB_LAMP_HOUSE, white_balance.lightSource);
        GX_VERIFY(status);

        //设置连续自动白平衡
        status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        GX_VERIFY(status);
    }
    else
    {
        status = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
        GX_VERIFY(status);
    }

    return GX_STATUS_SUCCESS;
}

GX_STATUS GxCamera::setSharpness()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //使能锐化
    GX_SHARPNESS_MODE_ENTRY nValue;
    nValue = GX_SHARPNESS_MODE_ON;
    status = GXSetEnum(g_hDevice, GX_ENUM_SHARPNESS_MODE, nValue);
    //获取锐度的值 0.0 - 3.0
    double dColorParam = 2.0;
    status = GXGetFloat(g_hDevice, GX_FLOAT_SHARPNESS, &dColorParam);
    return status;
}

GX_STATUS GxCamera::setNoiseReduction()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //使能降噪
    GX_NOISE_REDUCTION_MODE_ENTRY nValue;
    nValue = GX_NOISE_REDUCTION_MODE_ON;
    status = GXSetEnum(g_hDevice, GX_ENUM_NOISE_REDUCTION_MODE, nValue);
    //获取降噪的值 0.0 - 4.0
    double dNoiseReductionParam = 2.0;
    status = GXGetFloat(g_hDevice, GX_FLOAT_NOISE_REDUCTION, &dNoiseReductionParam);
    return status;
}

// 图像回调处理函数
void GxCamera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS)
    {
        camera_info_msg_.header.stamp = image_msg_.header.stamp = rclcpp::Clock().now();
        DxRaw8toRGB24((unsigned char*)pFrame->pImgBuf, image_msg_.data.data(), pFrame->nWidth, pFrame->nHeight, 
            RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERRG), false);    
        camera_pub_.publish(image_msg_, camera_info_msg_);
    }
    else if(pFrame->status == GX_FRAME_STATUS_INCOMPLETE)
        RCLCPP_ERROR(rclcpp::get_logger("galaxy_camera_node"),"Frame status incomplete");
    else if (pFrame->status == GX_FRAME_STATUS_INVALID_IMAGE_INFO)
        RCLCPP_ERROR(rclcpp::get_logger("galaxy_camera_node"),"Frame status invalid");
        
}

GX_STATUS GxCamera::initCamera()
{
    GX_STATUS emStatus;
    // Set Roi
    emStatus = setRoi();
    GX_VERIFY_EXIT(emStatus);
    
    // Set Exposure and Gain
    emStatus = setExposureGain();
    GX_VERIFY_EXIT(emStatus);

    // Set WhiteBalance
    emStatus = setWhiteBalance();
    GX_VERIFY_EXIT(emStatus);

    // Set Sharpness
    // emStatus = setSharpness();
    // GX_VERIFY_EXIT(emStatus);

    /// Set camera NoiseReduction
    // emStatus = setNoiseReduction();
    // GX_VERIFY_EXIT(emStatus);

    // Set acquisition mode
    // emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    // GX_VERIFY_EXIT(emStatus);

    if (enable_trigger_mode_)
    {
        // 关闭定帧率模式
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_OFF);
        GX_VERIFY_EXIT(emStatus);
        // Set trigger mode
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
        GX_VERIFY_EXIT(emStatus);
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
        GX_VERIFY_EXIT(emStatus);
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
        GX_VERIFY_EXIT(emStatus);
        //注册图像处理回调函数
        emStatus = GXRegisterCaptureCallback(g_hDevice, NULL, OnFrameCallbackFun);
    }
    else
    {
        // Set trigger mode
        // emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        // GX_VERIFY_EXIT(emStatus);

        // enable frame rate setting    GX_ACQUISITION_FRAME_RATE_MODE_ON
        emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
        GX_VERIFY_EXIT(emStatus);

        // setting the frame rate
        emStatus = GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, fps_);
        emStatus = GXRegisterCaptureCallback(g_hDevice, NULL, OnFrameCallbackFun);
        emStatus = GXStreamOn(g_hDevice);
        emStatus = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);
        if (emStatus == GX_STATUS_SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Stream On.");
            RCLCPP_INFO(this->get_logger(), "Publishing image!");
        }
    }

    if (emStatus != GX_STATUS_SUCCESS)
    {
        GX_VERIFY_EXIT(emStatus);
    }
    return emStatus;
}

/// Main function to run the whole program
GX_STATUS GxCamera::acquisitionStart()
{
    GX_STATUS emStatus;
    
    emStatus = initCamera();
    GX_VERIFY_EXIT(emStatus);
    return 0;
}

GxCamera::~GxCamera()
{
    // Device stop acquisition
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // if (capture_thread_.joinable())
    // {
    //     capture_thread_.join();
    // }
    // RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    emStatus = GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);
    emStatus = GXStreamOff(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GX_VERIFY_EXIT(emStatus);
    }

    emStatus = GXUnregisterCaptureCallback(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GX_VERIFY_EXIT(emStatus);
    }

    // Close device
    emStatus = GXCloseDevice(g_hDevice);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        exit(0);
    }

    // Release libary
    emStatus = GXCloseLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        exit(0);
    }
    RCLCPP_INFO(this->get_logger(), "<App exit!>");
}
//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code
\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS emErrorStatus)
{
    char* error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return;
    }

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete[] error_info;
        error_info = NULL;
    }
}

GX_DEV_HANDLE GxCamera::g_hDevice;
sensor_msgs::msg::Image GxCamera::image_msg_;
char* GxCamera::img_;
sensor_msgs::msg::CameraInfo GxCamera::camera_info_msg_;
image_transport::CameraPublisher GxCamera::camera_pub_;
double GxCamera::gamma_param_{};
int64_t GxCamera::contrast_param_{};
int GxCamera::improve_mode_ = 1;

}  // namespace galaxy_camera

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(galaxy_camera::GxCamera)

