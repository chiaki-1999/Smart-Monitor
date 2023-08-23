#include "actuator.h"
#include "trt-module.h"
#include "dml-module.h"
#include "imgui-module.h"
#include "lock-module.h"
#include <SFDXGI.h>

DWORD WINAPI IThread(void* args) {
    Actuator* actuator = static_cast<Actuator*>(args);
    Parameter* parame = actuator->getParameterPtr();
    YOLO* yolo = actuator->getYoloConfig();
    Frame* frame = actuator->getFrame();
    LockMode* lock = &LockMode::Get();
    lock->InitLock(parame->move_way);
    // 初始化截图
    SF_DXGI* sf = &SF_DXGI::Get();
    sf->CaptureResource(yolo->getInputDims()[2], yolo->getInputDims()[3]);
    cv::Mat img;
    while (parame->executionStatus && parame->ai_is_run) {
        // 截图
        sf->BitmapToMat(&img);
        // 推理
        frame->Detect(img);
        // 自瞄函数
        lock->StratLock(parame);
    }
    // 释放资源
    if (cv::getWindowProperty(WINDOWS_NAME, cv::WND_PROP_VISIBLE))
        cv::destroyWindow(WINDOWS_NAME);
    lock->Release();
    frame->Release();
    sf->Release();
    yolo->Release();
    // 复位状态
    parame->executionStatus = true;
    parame->ai_is_run = false;
    return 0;
}


sf::Type::YoloType convertYoloType(int type) {
	switch (type) {
	case 0: return sf::Type::TYPE_YOLOV5;
	case 1: return sf::Type::TYPE_YOLOV8;
	case 2: return sf::Type::TYPE_YOLOX;
	}
}

FrameFactory *convertBackendType(int backend) {
    switch (backend) {
        case 0:
            return sf::trt::TRTFactory::createTRTFactory();
        case 1:
            return sf::dml::DMLFactory::createDMLFactory();
    }
}

bool Actuator::executionThread(int yolotype, int backend, char* model_path) {
    // 防止多次创建线程
    if (_parame->ai_is_run) {
        return true; // 已经在运行
    }

    // 初始化yolo配置表
    YoloFactory* factory = YoloFactory::createYoloFactory();
    factory->AcquireYoloType(convertYoloType(yolotype));
    factory->AcquireConfidencePtr(&_parame->conf);
    factory->AcquireIOUPtr(&_parame->iou);
    factory->AcquireProcessPtr(&_parame->process);
    factory->AcquireShowWindowPtr(&_parame->showWindow);
    IStates hr = factory->QueryInterface(reinterpret_cast<void**>(&_yolo));
    factory->Release();

    if (hr.is_error()) {
        LOG_ERROR("Yolo初始化失败: {}", hr.msg());
        return false;
    }

    // 初始化后端
    FrameFactory* factory_frame = convertBackendType(backend);
    factory_frame->AcquireYoloPtr(_yolo);
    factory_frame->AcquireLoggerPtr(_logger);
    factory_frame->AcquireEquipmen(0); //屏幕默认0
    hr = factory_frame->QueryInterface(reinterpret_cast<void**>(&_frame));
    factory_frame->Release();
    if (hr.is_error()) {
        LOG_ERROR("后端初始化失败: {}", hr.msg());
        return false;
    }
    // 加载模型
    if (!_frame->AnalyticalModel(model_path)) {
        LOG_ERROR("加载模型失败. 错误码: {}, 错误消息: {}", _frame->getLastErrorInfo().err_code(), _frame->getLastErrorInfo().msg());
        return false;
    }
    // 创建执行线程
    thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)IThread, this, 0, 0);
    if (thread == NULL) {
        LOG_ERROR("创建AI线程失败");
        MESSAGEBOXA("创建Ai线程失败", MB_OK);
        return false;
    }
    return true;
}


Parameter* Actuator::getParameterPtr() {
	return _parame;
}

std::shared_ptr<spdlog::logger> Actuator::getLoggerPtr()
{
	return std::shared_ptr<spdlog::logger>();
}

YOLO* Actuator::getYoloConfig() {
	return _yolo;
}

Frame* Actuator::getFrame() {
	return _frame;
}

IStates actuatorFactory::QueryInterface(void** actuator){
	*actuator = new Actuator(_parame, _logger);
	return IStates();
}

IStates actuatorFactory::AcquireLoggerPtr(std::shared_ptr<spdlog::logger> logger) {
	_logger = logger;
	return IStates();
}

IStates actuatorFactory::AcquireParamePtr(Parameter* parame) {
	_parame = parame;
	return IStates();
}
