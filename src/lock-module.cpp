
#include "lock-module.h"
#include "move-way.h"
#include "parameter.h"
#include <SFDXGI.h>


BOOL LockMode::CreateLockEvent() {
    char new_name[MAX_PATH];
    _snprintf_s(new_name, _TRUNCATE, "%s%lu", "SF_TRT", static_cast<unsigned long>(GetCurrentProcessId()));
    lock_event = CreateEventA(nullptr, FALSE, FALSE, new_name);
    if (lock_event == nullptr) {
        MessageBoxA(nullptr, "创建事件失败，错误码查看日志", MESSAGEBOX_TITLE, MB_OK);
        return FALSE;
    }
    return TRUE;
}


static void PushIndices(std::vector<float>& EuclideanDistance,
                        std::vector<int>& indices,
                        Process* process,
                        const int& idx,
                        const bool& classes) {
    SFPoint* point = &SF_DXGI::Get().point;
    if (classes) {
        int index = process->_indices[idx];
        const Box& box = process->_boxes[index];
        float distance = pow((point->origin_x + box.x) - point->center_x, 2) +
                         pow((point->origin_y + box.y) - point->center_y, 2);
        indices.push_back(index);
        EuclideanDistance.push_back(distance);
    }
}

static inline BOOL IGetAsyncKeyState(SF::Value* value) {
    if (value->lock_key2 == 0)
        return GetAsyncKeyState(value->lock_key);
    else
        return GetAsyncKeyState(value->lock_key) || GetAsyncKeyState(value->lock_key2);
}

static std::chrono::system_clock::time_point auto_start = std::chrono::system_clock::now();

static inline BOOL AutoComplete(SF::Value* value) {
    int random_time = 0;
    if (value->auto_random != 0) {
        random_time = static_cast<int>(value->auto_random * rand() / (RAND_MAX + 1));
    }
    const std::chrono::system_clock::time_point auto_end = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(auto_end - auto_start).count() >= (value->auto_interval + random_time) * 1.0;
}


static inline VOID LeftButtonClick() {
    INPUT input{};
    input.type = INPUT_MOUSE;
    input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;   //MOUSEEVENTF_LEFTDOWN 左键按下
    input.mi.time = 0;
    input.mi.dwExtraInfo = 0;
    SendInput(1, &input, sizeof(INPUT));
    input.type = INPUT_MOUSE;
    input.mi.dwFlags = MOUSEEVENTF_LEFTUP;   // MOUSEEVENTF_LEFTUP  左键松开
    input.mi.time = 0;
    input.mi.dwExtraInfo = 0;
    SendInput(1, &input, sizeof(INPUT));
    auto_start = std::chrono::system_clock::now();
}


DWORD WINAPI IThread(void* args) {
    Actuator* actuator = static_cast<Actuator*>(args);
    Parameter* param = actuator->getParameterPtr();
    Process* process = &param->process;

    while (signl->ThreadStopSignl && signl->ImGuiWinStop) {
        WaitForSingleObject(lock->lock_event, INFINITE);
        if (!process->indices.empty()) {
            cv::Rect target_xywh;
            bool category = signl->ModelClasses;
            std::vector<float> EuclideanDistance(process->indices.size());
            std::vector<int> indices;

            for (int i = 0; i < process->indices.size(); i++) {
                if (category) {
                    int cls = process->classes[process->indices[i]];
                    if (cls >= 0 && cls <= 3) {
                        PushIndices(EuclideanDistance, indices, process, i, value->class[cls]);
                    }
                } else {
                    EuclideanDistance[i] = pow((point->origin_x + process->boxes[process->indices[i]].x) - point->center_x, 2) +
                                           pow((point->origin_y + process->boxes[process->indices[i]].y) - point->center_y, 2);
                }
            }

            int idx = -1;
            if (!indices.empty()) {
                auto min_it = std::min_element(indices.begin(), indices.end(),
                                               [&EuclideanDistance](int a, int b) { return EuclideanDistance[a] < EuclideanDistance[b]; }
                );
                idx = *min_it;
            }
            if (idx >= 0) {
                target_xywh = process->boxes[idx];
                if (value->location) {
                    float offset = (target_xywh.height * 0.5) - (target_xywh.height * value->location);
                    target_xywh.y += offset;
                }

                float x = (point->origin_x + target_xywh.x) - point->center_x;
                float y = (point->origin_y + target_xywh.y) - point->center_y;

                if (abs(x) <= value->effectiverange * 0.5 && abs(y) <= value->effectiverange * 0.5) {
                    if (variables->fov_algorithm) {
                        algorithm_x.FOVControl(&x, value->hfov, value->game_x_pixel, point->CapWidth, 360);
                        algorithm_x.FOVControl(&y, value->vfov, value->game_y_pixel, point->CapHeight, 180);
                    }

                    algorithm_x.PidControl(&x, value->P_x, value->I_x, value->D_x);
                    algorithm_y.PidControl(&y, value->P_y, value->I_y, value->D_y);

                    algorithm_x.MovePixel(&x, param->max_pixels, param->min_pixels);
                    algorithm_y.MovePixel(&y, param->max_pixels, param->min_pixels);

                    if (value->auto_fire && IGetAsyncKeyState(value) && signl->AimSwitch) {
                        moveway->MoveR(static_cast<int>(x), static_cast<int>(y));
                        if ((target_xywh.width * 0.4f) > abs(x) && (target_xywh.height * 0.4f) > abs(y) && AutoComplete(value)) {
                            LeftButtonClick();
                        }
                    }
                }
            }
        }
    }
}

VOID SetLockEvent() {
    SetEvent(LockMode::Get().lock_event);
}

BOOL LockMode::InitLock(int way,VOID (**f)()) {	// 二级指针修改，不能直接修改
    // 多线程Lock
    if (way == 3) {		// 函数式Lock
    }
    else {
        // 初始化事件
        if (!CreateLockEvent()) {
            return FALSE;
        }
        // 初始化移动
        if (!MoveWay::Get().InitMove(way)) return FALSE;
        // 创建线程
        lock_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)LockThread, this, 0, 0);
        if (lock_thread == NULL) {
            MESSAGEBOXA("创建Lock线程失败，错误码查看日志", MB_OK);
            return FALSE;
        }
        *f = SetLockEvent;
    }
    return TRUE;
}
