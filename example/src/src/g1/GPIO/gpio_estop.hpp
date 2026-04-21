#pragma once

/**
 * gpio_estop.hpp
 *
 * 宇树 G1 急停按钮检测器（libgpiod 后端）
 *
 * ═══════════════════════════════════════════════════════════
 * 当前部署映射（G1）
 * ═══════════════════════════════════════════════════════════
 *   输入线 A: gpiochip1 line 15 (PCC.03)
 *   输入线 B: 默认禁用（可通过 kInputLineB 改为有效 line）
 *   输出辅助: gpiochip1 line 14 (PCC.02) 固定输出低电平
 *
 * ═══════════════════════════════════════════════════════════
 * 构建方式
 * ═══════════════════════════════════════════════════════════
 *   sudo apt install gpiod libgpiod-dev
 *   cmake -S . -B build
 *   cmake --build build
 *   ./build/estop_demo
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <cerrno>
#include <unistd.h>
#include <sys/epoll.h>
#ifndef USE_LIBGPIOD
#define USE_LIBGPIOD 1
#endif

#if USE_LIBGPIOD
#include <gpiod.h>

class EStopDetector {
public:
    using Callback = std::function<void()>;
    // 启动自检结果分三类：
    // - kPassed: 当前电平符合“未触发”预期；
    // - kTriggered: 按钮可能处于按下/回路已断开，但检测器本身还能工作；
    // - kFault: GPIO 资源或读值本身异常，保护链不可用。
    enum class StartupSelfCheckState {
        kPassed,
        kTriggered,
        kFault
    };

    // 自检的结构化结果：状态供程序分支，report 供日志/TTS 直接复用。
    struct StartupSelfCheckResult {
        StartupSelfCheckState state;
        std::string report;
    };

    /**
     * libgpiod 后端构造函数
     * 当前固定配置（已实测可用）：
     *   输入线   = gpiochip1 line 15
     *   输出线   = gpiochip1 line 14（保持为 LOW）
     *   判定模式 = 单线触发（不要求双线同时有效）
     *
     * @param debounce_ms  消抖窗口（毫秒）
     * @param active_low   true = 低电平有效
     */
    explicit EStopDetector(int debounce_ms = 20,
                           bool active_low = false)
        : debounce_ms_(debounce_ms)
        , active_low_(active_low)
        , active_(false)
        , running_(false)
        , callback_worker_running_(false)
        , stop_in_progress_(false)
        , run_generation_(0)
        , epoll_fd_(-1)
        , chip_(nullptr)
        , line5_(nullptr)
        , line6_(nullptr)
        , line_out_(nullptr)
        , event_seq_(0)
    {}

    ~EStopDetector() { stop(); }

    // 注册“按下沿”回调；回调在线程序列化执行，不会和释放回调并发重入。
    void onPressed(Callback cb) {
        std::lock_guard<std::mutex> lock(cb_mtx_);
        cb_pressed_ = std::move(cb);
    }

    // 注册“释放沿”回调；新的回调会覆盖旧值。
    void onReleased(Callback cb) {
        std::lock_guard<std::mutex> lock(cb_mtx_);
        cb_released_ = std::move(cb);
    }

    // 启动自检：确认输入线电平符合“未按下”预期，避免无保护运行。
    StartupSelfCheckResult startupSelfCheckDetailed() const {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mtx_);
        if (!line5_) {
            return {StartupSelfCheckState::kFault,
                    "启动自检失败: 输入线 A 未初始化"};
        }

        int v_in_a = gpiod_line_get_value(line5_);
        if (v_in_a < 0) {
            return {StartupSelfCheckState::kFault,
                    "启动自检失败: 无法读取输入线 A 电平（line " +
                        std::to_string(kInputLineA) + ")"};
        }

        int v_in_b = -1;
        if (line6_) {
            v_in_b = gpiod_line_get_value(line6_);
            if (v_in_b < 0) {
                return {StartupSelfCheckState::kFault,
                        "启动自检失败: 无法读取输入线 B 电平（line " +
                            std::to_string(kInputLineB) + ")"};
            }
        }

        int expected_normal = active_low_ ? 1 : 0;
        if (v_in_a != expected_normal) {
            return {StartupSelfCheckState::kTriggered,
                    "启动自检失败: 启动时输入线 A 电平异常（line " +
                        std::to_string(kInputLineA) + "=" +
                        std::to_string(v_in_a) + "，期望=" +
                        std::to_string(expected_normal) +
                        "），请确认急停按钮已复位且接线正常"};
        }
        if (line6_ && v_in_b != expected_normal) {
            return {StartupSelfCheckState::kTriggered,
                    "启动自检失败: 启动时输入线 B 电平异常（line " +
                        std::to_string(kInputLineB) + "=" +
                        std::to_string(v_in_b) + "，期望=" +
                        std::to_string(expected_normal) +
                        "），请确认急停按钮已复位且接线正常"};
        }

        std::ostringstream oss;
        oss << "启动自检通过: 常闭回路导通正常 line " << kInputLineA << "=" << v_in_a;
        if (line6_) {
            oss << ", line " << kInputLineB << "=" << v_in_b;
        } else {
            oss << ", line B=禁用";
        }
        if (line_out_) {
            oss << "; 输出线 line " << kOutputLowLine << " 已驱动为 LOW（参考地）";
        } else {
            oss << "; 输出线=禁用";
        }
        return {StartupSelfCheckState::kPassed, oss.str()};
    }

    // 兼容旧接口：仅返回通过/失败，并把详细原因写回 report。
    bool startupSelfCheck(std::string& report) const {
        const StartupSelfCheckResult result = startupSelfCheckDetailed();
        report = result.report;
        return result.state == StartupSelfCheckState::kPassed;
    }

    // 返回最近一次采样/边沿处理后缓存的急停激活状态。
    bool isActive() const { return active_.load(std::memory_order_acquire); }

    // 读取当前轮运行锁存的运行时错误；返回 true 表示已有故障需要上层处理。
    bool getRuntimeError(std::string& err) const {
        std::lock_guard<std::mutex> lock(error_mtx_);
        err = runtime_error_;
        return !runtime_error_.empty();
    }

    // 读取事件序列号，供外部以“等待变化”的方式做健康巡检。
    uint64_t getEventSequence() const {
        std::lock_guard<std::mutex> lock(event_mtx_);
        return event_seq_;
    }

    // 等待事件序列变化或检测器停止；超时返回 false，发生变化/停止返回 true。
    bool waitForEvent(uint64_t& last_seen_seq,
                      std::chrono::milliseconds timeout) const {
        std::unique_lock<std::mutex> lock(event_mtx_);
        bool changed = event_cv_.wait_for(
            lock,
            timeout,
            [&] {
                return event_seq_ != last_seen_seq ||
                       !running_.load(std::memory_order_acquire);
            });
        last_seen_seq = event_seq_;
        return changed;
    }

private:
    // 回调队列中的事件类型。
    enum class CallbackEvent {
        Pressed,
        Released
    };

    int debounce_ms_;
    bool active_low_;
    // 急停当前状态、主监听线程状态、回调线程状态、stop 重入保护、运行代次。
    std::atomic<bool> active_;
    std::atomic<bool> running_;
    std::atomic<bool> callback_worker_running_;
    std::atomic<bool> stop_in_progress_;
    std::atomic<uint64_t> run_generation_;
    std::thread thread_;
    std::thread callback_thread_;
    int epoll_fd_;

    gpiod_chip* chip_;
    gpiod_line* line5_;
    gpiod_line* line6_;
    gpiod_line* line_out_;

    // 当前部署固定映射：A 线负责输入检测，B 线默认禁用，输出线提供固定 LOW 参考。
    static constexpr int kChipIndex = 1;
    static constexpr int kInputLineA = 15;
    static constexpr int kInputLineB = -1;   // -1 表示单线模式
    static constexpr int kOutputLowLine = 14;
    static constexpr bool kRequireBoth = false;  // 双线启用时：false=任一线触发即判定急停

    Callback cb_pressed_;
    Callback cb_released_;
    mutable std::mutex cb_mtx_;

    mutable std::mutex lifecycle_mtx_;
    std::condition_variable lifecycle_cv_;

    std::mutex callback_queue_mtx_;
    std::condition_variable callback_queue_cv_;
    std::deque<CallbackEvent> callback_queue_;

    mutable std::mutex event_mtx_;
    mutable std::condition_variable event_cv_;
    uint64_t event_seq_;

    mutable std::mutex error_mtx_;
    std::string runtime_error_;

    // 运行时故障只记录第一次，避免故障风暴期间重复覆盖更早的根因。
    void setRuntimeError(const std::string& err) {
        bool updated = false;
        {
            std::lock_guard<std::mutex> lock(error_mtx_);
            if (runtime_error_.empty()) {
                runtime_error_ = err;
                updated = true;
            }
        }
        if (updated) notifyEvent();
    }

    // 仅允许当前运行代次在非 stop() 流程中上报故障，避免旧线程污染新一轮状态。
    bool shouldReportRuntimeError(uint64_t run_generation) const {
        return run_generation_.load(std::memory_order_acquire) == run_generation &&
               !stop_in_progress_.load(std::memory_order_acquire);
    }

    void setRuntimeErrorForGeneration(uint64_t run_generation,
                                      const std::string& err) {
        if (!shouldReportRuntimeError(run_generation)) return;
        setRuntimeError(err);
    }

    // 某一代线程检测到致命故障后，只负责把本代运行标志拉低，由外层决定后续动作。
    void requestStopForGeneration(uint64_t run_generation) {
        if (run_generation_.load(std::memory_order_acquire) != run_generation) return;
        running_.store(false, std::memory_order_release);
        callback_worker_running_.store(false, std::memory_order_release);
        callback_queue_cv_.notify_all();
    }

    void clearRuntimeError() {
        std::lock_guard<std::mutex> lock(error_mtx_);
        runtime_error_.clear();
    }

    // epoll fd 可能在 stop()/异常恢复路径多次经过，这里做幂等关闭。
    void closeEpollFd() {
        if (epoll_fd_ >= 0) {
            ::close(epoll_fd_);
            epoll_fd_ = -1;
        }
    }

    // 推进事件序列号，唤醒外部监工线程和 waitForEvent() 调用方。
    void notifyEvent() {
        {
            std::lock_guard<std::mutex> lock(event_mtx_);
            ++event_seq_;
        }
        event_cv_.notify_all();
    }

    void clearCallbackQueue() {
        std::lock_guard<std::mutex> lock(callback_queue_mtx_);
        callback_queue_.clear();
    }

    void enqueueCallbackEvent(CallbackEvent event) {
        {
            std::lock_guard<std::mutex> lock(callback_queue_mtx_);
            if (!callback_worker_running_.load(std::memory_order_acquire)) {
                return;
            }
            callback_queue_.push_back(event);
        }
        callback_queue_cv_.notify_one();
    }

    // 释放 GPIO line/chip 句柄；线程与 epoll fd 需在外层先收好后再调用这里。
    void cleanupResources() {
        if (line5_) { gpiod_line_release(line5_); line5_ = nullptr; }
        if (line6_) { gpiod_line_release(line6_); line6_ = nullptr; }
        if (line_out_) { gpiod_line_release(line_out_); line_out_ = nullptr; }
        if (chip_) { gpiod_chip_close(chip_); chip_ = nullptr; }
    }

    // 读取当前 GPIO 电平并计算急停状态；读取失败时明确返回错误而不是静默沿用旧状态。
    bool sampleEStop(bool& sampled_state, std::string& error) const {
        int v5 = gpiod_line_get_value(line5_);
        if (v5 < 0) {
            error = "gpiod_line_get_value failed for input line A";
            return false;
        }
        bool p5 = active_low_ ? (v5 == 0) : (v5 == 1);

        if (!line6_) {
            sampled_state = p5;
            return true;
        }

        int v6 = gpiod_line_get_value(line6_);
        if (v6 < 0) {
            error = "gpiod_line_get_value failed for input line B";
            return false;
        }
        bool p6 = active_low_ ? (v6 == 0) : (v6 == 1);
        sampled_state = kRequireBoth ? (p5 && p6) : (p5 || p6);
        return true;
    }

    // 防止旧一轮遗留线程在新一轮 start() 后继续操作共享状态。
    bool isCurrentGeneration(uint64_t run_generation) const {
        return run_generation_.load(std::memory_order_acquire) == run_generation;
    }

    // 回调消费线程：串行执行按下/释放回调，避免并发重入用户逻辑。
    void callbackLoop(uint64_t run_generation) {
        while (true) {
            CallbackEvent event = CallbackEvent::Released;
            {
                std::unique_lock<std::mutex> lock(callback_queue_mtx_);
                callback_queue_cv_.wait(lock, [&] {
                    return !callback_queue_.empty() ||
                           !callback_worker_running_.load(std::memory_order_acquire) ||
                           !isCurrentGeneration(run_generation);
                });

                if (!callback_worker_running_.load(std::memory_order_acquire) ||
                    !isCurrentGeneration(run_generation)) {
                    callback_queue_.clear();
                    break;
                }

                event = callback_queue_.front();
                callback_queue_.pop_front();
            }

            Callback cb;
            const char* name = (event == CallbackEvent::Pressed) ? "pressed" : "released";
            {
                std::lock_guard<std::mutex> lock(cb_mtx_);
                cb = (event == CallbackEvent::Pressed) ? cb_pressed_ : cb_released_;
            }
            if (!cb) continue;

            try {
                cb();
            } catch (const std::exception& e) {
                setRuntimeErrorForGeneration(
                    run_generation,
                    std::string("回调异常(") + name + "): " + e.what());
                requestStopForGeneration(run_generation);
                notifyEvent();
                break;
            } catch (...) {
                setRuntimeErrorForGeneration(
                    run_generation,
                    std::string("回调异常(") + name + "): unknown");
                requestStopForGeneration(run_generation);
                notifyEvent();
                break;
            }
        }
    }

    // GPIO 事件线程：epoll 等待边沿事件 + 消抖 + 状态变更上报。
    void eventLoop(uint64_t run_generation) {
        if (epoll_fd_ < 0) {
            setRuntimeErrorForGeneration(run_generation, "epoll fd not initialized");
            requestStopForGeneration(run_generation);
            notifyEvent();
            return;
        }

        epoll_event events[4];
        while (running_.load(std::memory_order_acquire) &&
               isCurrentGeneration(run_generation)) {
            int n = epoll_wait(epoll_fd_, events, 4, 200);
            if (n < 0) {
                int saved_errno = errno;
                if (errno == EINTR) continue;

                bool is_shutdown_close = (saved_errno == EBADF || saved_errno == EINVAL) &&
                                         stop_in_progress_.load(std::memory_order_acquire);
                if (running_.load(std::memory_order_acquire) && !is_shutdown_close) {
                    setRuntimeErrorForGeneration(run_generation, "epoll_wait failed");
                }
                requestStopForGeneration(run_generation);
                break;
            }
            if (n == 0) continue;

            gpiod_line_event evt{};
            bool read_failed = false;
            for (int i = 0; i < n; ++i) {
                gpiod_line* line = nullptr;
                if (events[i].data.fd == gpiod_line_event_get_fd(line5_)) {
                    line = line5_;
                } else if (line6_ &&
                           events[i].data.fd == gpiod_line_event_get_fd(line6_)) {
                    line = line6_;
                }
                if (line && gpiod_line_event_read(line, &evt) < 0) {
                    setRuntimeErrorForGeneration(run_generation, "gpiod_line_event_read failed");
                    requestStopForGeneration(run_generation);
                    read_failed = true;
                    break;
                }
            }
            if (read_failed) break;

            // 统一消抖窗口，降低机械抖动带来的误触发概率。
            std::this_thread::sleep_for(std::chrono::milliseconds(debounce_ms_));

            bool new_state = false;
            std::string sample_error;
            if (!sampleEStop(new_state, sample_error)) {
                setRuntimeErrorForGeneration(run_generation, sample_error);
                requestStopForGeneration(run_generation);
                break;
            }
            bool old_state = active_.load(std::memory_order_acquire);
            if (new_state == old_state) continue;

            active_.store(new_state, std::memory_order_release);
            enqueueCallbackEvent(new_state ? CallbackEvent::Pressed
                                           : CallbackEvent::Released);
            notifyEvent();
        }

        notifyEvent();
    }

    // 当上一轮运行已自行停止（例如运行时故障）但外部尚未调用 stop() 时，
    // 在新一轮 start() 前回收陈旧资源，避免线程/FD/GPIO 句柄残留。
    void recoverStoppedRunStateLocked() {
        if (running_.load(std::memory_order_acquire)) {
            return;
        }

        callback_worker_running_.store(false, std::memory_order_release);
        closeEpollFd();
        callback_queue_cv_.notify_all();

        if (thread_.joinable()) {
            if (std::this_thread::get_id() != thread_.get_id()) {
                thread_.join();
            } else {
                thread_.detach();
            }
        }

        if (callback_thread_.joinable()) {
            if (std::this_thread::get_id() != callback_thread_.get_id()) {
                callback_thread_.join();
            } else {
                callback_thread_.detach();
            }
        }

        clearCallbackQueue();
        cleanupResources();
    }

public:
    // 初始化 GPIO 资源、启动事件线程和回调线程，并在启动末尾完成一次初始采样。
    void start() {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mtx_);
        if (stop_in_progress_.load(std::memory_order_acquire)) {
            throw std::runtime_error("start called while stop() is in progress");
        }

        // 已运行则直接返回；若前一轮异常退出，则先做一次资源回收。
        if (running_.load(std::memory_order_acquire)) return;
        recoverStoppedRunStateLocked();

        if (running_.exchange(true, std::memory_order_acq_rel)) return;
        const uint64_t run_generation =
            run_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;

        clearRuntimeError();
        clearCallbackQueue();

        try {
            if (kOutputLowLine >= 0 &&
                (kOutputLowLine == kInputLineA ||
                 kOutputLowLine == kInputLineB)) {
                throw std::runtime_error("Output line conflicts with input lines");
            }

            chip_ = gpiod_chip_open_by_number(kChipIndex);
            if (!chip_) {
                throw std::runtime_error(
                    "Cannot open gpiochip" + std::to_string(kChipIndex) +
                    "\n  → 请先安装：sudo apt install gpiod");
            }

            line5_ = gpiod_chip_get_line(chip_, kInputLineA);
            if (!line5_) {
                throw std::runtime_error(
                    "Cannot get input line=" + std::to_string(kInputLineA));
            }

            if (kInputLineB >= 0) {
                line6_ = gpiod_chip_get_line(chip_, kInputLineB);
                if (!line6_) {
                    throw std::runtime_error(
                        "Cannot get input line=" + std::to_string(kInputLineB));
                }
            }

            if (kOutputLowLine >= 0) {
                line_out_ = gpiod_chip_get_line(chip_, kOutputLowLine);
                if (!line_out_) {
                    throw std::runtime_error(
                        "Cannot get output line=" + std::to_string(kOutputLowLine));
                }

                gpiod_line_request_config out_cfg{};
                out_cfg.consumer = "estop_detector_out";
                out_cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
                out_cfg.flags = 0;

                if (gpiod_line_request(line_out_, &out_cfg, 0) < 0) {
                    throw std::runtime_error(
                        "Cannot request output line=" + std::to_string(kOutputLowLine));
                }
            }

            gpiod_line_request_config cfg{};
            cfg.consumer = "estop_detector";
            cfg.request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
            cfg.flags = 0;

            if (gpiod_line_request(line5_, &cfg, 0) < 0) {
                throw std::runtime_error(
                    "Cannot request input line=" + std::to_string(kInputLineA));
            }

            if (line6_ && gpiod_line_request(line6_, &cfg, 0) < 0) {
                throw std::runtime_error(
                    "Cannot request input line=" + std::to_string(kInputLineB));
            }

            epoll_fd_ = epoll_create1(0);
            if (epoll_fd_ < 0) {
                throw std::runtime_error("epoll_create1 failed");
            }

            // 将输入线事件 fd 挂到 epoll，统一由事件线程处理。
            auto addLine = [&](gpiod_line* line) {
                int fd = gpiod_line_event_get_fd(line);
                if (fd < 0) return false;
                epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = fd;
                return epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) == 0;
            };

            if (!addLine(line5_) || (line6_ && !addLine(line6_))) {
                throw std::runtime_error("epoll_ctl add fd failed");
            }

            callback_worker_running_.store(true, std::memory_order_release);
            callback_thread_ = std::thread(&EStopDetector::callbackLoop,
                                           this,
                                           run_generation);

            // 启动时先采样一次，避免第一拍状态未知；若此时读值失败，直接按启动失败处理。
            bool initial_state = false;
            std::string sample_error;
            if (!sampleEStop(initial_state, sample_error)) {
                throw std::runtime_error(sample_error);
            }
            active_.store(initial_state, std::memory_order_release);
            thread_ = std::thread(&EStopDetector::eventLoop, this, run_generation);
            notifyEvent();
        } catch (const std::exception& e) {
            running_.store(false, std::memory_order_release);
            callback_worker_running_.store(false, std::memory_order_release);
            callback_queue_cv_.notify_all();

            if (callback_thread_.joinable()) {
                callback_thread_.join();
            }

            setRuntimeErrorForGeneration(run_generation, e.what());
            closeEpollFd();
            cleanupResources();
            throw;
        } catch (...) {
            running_.store(false, std::memory_order_release);
            callback_worker_running_.store(false, std::memory_order_release);
            callback_queue_cv_.notify_all();

            if (callback_thread_.joinable()) {
                callback_thread_.join();
            }

            setRuntimeErrorForGeneration(run_generation, "libgpiod start unknown error");
            closeEpollFd();
            cleanupResources();
            throw;
        }
    }

    // 同步停止检测器：关闭 fd、唤醒线程、等待退出并回收全部 GPIO 资源。
    void stop() {
        std::thread event_thread_to_join;
        std::thread callback_thread_to_join;

        {
            std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mtx_);
            // 防止并发 stop：后到者等待前一次 stop 完整结束。
            if (stop_in_progress_.load(std::memory_order_acquire)) {
                lifecycle_cv_.wait(lifecycle_lock, [&] {
                    return !stop_in_progress_.load(std::memory_order_acquire);
                });
                return;
            }

            stop_in_progress_.store(true, std::memory_order_release);

            running_.store(false, std::memory_order_release);
            callback_worker_running_.store(false, std::memory_order_release);
            closeEpollFd();
            callback_queue_cv_.notify_all();
            notifyEvent();

            if (thread_.joinable()) {
                if (std::this_thread::get_id() != thread_.get_id()) {
                    event_thread_to_join = std::move(thread_);
                } else {
                    thread_.detach();
                }
            }

            if (callback_thread_.joinable()) {
                if (std::this_thread::get_id() != callback_thread_.get_id()) {
                    callback_thread_to_join = std::move(callback_thread_);
                } else {
                    callback_thread_.detach();
                }
            }
        }

        if (event_thread_to_join.joinable()) {
            event_thread_to_join.join();
        }
        if (callback_thread_to_join.joinable()) {
            callback_thread_to_join.join();
        }

        {
            std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mtx_);
            clearCallbackQueue();
            cleanupResources();
            stop_in_progress_.store(false, std::memory_order_release);
        }
        lifecycle_cv_.notify_all();
    }
};

#else
#error "USE_LIBGPIOD=OFF is not supported by current source tree. Configure with -DUSE_LIBGPIOD=ON."
#endif
