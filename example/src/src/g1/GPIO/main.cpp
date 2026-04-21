/**
 * main.cpp — 宇树 G1 急停按钮检测
 *
 * ═══════════════════════════════════════════════════════════
 * 官方文档确认的 GPIO 信息
 * ═══════════════════════════════════════════════════════════
 *
 *  输入线 A: gpiochip1 line 15 (PCC.03)
 *  输出辅助: gpiochip1 line 14 (PCC.02, 固定 LOW)
 *
 * ═══════════════════════════════════════════════════════════
 * 编译方式
 * ═══════════════════════════════════════════════════════════
 *
 *  统一方式（libgpiod，固定使用 build/ 目录）
 *    sudo apt install gpiod libgpiod-dev
 *    cmake -S . -B build
 *    cmake --build build
 *    ./build/estop_demo
 */

#include "gpio_estop.hpp"

#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <ctime>

volatile std::sig_atomic_t g_shutdown = 0;
void sigHandler(int) { g_shutdown = 1; }

static void log(const std::string& msg) {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::cout << "[" << std::put_time(&tm_buf, "%H:%M:%S") << "] "
              << msg << "\n" << std::flush;
}

static std::string runCommand(const std::string& cmd) {
    std::array<char, 256> buf{};
    std::string out;

    FILE* pipe = ::popen(cmd.c_str(), "r");
    if (!pipe) return "(命令执行失败)\n";

    while (::fgets(buf.data(), static_cast<int>(buf.size()), pipe)) {
        out += buf.data();
    }
    ::pclose(pipe);

    if (out.empty()) out = "(无输出)\n";
    return out;
}

static bool isLikelyOwnershipError(const std::string& err) {
    return err.find("Cannot request output line=") != std::string::npos ||
           err.find("Cannot request input line=")  != std::string::npos ||
           err.find("epoll_ctl add fd failed")     != std::string::npos;
}

static void printStartupFailureDiagnostics(const std::string& err) {
    if (!isLikelyOwnershipError(err)) return;

    std::cerr << "[诊断] 检测到疑似 GPIO 占用冲突，正在收集现场信息...\n";
    std::cerr << "[诊断] gpiochip1 关键行状态（line 14/15）:\n"
              << runCommand("gpioinfo gpiochip1 | grep -E 'line[[:space:]]+14:|line[[:space:]]+15:' || true");
    std::cerr << "[诊断] 相关进程（可能占用 GPIO）:\n"
              << runCommand("pgrep -af 'estop|emergency_stop|estop_relay|ros2 run|estop_demo' || true");
    std::cerr << "[建议] 可优先排查并停止: estop_relay emergency_stop_node（或其他占用 GPIO 的服务）。\n";
}

int main()
{
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);
    const int  debounce_ms = 20;
    const bool active_low  = false;  // 已实测：按下时输入线为高电平

    log("后端: libgpiod  (固定: input=gpiochip1:15, output-low=gpiochip1:14, detection-only)");
    log(std::string("配置: active_low=") + (active_low ? "true" : "false") +
        ", debounce_ms=" + std::to_string(debounce_ms) +
        ", mode=single-line");

    try {
        // 构造函数已内置官方文档的映射，无需传入编号
        EStopDetector detector(
            /*消抖毫秒=*/ debounce_ms,
            /*低电平有效=*/ active_low
        );

        detector.onPressed([]() {
            log("★★★ 急停触发！E-Stop ACTIVE ★★★");
            // 检测模式：仅上报状态，不直接执行运控动作。
        });

        detector.onReleased([]() {
            log("✓ 急停解除，状态恢复为正常。E-Stop RELEASED");
            // 检测模式：仅上报状态，不直接执行运控动作。
        });

        detector.start();

        std::string self_check_report;
        if (!detector.startupSelfCheck(self_check_report)) {
            std::cerr << "\n[错误] " << self_check_report
                      << "\n[错误] 自检失败，程序退出以避免无保护运行。\n";
            detector.stop();
            return 1;
        }
        log(self_check_report);

        log("监听中（epoll 边沿触发）。按 Ctrl+C 退出。\n");

        // 主循环：事件驱动等待 + 每秒状态打印
        auto last_print = std::chrono::steady_clock::now();
        bool runtime_fault = false;
        std::string runtime_error;
        uint64_t event_seq = detector.getEventSequence();
        while (g_shutdown == 0) {
            detector.waitForEvent(event_seq, std::chrono::milliseconds(250));

            if (detector.getRuntimeError(runtime_error)) {
                std::cerr << "\n[错误] 监听线程故障: " << runtime_error << "\n";
                runtime_fault = true;
                break;
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_print >= std::chrono::seconds(1)) {
                log(std::string("[状态] E-Stop: ") +
                    (detector.isActive() ? "【急停中】" : "正常"));
                last_print = now;
            }
        }

        detector.stop();
        if (runtime_fault) return 1;

    } catch (const std::exception& e) {
        std::cerr << "\n[错误] " << e.what() << "\n";
        printStartupFailureDiagnostics(e.what());
        return 1;
    }

    log("程序正常退出。");
    return 0;
}
