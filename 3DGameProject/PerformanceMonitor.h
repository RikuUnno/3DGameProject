#pragma once
#pragma once

#include <cstdint>
#include <chrono>
#include <string>
#include <string_view>
#include <unordered_map>
#include <mutex>
#include <vector>

// PerformanceMonitor
// - CPU使用率, メモリ使用量, GPU使用率, スレッドプール状態 を毎フレーム計測
// - DxLib の DrawFormatString で画面にオーバーレイ表示
// - _DEBUG 時のみ有効（Release では描画を省略）
//
// 使い方:
//   PerformanceMonitor::Instance().Update();   // 毎フレーム呼ぶ
//   PerformanceMonitor::Instance().Draw(x, y); // 描画

class PerformanceMonitor {
public:
    static PerformanceMonitor& Instance() noexcept;

    // フレーム開始時に呼び出して、区間計測（セクション）をリセットする
    void BeginFrame() noexcept;

    // 現在スレッドの「今どこを実行中か」を記録する（ハング時の切り分け用）
    void SetThreadState(std::string_view state) noexcept;

    // 毎フレーム呼び出して計測値を更新する
    void Update();

    // 画面上に計測結果を描画する (左上 x, y)
    void Draw(int x = 10, int y = 10) const;

    struct ScopedSection {
        ScopedSection() noexcept = default;
        ScopedSection(PerformanceMonitor* pm, std::string_view name) noexcept;
        ScopedSection(const ScopedSection&) = delete;
        ScopedSection& operator=(const ScopedSection&) = delete;
        ScopedSection(ScopedSection&& other) noexcept;
        ScopedSection& operator=(ScopedSection&& other) noexcept;
        ~ScopedSection();

    private:
        PerformanceMonitor* _pm = nullptr;
        std::string _name;
        std::string _prevState;
        uint64_t _startUs = 0;
    };

    [[nodiscard]] ScopedSection Scope(std::string_view name) noexcept {
        return ScopedSection(this, name);
    }

    // 表示の ON/OFF
    void SetVisible(bool visible) noexcept { _visible = visible; }
    bool IsVisible() const noexcept { return _visible; }
    void ToggleVisible() noexcept { _visible = !_visible; }

    // --- 詳細ログ機能 ---
    // 詳細ログの有効化/無効化(デフォルト: 無効)
    void EnableDetailedLogging(bool enable) noexcept { _detailedLoggingEnabled = enable; }
    bool IsDetailedLoggingEnabled() const noexcept { return _detailedLoggingEnabled; }

    // 詳細ログを手動保存(ファイル名省略時は自動生成)
    void SaveDetailedLog(const char* filename = nullptr) const;

    // 現在のシーン名を設定(SceneManagerから呼ばれる)
    void SetCurrentSceneName(std::string_view sceneName) noexcept { _currentSceneName = sceneName; }
    std::string GetCurrentSceneName() const noexcept { return _currentSceneName; }

    // --- 取得API ---
    float CpuUsagePercent()  const noexcept { return _cpuPercent; }
    float GpuUsagePercent()  const noexcept { return _gpuPercent; }
    float WorkingSetMB()     const noexcept { return _workingSetMB; }
    float VirtualMemMB()     const noexcept { return _virtualMemMB; }
    float GpuDedicatedMB()   const noexcept { return _gpuDedicatedMB; }
    float GpuSharedMB()      const noexcept { return _gpuSharedMB; }
    float Fps()              const noexcept { return _fps; }
    float FrameTimeMs()      const noexcept { return _frameTimeMs; }

    // スレッドプール情報
    size_t ThreadWorkerCount()  const noexcept { return _threadWorkerCount; }
    size_t ThreadQueueSize()    const noexcept { return _threadQueueSize; }

    // スレッド別CPU使用率 (index 0 = main thread, 1..N = worker threads)
    struct ThreadInfo {
        std::string name;
        uint32_t    threadId = 0;
        float       cpuPercent = 0.0f;
        uint64_t    kernelTimeUs = 0; // カーネル時間 (マイクロ秒)
        uint64_t    userTimeUs   = 0; // ユーザー時間 (マイクロ秒)
    };
    const std::vector<ThreadInfo>& GetThreadInfos() const noexcept { return _threads; }

    struct SectionInfo {
        std::string name;
        uint64_t timeUs = 0;
        uint32_t calls = 0;
    };
    std::vector<SectionInfo> GetTopSections(size_t maxCount = 10) const;

    struct ThreadStateInfo {
        uint32_t threadId = 0;
        std::string state;
        uint64_t lastUpdateUs = 0;
    };
    std::vector<ThreadStateInfo> GetThreadStates() const;

    PerformanceMonitor(const PerformanceMonitor&) = delete;
    PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;

private:
    PerformanceMonitor();
    ~PerformanceMonitor() = default;

    void UpdateCpu();
    void UpdateMemory();
    void UpdateGpu();
    void UpdateThreads();
    void UpdateFps();

    static uint64_t NowMicroseconds_() noexcept {
        using clock = std::chrono::steady_clock;
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            clock::now().time_since_epoch()).count());
    }

    void AddSectionTimeUs_(std::string_view name, uint64_t us) noexcept;

    void WatchdogLoop_();
    void DumpHangInfo_();
    void DumpSlowFrameInfo_() const;

    bool _visible = true;

    // CPU
    float   _cpuPercent = 0.0f;
    int     _processorCount = 1;
    int64_t _prevProcessKernel = 0;
    int64_t _prevProcessUser   = 0;
    int64_t _prevSystemTime    = 0;

    // Memory
    float _workingSetMB = 0.0f;
    float _virtualMemMB = 0.0f;

    // GPU (DXGI) ? アダプタは初期化時にキャッシュし、毎フレームの COM 生成を回避
    float _gpuPercent     = 0.0f;
    float _gpuDedicatedMB = 0.0f;
    float _gpuSharedMB    = 0.0f;
    bool  _gpuQueryAvailable = false;
    void* _cachedAdapter3 = nullptr; // IDXGIAdapter3* (ヘッダで COM 型を避ける)
    int64_t _prevGpuQueryTime = 0;   // 前回 GPU 問い合わせ時刻 (100ns単位)
    int64_t _prevMemQueryTime = 0;   // 前回 Memory 問い合わせ時刻

    // FPS
    float    _fps = 0.0f;
    float    _frameTimeMs = 0.0f;
    int      _frameCount = 0;
    int64_t  _fpsAccumTime = 0;
    int64_t  _prevFrameTime = 0;

    // Slow-frame dump (debug aid)
    mutable int64_t _lastSlowDumpUs = 0;

    // ThreadPool
    size_t _threadWorkerCount = 0;
    size_t _threadQueueSize   = 0;

    // Per-thread info
    std::vector<ThreadInfo> _threads;

    // Win32 handles for per-thread measurement
    struct ThreadHandle {
        void*    handle = nullptr; // HANDLE
        uint32_t threadId = 0;
        int64_t  prevKernel = 0;
        int64_t  prevUser   = 0;
    };
    std::vector<ThreadHandle> _threadHandles;
    bool _threadHandlesInitialized = false;
    void InitThreadHandles();

    struct SectionStat {
        uint64_t timeUs = 0;
        uint32_t calls = 0;
    };
    struct SectionHash {
        using is_transparent = void;
        size_t operator()(std::string_view sv) const noexcept { return std::hash<std::string_view>{}(sv); }
        size_t operator()(const std::string& s) const noexcept { return operator()(std::string_view{s}); }
    };
    struct SectionEq {
        using is_transparent = void;
        bool operator()(std::string_view a, std::string_view b) const noexcept { return a == b; }
        bool operator()(const std::string& a, std::string_view b) const noexcept { return a == b; }
        bool operator()(std::string_view a, const std::string& b) const noexcept { return a == b; }
        bool operator()(const std::string& a, const std::string& b) const noexcept { return a == b; }
    };
    mutable std::mutex _sectionMtx;
    std::unordered_map<std::string, SectionStat, SectionHash, SectionEq> _sections;
    std::vector<std::string> _sectionOrder;

    struct ThreadState {
        std::string state;
        uint64_t lastUpdateUs = 0;
    };
    mutable std::mutex _threadStateMtx;
    std::unordered_map<uint32_t, ThreadState> _threadStates;

    std::atomic<uint64_t> _lastHeartbeatUs{ 0 };
    std::atomic<uint64_t> _frameIndex{ 0 };
    std::atomic<bool> _watchdogStarted{ false };
    std::atomic<bool> _watchdogStop{ false };

    // 詳細ログ機能
    bool _detailedLoggingEnabled = false;
    std::string _currentSceneName = "Unknown";
};
