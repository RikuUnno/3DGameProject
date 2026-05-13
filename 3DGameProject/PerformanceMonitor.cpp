#include "PerformanceMonitor.h"
#include "DxLib.h"
#include "ThreadPool.h"

#include <algorithm>
#include <fstream>
#include <thread>

namespace {
    thread_local std::string g_tlsState = "Idle";
}

// Windows API for CPU/Memory/GPU
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Psapi.h>
#include <dxgi1_4.h>
#include <wrl/client.h>

#pragma comment(lib, "psapi.lib")
#pragma comment(lib, "dxgi.lib")

namespace {
    // FILETIME → int64 (100ns 単位)
    inline int64_t FileTimeToInt64(const FILETIME& ft) noexcept {
        ULARGE_INTEGER li;
        li.LowPart  = ft.dwLowDateTime;
        li.HighPart = ft.dwHighDateTime;
        return static_cast<int64_t>(li.QuadPart);
    }

    // QueryPerformanceCounter をマイクロ秒で返す
    inline int64_t NowMicroseconds() noexcept {
        LARGE_INTEGER freq, now;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&now);
        return now.QuadPart * 1000000LL / freq.QuadPart;
    }

    // GetSystemTimeAsFileTime を int64(100ns) にする
    inline int64_t NowFileTimeUnits() noexcept {
        FILETIME ft;
        GetSystemTimeAsFileTime(&ft);
        return FileTimeToInt64(ft);
    }
}

void PerformanceMonitor::DumpSlowFrameInfo_() const {
    // Heavy-weight dump: throttle to avoid excessive I/O
    const uint64_t nowUs = NowMicroseconds_();
    if (nowUs - static_cast<uint64_t>(_lastSlowDumpUs) < 1000 * 1000) return; // 1 sec
    _lastSlowDumpUs = static_cast<int64_t>(nowUs);

    std::ofstream ofs("ProfileSlowFrame.txt", std::ios::out | std::ios::app);
    if (!ofs) return;

    const uint64_t frame = _frameIndex.load(std::memory_order_relaxed);
    ofs << "\n[SLOW] frame=" << frame
        << " frameTimeMs=" << _frameTimeMs
        << " fps=" << _fps
        << " cpu%=" << _cpuPercent
        << " workers=" << _threadWorkerCount
        << " queue=" << _threadQueueSize
        << "\n";

    // Section snapshot
    {
        std::lock_guard<std::mutex> lk(_sectionMtx);
        std::vector<SectionInfo> secs;
        secs.reserve(_sections.size());
        for (const auto& name : _sectionOrder) {
            auto it = _sections.find(name);
            if (it == _sections.end()) continue;
            const auto& st = it->second;
            if (st.calls == 0) continue;
            secs.push_back(SectionInfo{ name, st.timeUs, st.calls });
        }
        std::sort(secs.begin(), secs.end(), [](const SectionInfo& a, const SectionInfo& b) {
            return a.timeUs > b.timeUs;
        });
        if (secs.size() > 16) secs.resize(16);
        ofs << "-- SectionsTop --\n";
        for (const auto& s : secs) {
            ofs << s.name << " us=" << s.timeUs << " calls=" << s.calls << "\n";
        }
    }
}

// ============================================================
//  Section profiling
// ============================================================
PerformanceMonitor::ScopedSection::ScopedSection(PerformanceMonitor* pm, std::string_view name) noexcept
    : _pm(pm), _name(name), _startUs(pm ? pm->NowMicroseconds_() : 0) {
    _prevState = g_tlsState;
    g_tlsState = _name;
    if (_pm) _pm->SetThreadState(_name);
}

PerformanceMonitor::ScopedSection::ScopedSection(ScopedSection&& other) noexcept {
    _pm = other._pm;
    _name = other._name;
    _prevState = other._prevState;
    _startUs = other._startUs;
    other._pm = nullptr;
    other._startUs = 0;
}

PerformanceMonitor::ScopedSection& PerformanceMonitor::ScopedSection::operator=(ScopedSection&& other) noexcept {
    if (this == &other) return *this;
    if (_pm) {
        const uint64_t endUs = _pm->NowMicroseconds_();
        _pm->AddSectionTimeUs_(_name, endUs - _startUs);
    }
    _pm = other._pm;
    _name = other._name;
    _prevState = other._prevState;
    _startUs = other._startUs;
    other._pm = nullptr;
    other._startUs = 0;
    return *this;
}

PerformanceMonitor::ScopedSection::~ScopedSection() {
    if (!_pm) return;
    const uint64_t endUs = _pm->NowMicroseconds_();
    _pm->AddSectionTimeUs_(_name, endUs - _startUs);
    g_tlsState = _prevState;
    _pm->SetThreadState(g_tlsState);
}

void PerformanceMonitor::BeginFrame() noexcept {
    if (!_watchdogStarted.exchange(true, std::memory_order_acq_rel)) {
        _watchdogStop.store(false, std::memory_order_release);
        std::thread([this]() { WatchdogLoop_(); }).detach();
    }

    _lastHeartbeatUs.store(NowMicroseconds_(), std::memory_order_release);
    _frameIndex.fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lk(_sectionMtx);
    for (auto& kv : _sections) {
        kv.second.timeUs = 0;
        kv.second.calls = 0;
    }
}

void PerformanceMonitor::SetThreadState(std::string_view state) noexcept {
    if (state.empty()) state = "Idle";
    const uint64_t nowUs = NowMicroseconds_();
    const uint32_t tid = ::GetCurrentThreadId();
    std::lock_guard<std::mutex> lk(_threadStateMtx);
    auto& st = _threadStates[tid];
    st.state = std::string(state);
    st.lastUpdateUs = nowUs;
}

void PerformanceMonitor::AddSectionTimeUs_(std::string_view name, uint64_t us) noexcept {
    if (name.empty() || us == 0) return;
    std::lock_guard<std::mutex> lk(_sectionMtx);
    auto it = _sections.find(name);
    if (it == _sections.end()) {
        auto [insIt, inserted] = _sections.try_emplace(std::string(name), SectionStat{});
        it = insIt;
        if (inserted) {
            _sectionOrder.emplace_back(it->first);
        }
    }
    it->second.timeUs += us;
    it->second.calls += 1;
}

std::vector<PerformanceMonitor::ThreadStateInfo> PerformanceMonitor::GetThreadStates() const {
    std::vector<ThreadStateInfo> out;
    std::lock_guard<std::mutex> lk(_threadStateMtx);
    out.reserve(_threadStates.size());
    for (const auto& [tid, st] : _threadStates) {
        out.push_back(ThreadStateInfo{ tid, st.state, st.lastUpdateUs });
    }
    std::sort(out.begin(), out.end(), [](const ThreadStateInfo& a, const ThreadStateInfo& b) {
        return a.threadId < b.threadId;
    });
    return out;
}

void PerformanceMonitor::WatchdogLoop_() {
    // 画面が止まっても情報が取れるよう、ハング検知時にファイルへ吐く
    constexpr uint64_t kHangThresholdUs = 1500 * 1000;
    constexpr uint64_t kDumpIntervalUs = 1000 * 1000;
    uint64_t lastDumpUs = 0;

    for (;;) {
        if (_watchdogStop.load(std::memory_order_relaxed)) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        const uint64_t nowUs = NowMicroseconds_();
        const uint64_t hbUs = _lastHeartbeatUs.load(std::memory_order_acquire);
        const uint64_t sinceHb = (hbUs <= nowUs) ? (nowUs - hbUs) : 0;
        if (sinceHb < kHangThresholdUs) continue;
        if (nowUs - lastDumpUs < kDumpIntervalUs) continue;
        lastDumpUs = nowUs;
        DumpHangInfo_();
    }
}

void PerformanceMonitor::DumpHangInfo_() {
    std::ofstream ofs("ProfileHang.txt", std::ios::out | std::ios::app);
    if (!ofs) return;

    const uint64_t nowUs = NowMicroseconds_();
    const uint64_t hbUs = _lastHeartbeatUs.load(std::memory_order_acquire);
    const uint64_t sinceHb = (hbUs <= nowUs) ? (nowUs - hbUs) : 0;
    const uint64_t frame = _frameIndex.load(std::memory_order_relaxed);

    ofs << "\n[HANG] frame=" << frame << " sinceHeartbeatUs=" << sinceHb << "\n";

    // Thread state snapshot
    {
        std::lock_guard<std::mutex> lk(_threadStateMtx);
        ofs << "-- ThreadStates --\n";
        for (const auto& [tid, st] : _threadStates) {
            ofs << "tid=" << tid << " lastUpdateUs=" << st.lastUpdateUs << " state=" << st.state << "\n";
        }
    }

    // Section snapshot (best-effort: avoid blocking on lock)
    {
        std::unique_lock<std::mutex> lk(_sectionMtx, std::try_to_lock);
        if (!lk.owns_lock()) {
            ofs << "-- Sections -- (lock busy)\n";
        } else {
            std::vector<SectionInfo> secs;
            secs.reserve(_sections.size());
            for (const auto& name : _sectionOrder) {
                auto it = _sections.find(name);
                if (it == _sections.end()) continue;
                const auto& st = it->second;
                if (st.calls == 0) continue;
                secs.push_back(SectionInfo{ name, st.timeUs, st.calls });
            }
            std::sort(secs.begin(), secs.end(), [](const SectionInfo& a, const SectionInfo& b) {
                return a.timeUs > b.timeUs;
            });
            if (secs.size() > 16) secs.resize(16);
            ofs << "-- SectionsTop --\n";
            for (const auto& s : secs) {
                ofs << s.name << " us=" << s.timeUs << " calls=" << s.calls << "\n";
            }
        }
    }
}

std::vector<PerformanceMonitor::SectionInfo> PerformanceMonitor::GetTopSections(size_t maxCount) const {
    std::vector<SectionInfo> out;
    out.reserve(_sections.size());
    {
        std::lock_guard<std::mutex> lk(_sectionMtx);
        for (const auto& name : _sectionOrder) {
            auto it = _sections.find(name);
            if (it == _sections.end()) continue;
            const auto& st = it->second;
            if (st.calls == 0) continue;
            out.push_back(SectionInfo{ name, st.timeUs, st.calls });
        }
    }
    std::sort(out.begin(), out.end(), [](const SectionInfo& a, const SectionInfo& b) {
        return a.timeUs > b.timeUs;
    });
    if (maxCount > 0 && out.size() > maxCount) out.resize(maxCount);
    return out;
}

// ============================================================
//  Singleton
// ============================================================
PerformanceMonitor& PerformanceMonitor::Instance() noexcept {
    static PerformanceMonitor inst;
    return inst;
}

PerformanceMonitor::PerformanceMonitor() {
    SYSTEM_INFO si{};
    GetSystemInfo(&si);
    _processorCount = static_cast<int>(si.dwNumberOfProcessors);
    if (_processorCount < 1) _processorCount = 1;

    // 初期タイムスタンプ
    FILETIME creation, exit, kernel, user;
    if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) {
        _prevProcessKernel = FileTimeToInt64(kernel);
        _prevProcessUser   = FileTimeToInt64(user);
    }
    _prevSystemTime = NowFileTimeUnits();
    _prevFrameTime  = NowMicroseconds();

    // GPU クエリ初期化 ? アダプタをキャッシュして毎フレームの COM 生成を回避
    {
        Microsoft::WRL::ComPtr<IDXGIFactory4> factory;
        if (SUCCEEDED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) {
            Microsoft::WRL::ComPtr<IDXGIAdapter1> adapter1;
            if (SUCCEEDED(factory->EnumAdapters1(0, &adapter1))) {
                IDXGIAdapter3* raw = nullptr;
                if (SUCCEEDED(adapter1->QueryInterface(IID_PPV_ARGS(&raw)))) {
                    _cachedAdapter3 = raw; // AddRef 済み。デストラクタで Release 不要（プロセス終了まで保持）
                    _gpuQueryAvailable = true;
                }
            }
        }
    }
}

// ============================================================
//  Update
// ============================================================
void PerformanceMonitor::Update() {
    UpdateFps();
    UpdateCpu();
    UpdateMemory();
    UpdateGpu();
    UpdateThreads();

    // 詳細ログ自動保存（設定された間隔で保存）
    if (_detailedLoggingEnabled) {
        static int64_t s_lastDetailedLog = 0;
        const int64_t now = NowMicroseconds();
        const int64_t intervalUs = static_cast<int64_t>(_autoSaveIntervalSec * 1000000LL);
        if (now - s_lastDetailedLog > intervalUs) {
            s_lastDetailedLog = now;
            SaveDetailedLog();
        }
    }
}

// ============================================================
//  CPU usage (process-wide)
// ============================================================
void PerformanceMonitor::UpdateCpu() {
    FILETIME creation, exit, kernel, user;
    if (!GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user)) return;

    const int64_t curKernel = FileTimeToInt64(kernel);
    const int64_t curUser   = FileTimeToInt64(user);
    const int64_t curSystem = NowFileTimeUnits();

    const int64_t dKernel = curKernel - _prevProcessKernel;
    const int64_t dUser   = curUser   - _prevProcessUser;
    const int64_t dSystem = curSystem - _prevSystemTime;

    if (dSystem > 0) {
        // CPU% = (process time / wall time) / numProcessors * 100
        const double cpuTime = static_cast<double>(dKernel + dUser);
        const double wallTime = static_cast<double>(dSystem);
        _cpuPercent = static_cast<float>(cpuTime / wallTime / _processorCount * 100.0);
        if (_cpuPercent > 100.0f * _processorCount) _cpuPercent = 100.0f * _processorCount;
    }

    _prevProcessKernel = curKernel;
    _prevProcessUser   = curUser;
    _prevSystemTime    = curSystem;
}

// ============================================================
//  Memory
// ============================================================
void PerformanceMonitor::UpdateMemory() {
    // 0.5秒間隔でクエリ (カーネル呼び出し頻度を抑える)
    const int64_t now = NowFileTimeUnits();
    if (now - _prevMemQueryTime < 5000000LL) return;
    _prevMemQueryTime = now;

    PROCESS_MEMORY_COUNTERS_EX pmc{};
    pmc.cb = sizeof(pmc);
    if (GetProcessMemoryInfo(GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc), sizeof(pmc))) {
        _workingSetMB = static_cast<float>(pmc.WorkingSetSize) / (1024.0f * 1024.0f);
        _virtualMemMB = static_cast<float>(pmc.PrivateUsage)  / (1024.0f * 1024.0f);
    }
}

// ============================================================
//  GPU (DXGI 1.4)
// ============================================================
void PerformanceMonitor::UpdateGpu() {
    if (!_gpuQueryAvailable || !_cachedAdapter3) return;

    // 0.5秒間隔でクエリ (DXGI 呼び出しは重い)
    const int64_t now = NowFileTimeUnits();
    if (now - _prevGpuQueryTime < 5000000LL) return;
    _prevGpuQueryTime = now;

    auto* adapter3 = static_cast<IDXGIAdapter3*>(_cachedAdapter3);

    // 専用メモリ
    DXGI_QUERY_VIDEO_MEMORY_INFO dedicatedInfo{};
    if (SUCCEEDED(adapter3->QueryVideoMemoryInfo(
            0, DXGI_MEMORY_SEGMENT_GROUP_LOCAL, &dedicatedInfo))) {
        _gpuDedicatedMB = static_cast<float>(dedicatedInfo.CurrentUsage) / (1024.0f * 1024.0f);
        const float budgetMB = static_cast<float>(dedicatedInfo.Budget) / (1024.0f * 1024.0f);
        if (budgetMB > 0.0f) {
            _gpuPercent = (_gpuDedicatedMB / budgetMB) * 100.0f;
        }
    }

    // 共有メモリ
    DXGI_QUERY_VIDEO_MEMORY_INFO sharedInfo{};
    if (SUCCEEDED(adapter3->QueryVideoMemoryInfo(
            0, DXGI_MEMORY_SEGMENT_GROUP_NON_LOCAL, &sharedInfo))) {
        _gpuSharedMB = static_cast<float>(sharedInfo.CurrentUsage) / (1024.0f * 1024.0f);
    }
}

// ============================================================
//  Per-thread CPU usage
// ============================================================
void PerformanceMonitor::InitThreadHandles() {
    _threadHandles.clear();
    _threads.clear();

    // メインスレッド
    {
        HANDLE h = nullptr;
        DuplicateHandle(GetCurrentProcess(), GetCurrentThread(),
                        GetCurrentProcess(), &h, THREAD_QUERY_INFORMATION, FALSE, 0);
        ThreadHandle th;
        th.handle   = h;
        th.threadId = GetCurrentThreadId();

        FILETIME creation, exit, kernel, user;
        if (h && GetThreadTimes(h, &creation, &exit, &kernel, &user)) {
            th.prevKernel = FileTimeToInt64(kernel);
            th.prevUser   = FileTimeToInt64(user);
        }
        _threadHandles.push_back(th);

        ThreadInfo info;
        info.name = "Main";
        info.threadId = th.threadId;
        _threads.push_back(info);
    }

    // ワーカースレッド
    auto handles = ThreadPool::Instance().WorkerNativeHandles();
    auto ids     = ThreadPool::Instance().WorkerThreadIds();
    for (size_t i = 0; i < handles.size(); ++i) {
        HANDLE h = nullptr;
        DuplicateHandle(GetCurrentProcess(), static_cast<HANDLE>(handles[i]),
                        GetCurrentProcess(), &h, THREAD_QUERY_INFORMATION, FALSE, 0);
        ThreadHandle th;
        th.handle   = h;
        // native_handle_type から thread id を取得
        th.threadId = GetThreadId(static_cast<HANDLE>(handles[i]));

        FILETIME creation, exit, kernel, user;
        if (h && GetThreadTimes(h, &creation, &exit, &kernel, &user)) {
            th.prevKernel = FileTimeToInt64(kernel);
            th.prevUser   = FileTimeToInt64(user);
        }
        _threadHandles.push_back(th);

        ThreadInfo info;
        info.name = "Worker " + std::to_string(i);
        info.threadId = th.threadId;
        _threads.push_back(info);
    }

    _threadHandlesInitialized = true;
}

void PerformanceMonitor::UpdateThreads() {
    // ThreadPool 情報
    _threadWorkerCount = ThreadPool::Instance().WorkerCount();
    _threadQueueSize   = ThreadPool::Instance().QueueSize();

    if (!_threadHandlesInitialized) {
        InitThreadHandles();
    }

    const int64_t curSystem = NowFileTimeUnits();
    // 前回との差分が小さすぎる場合はスキップ (ちらつき防止)
    static int64_t s_prevThreadUpdate = 0;
    const int64_t dSystem = curSystem - s_prevThreadUpdate;
    if (dSystem < 5000000LL) return; // 0.5秒未満は更新しない
    s_prevThreadUpdate = curSystem;

    for (size_t i = 0; i < _threadHandles.size() && i < _threads.size(); ++i) {
        auto& th = _threadHandles[i];
        auto& info = _threads[i];
        if (!th.handle) continue;

        FILETIME creation, exit, kernel, user;
        if (!GetThreadTimes(static_cast<HANDLE>(th.handle), &creation, &exit, &kernel, &user))
            continue;

        const int64_t curKernel = FileTimeToInt64(kernel);
        const int64_t curUser   = FileTimeToInt64(user);

        const int64_t dk = curKernel - th.prevKernel;
        const int64_t du = curUser   - th.prevUser;

        if (dSystem > 0) {
            const double threadTime = static_cast<double>(dk + du);
            const double wallTime   = static_cast<double>(dSystem);
            info.cpuPercent = static_cast<float>(threadTime / wallTime * 100.0);
        }
        info.kernelTimeUs = static_cast<uint64_t>(curKernel / 10); // 100ns → μs
        info.userTimeUs   = static_cast<uint64_t>(curUser / 10);

        th.prevKernel = curKernel;
        th.prevUser   = curUser;
    }
}

// ============================================================
//  詳細ログ保存
// ============================================================
void PerformanceMonitor::SaveDetailedLog(const char* filename) const {
    std::string fname;
    if (filename) {
        fname = filename;
    } else {
        fname = "PerformanceLog.txt";
    }

    std::ofstream ofs(fname, std::ios::out | std::ios::trunc);
    if (!ofs) return;

    const uint64_t frame = _frameIndex.load(std::memory_order_relaxed);

    ofs << "\n========================================\n";
    ofs << "[DETAILED LOG] Frame: " << frame << "\n";
    ofs << "========================================\n";

    // タイムスタンプ
    {
        SYSTEMTIME st;
        GetLocalTime(&st);
        ofs << "Timestamp: " << st.wYear << "/" << st.wMonth << "/" << st.wDay
            << " " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "\n";
    }

    // シーン情報
    ofs << "\n--- Scene Information ---\n";
    ofs << "Current Scene: " << _currentSceneName << "\n";

    // パフォーマンス概要
    ofs << "\n--- Performance Summary ---\n";
    ofs << "FPS: " << _fps << "\n";
    ofs << "Frame Time: " << _frameTimeMs << " ms\n";
    ofs << "CPU Usage: " << _cpuPercent << " %\n";
    ofs << "GPU Usage: " << _gpuPercent << " %\n";

    // メモリ使用量
    ofs << "\n--- Memory Usage ---\n";
    ofs << "Working Set: " << _workingSetMB << " MB\n";
    ofs << "Virtual Memory: " << _virtualMemMB << " MB\n";
    ofs << "GPU Dedicated: " << _gpuDedicatedMB << " MB\n";
    ofs << "GPU Shared: " << _gpuSharedMB << " MB\n";

    // システム情報
    ofs << "\n--- System Information ---\n";
    ofs << "Processor Count: " << _processorCount << "\n";

    // メモリ情報(物理/仮想総量)
    MEMORYSTATUSEX memStatus;
    memStatus.dwLength = sizeof(memStatus);
    if (GlobalMemoryStatusEx(&memStatus)) {
        ofs << "Total Physical Memory: " << (memStatus.ullTotalPhys / (1024.0 * 1024.0)) << " MB\n";
        ofs << "Available Physical Memory: " << (memStatus.ullAvailPhys / (1024.0 * 1024.0)) << " MB\n";
        ofs << "Memory Load: " << memStatus.dwMemoryLoad << " %\n";
    }

    // ThreadPool情報
    ofs << "\n--- ThreadPool Status ---\n";
    ofs << "Worker Count: " << _threadWorkerCount << "\n";
    ofs << "Queue Size: " << _threadQueueSize << "\n";

    // スレッド詳細
    ofs << "\n--- Thread Details ---\n";
    for (const auto& t : _threads) {
        ofs << "[" << t.name << "] ";
        ofs << "TID=" << t.threadId << " ";
        ofs << "CPU=" << t.cpuPercent << "% ";
        ofs << "Kernel=" << t.kernelTimeUs << "us ";
        ofs << "User=" << t.userTimeUs << "us\n";
    }

    // スレッド状態
    ofs << "\n--- Thread States ---\n";
    {
        std::lock_guard<std::mutex> lk(_threadStateMtx);
        for (const auto& [tid, st] : _threadStates) {
            ofs << "TID=" << tid << " State=[" << st.state << "] ";
            ofs << "LastUpdate=" << st.lastUpdateUs << "us\n";
        }
    }

    // セクション詳細(全セクション、ソート済み)
    ofs << "\n--- Profiling Sections (All) ---\n";
    {
        std::lock_guard<std::mutex> lk(_sectionMtx);
        std::vector<SectionInfo> secs;
        secs.reserve(_sections.size());
        for (const auto& name : _sectionOrder) {
            auto it = _sections.find(name);
            if (it == _sections.end()) continue;
            const auto& st = it->second;
            secs.push_back(SectionInfo{ name, st.timeUs, st.calls });
        }
        std::sort(secs.begin(), secs.end(), [](const SectionInfo& a, const SectionInfo& b) {
            return a.timeUs > b.timeUs;
        });
        for (const auto& s : secs) {
            ofs << s.name << ": ";
            ofs << "Time=" << s.timeUs << "us ";
            ofs << "Calls=" << s.calls;
            if (s.calls > 0) {
                ofs << " Avg=" << (s.timeUs / s.calls) << "us";
            }
            ofs << "\n";
        }
    }

    ofs << "\n========================================\n\n";
}

// ============================================================
//  FPS
// ============================================================
void PerformanceMonitor::UpdateFps() {
    const int64_t now = NowMicroseconds();
    const int64_t frameDt = now - _prevFrameTime;
    _prevFrameTime = now;

    if (frameDt > 0) {
        _frameTimeMs = static_cast<float>(frameDt) / 1000.0f;
    }

#ifdef _DEBUG
    // Dump top sections when a frame becomes slow (helps find perf bottlenecks when many objects exist)
    // Threshold: ~30 FPS
    if (_frameTimeMs >= 33.0f) {
        DumpSlowFrameInfo_();
    }
#endif

    _fpsAccumTime += frameDt;
    ++_frameCount;
    if (_fpsAccumTime >= 1000000LL) { // 1秒ごとに更新
        _fps = static_cast<float>(_frameCount) * 1000000.0f / static_cast<float>(_fpsAccumTime);
        _frameCount = 0;
        _fpsAccumTime = 0;
    }
}

// ============================================================
//  Draw ? DxLib の DrawFormatString で描画
// ============================================================
void PerformanceMonitor::Draw(int x, int y) const {
#ifdef _DEBUG
    if (!_visible) return;

    const unsigned int white   = GetColor(255, 255, 255);
    const unsigned int yellow  = GetColor(255, 255, 0);
    const unsigned int green   = GetColor(100, 255, 100);
    const unsigned int cyan    = GetColor(100, 255, 255);
    const unsigned int red     = GetColor(255, 100, 100);
    const unsigned int orange  = GetColor(255, 180, 50);
    const unsigned int bgColor = GetColor(0, 0, 0);
    constexpr int lineH = 16;
    int curY = y;

    // 半透明背景
    const int bgW = 520;
    const auto sections = GetTopSections(10);
    const int totalLines = 6 + static_cast<int>(_threads.size()) + 2 + 2 + static_cast<int>(sections.size());
    SetDrawBlendMode(DX_BLENDMODE_ALPHA, 160);
    DrawBox(x - 4, y - 2, x + bgW, y + totalLines * lineH + 4, bgColor, TRUE);
    SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);

    // FPS / FrameTime
    const unsigned int fpsColor = (_fps >= 55.0f) ? green : (_fps >= 30.0f) ? yellow : red;
    DrawFormatString(x, curY, fpsColor, "FPS: %.1f  (%.2f ms)", _fps, _frameTimeMs);
    curY += lineH;

    // CPU
    const unsigned int cpuColor = (_cpuPercent < 50.0f) ? green : (_cpuPercent < 80.0f) ? yellow : red;
    DrawFormatString(x, curY, cpuColor, "CPU: %.1f%%  (%d cores)", _cpuPercent, _processorCount);
    curY += lineH;

    // Memory
    DrawFormatString(x, curY, cyan,
        "MEM: WorkingSet %.0f MB / Virtual %.0f MB",
        _workingSetMB, _virtualMemMB);
    curY += lineH;

    // GPU
    if (_gpuQueryAvailable) {
        const unsigned int gpuColor = (_gpuPercent < 60.0f) ? green : (_gpuPercent < 85.0f) ? yellow : red;
        DrawFormatString(x, curY, gpuColor,
            "GPU: VRAM %.0f MB (%.0f%%)  Shared %.0f MB",
            _gpuDedicatedMB, _gpuPercent, _gpuSharedMB);
    } else {
        DrawFormatString(x, curY, white, "GPU: N/A (DXGI 1.4 required)");
    }
    curY += lineH;

    // ThreadPool
    DrawFormatString(x, curY, orange,
        "ThreadPool: %zu workers / Queue: %zu",
        _threadWorkerCount, _threadQueueSize);
    curY += lineH;

    // Separator
    DrawFormatString(x, curY, white, "--- Per-Thread CPU ---");
    curY += lineH;

    // Per-thread
    for (const auto& info : _threads) {
        const unsigned int tColor = (info.cpuPercent < 30.0f) ? green
            : (info.cpuPercent < 70.0f) ? yellow : red;
        DrawFormatString(x, curY, tColor,
            "%-10s [%5u] CPU: %5.1f%%  K:%llu U:%llu us",
            info.name.c_str(), info.threadId,
            info.cpuPercent, info.kernelTimeUs, info.userTimeUs);
        curY += lineH;
    }

    // Separator
    DrawFormatString(x, curY, white, "--- Frame Sections (Top %zu) ---", sections.size());
    curY += lineH;
    for (const auto& s : sections) {
        const float ms = static_cast<float>(s.timeUs) / 1000.0f;
        const unsigned int c = (ms < 1.0f) ? green : (ms < 4.0f) ? yellow : red;
        DrawFormatString(x, curY, c, "%-32s %7.2f ms  (%u)",
            s.name.c_str(), ms, s.calls);
        curY += lineH;
    }
#else
    (void)x; (void)y;
#endif
}
