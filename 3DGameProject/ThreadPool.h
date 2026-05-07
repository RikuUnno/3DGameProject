#pragma once

#include <cstddef>
#include <future>
#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <algorithm>
#include <atomic>

#ifdef _DEBUG
#include "PerformanceMonitor.h"
#endif

#if defined(_MSC_VER)
#include <intrin.h>
#endif

// 物理演算・衝突判定などの高頻度な並列 for ループに最適化されたスレッドプール。
//
// 設計方針:
//   - Lock-free なジョブ公開 (_jobGen の release/acquire 同期)
//   - ワーカーは短期スピン後に atomic wait へ移行（連続バリア呼び出しで低遅延）
//   - False Sharing 回避: 頻繁に更新する atomic 変数を 64B 境界に配置
//   - バリアパスでヒープ割り当てゼロ
//
class ThreadPool {
public:
    // シングルトン
    static ThreadPool& Instance() noexcept {
        static ThreadPool inst;
        return inst;
    }

    size_t WorkerCount() const noexcept { return _workers.size(); }

    size_t QueueSize() const noexcept {
        std::lock_guard<std::mutex> lk(_mtx);
        return _tasks.size();
    }

    std::vector<std::thread::id> WorkerThreadIds() const noexcept {
        std::vector<std::thread::id> ids;
        ids.reserve(_workers.size());
        for (const auto& w : _workers)
            ids.push_back(w.get_id());
        return ids;
    }

    std::vector<std::thread::native_handle_type> WorkerNativeHandles() noexcept {
        std::vector<std::thread::native_handle_type> handles;
        handles.reserve(_workers.size());
        for (auto& w : _workers)
            handles.push_back(w.native_handle());
        return handles;
    }

    // ---- Enqueue -------------------------------------------------------
    // std::function は CopyConstructible を要求するため packaged_task を直接格納できない。
    // unique_ptr ベースの型消去 (MoveOnlyTask) で move-only なタスクをキューに格納する。

    struct ITask { virtual void invoke() = 0; virtual ~ITask() = default; };

    template<typename PT>
    struct TaskHolder final : ITask {
        PT pt;
        explicit TaskHolder(PT&& p) : pt(std::move(p)) {}
        void invoke() override { pt(); }
    };

    struct MoveOnlyTask {
        std::unique_ptr<ITask> ptr;
        void operator()() { ptr->invoke(); }
    };

    // タスクをキューに投入し、ワーカースレッドで非同期実行する。
    // 戻り値の std::future で結果を受け取れる。
    template<typename F>
    auto Enqueue(F&& f) -> std::future<std::invoke_result_t<F>> {
        using R  = std::invoke_result_t<F>;
        using PT = std::packaged_task<R()>;
        PT pt(std::forward<F>(f));
        std::future<R> result = pt.get_future();
        {
            std::lock_guard<std::mutex> lk(_mtx);
            if (_stop) return result;
            _tasks.push(MoveOnlyTask{ std::make_unique<TaskHolder<PT>>(std::move(pt)) });
        }
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_one();
        return result;
    }

    // ---- ParallelFor (ParallelForBarrier へのエイリアス) ----------------

    template<typename Func>
    void ParallelFor(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        ParallelForBarrier(begin, end, std::forward<Func>(func), grainSize);
    }

    template<typename Func>
    void ParallelFor(size_t begin, size_t end, Func&& func, size_t grainSize, size_t maxWorkers) {
        ParallelForBarrier(begin, end, std::forward<Func>(func), grainSize, maxWorkers);
    }

    // ---- ParallelForBarrier --------------------------------------------
    // [begin, end) を並列実行し、全ワーカー完了まで呼び出し元をブロックする。
    // バリア実行中に再入した場合はシリアル実行にフォールバックする。
    // count < 32 の場合もシリアル実行。

    template<typename Func>
    void ParallelForBarrier(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        if (_barrierActive.test_and_set(std::memory_order_acquire)) {
#ifdef _DEBUG
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ReentrantSerial");
#endif
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }
        struct Guard {
            std::atomic_flag& f;
            ~Guard() { f.clear(std::memory_order_release); }
        } guard{ _barrierActive };
        DispatchParallel_(begin, end, std::forward<Func>(func), grainSize, _workers.size(), kDefaultMinParallel);
    }

    // maxWorkers: 使用するワーカー数の上限 (0 = 制限なし)
    template<typename Func>
    void ParallelForBarrier(size_t begin, size_t end, Func&& func, size_t grainSize, size_t maxWorkers) {
        if (_barrierActive.test_and_set(std::memory_order_acquire)) {
#ifdef _DEBUG
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ReentrantSerial");
#endif
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }
        struct Guard {
            std::atomic_flag& f;
            ~Guard() { f.clear(std::memory_order_release); }
        } guard{ _barrierActive };
        size_t wc = _workers.size();
        if (maxWorkers > 0 && maxWorkers < wc) wc = maxWorkers;
        DispatchParallel_(begin, end, std::forward<Func>(func), grainSize, wc, kDefaultMinParallel);
    }

    // ---- ParallelForBarrierHeavy ---------------------------------------
    // 1アイテムのコストが大きい場合専用。kDefaultMinParallel(32) を無視し
    // アイテム数 >= 2 で並列化する（アイランドソルバー等）。

    template<typename Func>
    void ParallelForBarrierHeavy(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        if (_barrierActive.test_and_set(std::memory_order_acquire)) {
#ifdef _DEBUG
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ReentrantSerial");
#endif
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }
        struct Guard {
            std::atomic_flag& f;
            ~Guard() { f.clear(std::memory_order_release); }
        } guard{ _barrierActive };
        DispatchParallel_(begin, end, std::forward<Func>(func), grainSize, _workers.size(), 2);
    }

    ~ThreadPool() {
        _stop.store(true, std::memory_order_release);
        _jobGen.fetch_add(1, std::memory_order_release);
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_all();
        for (auto& w : _workers) {
            if (w.joinable()) w.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

private:
    // 軽量処理でオーバーヘッド > 効果になる並列化の最小アイテム数
    static constexpr size_t kDefaultMinParallel  = 32;
    // ワーカーの短期スピン回数（フレーム内連続バリアに即応するため）
    static constexpr int    kWorkerSpinLimit     = 256;
    // 完了待機スピン回数（atomic wait 移行前）
    static constexpr int    kCompletionSpinLimit = 128;

    // ---- DispatchParallel_ ---------------------------------------------
    // ParallelForBarrier* の共通実装。wc = 使用ワーカー数、minCount = 並列化最小数。

    template<typename Func>
    void DispatchParallel_(size_t begin, size_t end, Func&& func,
        size_t grainSize, size_t wc, size_t minCount) noexcept
    {
        if (begin >= end) return;
        const size_t count = end - begin;
        if (wc == 0 || count <= grainSize || count < minCount) {
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }

        const size_t cs = (std::max)(grainSize, count / (wc + 1));
        const size_t nc = (count + cs - 1) / cs;

        using FuncT   = std::remove_reference_t<Func>;
        using Invoker = void(*)(size_t, void*);
        Invoker invoker = [](size_t i, void* ctx) { (*static_cast<FuncT*>(ctx))(i); };

        _job.begin     = begin;
        _job.end       = end;
        _job.chunkSize = cs;
        _job.numChunks = static_cast<int>(nc);
        _job.nextChunk.store(0, std::memory_order_relaxed);
        _job.chunksCompleted.store(0, std::memory_order_relaxed);
        _job.invoker   = invoker;
        _job.ctx       = static_cast<void*>(const_cast<FuncT*>(&func));

        const uint64_t jobGen = _jobGen.fetch_add(1, std::memory_order_release) + 1;
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_all();

        ProcessChunks_(jobGen);

        const int target = static_cast<int>(nc);

        // 最速パス: ProcessChunks_ でメインが全チャンクを処理済みなら即リターン
        if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
            _jobGen.fetch_add(1, std::memory_order_release); return;
        }

        // スピン待機
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.SpinWait");
            for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
                if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                    _jobGen.fetch_add(1, std::memory_order_release); return;
                }
#if defined(_MSC_VER)
                _mm_pause();
#endif
            }
        }
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.CvWait");
            int cur = _job.chunksCompleted.load(std::memory_order_acquire);
            while (cur < target) {
                _job.chunksCompleted.wait(cur, std::memory_order_relaxed);
                cur = _job.chunksCompleted.load(std::memory_order_acquire);
            }
        }
#else
        for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
            if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                _jobGen.fetch_add(1, std::memory_order_release); return;
            }
#if defined(_MSC_VER)
            _mm_pause();
#endif
        }
        {
            int cur = _job.chunksCompleted.load(std::memory_order_acquire);
            while (cur < target) {
                _job.chunksCompleted.wait(cur, std::memory_order_relaxed);
                cur = _job.chunksCompleted.load(std::memory_order_acquire);
            }
        }
#endif
        _jobGen.fetch_add(1, std::memory_order_release);
    }

    // ---- JobDesc -------------------------------------------------------
    // バリアジョブの記述子。nextChunk と chunksCompleted は別キャッシュラインに配置して
    // 複数スレッドからの書き込みによる False Sharing を防ぐ。

    struct JobDesc {
        size_t begin   = 0;
        size_t end     = 0;
        size_t chunkSize = 1;
        int    numChunks = 0;
        void (*invoker)(size_t, void*) = nullptr;
        void*  ctx = nullptr;

        alignas(64) std::atomic<int> nextChunk{ 0 };      // ワーカーが fetch_add で取得
        alignas(64) std::atomic<int> chunksCompleted{ 0 }; // 完了待機に使用
    };

    // ---- 内部状態（64B 境界アライン、False Sharing 回避）---------------

    alignas(64) JobDesc                  _job;
    alignas(64) std::atomic<uint64_t>    _jobGen{ 0 };    // ジョブ世代（奇数=実行中）
    alignas(64) std::atomic<uint64_t>    _wakeGen{ 0 };   // ワーカー起床カウンタ
    alignas(64) std::atomic<bool>        _stop{ false };

    std::atomic_flag _barrierActive = ATOMIC_FLAG_INIT;   // バリア再入防止

    // ---- ProcessChunks_ ------------------------------------------------
    // ワーカーとメインが並列に呼び出すチャンク実行ループ。
    // jobGenSnapshot が変わった時点で即 break（次ジョブとの混線防止）。

    void ProcessChunks_(uint64_t jobGenSnapshot) noexcept {
        auto inv         = _job.invoker;
        void*  ctx       = _job.ctx;
        const size_t beg = _job.begin;
        const size_t end = _job.end;
        const size_t csz = _job.chunkSize;
        const int    nc  = _job.numChunks;

        for (;;) {
            if (_jobGen.load(std::memory_order_acquire) != jobGenSnapshot) break;

            const int c = _job.nextChunk.fetch_add(1, std::memory_order_relaxed);
            if (c >= nc) break;

            const size_t lo = beg + static_cast<size_t>(c) * csz;
            const size_t hi = (std::min)(lo + csz, end);
            for (size_t i = lo; i < hi; ++i) inv(i, ctx);

            const int done = _job.chunksCompleted.fetch_add(1, std::memory_order_release) + 1;
            if (done >= nc) _job.chunksCompleted.notify_all();
        }
    }

    // ---- WorkerLoop_ ---------------------------------------------------
    // ワーカースレッドのメインループ。
    //
    // Phase 1（短期スピン）: 連続バリア呼び出し時の遅延を最小化
    // Phase 2（atomic wait）: 長時間アイドル時は OS に制御を渡して省電力

    void WorkerLoop_() {
        uint64_t myGen = _jobGen.load(std::memory_order_relaxed);

        for (;;) {
            // Phase 1: スピン
#ifdef _DEBUG
            PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.Spin");
#endif
            for (int spin = 0; spin < kWorkerSpinLimit; ++spin) {
                if (_stop.load(std::memory_order_relaxed)) return;

                const uint64_t cur = _jobGen.load(std::memory_order_acquire);
                if (cur != myGen && (cur & 1) == 1) {
                    myGen = cur;
#ifdef _DEBUG
                    PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.ProcessChunks");
#endif
                    ProcessChunks_(myGen);
                    spin = -1; // 完了後もスピン継続（次バリアに即対応）
                    continue;
                }
#if defined(_MSC_VER)
                _mm_pause();
#endif
            }

            // Phase 2: スリープ
            {
                const uint64_t expectedWake = _wakeGen.load(std::memory_order_acquire);

                // Enqueue タスクがあれば優先処理
                {
                    std::unique_lock<std::mutex> lk(_mtx);
                    if (_stop.load(std::memory_order_relaxed)) return;
                    if (!_tasks.empty()) {
                        auto task = std::move(_tasks.front());
                        _tasks.pop();
                        lk.unlock();
#ifdef _DEBUG
                        PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.Task");
#endif
                        task();
                        continue;
                    }
                }

                // バリアジョブが来ていれば処理
                const uint64_t cur = _jobGen.load(std::memory_order_acquire);
                if (cur != myGen && (cur & 1) == 1) {
                    myGen = cur;
#ifdef _DEBUG
                    PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.ProcessChunks");
#endif
                    ProcessChunks_(myGen);
                    continue;
                }
                myGen = cur;

                if (_stop.load(std::memory_order_relaxed)) return;
#ifdef _DEBUG
                PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.AtomicWait");
#endif
                _wakeGen.wait(expectedWake, std::memory_order_relaxed);
            }
        }
    }

    // ---- コンストラクタ（シングルトン用、private）----------------------

    ThreadPool() {
        const unsigned int hw = std::thread::hardware_concurrency();
        constexpr unsigned int kMaxWorkers = 16;
        const unsigned int nw = (hw > 1) ? (std::min)(hw - 1, kMaxWorkers) : 1;
        _workers.reserve(nw);
        for (unsigned int i = 0; i < nw; ++i)
            _workers.emplace_back([this]() { WorkerLoop_(); });
    }

    // ---- メンバ変数 ----------------------------------------------------

    std::vector<std::thread> _workers;
    std::queue<MoveOnlyTask> _tasks;
    mutable std::mutex       _mtx;
};
