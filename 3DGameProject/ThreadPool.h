#pragma once
#pragma once

#include <cstddef>
#include <functional>
#include <future>
#include <mutex>
#include <condition_variable>
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

// ================================================================
//  ThreadPool - 高頻度な並列forループに最適化されたスレッドプール
// ================================================================
//
//  設計方針:
//    - Lock-freeなジョブ公開 (atomic _jobGen で release/acquire 同期)
//    - ワーカーは短時間スピン後にCV待機に移行 → 連続バリア呼び出しで低遅延
//    - False Sharing 回避: 頻繁に更新されるatomic変数を64B境界に配置
//    - バリアパスでヒープ割り当てゼロ
//    - バリアごとに1回の notify_all (チャンクごとではない)
//
//  主な用途:
//    - 物理演算・衝突判定などのフレーム内並列処理 (ParallelForBarrier)
//    - 非同期タスク実行 (Enqueue)
//
class ThreadPool {
public:
    // シングルトンインスタンス取得
    static ThreadPool& Instance() noexcept {
        static ThreadPool inst;
        return inst;
    }

    // ワーカースレッド数を取得
    size_t WorkerCount() const noexcept { return _workers.size(); }

    // Enqueue タスクキューのサイズを取得
    size_t QueueSize() const noexcept {
        std::lock_guard<std::mutex> lk(_mtx);
        return _tasks.size();
    }

    // ワーカースレッドのIDリストを取得 (デバッグ・計測用)
    std::vector<std::thread::id> WorkerThreadIds() const noexcept {
        std::vector<std::thread::id> ids;
        ids.reserve(_workers.size());
        for (const auto& w : _workers)
            ids.push_back(w.get_id());
        return ids;
    }

    // ワーカースレッドのネイティブハンドルを取得 (パフォーマンス計測用)
    std::vector<std::thread::native_handle_type> WorkerNativeHandles() noexcept {
        std::vector<std::thread::native_handle_type> handles;
        handles.reserve(_workers.size());
        for (auto& w : _workers)
            handles.push_back(w.native_handle());
        return handles;
    }

    // ================================================================
    //  Enqueue - 非同期タスク投入
    // ================================================================
    // タスクをキューに登録し、ワーカースレッドで実行
    // 戻り値は std::future で受け取れる
    //
    // 使用例:
    //   auto future = ThreadPool::Instance().Enqueue([]() { return 42; });
    //   int result = future.get(); // ブロック待機
    //
    template<typename F>
    auto Enqueue(F&& f) -> std::future<decltype(f())> {
        using R = decltype(f());
        auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
        std::future<R> result = task->get_future();
        {
            std::lock_guard<std::mutex> lk(_mtx);
            if (_stop) return result; // シャットダウン中は投入しない
            _tasks.emplace([task]() { (*task)(); });
        }
        // ワーカーを起床
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_one();
        return result;
    }

    // ================================================================
    //  ParallelFor - ParallelForBarrier へのエイリアス
    // ================================================================
    template<typename Func>
    void ParallelFor(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        ParallelForBarrier(begin, end, std::forward<Func>(func), grainSize);
    }

    template<typename Func>
    void ParallelFor(size_t begin, size_t end, Func&& func, size_t grainSize, size_t maxWorkers) {
        ParallelForBarrier(begin, end, std::forward<Func>(func), grainSize, maxWorkers);
    }

    // ================================================================
    //  ParallelForBarrier - 並列forループ（全スレッド同期待ち）
    // ================================================================
    // for (size_t i = begin; i < end; ++i) func(i); を並列実行
    // 全ワーカーが完了するまで呼び出し元はブロックされる
    //
    // 引数:
    //   begin, end: ループ範囲 [begin, end)
    //   func: ラムダ/関数オブジェクト。 void func(size_t index) の形式
    //   grainSize: チャンク粒度（最小でもこの単位で分割）
    //
    // 使用例:
    //   ThreadPool::Instance().ParallelForBarrier(0, 1000, [&](size_t i) {
    //       bodies[i]->Update();
    //   }, 64);
    //
    // 注意:
    //   - 再入禁止（バリア実行中に別のバリアを呼ぶと自動的にシリアル実行）
    //   - count < 32 の場合は並列化せずシリアル実行
    //
    template<typename Func>
    void ParallelForBarrier(size_t begin, size_t end, Func&& func, size_t grainSize = 1) {
        // --- 再入防止 ---
        // バリア実行中に再度バリアを呼ぶとデッドロック or ジョブ記述子破壊
        // → _barrierActive フラグで排他し、衝突時はシリアル実行にフォールバック
        if (_barrierActive.test_and_set(std::memory_order_acquire)) {
#ifdef _DEBUG
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ReentrantSerial");
#endif
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }
        // RAII でフラグを自動クリア
        struct BarrierGuard {
            std::atomic_flag& f;
            ~BarrierGuard() { f.clear(std::memory_order_release); }
        } guard{ _barrierActive };

        // --- 早期リターン（並列化不要な場合） ---
        if (begin >= end) return;
        const size_t count = end - begin;
        const size_t wc = _workers.size();
        // 閾値未満 or ワーカーなし → シリアル実行の方が速い
        constexpr size_t kMinParallelCount = 32;
        if (wc == 0 || count <= grainSize || count < kMinParallelCount) {
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }

        // --- チャンク分割 ---
        // 目標: ~(ワーカー数 + メイン) 個のチャンクに分割
        // 粗めにすることで fetch_add 競合を削減
        const size_t cs = (std::max)(grainSize, count / (wc + 1));
        const size_t nc = (count + cs - 1) / cs;

        // 関数ポインタ経由で呼び出し（テンプレート型消去）
        using FuncT = std::remove_reference_t<Func>;
        using Invoker = void(*)(size_t, void*);
        Invoker invoker = [](size_t i, void* ctx) {
            (*static_cast<FuncT*>(ctx))(i);
        };

        // --- ジョブ記述子を埋める ---
        // ワーカーは _jobGen が変わった後に読むので、ここは relaxed でOK
        _job.begin     = begin;
        _job.end       = end;
        _job.chunkSize = cs;
        _job.numChunks = static_cast<int>(nc);
        _job.nextChunk.store(0, std::memory_order_relaxed);
        _job.chunksCompleted.store(0, std::memory_order_relaxed);
        _job.invoker   = invoker;
        _job.ctx       = static_cast<void*>(const_cast<FuncT*>(&func));

        // --- ジョブ公開（奇数世代 = 実行中） ---
        const uint64_t jobGen = _jobGen.fetch_add(1, std::memory_order_release) + 1;

        // --- ワーカー起床 ---
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_all();

        // --- メインスレッドもチャンク処理に参加 ---
        // 遊ばせると無駄なので、メインも ProcessChunks_ を実行
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ProcessChunks");
            ProcessChunks_(jobGen);
        }
#else
        ProcessChunks_(jobGen);
#endif

        // --- 完了待機 (2段階: スピン → atomic wait) ---
        const int target = static_cast<int>(nc);

        // (1) 短期スピン（128回まで）
        //     フレーム内で連続してバリアが呼ばれる場合、OSスケジューラを介さず即復帰
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.SpinWait");
            for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
                if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                    _jobGen.fetch_add(1, std::memory_order_release); // 偶数に戻す（アイドル）
                    return;
                }
    #if defined(_MSC_VER)
                _mm_pause(); // CPUヒント（省電力スピン）
    #endif
            }
        }
#else
        for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
            if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                _jobGen.fetch_add(1, std::memory_order_release);
                return;
            }
#if defined(_MSC_VER)
            _mm_pause();
#endif
        }
#endif

        // (2) フォールバック: C++20 atomic wait
        //     スピン限界を超えたら、OSベースの待機に切り替え（電力効率）
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.CvWait");
            int cur = _job.chunksCompleted.load(std::memory_order_acquire);
            while (cur < target) {
                _job.chunksCompleted.wait(cur, std::memory_order_relaxed);
                cur = _job.chunksCompleted.load(std::memory_order_acquire);
            }
        }
#else
        {
            int cur = _job.chunksCompleted.load(std::memory_order_acquire);
            while (cur < target) {
                _job.chunksCompleted.wait(cur, std::memory_order_relaxed);
                cur = _job.chunksCompleted.load(std::memory_order_acquire);
            }
        }
#endif

        // --- ジョブ完了マーク（偶数世代 = アイドル） ---
        _jobGen.fetch_add(1, std::memory_order_release);
    }

    // ================================================================
    //  ParallelForBarrier (maxWorkers対応オーバーロード)
    // ================================================================
    // 最大ワーカー数を制限して並列実行
    // 軽い処理で少数スレッドのみ使用したい場合に有効
    //
    // 引数:
    //   maxWorkers: 使用するワーカー数の上限 (0 = 制限なし)
    //
    template<typename Func>
    void ParallelForBarrier(size_t begin, size_t end, Func&& func, size_t grainSize, size_t maxWorkers) {
        // --- 再入防止 ---
        if (_barrierActive.test_and_set(std::memory_order_acquire)) {
#ifdef _DEBUG
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ReentrantSerial");
#endif
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }
        struct BarrierGuard {
            std::atomic_flag& f;
            ~BarrierGuard() { f.clear(std::memory_order_release); }
        } guard{ _barrierActive };

        // --- 早期リターン ---
        if (begin >= end) return;
        const size_t count = end - begin;

        // ワーカー数を制限
        size_t wc = _workers.size();
        if (maxWorkers > 0 && maxWorkers < wc) {
            wc = maxWorkers;
        }

        constexpr size_t kMinParallelCount = 32;
        if (wc == 0 || count <= grainSize || count < kMinParallelCount) {
            for (size_t i = begin; i < end; ++i) func(i);
            return;
        }

        // --- チャンク分割（制限されたワーカー数で計算）---
        const size_t cs = (std::max)(grainSize, count / (wc + 1));
        const size_t nc = (count + cs - 1) / cs;

        using FuncT = std::remove_reference_t<Func>;
        using Invoker = void(*)(size_t, void*);
        Invoker invoker = [](size_t i, void* ctx) {
            (*static_cast<FuncT*>(ctx))(i);
        };

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

#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.ProcessChunks");
            ProcessChunks_(jobGen);
        }
#else
        ProcessChunks_(jobGen);
#endif

        const int target = static_cast<int>(nc);

#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.SpinWait");
            for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
                if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                    _jobGen.fetch_add(1, std::memory_order_release);
                    return;
                }
    #if defined(_MSC_VER)
                _mm_pause();
    #endif
            }
        }
#else
        for (int spin = 0; spin < kCompletionSpinLimit; ++spin) {
            if (_job.chunksCompleted.load(std::memory_order_acquire) >= target) {
                _jobGen.fetch_add(1, std::memory_order_release);
                return;
            }
#if defined(_MSC_VER)
            _mm_pause();
#endif
        }
#endif

#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("ThreadPool.Barrier.CvWait");
            int cur = _job.chunksCompleted.load(std::memory_order_acquire);
            while (cur < target) {
                _job.chunksCompleted.wait(cur, std::memory_order_relaxed);
                cur = _job.chunksCompleted.load(std::memory_order_acquire);
            }
        }
#else
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

    // デストラクタ - 全ワーカースレッドを終了
    ~ThreadPool() {
        _stop.store(true, std::memory_order_release);
        _jobGen.fetch_add(1, std::memory_order_release);
        _wakeGen.fetch_add(1, std::memory_order_release);
        _wakeGen.notify_all();
        _doneCv.notify_all();
        for (auto& w : _workers) {
            if (w.joinable()) w.join();
        }
    }

    // コピー・ムーブ禁止
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

private:
    // ================================================================
    //  JobDesc - バリアジョブ記述子（False Sharing 回避）
    // ================================================================
    // 各フィールドを64バイト境界に配置して、異なるキャッシュラインに分離
    // → 複数スレッドからの書き込みによる不要なキャッシュ無効化を防止
    //
    struct JobDesc {
        size_t begin = 0;           // ループ開始インデックス
        size_t end = 0;             // ループ終了インデックス（含まない）
        size_t chunkSize = 1;       // 1チャンクのサイズ
        int numChunks = 0;          // 総チャンク数
        void (*invoker)(size_t, void*) = nullptr;  // 関数ポインタ
        void* ctx = nullptr;        // ユーザー関数オブジェクトへのポインタ

        // 各ワーカーが fetch_add で取得 → 別キャッシュラインに配置
        alignas(64) std::atomic<int> nextChunk{0};
        // 完了チャンク数（完了待機に使用）
        alignas(64) std::atomic<int> chunksCompleted{0};
    };

    // ================================================================
    //  内部状態（全て64バイト境界アライン）
    // ================================================================
    alignas(64) JobDesc _job;                   // バリアジョブ記述子
    alignas(64) std::atomic<uint64_t> _jobGen{0};   // ジョブ世代（偶数=アイドル、奇数=実行中）
    alignas(64) std::atomic<uint64_t> _wakeGen{0};  // ワーカー起床用カウンタ
    alignas(64) std::atomic<bool> _stop{false};     // シャットダウンフラグ
    std::atomic_flag _barrierActive = ATOMIC_FLAG_INIT; // バリア再入防止フラグ

    // スピン回数の調整値（1?2μsバリア間隔を想定）
    static constexpr int kWorkerSpinLimit = 256;       // ワーカースレッドのスピン回数
    static constexpr int kCompletionSpinLimit = 128;   // 完了待機スピン回数

    // ================================================================
    //  ProcessChunks_ - チャンク取得＆実行ループ
    // ================================================================
    // ワーカー（とメイン）が並列に呼び出す内部関数
    // _job.nextChunk を fetch_add でlock-freeに取得し、対応する範囲を実行
    //
    // 引数:
    //   jobGenSnapshot: このジョブ開始時の _jobGen 値
    //                   ループ中に世代が変わったら即座に break（次ジョブ混線防止）
    //
    void ProcessChunks_(uint64_t jobGenSnapshot) noexcept {
        // ジョブ記述子をローカルにコピー（最初に1回だけ読む）
        auto inv = _job.invoker;
        void* ctx = _job.ctx;
        const size_t beg = _job.begin;
        const size_t end = _job.end;
        const size_t csz = _job.chunkSize;
        const int nc = _job.numChunks;

        for (;;) {
            // ★世代チェック（重要）
            // 前のバリアから戻り途中のワーカーが、次バリアのチャンクを誤って消費するのを防ぐ
            // メインスレッドは完了後すぐに次のバリアを publish できるため、
            // ここで世代が変わったら即座に break する
            if (_jobGen.load(std::memory_order_acquire) != jobGenSnapshot) break;

            // 次のチャンク番号を取得（lock-free）
            const int c = _job.nextChunk.fetch_add(1, std::memory_order_relaxed);
            if (c >= nc) break; // 全チャンク消費済み

            // チャンク範囲を計算して実行
            const size_t lo = beg + static_cast<size_t>(c) * csz;
            const size_t hi = (std::min)(lo + csz, end);
            for (size_t i = lo; i < hi; ++i) inv(i, ctx);

            // 完了チャンク数をインクリメント
            const int done = _job.chunksCompleted.fetch_add(1, std::memory_order_release) + 1;
            if (done >= nc) {
                // 全チャンク完了 → 待機中スレッドを起床
                _job.chunksCompleted.notify_all();
            }
        }
    }

    // ================================================================
    //  WorkerLoop_ - ワーカースレッドのメインループ
    // ================================================================
    // 各ワーカースレッドがコンストラクタで起動し、シャットダウンまで実行し続ける
    //
    // 2段階待機戦略:
    //   Phase 1: 短期スピン（256回まで）
    //            → フレーム内で連続してバリアが呼ばれる場合、遅延ほぼゼロで即応答
    //   Phase 2: CV待機（atomic wait）
    //            → 長時間アイドル時はOSスケジューラに yield して省電力
    //
    void WorkerLoop_() {
        uint64_t myGen = _jobGen.load(std::memory_order_relaxed);

        for (;;) {
            // ============================================================
            // Phase 1: 短期スピン（ホットループ）
            // ============================================================
            // 連続バリア呼び出し時の遅延を最小化するため、
            // しばらくはCPUをスピンさせて _jobGen の変化を監視
#ifdef _DEBUG
            PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.Spin");
#endif
            for (int spin = 0; spin < kWorkerSpinLimit; ++spin) {
                if (_stop.load(std::memory_order_relaxed)) return; // シャットダウン

                const uint64_t cur = _jobGen.load(std::memory_order_acquire);
                // 世代が変わった && 奇数（ジョブ実行中）
                if (cur != myGen && (cur & 1) == 1) {
                    myGen = cur;
#ifdef _DEBUG
                    PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.ProcessChunks");
#endif
                    ProcessChunks_(myGen); // チャンク処理
                    _doneCv.notify_all();
                    spin = -1; // 完了後もスピンを継続（次バリアに即対応）
                    continue;
                }
#if defined(_MSC_VER)
                _mm_pause(); // CPUヒント: 省電力スピン
#endif
            }

            // ============================================================
            // Phase 2: スリープ（長期アイドル時）
            // ============================================================
            {
                const uint64_t expectedWake = _wakeGen.load(std::memory_order_acquire);

                // (1) Enqueue タスクがあれば優先処理
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
                        continue; // タスク完了後、再度 Phase 1 へ
                    }
                }

                // (2) バリアジョブがあればまだ処理
                const uint64_t cur = _jobGen.load(std::memory_order_acquire);
                if (cur != myGen && (cur & 1) == 1) {
                    myGen = cur;
#ifdef _DEBUG
                    PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.ProcessChunks");
#endif
                    ProcessChunks_(myGen);
                    _doneCv.notify_all();
                    continue;
                }
                myGen = cur;

                // (3) どちらもなければ C++20 atomic wait でスリープ
                //     _wakeGen が変わるまでOSに制御を渡して省電力
                if (_stop.load(std::memory_order_relaxed)) return;
#ifdef _DEBUG
                PerformanceMonitor::Instance().SetThreadState("ThreadPool.Worker.AtomicWait");
#endif
                _wakeGen.wait(expectedWake, std::memory_order_relaxed);
            }
        }
    }

    // ================================================================
    //  コンストラクタ（private、シングルトン）
    // ================================================================
    // ワーカースレッドを起動
    // 物理演算・衝突判定のような50?200オブジェクト規模では、
    // 3?4ワーカーが最も効率が良い（それ以上増やすとバリアオーバーヘッドが支配的になる）
    //
    ThreadPool() {
        const unsigned int hw = std::thread::hardware_concurrency();
        // ワーカー数上限: 小規模?中規模の並列処理に最適化
        // オブジェクト数が500個を超える場合は kMaxWorkers を増やすと良い
        constexpr unsigned int kMaxWorkers = 8;
        const unsigned int nw = (hw > 1) ? (std::min)(hw - 1, kMaxWorkers) : 1;
        _workers.reserve(nw);
        for (unsigned int i = 0; i < nw; ++i)
            _workers.emplace_back([this]() { WorkerLoop_(); });
    }

    // ================================================================
    //  メンバ変数
    // ================================================================
    std::vector<std::thread> _workers;          // ワーカースレッド配列
    std::queue<std::function<void()>> _tasks;   // Enqueue タスクキュー
    mutable std::mutex _mtx;                    // タスクキュー保護用mutex
    std::condition_variable _cv;                // （未使用: 将来用）
    std::condition_variable _doneCv;            // ワーカー完了通知用
};
