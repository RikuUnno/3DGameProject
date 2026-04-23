#include "PhysicsManager.h"

// ============================================================
//  PhysicsManager.cpp
// ============================================================
// 物理エンジンの中核実装
//
// 主な機能:
//   - 固定時間ステップでの物理演算（重力・速度積分・衝突応答）
//   - Island（独立した物体グループ）による並列ソルバー
//   - Warm-start による高速収束（前フレームの累積インパルスを再利用）
//   - Speculative CCD（トンネリング防止の投機的接触生成）
//   - Split Impulse（位置補正を速度に影響させない方法）
//
// 参考文献:
//   - Erin Catto (Box2D/GDC講演): Sequential Impulse, Warm-starting, Island splitting
//   - Havok Physics: Speculative contacts, TOI backstep
//
#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "Assert.h"
#include "PhysicsController.h"
#include "PhysicsBody.h"
#include "PhysicsMaterial.h"
#include "GameObject.h"
#include "ColliderManager.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "HalfPlaneCollider.h"
#include "CompoundCollider.h"
#include "Transform.h"
#include "ThreadPool.h"
#include "PerformanceMonitor.h"

#if defined(_MSC_VER)
#include <intrin.h>
#endif

// ============================================================
// 内部ヘルパー関数（無名namespace）
// ============================================================
namespace {
    // ============================================================
    //  IsLikelyBadRef_ - デバッグ用: 不正なポインタ検出
    // ============================================================
    // メモリ破壊やダングリング参照を早期検出するため、
    // 明らかに不正なアドレス（nullptr、0xFFFFFFFF、カーネル空間など）を判定
    inline bool IsLikelyBadRef_(const void* p) noexcept {
        const auto u = static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(p));
        // よくあるセンチネル値をキャッチ
        if (u == 0 || u == static_cast<std::uintptr_t>(~0ULL)) return true;
#if defined(_WIN64)
        // ユーザーモードの正規アドレスは通常 0x0000800000000000 未満
        if (u >= 0x0000800000000000ULL) return true;
#endif
        return false;
    }

    // ============================================================
    //  ベクトル演算ヘルパー
    // ============================================================
    // DxLib の VECTOR 型に対する基本演算
    // デバッグビルドでは不正参照を検出してアサート

    /// 内積 (a・b)
    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
#ifndef NDEBUG
        if (IsLikelyBadRef_(&a) || IsLikelyBadRef_(&b)) {
            ASSERT_MSG(false, "Dot3: invalid VECTOR reference. &a=%p &b=%p", &a, &b);
            return 0.0f;
        }
#endif
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// 長さの二乗
    inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }

    /// 長さ（負にならないよう max(0) でクランプ）
    inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v), 0.0f)); }

    /// 安全な正規化（長さゼロの場合は fallback を返す）
    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback = VGet(0,1,0)) noexcept {
        const float len = Len3(v);
        if (len > 1e-6f) return VScale(v, 1.0f / len);
        return fallback;
    }

    /// ベクトルの長さを maxMag 以下にクランプ
    inline void ClampMagnitude(VECTOR& v, float maxMag) noexcept {
        if (maxMag <= 0.0f) return;
        const float ls = LenSq(v);
        if (ls <= maxMag * maxMag) return;
        v = VScale(v, maxMag / std::sqrt((std::max)(ls, 1e-8f)));
    }

    /// 有限値チェック（NaN/Inf 検出）
    inline bool IsFinite(float v) noexcept { return std::isfinite(v); }
    inline bool IsFiniteVec(const VECTOR& v) noexcept { return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z); }

    /// NaN/Inf が混入していたらゼロベクトルに修正
    inline void SanitizeVec(VECTOR& v) noexcept { if (!IsFiniteVec(v)) v = VGet(0,0,0); }

    // ============================================================
    //  物理演算パラメータ（調整可能な定数）
    // ============================================================
    constexpr float kBiasFactor   = 0.2f;   // Baumgarte安定化の係数（大きいほど補正が強い）
    constexpr float kSlop         = 0.005f; // 許容めり込み深度（これ以下は無視）
    constexpr float kMaxPen       = 5.0f;   // 最大めり込み深度（これを超えるとクランプ）
    constexpr float kMaxCorrection = 0.5f;  // 1フレームあたりの最大位置補正量
    constexpr float kRestitutionThreshold = 0.05f; // 反発を適用する最低速度
    constexpr float kWarmStartFactor = 0.8f;       // Warm-start の減衰率（1.0=完全再利用、0.0=無効）
    constexpr float kContactMatchDistSq = 0.04f;   // 接触点マッチング距離の二乗（0.2m^2）
    constexpr float kSplitBiasFactor = 0.1f;       // Split Impulse の補正係数
    constexpr float kSpeculativeMargin = 0.02f;    // Speculative 接触のマージン（2cm）

    // ============================================================
    //  GetColliderMinHalfExtent - コライダーの最小半径を取得
    // ============================================================
    // CCD（連続衝突検出）で「トンネリング判定に使う代表サイズ」を返す
    // 球→半径、箱→最小辺の半分、カプセル→半径
    inline float GetColliderMinHalfExtent(const Collider* col) noexcept {
        if (!col) return 0.5f;
        switch (col->GetKind()) {
        case Collider::Kind::Sphere: 
            return static_cast<const SphereCollider*>(col)->GetRadius();
        case Collider::Kind::Box: {
            const VECTOR he = static_cast<const BoxCollider*>(col)->GetHalfExtents();
            return (std::min)({he.x, he.y, he.z}); // 最小辺の半分
        }
        case Collider::Kind::Capsule: 
            return static_cast<const CapsuleCollider*>(col)->GetRadius();
        default: 
            return 0.5f;
        }
    }

    // ============================================================
    //  ComputeTangentBasis - 法線から接線基底を生成
    // ============================================================
    // 摩擦を2D円錐拘束（Coulomb cone）で解くため、法線 n に直交する2つの接線ベクトルを作る
    // n とほぼ平行でない軸（X or Y）と外積を取って t1 を生成し、t2 = n × t1 で完成
    inline void ComputeTangentBasis(const VECTOR& n, VECTOR& t1, VECTOR& t2) noexcept {
        if (std::fabs(n.x) < 0.9f)
            t1 = SafeNormalize(VCross(n, VGet(1,0,0))); // X軸と外積
        else
            t1 = SafeNormalize(VCross(n, VGet(0,1,0))); // Y軸と外積
        t2 = VCross(n, t1); // t2 = n × t1（既に正規化済み）
    }

    // ============================================================
    //  ComputeEffectiveInvMass - 有効逆質量の計算
    // ============================================================
    // 接触点でのインパルス応答を計算するため、
    // 並進質量 + 回転慣性（角運動量の寄与）を合算した「有効逆質量」を返す
    //
    // 公式:
    //   K = 1/mA + 1/mB + (rA × dir)^T * IA^-1 * (rA × dir)
    //                   + (rB × dir)^T * IB^-1 * (rB × dir)
    //
    // ここで dir は拘束方向（法線 or 接線）、rA/rB は接触点への腕ベクトル
    inline float ComputeEffectiveInvMass(
        float invA, float invB,
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& dir) noexcept
    {
        float result = invA + invB; // 並進成分
        // 回転成分（bodyA）
        if (bodyA && invA > 0.0f && !bodyA->_freezeRotation) {
            const VECTOR tmp = bodyA->ApplyInverseInertia(VCross(rA, dir));
            result += Dot3(VCross(tmp, rA), dir);
        }
        // 回転成分（bodyB）
        if (bodyB && invB > 0.0f && !bodyB->_freezeRotation) {
            const VECTOR tmp = bodyB->ApplyInverseInertia(VCross(rB, dir));
            result += Dot3(VCross(tmp, rB), dir);
        }
        return result;
    }

    // ============================================================
    //  ApplyImpulse - インパルス（瞬間力積）を2物体に適用
    // ============================================================
    // 接触拘束・摩擦拘束などで計算したインパルスを速度に反映
    //
    // 並進速度: v += J / m
    // 角速度:   ω += I^-1 * (r × J)
    //
    // bodyA には -impulse、bodyB には +impulse を適用（作用反作用の法則）
    inline void ApplyImpulse(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        float invA, float invB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& impulse) noexcept
    {
        if (bodyA && invA > 0.0f) {
            // 並進速度を減らす（-impulse 方向）
            bodyA->_velocity = VSub(bodyA->_velocity, VScale(impulse, invA));
            // 角速度も減らす（トルク = rA × (-impulse)）
            if (!bodyA->_freezeRotation)
                bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
                    bodyA->ApplyInverseInertia(VCross(rA, impulse)));
        }
        if (bodyB && invB > 0.0f) {
            // 並進速度を増やす（+impulse 方向）
            bodyB->_velocity = VAdd(bodyB->_velocity, VScale(impulse, invB));
            // 角速度も増やす（トルク = rB × impulse）
            if (!bodyB->_freezeRotation)
                bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
                    bodyB->ApplyInverseInertia(VCross(rB, impulse)));
        }
    }

    // ============================================================
    //  RelNormalVelocity - 法線方向の相対速度
    // ============================================================
    // 接触点での2物体の「近づく速度」を返す
    // 正の値 = 離れていく、負の値 = 近づいていく
    //
    // v_contact = (vB + ωB × rB) - (vA + ωA × rA)
    // 法線成分 = v_contact ・ n
    inline float RelNormalVelocity(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& n) noexcept
    {
        // 接触点での速度 = 並進速度 + 角速度×腕
        const VECTOR vA = VAdd(bodyA ? bodyA->_velocity : VGet(0,0,0),
            bodyA ? VCross(bodyA->_angularVelocity, rA) : VGet(0,0,0));
        const VECTOR vB = VAdd(bodyB ? bodyB->_velocity : VGet(0,0,0),
            bodyB ? VCross(bodyB->_angularVelocity, rB) : VGet(0,0,0));
        return Dot3(VSub(vB, vA), n); // (vB - vA) ・ n
    }

    /// 任意方向 d の相対速度（RelNormalVelocity のエイリアス）
    inline float RelDirVelocity(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& d) noexcept
    {
        return RelNormalVelocity(bodyA, bodyB, rA, rB, d);
    }

    // ============================================================
    //  HalfPlaneDistance - 半平面からの符号付き距離
    // ============================================================
    // 平面方程式: n ・ p - d = 0
    // 正の値 = 平面の表側（空間）、負の値 = 裏側（固体内）
    inline float HalfPlaneDistance(const VECTOR& point, const VECTOR& planeNormal, float planeD) noexcept {
        return Dot3(point, planeNormal) - planeD;
    }
}

// ============================================================
//  ライフサイクル
// ============================================================

/// <summary>
/// PhysicsManagerのシャットダウン処理
/// 非同期物理演算の完了を待機し、全ての物理オブジェクトをクリアする
/// </summary>
void PhysicsManager::Shutdown() {
    const bool was = _shuttingDown.exchange(true, std::memory_order_relaxed);
    if (was) return;
    // 状態をクリアする前に進行中の非同期物理演算を待機
    WaitForPhysics();
    std::lock_guard lk(_mtx);
    _controllers.clear();
    _bodies.clear();
    _solverContacts.clear();
    _prevSolverContacts.clear();
    _islands.clear();
    _bodyIslandMap.clear();
    _accumulator = 0.0f;
}

/// <summary>
/// 固定物理ステップのデルタタイムを設定
/// </summary>
/// <param name="fixedDeltaTime">物理シミュレーションの固定時間刻み（秒）。1e-4f未満の場合は1/120秒にクランプ</param>
void PhysicsManager::SetFixedDeltaTime(float fixedDeltaTime) noexcept {
    _fixedDeltaTime = (fixedDeltaTime > 1e-4f) ? fixedDeltaTime : (1.0f / 120.0f);
}

/// <summary>
/// 1フレームあたりの最大物理サブステップ数を設定
/// </summary>
/// <param name="maxSubSteps">最大サブステップ数。1未満の場合は1にクランプ</param>
void PhysicsManager::SetMaxSubSteps(int maxSubSteps) noexcept {
    _maxSubSteps = (maxSubSteps > 1) ? maxSubSteps : 1;
}

/// <summary>
/// ソルバーの反復回数を設定
/// </summary>
/// <param name="solverIterations">拘束ソルバーの反復回数。1未満の場合は1にクランプ</param>
void PhysicsManager::SetSolverIterations(int solverIterations) noexcept {
    _solverIterations = (solverIterations > 1) ? solverIterations : 1;
}

/// <summary>
/// アダプティブなソルバー反復回数を計算
/// 接触数が多いほど反復回数を増やし、[min, max]にクランプ
/// </summary>
/// <returns>計算された反復回数</returns>
int PhysicsManager::ComputeAdaptiveIterations() const noexcept {
    const int contactCount = static_cast<int>(_solverContacts.size());
    // ヒューリスティック: 基本反復回数 + 10接触ごとに1回追加
    int adaptive = _minSolverIterations + contactCount / 10;
    adaptive = (std::max)(adaptive, _minSolverIterations);
    adaptive = (std::min)(adaptive, _maxSolverIterations);
    // 範囲内であれば明示的な _solverIterations も尊重
    return (std::max)(adaptive, (std::min)(_solverIterations, _maxSolverIterations));
}

/// <summary>
/// 物理シミュレーションの更新処理（メインループから毎フレーム呼ばれる）
/// 非同期モード時はバックグラウンドスレッドで物理演算を実行し、同期モード時は即座に実行
/// </summary>
/// <param name="dt">前フレームからの経過時間（秒）</param>
void PhysicsManager::Update(float dt) {
    if (IsShuttingDown()) return;
    if (dt < 0.0f) dt = 0.0f;

#ifdef _DEBUG
    auto _scopeUpdate = PerformanceMonitor::Instance().Scope("Physics.Update");
#endif

    // コントローラー（常にメインスレッドで実行 — ゲームロジックに触れる可能性があるため）
    std::vector<PhysicsController*> ctrlSnapshot;
    {
        std::lock_guard lk(_mtx);
        ctrlSnapshot = _controllers;
    }
    for (auto* c : ctrlSnapshot) {
        if (!c) continue;
        c->Update(dt);
    }

    if (_asyncEnabled) {
        // --- 非同期モード: バックグラウンドで物理演算を開始し、即座にリターン ---
        // 前フレームの物理演算が完了するのを先に待機
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.WaitForPhysics");
            WaitForPhysics();
        }
#else
        WaitForPhysics();
#endif
        // 補間は*完了した*物理状態を使用
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation");
            ComputeInterpolation();
        }
#else
        ComputeInterpolation();
#endif
        // 新しい物理ステップを非同期で起動
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.AsyncEnqueue");
            _asyncFuture = ThreadPool::Instance().Enqueue([this, dt]() { RunAsyncStep(dt); });
        }
#else
        _asyncFuture = ThreadPool::Instance().Enqueue([this, dt]() { RunAsyncStep(dt); });
#endif
    } else {
        // --- 同期モード（元の動作）---
        const float maxDt = _fixedDeltaTime * static_cast<float>(_maxSubSteps);
        _accumulator += (std::min)(dt, maxDt);

        int sub = 0;
        while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < _maxSubSteps) {
#ifdef _DEBUG
            {
                auto _s = PerformanceMonitor::Instance().Scope("Physics.StepSimulation");
                StepSimulation(_fixedDeltaTime);
            }
#else
            StepSimulation(_fixedDeltaTime);
#endif
            _accumulator -= _fixedDeltaTime;
            ++sub;
        }
        if (_accumulator < 0.0f) _accumulator = 0.0f;

#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation");
            ComputeInterpolation();
        }
#else
        ComputeInterpolation();
#endif
    }
}

/// <summary>
/// 非同期物理演算の完了を待機
/// 非同期モード時に、前フレームの物理計算が終わるまでブロック
/// </summary>
void PhysicsManager::WaitForPhysics() {
    if (_asyncFuture.valid()) {
        _asyncFuture.get();
    }
}

/// <summary>
/// 非同期モード用の物理ステップ実行（バックグラウンドスレッドで呼ばれる）
/// </summary>
/// <param name="dt">経過時間（秒）</param>
void PhysicsManager::RunAsyncStep(float dt) {
    const float maxDt = _fixedDeltaTime * static_cast<float>(_maxSubSteps);
    _accumulator += (std::min)(dt, maxDt);

    int sub = 0;
    while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < _maxSubSteps) {
        StepSimulation(_fixedDeltaTime);
        _accumulator -= _fixedDeltaTime;
        ++sub;
    }
    if (_accumulator < 0.0f) _accumulator = 0.0f;
}

// ============================================================
//  キャッシュ構築と高速ルックアップ
// ============================================================
void PhysicsManager::BuildLookupCaches() {
	// 物体数やコライダー数に応じてハッシュテーブルのバケット数を調整し、挿入時の再ハッシュを減らす
	// 特に大規模シーンでは、毎フレームのキャッシュ構築コストを削減できる
    _bodyByOwner.clear();
    if (_bodyByOwner.bucket_count() < _bodies.size())
        _bodyByOwner.reserve(_bodies.size());
    for (auto* body : _bodies) {
        if (!body || !body->_owner) continue;
        _bodyByOwner.emplace(body->_owner, body);
    }
    _colliderByOwner.clear();
    const auto& colliders = ColliderManager::Instance().GetColliders();
    if (_colliderByOwner.bucket_count() < colliders.size())
        _colliderByOwner.reserve(colliders.size());
    for (auto* col : colliders) {
        if (!col || !col->owner) continue;
        _colliderByOwner.emplace(col->owner, col);
    }
}

/// <summary>
/// キャッシュからPhysicsBodyを高速検索
/// </summary>
/// <param name="owner">検索対象のGameObject</param>
/// <returns>見つかったPhysicsBody、存在しない場合はnullptr</returns>
PhysicsBody* PhysicsManager::CachedFindBody(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _bodyByOwner.find(owner);
    return (it != _bodyByOwner.end()) ? it->second : nullptr;
}

/// <summary>
/// キャッシュからColliderを高速検索
/// </summary>
/// <param name="owner">検索対象のGameObject</param>
/// <returns>見つかったCollider、存在しない場合はnullptr</returns>
Collider* PhysicsManager::CachedFindCollider(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _colliderByOwner.find(owner);
    return (it != _colliderByOwner.end()) ? it->second : nullptr;
}

// ============================================================
//  ステップシミュレーション
// ============================================================

/// <summary>
/// 物理シミュレーションの1ステップを実行
/// 速度積分 → 接触検出 → ソルバー → 位置補正の順に処理
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::StepSimulation(float stepDt) {
#ifdef _DEBUG
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildLookupCaches");
        BuildLookupCaches();
    }
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.IntegrateBodies");
        IntegrateBodies(stepDt);
    }
    {
        auto _s = PerformanceMonitor::Instance().Scope("Collider.Update");
        ColliderManager::Instance().Update(stepDt);
    }
	// デバッグビルドではキャッシュ構築・積分・コライダー更新を個別に計測
    // 衝突検出後は変化しない（位置のみが移動する）。
#else
    BuildLookupCaches();
    IntegrateBodies(stepDt);
    ColliderManager::Instance().Update(stepDt);
#endif

    // ============================================================
    // 物理シミュレーションのメインフェーズ
    // ============================================================

    // --- フェーズ 1: TOI イベント解決 ---
    // CCD有効な物体がトンネリングした場合、最も早いTOIでバックステップして再シミュレーション
#ifdef _DEBUG
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.ResolveToiEvents");
        ResolveToiEvents(stepDt);
    }
#else
    ResolveToiEvents(stepDt);
#endif

    // --- フェーズ 2: 接触拘束の構築 ---
    // 狭義相衝突検出で得た接触情報から、ソルバー用拘束を構築
    // 有効逆質量・反発係数・摩擦係数を計算し、Warm-start用に前フレームとマッチング
#ifdef _DEBUG
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildSolverContacts");
        BuildSolverContacts(stepDt);
    }
    // --- フェーズ 2b: Speculative 接触生成（オプション）---
    // 高速移動中の物体に対し、次フレームの衝突を予測して仮想接触を生成（トンネリング防止）
    if (_speculativeCcdEnabled) {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.GenerateSpeculativeContacts");
        GenerateSpeculativeContacts(stepDt);
    }
    // --- フェーズ 3: Island 構築 ---
    // 接触グラフから独立した物体グループ（Island）を構築し、並列ソルバーを準備
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildIslands");
        BuildIslands();
    }
    // --- フェーズ 4: Warm Start ---
    // 前フレームの累積インパルスを再利用して、ソルバーの収束を高速化
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.WarmStart");
        WarmStart();
    }
#else
    BuildSolverContacts(stepDt);
    if (_speculativeCcdEnabled) GenerateSpeculativeContacts(stepDt);
    BuildIslands();
    WarmStart();
#endif

    // --- フェーズ 5: ソルバー反復 ---
    // アダプティブ反復回数を計算し、各 Island を並列に解く
    const int iterations = ComputeAdaptiveIterations();
    const int savedIter = _solverIterations;
    _solverIterations = iterations; // 一時的に上書き
#ifdef _DEBUG
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.SolveAllIslands");
        SolveAllIslands(stepDt);
    }
#else
    SolveAllIslands(stepDt);
#endif
    _solverIterations = savedIter; // 復元

    // --- フェーズ 6: 位置補正 ---
    // Split Impulse 有効時: 速度に影響させずに位置を補正
    // Split Impulse 無効時: Baumgarte 安定化で位置を直接補正
    if (_splitImpulseEnabled) {
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.SplitImpulseCorrection");
            SplitImpulseCorrection(stepDt);
        }
#else
        SplitImpulseCorrection(stepDt);
#endif
    } else {
#ifdef _DEBUG
        {
            auto _s = PerformanceMonitor::Instance().Scope("Physics.PositionalCorrection");
            PositionalCorrection(stepDt);
        }
#else
        PositionalCorrection(stepDt);
#endif
    }

    // 前フレーム接触を保存（Warm-start用）
    _prevSolverContacts.swap(_solverContacts);

    // --- フェーズ 7: Island スリープ伝播 ---
    // Island 内のいずれかが起きていれば全員を起こす、全員寝ていればスリープ
#ifdef _DEBUG
    {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.PropagateIslandSleep");
        PropagateIslandSleep();
    }
#else
    PropagateIslandSleep();
#endif
}

// ============================================================
//  SoA gather / scatter — キャッシュフレンドリーな積分ループ用
// ============================================================

/// <summary>
/// PhysicsBodyの配列をSoA（Structure of Arrays）形式に変換
/// キャッシュ効率の良いSIMD処理を可能にする
/// </summary>
void PhysicsManager::GatherBodySoA() {
    const size_t n = _bodies.size();
    _bodySoA.Resize(n);
    ThreadPool::Instance().ParallelForBarrier(0, n, [&](size_t i) {
        PhysicsBody* b = _bodies[i];
        if (!b || !b->_owner) {
            _bodySoA.flags[i] = 0;
            return;
        }
        uint8_t f = 0;
        if (b->_enabled && b->_owner->IsActive()) f |= 1;
        if (b->_isKinematic)                       f |= 2;
        if (b->_isSleeping)                        f |= 4;
        if (b->_useGravity)                        f |= 8;
        if (b->_freezeRotation)                    f |= 16;
        if (b->_detectContinuous && b->_ccdQuality >= CcdQuality::Default) f |= 32;
        _bodySoA.flags[i]           = f;
        _bodySoA.position[i]        = b->_owner->transform.LocalPosition();
        _bodySoA.velocity[i]        = b->_velocity;
        _bodySoA.angularVelocity[i] = b->_angularVelocity;
        _bodySoA.force[i]           = b->_force;
        _bodySoA.torque[i]          = b->_torque;
        _bodySoA.inverseMass[i]     = b->InverseMass();
        _bodySoA.linearDamping[i]   = b->_linearDamping;
        _bodySoA.angularDamping[i]  = b->_angularDamping;
        _bodySoA.gravityScale[i]    = b->_gravityScale;
    }, 64);
}

/// <summary>
/// SoA形式から元のPhysicsBody配列に結果を書き戻す
/// </summary>
/// <param name="stepDt">未使用（将来の拡張用）</param>
void PhysicsManager::ScatterBodySoA(float /*stepDt*/) {
    const size_t n = _bodies.size();
    ThreadPool::Instance().ParallelForBarrier(0, n, [&](size_t i) {
        if (!(_bodySoA.flags[i] & 1)) return;
        PhysicsBody* b = _bodies[i];
        if (!b || !b->_owner) return;
        b->_velocity        = _bodySoA.velocity[i];
        b->_angularVelocity = _bodySoA.angularVelocity[i];
    }, 64);
}

// ============================================================
//  IntegrateBodies — SoA で速度積分を最適化
// ============================================================

/// <summary>
/// 全ボディの速度積分と位置/回転の更新
/// 重力、力、トルクを適用し、CCD速度クランプと地面衝突を処理
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::IntegrateBodies(float stepDt) {
    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    const bool groundEnabled = _groundPlaneEnabled;
    const VECTOR groundN = _groundPlaneNormal;
    const float groundD = _groundPlaneD;
    const VECTOR gravity = _gravity;

    // --- フェーズ 1: Gather SoA + 速度積分（統合、1バリア）---
    _bodySoA.Resize(bodyCount);
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        // 次のボディの AoS データを L1 キャッシュにプリフェッチしてメモリレイテンシを隠蔽。
        // bodies 配列はポインタ間接なので、順次プリフェッチが有効。
        if (idx + 2 < bodyCount && _bodies[idx + 2]) {
#if defined(_MSC_VER)
            _mm_prefetch(reinterpret_cast<const char*>(_bodies[idx + 2]), _MM_HINT_T0);
#endif
        }
        // === 収集 ===
        PhysicsBody* b = _bodies[idx];
        if (!b || !b->_owner) {
            _bodySoA.flags[idx] = 0;
            return;
        }
        uint8_t f = 0;
        if (b->_enabled && b->_owner->IsActive()) f |= 1;
        if (b->_isKinematic)                       f |= 2;
        if (b->_isSleeping)                        f |= 4;
        if (b->_useGravity)                        f |= 8;
        if (b->_freezeRotation)                    f |= 16;
        if (b->_detectContinuous && b->_ccdQuality >= CcdQuality::Default) f |= 32;
        _bodySoA.flags[idx]           = f;
        _bodySoA.position[idx]        = b->_owner->transform.LocalPosition();
        _bodySoA.velocity[idx]        = b->_velocity;
        _bodySoA.angularVelocity[idx] = b->_angularVelocity;
        _bodySoA.force[idx]           = b->_force;
        _bodySoA.torque[idx]          = b->_torque;
        _bodySoA.inverseMass[idx]     = b->InverseMass();
        _bodySoA.linearDamping[idx]   = b->_linearDamping;
        _bodySoA.angularDamping[idx]  = b->_angularDamping;
        _bodySoA.gravityScale[idx]    = b->_gravityScale;

        // === 速度積分 ===
        if (!(f & 1)) return;
        if (f & 2)    return;
        if ((f & 4) && LenSq(_bodySoA.force[idx]) <= 1e-8f
                    && LenSq(_bodySoA.torque[idx]) <= 1e-8f) return;

        const float invM = _bodySoA.inverseMass[idx];
        if (invM <= 0.0f) return;

        VECTOR acc = VScale(_bodySoA.force[idx], invM);
        if (f & 8) acc = VAdd(acc, VScale(gravity, _bodySoA.gravityScale[idx]));
        _bodySoA.velocity[idx] = VAdd(_bodySoA.velocity[idx], VScale(acc, stepDt));

        if (!(f & 16)) {
            const VECTOR angAcc = b->ApplyInverseInertia(_bodySoA.torque[idx]);
            _bodySoA.angularVelocity[idx] = VAdd(_bodySoA.angularVelocity[idx], VScale(angAcc, stepDt));
        }

        if (_bodySoA.linearDamping[idx] > 0.0f)
            _bodySoA.velocity[idx] = VScale(_bodySoA.velocity[idx],
                1.0f / (1.0f + _bodySoA.linearDamping[idx] * stepDt));
        if (!(f & 16) && _bodySoA.angularDamping[idx] > 0.0f)
            _bodySoA.angularVelocity[idx] = VScale(_bodySoA.angularVelocity[idx],
                1.0f / (1.0f + _bodySoA.angularDamping[idx] * stepDt));
    }, 64);

    // --- フェーズ 2: Scatter + 位置/回転更新（統合、1バリア）---
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        // === 分散 ===
        if (_bodySoA.flags[idx] & 1) {
            PhysicsBody* bs = _bodies[idx];
            if (bs && bs->_owner) {
                bs->_velocity        = _bodySoA.velocity[idx];
                bs->_angularVelocity = _bodySoA.angularVelocity[idx];
            }
        }

        // === 位置/回転更新 ===
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_enabled || !body->_owner || !body->_owner->IsActive()) return;

        body->_previousPosition = body->_owner->transform.LocalPosition();
        body->_previousRotation = body->_owner->transform.LocalRotation();

        if (body->_isKinematic) {
            if (body->_hasMovePositionTarget) {
                body->_owner->transform.SetLocalPosition(body->_movePositionTarget);
                body->_hasMovePositionTarget = false;
            }
            if (body->_hasMoveRotationTarget) {
                body->_owner->transform.SetLocalRotation(body->_moveRotationTarget);
                body->_hasMoveRotationTarget = false;
            }
            body->ClearAccumulators();
            body->_velocity = VGet(0,0,0);
            body->_angularVelocity = VGet(0,0,0);
            return;
        }

        if (body->_isSleeping && LenSq(body->_force) <= 1e-8f && LenSq(body->_torque) <= 1e-8f) return;
        if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) body->WakeUp();

        const float inverseMass = body->InverseMass();
        if (inverseMass <= 0.0f) { body->ClearAccumulators(); return; }

        ApplyBodyConstraints(body);
        ClampMagnitude(body->_velocity, body->_maxLinearSpeed);
        if (!body->_freezeRotation) ClampMagnitude(body->_angularVelocity, body->_maxAngularSpeed);

        // CCD 速度クランプ
        if (body->_detectContinuous && body->_ccdQuality >= CcdQuality::Default) {
            Collider* col = CachedFindCollider(body->_owner);
            if (col) {
                const float minHE = GetColliderMinHalfExtent(col);
                const float allowedPen = (body->_allowedPenetrationDepth > 0.0f)
                    ? body->_allowedPenetrationDepth : (minHE * 0.8f);
                const float linearSpeed = Len3(body->_velocity);
                const float angularSpeed = Len3(body->_angularVelocity);
                float maxHE = minHE;
                if (col->GetKind() == Collider::Kind::Box) {
                    const auto* boxCol = static_cast<const BoxCollider*>(col);
                    const VECTOR he = boxCol->GetHalfExtents();
                    maxHE = std::sqrt(he.x * he.x + he.y * he.y + he.z * he.z);
                } else if (col->GetKind() == Collider::Kind::Capsule) {
                    const auto* capCol = static_cast<const CapsuleCollider*>(col);
                    maxHE = capCol->GetRadius() + Len3(VSub(capCol->GetTop(), capCol->GetBottom())) * 0.5f;
                }
                const float angularSurfaceSpeed = angularSpeed * maxHE;
                const float effectiveSpeed = linearSpeed + angularSurfaceSpeed;
                if (stepDt > 1e-6f && effectiveSpeed > 1e-6f) {
                    const float maxDisplacement = allowedPen;
                    const float toi = maxDisplacement / effectiveSpeed;
                    if (toi < stepDt) {
                        const float qualityScale = (body->_ccdQuality == CcdQuality::Critical) ? 0.5f : 1.0f;
                        const float clampDisp = maxDisplacement * qualityScale;
                        const float linearFraction = (effectiveSpeed > 1e-6f) ? (linearSpeed / effectiveSpeed) : 1.0f;
                        ClampMagnitude(body->_velocity, clampDisp * linearFraction / stepDt);
                        if (!body->_freezeRotation && angularSurfaceSpeed > 1e-6f) {
                            const float angFraction = 1.0f - linearFraction;
                            const float maxAngDisp = clampDisp * angFraction;
                            if (maxHE > 1e-6f) {
                                const float maxAngSpeed = maxAngDisp / (maxHE * stepDt);
                                ClampMagnitude(body->_angularVelocity, maxAngSpeed);
                            }
                        }
                    }
                }
            }
        }

        VECTOR pos = body->_owner->transform.LocalPosition();
        pos = VAdd(pos, VScale(body->_velocity, stepDt));

        if (groundEnabled) {
            const float dist = HalfPlaneDistance(pos, groundN, groundD);
            if (dist < 0.0f) {
                pos = VSub(pos, VScale(groundN, dist));
                const float vn = Dot3(body->_velocity, groundN);
                if (vn < 0.0f) {
                    body->_velocity = VSub(body->_velocity, VScale(groundN, vn));
                }
                if (!body->_freezeRotation && body->_friction > 0.0f) {
                    body->_angularVelocity = VScale(body->_angularVelocity,
                        1.0f / (1.0f + body->_friction * 0.5f * stepDt));
                }
            }
        }
        body->_owner->transform.SetLocalPosition(pos);

        if (!body->_freezeRotation) {
            const VECTOR& w = body->_angularVelocity;
            const float angSpd = Len3(w);
            if (angSpd > 1e-6f) {
                Quaternion q = body->_owner->transform.LocalRotation();
                Quaternion wq(w.x * 0.5f * stepDt, w.y * 0.5f * stepDt, w.z * 0.5f * stepDt, 0.0f);
                Quaternion dq = Quaternion::Multiply(wq, q);
                q.x += dq.x; q.y += dq.y; q.z += dq.z; q.w += dq.w;
                body->_owner->transform.SetLocalRotation(q.Normalized());
            }
        }

        body->_hasMovePositionTarget = false;
        body->_hasMoveRotationTarget = false;
        body->ClearAccumulators();

        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
        {
            VECTOR p = body->_owner->transform.LocalPosition();
            if (!IsFiniteVec(p)) {
                body->_owner->transform.SetLocalPosition(body->_previousPosition);
                body->_velocity = VGet(0,0,0);
                body->_angularVelocity = VGet(0,0,0);
            }
        }
    }, 64);
}

// ============================================================
//  BuildSolverContacts - ソルバー用接触拘束の構築
// ============================================================

/// <summary>
/// ColliderManagerから得た接触情報を速度ソルバーで解ける形式に変換
/// 主な処理:
///   1. 有効逆質量を計算（並進 + 回転慣性）
///   2. 摩擦係数をマテリアルから合成（static/kinetic 別々）
///   3. 前フレームの接触とマッチングしてWarm-start用の累積インパルスを復元
///   4. Baumgarteバイアス / Split Impulseバイアスを計算
/// 並列化: 各接触の解決は独立なのでParallelForBarrierで構築
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::BuildSolverContacts(float stepDt) {
    const auto& rawContacts = ColliderManager::Instance().GetContacts();
    _solverContacts.clear();

    const size_t rawCount = rawContacts.size();
    if (rawCount == 0) return;

    const float invDt = (stepDt > 1e-6f) ? (1.0f / stepDt) : 0.0f;

    // --- 前フレーム接触の高速ルックアップ用ハッシュマップ ---
    struct PrevKey {
        Collider* a; Collider* b;
        bool operator==(const PrevKey& o) const noexcept { return a == o.a && b == o.b; }
    };
    struct PrevHash {
        size_t operator()(const PrevKey& k) const noexcept {
            return (reinterpret_cast<size_t>(k.a) >> 4) ^ (reinterpret_cast<size_t>(k.b) << 1);
        }
    };
    std::unordered_multimap<PrevKey, size_t, PrevHash> prevMap;
    prevMap.reserve(_prevSolverContacts.size());
    for (size_t i = 0; i < _prevSolverContacts.size(); ++i) {
        const auto& p = _prevSolverContacts[i];
        prevMap.emplace(PrevKey{p.colA, p.colB}, i);
    }

    // --- 並列構築 ---
    struct TaggedContact { SolverContact sc; bool valid = false; };
    std::vector<TaggedContact> results(rawCount);

    ThreadPool::Instance().ParallelForBarrier(0, rawCount, [&](size_t idx) {
        const auto& ct = rawContacts[idx];
        if (!ct.a || !ct.b) return;
        if (!IsFinite(ct.penetration) || ct.penetration <= 0.0f) return;
        if (!IsFiniteVec(ct.normal)) return;

        GameObject* ownerA = ct.a->owner;
        GameObject* ownerB = ct.b->owner;
        PhysicsBody* bodyA = CachedFindBody(ownerA);
        PhysicsBody* bodyB = CachedFindBody(ownerB);

        const float invA = (bodyA && bodyA->IsDynamic() && ownerA && ownerA->IsActive()) ? bodyA->InverseMass() : 0.0f;
        const float invB = (bodyB && bodyB->IsDynamic() && ownerB && ownerB->IsActive()) ? bodyB->InverseMass() : 0.0f;
        if (invA + invB <= 1e-8f) return;

        SolverContact sc{};
        sc.colA = ct.a;
        sc.colB = ct.b;
        sc.normal = SafeNormalize(ct.normal, VGet(0,1,0));
        sc.point = ct.point;
        sc.penetration = (std::min)(ct.penetration, kMaxPen);
        sc.bodyA = bodyA;
        sc.bodyB = bodyB;
        sc.invA = invA;
        sc.invB = invB;

        const VECTOR centerA = ownerA ? ownerA->transform.WorldPosition() : VGet(0,0,0);
        const VECTOR centerB = ownerB ? ownerB->transform.WorldPosition() : VGet(0,0,0);
        sc.rA = VSub(sc.point, centerA);
        sc.rB = VSub(sc.point, centerB);

        ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);

        sc.effectiveInvMassN  = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        sc.effectiveInvMassT1 = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent1);
        sc.effectiveInvMassT2 = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent2);
        if (sc.effectiveInvMassN <= 1e-8f) return;
        if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
        if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;

        float rest = 0.0f;
        if (bodyA && bodyB) rest = PhysicsMaterial::CombineRestitution(bodyA->_material, bodyB->_material);
        else if (bodyA) rest = bodyA->_restitution;
        else if (bodyB) rest = bodyB->_restitution;
        const float vn = RelNormalVelocity(bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        if (std::fabs(vn) < kRestitutionThreshold) rest = 0.0f;
        sc.restitution = rest;

        if (bodyA && bodyB) sc.friction = PhysicsMaterial::CombineFriction(bodyA->_material, bodyB->_material);
        else if (bodyA) sc.friction = (std::max)(0.0f, bodyA->_friction);
        else if (bodyB) sc.friction = (std::max)(0.0f, bodyB->_friction);
        if (bodyA && bodyB) sc.staticFriction = PhysicsMaterial::CombineStaticFriction(bodyA->_material, bodyB->_material);
        else if (bodyA) sc.staticFriction = sc.friction * 1.2f;
        else if (bodyB) sc.staticFriction = sc.friction * 1.2f;

        sc.normalBias = kBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        if (vn < -kRestitutionThreshold) sc.normalBias += rest * (-vn);
        sc.splitBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        sc.splitNormalLambda = 0.0f;
        sc.speculative = false;

        sc.localA = ownerA ? VSub(sc.point, centerA) : sc.point;
        sc.localB = ownerB ? VSub(sc.point, centerB) : sc.point;

        sc.normalLambda = 0.0f;
        sc.frictionLambda1 = 0.0f;
        sc.frictionLambda2 = 0.0f;
        auto range = prevMap.equal_range(PrevKey{sc.colA, sc.colB});
        for (auto it = range.first; it != range.second; ++it) {
            const auto& prev = _prevSolverContacts[it->second];
            if (LenSq(VSub(prev.localA, sc.localA)) > kContactMatchDistSq) continue;
            if (LenSq(VSub(prev.localB, sc.localB)) > kContactMatchDistSq) continue;
            sc.normalLambda    = prev.normalLambda;
            sc.frictionLambda1 = prev.frictionLambda1;
            sc.frictionLambda2 = prev.frictionLambda2;
            break;
        }

        results[idx] = { sc, true };
    }, 64);

    _solverContacts.reserve(rawCount);
    for (auto& r : results) {
        if (r.valid) _solverContacts.push_back(std::move(r.sc));
    }
}

// ============================================================
//  Union-Find（ランク付き、再割り当てを避けるため永続配列を使用）
// ============================================================

/// <summary>
/// Union-Findのルート検索（経路圧縮付き）
/// </summary>
/// <param name="x">検索するノードのインデックス</param>
/// <returns>ルートノードのインデックス</returns>
int PhysicsManager::UFFind(int x) noexcept {
    while (_ufParent[x] != x) {
        _ufParent[x] = _ufParent[_ufParent[x]]; // 経路半減
        x = _ufParent[x];
    }
    return x;
}

/// <summary>
/// Union-Findの統合操作（ランクによる統合で木の高さを最適化）
/// </summary>
/// <param name="a">統合する第1ノード</param>
/// <param name="b">統合する第2ノード</param>
void PhysicsManager::UFUnite(int a, int b) noexcept {
    a = UFFind(a); b = UFFind(b);
    if (a == b) return;
    // ランクによる統合
    if (_ufRank[a] < _ufRank[b]) std::swap(a, b);
    _ufParent[b] = a;
    if (_ufRank[a] == _ufRank[b]) ++_ufRank[a];
}

// ============================================================
//  BuildIslands — 接触グラフ上でのUnion-Find
// ============================================================

/// <summary>
/// 接触グラフからアイランド（独立した物体グループ）を構築
/// Union-Findで接触しているボディを同じグループにまとめ、並列ソルバーを準備
/// 大規模アイランドは分割して効率化
/// </summary>
void PhysicsManager::BuildIslands() {
    _islands.clear();
    _bodyIslandMap.clear();
    if (_bodies.empty()) return;

    // body ptr → index: unordered_map のバケットを再利用
    std::unordered_map<PhysicsBody*, int> bodyIndex;
    if (bodyIndex.bucket_count() < _bodies.size())
        bodyIndex.reserve(_bodies.size());
    for (int i = 0; i < static_cast<int>(_bodies.size()); ++i) {
        if (_bodies[i]) bodyIndex.emplace(_bodies[i], i);
    }

    const int n = static_cast<int>(_bodies.size());
    _ufParent.resize(n);
    _ufRank.assign(n, 0);
    for (int i = 0; i < n; ++i) _ufParent[i] = i;

    for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
        auto& sc = _solverContacts[ci];
        int ia = -1, ib = -1;
        if (sc.bodyA) { auto it = bodyIndex.find(sc.bodyA); if (it != bodyIndex.end()) ia = it->second; }
        if (sc.bodyB) { auto it = bodyIndex.find(sc.bodyB); if (it != bodyIndex.end()) ib = it->second; }
        if (ia >= 0 && ib >= 0) UFUnite(ia, ib);
    }

    std::unordered_map<int, int> rootToIsland;
    rootToIsland.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (!_bodies[i]) continue;
        const int root = UFFind(i);
        auto it = rootToIsland.find(root);
        int islandIdx;
        if (it == rootToIsland.end()) {
            islandIdx = static_cast<int>(_islands.size());
            rootToIsland[root] = islandIdx;
            _islands.emplace_back();
        } else {
            islandIdx = it->second;
        }
        _islands[islandIdx].bodies.push_back(_bodies[i]);
        _bodyIslandMap[_bodies[i]] = islandIdx;
    }

    for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
        auto& sc = _solverContacts[ci];
        PhysicsBody* repBody = sc.bodyA ? sc.bodyA : sc.bodyB;
        if (!repBody) continue;
        auto it = _bodyIslandMap.find(repBody);
        if (it == _bodyIslandMap.end()) continue;
        sc.islandId = it->second;
        _islands[it->second].contactIndices.push_back(ci);
    }

    for (auto& island : _islands) {
        bool allSleep = true;
        for (auto* body : island.bodies) {
            if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
        }
        island.allSleeping = allSleep;
    }

    // --- 大きなアイランドのための拘束グラフ着色 ---
    // 各バッチ内の接触がボディを共有しないように独立したバッチを構築し、
    // 並列で解けるようにする。
    for (auto& island : _islands) {
        if (!island.allSleeping && static_cast<int>(island.contactIndices.size()) >= kBatchingThreshold) {
            BuildConstraintBatches(island);
        }
    }

    // --- 大規模アイランド分割 ---
    // 大きなアイランドを分割して並列 PGS の効率を上げる。
    // kIslandSplitThreshold を超えるアイランドを検出し、
    // 接触グラフの色分割で小アイランドに分解する。
    {
        const int origCount = static_cast<int>(_islands.size());
        // 途中で追加される新アイランド用にあらかじめ予約し、
        // emplace_back による確保を回避。
        _islands.reserve(origCount * 2);
        for (int ii = 0; ii < origCount; ++ii) {
            if (ii < 0 || static_cast<size_t>(ii) >= _islands.size()) {
                ASSERT_MSG(false, "BuildIslands: island index out of range. ii=%d size=%zu", ii, _islands.size());
                break;
            }
            if (_islands[ii].allSleeping) continue;
            if (static_cast<int>(_islands[ii].bodies.size()) > kIslandSplitThreshold) {
                SplitLargeIsland(ii, kIslandSplitThreshold);
            }
        }
    }
}

// ============================================================
//  SplitLargeIsland — 大規模アイランドを小ソルバー単位に分割
// ============================================================

/// <summary>
/// 大規模アイランドをグラフ2着色で2つのサブアイランドに分割
/// 完全な独立性は保証されないが、PGSの並列効率が向上する
/// </summary>
/// <param name="islandIdx">分割対象のアイランドインデックス</param>
/// <param name="maxBodiesPerSplit">分割を行うボディ数の閾値</param>
void PhysicsManager::SplitLargeIsland(int islandIdx, int maxBodiesPerSplit) {
    if (islandIdx < 0 || static_cast<size_t>(islandIdx) >= _islands.size()) {
        ASSERT_MSG(false, "SplitLargeIsland: islandIdx out of range. islandIdx=%d size=%zu", islandIdx, _islands.size());
        return;
    }
    // ����: _islands �ւ� emplace_back �͓���o�b�t�@��Ċm�ۂ���\��������A
    // �����̎Q�ƁE�|�C���^�𖳌�������B���̂��� island ��Q�Ƃł͂Ȃ�
    // �C���f�b�N�X�A�N�Z�X�ň����Aemplace_back ��͎Q�Ƃ�Ď擾����B
    const int bodyCount = static_cast<int>(_islands[islandIdx].bodies.size());
    if (bodyCount <= maxBodiesPerSplit) return;

    // body → local index
    std::unordered_map<PhysicsBody*, int> localIdx;
    localIdx.reserve(bodyCount);
    for (int i = 0; i < bodyCount; ++i) {
        localIdx[_islands[islandIdx].bodies[i]] = i;
    }

    // 隣接リスト: 接触から構築
    std::vector<std::vector<int>> adj(bodyCount);
    for (int ci : _islands[islandIdx].contactIndices) {
        const auto& sc = _solverContacts[ci];
        auto itA = localIdx.find(sc.bodyA);
        auto itB = localIdx.find(sc.bodyB);
        if (itA == localIdx.end() || itB == localIdx.end()) continue;
        adj[itA->second].push_back(itB->second);
        adj[itB->second].push_back(itA->second);
    }

    // 貪欲 2 着色（BFS）
    std::vector<int> color(bodyCount, -1);
    color[0] = 0;
    std::vector<int> queue;
    queue.reserve(bodyCount);
    queue.push_back(0);
    for (size_t head = 0; head < queue.size(); ++head) {
        const int u = queue[head];
        for (int v : adj[u]) {
            if (color[v] < 0) {
                color[v] = 1 - color[u];
                queue.push_back(v);
            }
        }
    }
    // 未訪問ボディはグループ 0 へ
    for (int i = 0; i < bodyCount; ++i) {
        if (color[i] < 0) color[i] = 0;
    }

    // グループサイズをカウント; 分割が偏りすぎていれば中止
    int count0 = 0, count1 = 0;
    for (int c : color) { if (c == 0) ++count0; else ++count1; }
    if (count1 == 0 || count0 == 0) return; // 分割不可

    // bodies と contactIndices のコピーを先に取得（emplace_back 前）
    // emplace_back で _islands が再確保されると既存の参照が無効になるため。
    std::vector<PhysicsBody*> origBodies = _islands[islandIdx].bodies;
    std::vector<int> origContactIndices = _islands[islandIdx].contactIndices;

    // グループ 1 用の新アイランドを構築
    const int newIslandIdx = static_cast<int>(_islands.size());
    _islands.emplace_back();
    // !! �����ȍ~ _islands[islandIdx] �͍Ċm�ۂňړ��ς݂̉\�������邪�A
    //    origBodies / origContactIndices �̃R�s�[����č\�z����̂ň��S�B

    // Move bodies to appropriate island
    PhysicsIsland oldIslandNew;
    oldIslandNew.bodies.reserve(count0);
    _islands[newIslandIdx].bodies.reserve(count1);
    for (int i = 0; i < bodyCount; ++i) {
        if (color[i] == 0) {
            oldIslandNew.bodies.push_back(origBodies[i]);
        } else {
            _islands[newIslandIdx].bodies.push_back(origBodies[i]);
            _bodyIslandMap[origBodies[i]] = newIslandIdx;
        }
    }

    // 接触の再分配: 両ボディがグループ 1 の接触は newIsland へ、
    // グループをまたぐ接触は元のまま（境界接触）。
    oldIslandNew.contactIndices.reserve(origContactIndices.size());
    _islands[newIslandIdx].contactIndices.reserve(origContactIndices.size() / 2);
    for (int ci : origContactIndices) {
        const auto& sc = _solverContacts[ci];
        auto itA = localIdx.find(sc.bodyA);
        auto itB = localIdx.find(sc.bodyB);
        const int cA = (itA != localIdx.end()) ? color[itA->second] : 0;
        const int cB = (itB != localIdx.end()) ? color[itB->second] : 0;
        if (cA == 1 && cB == 1) {
            _islands[newIslandIdx].contactIndices.push_back(ci);
            _solverContacts[ci].islandId = newIslandIdx;
        } else {
            oldIslandNew.contactIndices.push_back(ci);
        }
    }

    // 元のアイランドを置き換え
    _islands[islandIdx].bodies = std::move(oldIslandNew.bodies);
    _islands[islandIdx].contactIndices = std::move(oldIslandNew.contactIndices);

    // スリープフラグを再計算
    auto computeSleep = [](PhysicsIsland& isl) {
        bool allSleep = true;
        for (auto* body : isl.bodies) {
            if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
        }
        isl.allSleeping = allSleep;
    };
    computeSleep(_islands[islandIdx]);
    computeSleep(_islands[newIslandIdx]);
}

// ============================================================
//  BuildConstraintBatches — 拘束グラフ上での貪欲グラフ着色
// ============================================================

/// <summary>
/// 大規模アイランド用に接触を独立したバッチに分割
/// ボディを共有する接触は同じバッチに入らないよう貪欲着色アルゴリズムで分類
/// </summary>
/// <param name="island">バッチ化対象のアイランド</param>
void PhysicsManager::BuildConstraintBatches(PhysicsIsland& island) {
    const int numContacts = static_cast<int>(island.contactIndices.size());
    if (numContacts <= 0) return;

    // body → 接触ローカルインデックスの隣接リストを構築
    std::unordered_map<PhysicsBody*, std::vector<int>> bodyToContacts;
    bodyToContacts.reserve(island.bodies.size());
    for (int li = 0; li < numContacts; ++li) {
        const auto& sc = _solverContacts[island.contactIndices[li]];
        if (sc.bodyA) bodyToContacts[sc.bodyA].push_back(li);
        if (sc.bodyB) bodyToContacts[sc.bodyB].push_back(li);
    }

    // 貪欲着色
    std::vector<int> contactColor(numContacts, -1);
    int maxColor = 0;
    for (int li = 0; li < numContacts; ++li) {
        // 隣接が使用している色を収集
        std::vector<bool> usedColors(maxColor + 2, false);
        const auto& sc = _solverContacts[island.contactIndices[li]];
        auto addUsed = [&](PhysicsBody* body) {
            if (!body) return;
            auto it = bodyToContacts.find(body);
            if (it == bodyToContacts.end()) return;
            for (int adj : it->second) {
                if (adj != li && contactColor[adj] >= 0) {
                    if (contactColor[adj] < static_cast<int>(usedColors.size()))
                        usedColors[contactColor[adj]] = true;
                }
            }
        };
        addUsed(sc.bodyA);
        addUsed(sc.bodyB);

        // 未使用の最小色を探す
        int color = 0;
        while (color < static_cast<int>(usedColors.size()) && usedColors[color]) ++color;
        contactColor[li] = color;
        if (color > maxColor) maxColor = color;
    }

    // 着色からバッチを構築
    island.constraintBatches.resize(maxColor + 1);
    for (auto& batch : island.constraintBatches) batch.clear();
    for (int li = 0; li < numContacts; ++li) {
        island.constraintBatches[contactColor[li]].push_back(island.contactIndices[li]);
    }
}

// ============================================================
//  ウォームスタート
// ============================================================

/// <summary>
/// 前フレームの累積インパルスを再適用してソルバーの収束を高速化
/// 接触が連続する場合、前フレームの解を初期値として使うことで反復回数を削減
/// </summary>
void PhysicsManager::WarmStart() {
    for (auto& sc : _solverContacts) {
        if (sc.effectiveInvMassN <= 1e-8f) continue;

        const VECTOR warmN  = VScale(sc.normal,   sc.normalLambda    * kWarmStartFactor);
        const VECTOR warmT1 = VScale(sc.tangent1, sc.frictionLambda1 * kWarmStartFactor);
        const VECTOR warmT2 = VScale(sc.tangent2, sc.frictionLambda2 * kWarmStartFactor);
        const VECTOR totalImpulse = VAdd(VAdd(warmN, warmT1), warmT2);

        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, totalImpulse);

        if (sc.bodyA) sc.bodyA->WakeUp();
        if (sc.bodyB) sc.bodyB->WakeUp();
    }
}

// ============================================================
//  SolveIsland — 円形摩擦コーン + split impulse
// ============================================================

/// <summary>
/// 単一アイランドの接触拘束を解く（1反復分）
/// 法線拘束と円形クーロン摩擦コーンを逐次的に解決
/// </summary>
/// <param name="island">解くアイランド</param>
/// <param name="stepDt">未使用</param>
void PhysicsManager::SolveIsland(const PhysicsIsland& island, float /*stepDt*/) {
    for (int ci : island.contactIndices) {
        if (ci < 0 || static_cast<size_t>(ci) >= _solverContacts.size()) {
            ASSERT_MSG(false, "SolveIsland: contact index out of range. ci=%d size=%zu", ci, _solverContacts.size());
            continue;
        }
        SolverContact& sc = _solverContacts[ci];
        if (sc.effectiveInvMassN <= 1e-8f) continue;

        // --- 累積クランプ付き法線拘束 ---
        {
            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
            // split impulse が有効な場合、Baumgarte バイアスは分離され、
            // 速度ソルバーには反発バイアスのみが入る。
            float bias = 0.0f;
            if (!_splitImpulseEnabled) {
                bias = sc.normalBias;
            } else {
                // Only restitution bias in velocity solver when split impulse is active
                if (vn < -kRestitutionThreshold) {
                    bias = sc.restitution * (-vn);
                }
            }

            // 投機的接触では、非負の法線インパルスのみを許可し、
            // バイアスを接近速度成分の除去のみにクランプ。
            // これにより投機的接触が分離エネルギーを注入するのを防ぐ。
            if (sc.speculative) {
                // Havok スタイル: bias = min(normalBias, -vn) で接近のみを停止
                const float approachSpeed = (std::max)(-vn, 0.0f);
                bias = (std::min)(sc.normalBias, approachSpeed);
            }

            float deltaLambda = (-vn + bias) / sc.effectiveInvMassN;
            const float oldLambda = sc.normalLambda;
            sc.normalLambda = (std::max)(oldLambda + deltaLambda, 0.0f);
            deltaLambda = sc.normalLambda - oldLambda;

            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
                VScale(sc.normal, deltaLambda));
        }

        // --- 摩擦: 円形クーロンコーン（2D）---
        // 接線相対速度を計算
        const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
        const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
        const float tangentSpeed = std::sqrt(vt1 * vt1 + vt2 * vt2);

        // 接線速度が低い場合は静止摩擦、それ以外は動摩擦を使用
        const float frictionToUse = (tangentSpeed < 0.1f) ? sc.staticFriction : sc.friction;
        const float maxFriction = frictionToUse * sc.normalLambda;

        // 両接線軸を解く
        float dLambda1 = -vt1 / sc.effectiveInvMassT1;
        float dLambda2 = -vt2 / sc.effectiveInvMassT2;
        float newF1 = sc.frictionLambda1 + dLambda1;
        float newF2 = sc.frictionLambda2 + dLambda2;

        // 円形コーンクランプ: √(f1² + f2²) ≤ μN
        const float fMag = std::sqrt(newF1 * newF1 + newF2 * newF2);
        if (fMag > maxFriction && fMag > 1e-8f) {
            const float scale = maxFriction / fMag;
            newF1 *= scale;
            newF2 *= scale;
        }

        dLambda1 = newF1 - sc.frictionLambda1;
        dLambda2 = newF2 - sc.frictionLambda2;
        sc.frictionLambda1 = newF1;
        sc.frictionLambda2 = newF2;

        const VECTOR frictionImpulse = VAdd(
            VScale(sc.tangent1, dLambda1),
            VScale(sc.tangent2, dLambda2));
        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, frictionImpulse);
    }
}

// ============================================================
//  SolveAllIslands — 並列、アダプティブ反復、ロードバランス
// ============================================================

/// <summary>
/// 全アイランドを並列に解く
/// 接触数でソートしてロードバランスを改善し、バッチ化で収束を高速化
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::SolveAllIslands(float stepDt) {
    if (_islands.empty()) return;

    const int iterations = _solverIterations;
    const size_t islandCount = _islands.size();

    // 接触数の降順でアイランドをソートし、最も重いアイランドを
    // 最初に処理することで、ロード不均衡によるテール待機を削減。
    std::vector<size_t> islandOrder(islandCount);
    for (size_t i = 0; i < islandCount; ++i) islandOrder[i] = i;
    std::sort(islandOrder.begin(), islandOrder.end(), [&](size_t a, size_t b) {
        return _islands[a].contactIndices.size() > _islands[b].contactIndices.size();
    });

    ThreadPool::Instance().ParallelForBarrier(0, islandCount, [&](size_t orderIdx) {
        const size_t i = islandOrder[orderIdx];
        if (i >= _islands.size()) {
            ASSERT_MSG(false, "SolveAllIslands: index out of range. i=%zu size=%zu", i, _islands.size());
            return;
        }
        if (_islands[i].allSleeping) return;
        if (_islands[i].contactIndices.empty()) return;

        // 拘束バッチが利用可能な場合、各反復内でバッチ解法を使用し、
        // 大規模アイランドでの収束を改善。
        if (!_islands[i].constraintBatches.empty()) {
            for (int iter = 0; iter < iterations; ++iter) {
                for (const auto& batch : _islands[i].constraintBatches) {
                    for (int ci : batch) {
                        if (ci < 0 || static_cast<size_t>(ci) >= _solverContacts.size()) continue;
                        SolverContact& sc = _solverContacts[ci];
                        if (sc.effectiveInvMassN <= 1e-8f) continue;

                        // --- 法線 ---
                        {
                            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
                            float bias = 0.0f;
                            if (!_splitImpulseEnabled) bias = sc.normalBias;
                            else if (vn < -kRestitutionThreshold) bias = sc.restitution * (-vn);
                            if (sc.speculative) {
                                const float approach = (std::max)(-vn, 0.0f);
                                bias = (std::min)(sc.normalBias, approach);
                            }
                            float dl = (-vn + bias) / sc.effectiveInvMassN;
                            const float old = sc.normalLambda;
                            sc.normalLambda = (std::max)(old + dl, 0.0f);
                            dl = sc.normalLambda - old;
                            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, VScale(sc.normal, dl));
                        }
                        // --- 摩擦 ---
                        {
                            const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
                            const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
                            const float tSpd = std::sqrt(vt1*vt1 + vt2*vt2);
                            const float fri = (tSpd < 0.1f) ? sc.staticFriction : sc.friction;
                            const float maxF = fri * sc.normalLambda;
                            float d1 = -vt1 / sc.effectiveInvMassT1;
                            float d2 = -vt2 / sc.effectiveInvMassT2;
                            float n1 = sc.frictionLambda1 + d1;
                            float n2 = sc.frictionLambda2 + d2;
                            const float fMag = std::sqrt(n1*n1 + n2*n2);
                            if (fMag > maxF && fMag > 1e-8f) { const float s = maxF/fMag; n1 *= s; n2 *= s; }
                            d1 = n1 - sc.frictionLambda1; d2 = n2 - sc.frictionLambda2;
                            sc.frictionLambda1 = n1; sc.frictionLambda2 = n2;
                            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
                                VAdd(VScale(sc.tangent1, d1), VScale(sc.tangent2, d2)));
                        }
                    }
                }
            }
        } else {
            for (int iter = 0; iter < iterations; ++iter) {
                SolveIsland(_islands[i], stepDt);
            }
        }
    }, 1);

    const size_t bodyCount = _bodies.size();
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body) return;
        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
    }, 64);
}

// ============================================================
//  PositionalCorrection — 半平面対応（split impulse オフ時のフォールバック）
// ============================================================

/// <summary>
/// Baumgarte安定化による位置補正（Split Impulse無効時のフォールバック）
/// めり込みを直接位置で補正するが、エネルギー注入の副作用がある
/// </summary>
/// <param name="stepDt">未使用</param>
void PhysicsManager::PositionalCorrection(float /*stepDt*/) {
    for (const auto& sc : _solverContacts) {
        if (sc.penetration <= kSlop) continue;
        const float invSum = sc.invA + sc.invB;
        if (invSum <= 1e-8f) continue;

        const float clampedPen = (std::min)(sc.penetration, kMaxPen);
        float correctionMag = kBiasFactor * (std::max)(clampedPen - kSlop, 0.0f) / invSum;
        correctionMag = (std::min)(correctionMag, kMaxCorrection);
        if (correctionMag <= 1e-6f) continue;

        GameObject* ownerA = sc.colA ? sc.colA->owner : nullptr;
        GameObject* ownerB = sc.colB ? sc.colB->owner : nullptr;

        if (sc.bodyA && sc.invA > 0.0f && ownerA) {
            VECTOR p = ownerA->transform.LocalPosition();
            p = VSub(p, VScale(sc.normal, correctionMag * sc.invA));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerA->transform.SetLocalPosition(p);
            }
        }
        if (sc.bodyB && sc.invB > 0.0f && ownerB) {
            VECTOR p = ownerB->transform.LocalPosition();
            p = VAdd(p, VScale(sc.normal, correctionMag * sc.invB));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerB->transform.SetLocalPosition(p);
            }
        }
    }
}

// ============================================================
//  スリープ / 拘束
// ============================================================

/// <summary>
/// ボディのスリープ状態を更新
/// 速度が閾値以下で一定時間経過したらスリープ、力が加わったら起床
/// </summary>
/// <param name="body">更新対象のボディ</param>
/// <param name="stepDt">デルタタイム（秒）</param>
void PhysicsManager::UpdateSleepState(PhysicsBody* body, float stepDt) {
    if (!body || !body->IsDynamic()) return;
    if (body->_hasMovePositionTarget || body->_hasMoveRotationTarget) { body->WakeUp(); return; }
    if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) { body->WakeUp(); return; }

    const float lSq = body->_sleepLinearThreshold * body->_sleepLinearThreshold;
    const float aSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
    if (LenSq(body->_velocity) > lSq || LenSq(body->_angularVelocity) > aSq) { body->WakeUp(); return; }

    body->_sleepTimer += stepDt;
    if (body->_sleepTimer >= body->_sleepTimeThreshold) body->Sleep();
}

// ============================================================
//  SplitImpulseCorrection — 速度に影響しない位置補正
// ============================================================

/// <summary>
/// Split Impulse法による位置補正
/// 疑似速度チャンネルで貫通を解決し、速度ソルバーへのエネルギー注入を防ぐ
/// Baumgarte法より物理的に正確で安定
/// </summary>
/// <param name="stepDt">未使用</param>
void PhysicsManager::SplitImpulseCorrection(float /*stepDt*/) {
    // ボディごとの疑似速度: body index → VECTOR（フラット配列でハッシュ不要）
    const size_t bodyCount = _bodies.size();
    // _bodyByOwner は BuildLookupCaches 済みなので body→index の逆引き構築
    std::vector<VECTOR> pseudoVel(bodyCount, VGet(0, 0, 0));
    // body ptr → index（フラット配列）
    std::unordered_map<PhysicsBody*, size_t> bodyIdx;
    bodyIdx.reserve(bodyCount);
    for (size_t i = 0; i < bodyCount; ++i) {
        if (_bodies[i]) bodyIdx[_bodies[i]] = i;
    }

    // 反復解法（速度ソルバーより少ない反復で済む）
    const int posIter = (std::max)(_solverIterations / 2, 2);
    for (int iter = 0; iter < posIter; ++iter) {
        for (auto& sc : _solverContacts) {
            if (sc.penetration <= kSlop) continue;
            if (sc.effectiveInvMassN <= 1e-8f) continue;

            const size_t idxA = sc.bodyA ? (bodyIdx.count(sc.bodyA) ? bodyIdx[sc.bodyA] : SIZE_MAX) : SIZE_MAX;
            const size_t idxB = sc.bodyB ? (bodyIdx.count(sc.bodyB) ? bodyIdx[sc.bodyB] : SIZE_MAX) : SIZE_MAX;
            const VECTOR pvA = (idxA < bodyCount) ? pseudoVel[idxA] : VGet(0,0,0);
            const VECTOR pvB = (idxB < bodyCount) ? pseudoVel[idxB] : VGet(0,0,0);
            const float pvn = Dot3(VSub(pvB, pvA), sc.normal);

            float deltaLambda = (-pvn + sc.splitBias) / sc.effectiveInvMassN;
            const float oldLambda = sc.splitNormalLambda;
            sc.splitNormalLambda = (std::max)(oldLambda + deltaLambda, 0.0f);
            deltaLambda = sc.splitNormalLambda - oldLambda;
            if (std::fabs(deltaLambda) < 1e-8f) continue;

            const VECTOR impulse = VScale(sc.normal, deltaLambda);
            if (idxA < bodyCount && sc.invA > 0.0f)
                pseudoVel[idxA] = VSub(pseudoVel[idxA], VScale(impulse, sc.invA));
            if (idxB < bodyCount && sc.invB > 0.0f)
                pseudoVel[idxB] = VAdd(pseudoVel[idxB], VScale(impulse, sc.invB));
        }
    }

    // 累積された疑似速度を位置補正として適用（ボディごとに並列）
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        if (LenSq(pseudoVel[idx]) < 1e-10f) return;
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_owner) return;
        VECTOR p = body->_owner->transform.LocalPosition();
        VECTOR correction = VScale(pseudoVel[idx], _fixedDeltaTime);
        ClampMagnitude(correction, kMaxCorrection);
        p = VAdd(p, correction);
        if (IsFiniteVec(p)) {
            if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
            body->_owner->transform.SetLocalPosition(p);
        }
    }, 64);
}

// ============================================================
//  GenerateSpeculativeContacts
// ============================================================

/// <summary>
/// 投機的接触の生成（トンネリング防止用のCCD手法）
/// 高速移動ボディの次ステップ位置を予測し、事前に仮想接触を作成
/// サポート: 地面平面 + 全コライダーペア
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::GenerateSpeculativeContacts(float stepDt) {
    if (stepDt <= 1e-6f) return;
    const float invDt = 1.0f / stepDt;

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1.0f) continue; // 高速移動ボディのみ対象

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE = GetColliderMinHalfExtent(col);
        const float predictedDisplacement = speed * stepDt;
        if (predictedDisplacement < minHE) continue; // トンネリングするほど速くない

        // 未来の位置を予測
        const VECTOR curPos = body->_owner->transform.WorldPosition();
        const VECTOR predictedPos = VAdd(curPos, VScale(body->_velocity, stepDt));

        // 予測 AABB: 現在位置と予測位置の和集合を minHE で拡大
        AABB predictedAABB;
        {
            const AABB& curAABB = col->GetAABB();
            const VECTOR offset = VScale(body->_velocity, stepDt);
            predictedAABB.min.x = (std::min)(curAABB.min.x, curAABB.min.x + offset.x) - kSpeculativeMargin;
            predictedAABB.min.y = (std::min)(curAABB.min.y, curAABB.min.y + offset.y) - kSpeculativeMargin;
            predictedAABB.min.z = (std::min)(curAABB.min.z, curAABB.min.z + offset.z) - kSpeculativeMargin;
            predictedAABB.max.x = (std::max)(curAABB.max.x, curAABB.max.x + offset.x) + kSpeculativeMargin;
            predictedAABB.max.y = (std::max)(curAABB.max.y, curAABB.max.y + offset.y) + kSpeculativeMargin;
            predictedAABB.max.z = (std::max)(curAABB.max.z, curAABB.max.z + offset.z) + kSpeculativeMargin;
            predictedAABB.center = VScale(VAdd(predictedAABB.min, predictedAABB.max), 0.5f);
        }

        // 有効な場合、地面平面との衝突をチェック
        if (_groundPlaneEnabled) {
            const float dist = HalfPlaneDistance(predictedPos, _groundPlaneNormal, _groundPlaneD);
            if (dist < minHE + kSpeculativeMargin) {
                SolverContact sc{};
                sc.colA = col;
                sc.colB = nullptr;
                sc.normal = _groundPlaneNormal;
                sc.point = VSub(predictedPos, VScale(_groundPlaneNormal, dist));
                sc.penetration = minHE + kSpeculativeMargin - dist;
                sc.bodyA = body;
                sc.bodyB = nullptr;
                sc.invA = body->InverseMass();
                sc.invB = 0.0f;
                sc.rA = VSub(sc.point, curPos);
                sc.rB = VGet(0,0,0);
                ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
                sc.effectiveInvMassN = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.normal);
                sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent1);
                sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent2);
                if (sc.effectiveInvMassN <= 1e-8f) continue;
                if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
                if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;
                sc.friction = body->_friction;
                sc.staticFriction = sc.friction * 1.2f;
                sc.restitution = 0.0f;
                sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
                sc.splitBias = sc.normalBias;
                sc.localA = sc.rA;
                sc.localB = VGet(0,0,0);
                sc.speculative = true;
                _solverContacts.push_back(sc);
            }
        }

        // 投機的接触のため、他の全コライダーとのチェック
        const auto& allColliders = ColliderManager::Instance().GetColliders();

        // speculative �R���^�N�g��ꎞ�o�b�t�@�Ɏ��W���A���[�v��Ɉꊇ�ǉ�����B
        // _solverContacts �ւ� push_back �̓C�e���[�^��������N�������߁A
        // ���[�v���ɒ��ڒǉ����Ă͂Ȃ�Ȃ��B
        std::vector<SolverContact> specContacts;

        for (auto* otherCol : allColliders) {
            if (!otherCol || otherCol == col) continue;
            if (otherCol->owner == body->_owner) continue; // 自身はスキップ

            // 重複排除: このペアに実際の接触が既に存在する場合はスキップ
            bool alreadyHasContact = false;
            for (size_t ei = 0; ei < _solverContacts.size(); ++ei) {
                const auto& existing = _solverContacts[ei];
                if (existing.speculative) continue;
                if ((existing.colA == col && existing.colB == otherCol) ||
                    (existing.colA == otherCol && existing.colB == col)) {
                    alreadyHasContact = true;
                    break;
                }
            }
            if (alreadyHasContact) continue;

            // 予測 AABB との AABB 重なり判定
            const AABB& otherAABB = otherCol->GetAABB();
            if (predictedAABB.min.x > otherAABB.max.x || predictedAABB.max.x < otherAABB.min.x) continue;
            if (predictedAABB.min.y > otherAABB.max.y || predictedAABB.max.y < otherAABB.min.y) continue;
            if (predictedAABB.min.z > otherAABB.max.z || predictedAABB.max.z < otherAABB.min.z) continue;

            // 最接近方向と距離の近似計算
            const VECTOR otherCenter = otherCol->GetCenter();
            const VECTOR toPredicted = VSub(predictedPos, otherCenter);
            const float approachDist = Len3(toPredicted);
            const float otherMinHE = GetColliderMinHalfExtent(otherCol);
            const float combinedExtent = minHE + otherMinHE + kSpeculativeMargin;

            if (approachDist > combinedExtent * 3.0f) continue; // 投機的接触には遠すぎる

            // 投機的貫通の推定: ボディがどれだけ重なるか
            const VECTOR toOther = VSub(otherCenter, curPos);
            const float distNow = Len3(toOther);
            const float closingSpeed = -Dot3(body->_velocity, SafeNormalize(toOther)) * stepDt;
            const float specPen = combinedExtent - (distNow - closingSpeed);
            if (specPen <= 0.0f) continue;

            // 投機的接触を生成
            const VECTOR normal = SafeNormalize(VSub(curPos, otherCenter), VGet(0, 1, 0));
            const VECTOR contactPt = VAdd(otherCenter, VScale(normal, otherMinHE));

            PhysicsBody* otherBody = CachedFindBody(otherCol->owner);
            const float otherInv = (otherBody && otherBody->IsDynamic() && otherCol->owner && otherCol->owner->IsActive())
                ? otherBody->InverseMass() : 0.0f;

            if (body->InverseMass() + otherInv <= 1e-8f) continue;

            SolverContact sc{};
            sc.colA = col;
            sc.colB = otherCol;
            sc.normal = normal;
            sc.point = contactPt;
            sc.penetration = (std::min)(specPen, kMaxPen);
            sc.bodyA = body;
            sc.bodyB = otherBody;
            sc.invA = body->InverseMass();
            sc.invB = otherInv;
            sc.rA = VSub(sc.point, curPos);
            sc.rB = VSub(sc.point, otherCenter);
            ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
            sc.effectiveInvMassN = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.normal);
            sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent1);
            sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent2);
            if (sc.effectiveInvMassN <= 1e-8f) continue;
            if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
            if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;

            if (body && otherBody)
                sc.friction = PhysicsMaterial::CombineFriction(body->_material, otherBody->_material);
            else
                sc.friction = body->_friction;
            sc.staticFriction = sc.friction * 1.2f;
            sc.restitution = 0.0f;
            sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
            sc.splitBias = sc.normalBias;
            sc.localA = sc.rA;
            sc.localB = sc.rB;
            sc.speculative = true;

            // 接触溶接: 同じボディペアの既存投機的接触に近すぎる場合はスキップ
            // （重複接触によるソルバーアーティファクトを防止）
            bool welded = false;
            constexpr float kWeldDistSq = 0.01f; // 0.1 ワールド単位
            // 既存 _solverContacts と一時バッファ両方でチェック
            for (size_t ei = 0; ei < _solverContacts.size(); ++ei) {
                const auto& existing = _solverContacts[ei];
                if (!existing.speculative) continue;
                if (existing.bodyA != body && existing.bodyB != body) continue;
                if (LenSq(VSub(existing.point, sc.point)) < kWeldDistSq &&
                    Dot3(existing.normal, sc.normal) > 0.9f) {
                    welded = true;
                    break;
                }
            }
            if (!welded) {
                for (const auto& existing : specContacts) {
                    if (existing.bodyA != body && existing.bodyB != body) continue;
                    if (LenSq(VSub(existing.point, sc.point)) < kWeldDistSq &&
                        Dot3(existing.normal, sc.normal) > 0.9f) {
                        welded = true;
                        break;
                    }
                }
            }
            if (welded) continue;

            specContacts.push_back(sc);
        }

        // 一時バッファを一括追加
        for (auto& sc : specContacts) {
            _solverContacts.push_back(std::move(sc));
        }
    }
}

// ============================================================
//  ResolveToiEvents — 時間ソート済みイベントでのサブTOIステッピング
// ============================================================

/// <summary>
/// TOI（Time of Impact）イベントの解決
/// 薄いジオメトリを貫通したCCDボディを検出し、衝突時点にバックステップして再積分
/// Bullet/Critical品質のボディのみ処理
/// </summary>
/// <param name="stepDt">固定デルタタイム（秒）</param>
void PhysicsManager::ResolveToiEvents(float stepDt) {
    if (stepDt <= 1e-6f) return;

    struct ToiEvent {
        PhysicsBody* body = nullptr;
        float toi = 1.0f;          // stepDt 内の割合 [0..1]
        VECTOR toiPosition{};       // TOI での位置
        VECTOR clampedVelocity{};   // バックステップ後の速度
    };

    std::vector<ToiEvent> events;

    // 投機的接触で既にカバーされているボディのセットを構築 —
    // ボディが既に投機的接触を持っている場合、その TOI イベントは
    // ソルバーで処理されるため、高コストなバックステップをスキップ。
    std::unordered_set<PhysicsBody*> specCoveredBodies;
    if (_speculativeCcdEnabled) {
        for (const auto& sc : _solverContacts) {
            if (!sc.speculative) continue;
            if (sc.bodyA) specCoveredBodies.insert(sc.bodyA);
            if (sc.bodyB) specCoveredBodies.insert(sc.bodyB);
        }
    }

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        // Bullet と Critical 品質のみが TOI バックステップを取得
        if (body->_ccdQuality < CcdQuality::Bullet) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;
        if (body->_isSleeping) continue;

        // 投機的接触で既にカバーされているボディをスキップ
        // （トンネリングはソルバーで防止されるため、バックステップ不要）
        if (specCoveredBodies.count(body)) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1e-4f) continue;

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE = GetColliderMinHalfExtent(col);
        // 設定されている場合は allowedPenetrationDepth を使用
        const float allowedPen = (body->_allowedPenetrationDepth > 0.0f)
            ? body->_allowedPenetrationDepth : (minHE * 0.8f);
        const float displacement = speed * stepDt;
        if (displacement <= allowedPen) continue;

        const float toi = allowedPen / (speed * stepDt);
        if (toi >= 1.0f) continue;

        ToiEvent ev;
        ev.body = body;
        ev.toi = (std::max)(0.0f, toi);
        ev.toiPosition = VAdd(body->_previousPosition, VScale(body->_velocity, ev.toi * stepDt));
        ev.clampedVelocity = body->_velocity;
        events.push_back(ev);
    }

    if (events.empty()) return;

    // 最も早い TOI を優先してソート
    std::sort(events.begin(), events.end(), [](const ToiEvent& a, const ToiEvent& b) {
        return a.toi < b.toi;
    });

    // 時間順にイベントを処理
    for (auto& ev : events) {
        if (!ev.body || !ev.body->_owner) continue;

        // ボディを TOI 位置にバックステップ
        ev.body->_owner->transform.SetLocalPosition(ev.toiPosition);

        // 残り時間を再積分: (1 - toi) * stepDt
        const float remainDt = (1.0f - ev.toi) * stepDt;
        if (remainDt > 1e-6f) {
            Collider* col = CachedFindCollider(ev.body->_owner);
            const float allowedPen = col
                ? ((ev.body->_allowedPenetrationDepth > 0.0f)
                    ? ev.body->_allowedPenetrationDepth : (GetColliderMinHalfExtent(col) * 0.8f))
                : 0.4f;
            const float remainSpeed = Len3(ev.body->_velocity);
            if (remainSpeed * remainDt > allowedPen) {
                ClampMagnitude(ev.body->_velocity, allowedPen / remainDt);
            }

            // 残り分を進める
            VECTOR pos = ev.body->_owner->transform.LocalPosition();
            pos = VAdd(pos, VScale(ev.body->_velocity, remainDt));

            // 地面平面クランプ
            if (_groundPlaneEnabled) {
                const float dist = HalfPlaneDistance(pos, _groundPlaneNormal, _groundPlaneD);
                if (dist < 0.0f) {
                    pos = VSub(pos, VScale(_groundPlaneNormal, dist));
                    const float vn = Dot3(ev.body->_velocity, _groundPlaneNormal);
                    if (vn < 0.0f) {
                        // 速度成分を平面内に反射（TOI 衝突応答）
                        ev.body->_velocity = VSub(ev.body->_velocity, VScale(_groundPlaneNormal, vn * (1.0f + ev.body->_restitution)));
                    }
                }
            }
            ev.body->_owner->transform.SetLocalPosition(pos);
        }

        // 再配置後にコライダー形状を更新
        Collider* col = CachedFindCollider(ev.body->_owner);
        if (col) col->UpdateShape();
    }

    // 再配置されたボディに対して狭義相検出を再実行
    ColliderManager::Instance().Update(stepDt);
    BuildLookupCaches();
}

// ============================================================
//  PropagateIslandSleep — アイランド内のいずれかが起きていれば全員起こす
// ============================================================

/// <summary>
/// アイランド内のスリープ状態を伝播
/// 1つでも起きているボディがあれば全員起床、全員寝ていればスリープ維持
/// </summary>
void PhysicsManager::PropagateIslandSleep() {
    // --- アイランドスリープ伝播（アイランドごとに並列）---
    const size_t islandCount = _islands.size();
    if (islandCount > 0) {
        ThreadPool::Instance().ParallelForBarrier(0, islandCount, [&](size_t i) {
            auto& island = _islands[i];
            bool anyAwake = false;
            for (auto* body : island.bodies) {
                if (body && body->IsDynamic() && !body->_isSleeping) {
                    anyAwake = true;
                    break;
                }
            }
            if (anyAwake) {
                for (auto* body : island.bodies) {
                    if (body && body->IsDynamic() && body->_isSleeping) {
                        body->WakeUp();
                    }
                }
            }
        }, 1);
    }

    // --- 最終ボディ拘束 + スリープ状態（同一パスに統合）---
    const size_t bodyCount = _bodies.size();
    if (bodyCount > 0) {
        const float stepDt = _fixedDeltaTime;
        ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
            PhysicsBody* body = _bodies[idx];
            if (!body) return;
            if (body->_isSleeping) {
                if (LenSq(body->_velocity) > 1e-8f || LenSq(body->_angularVelocity) > 1e-8f)
                    body->WakeUp();
            }
            ApplyBodyConstraints(body);
            UpdateSleepState(body, stepDt);
        }, 64);
    }
}

// ============================================================
//  補間
// ============================================================

/// <summary>
/// 補間係数αを計算（固定ステップと可変フレームレートの差を埋める）
/// </summary>
/// <returns>補間係数 [0.0, 1.0]</returns>
float PhysicsManager::InterpolationAlpha() const noexcept {
    if (_fixedDeltaTime <= 1e-6f) return 1.0f;
    return (std::min)(_accumulator / _fixedDeltaTime, 1.0f);
}

/// <summary>
/// 全ボディの補間位置/回転を計算
/// 前フレームと現在の物理状態を補間してスムーズな描画を実現
/// </summary>
void PhysicsManager::ComputeInterpolation() noexcept {
    const float alpha = InterpolationAlpha();
    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_owner) return;
        if (!body->_useInterpolation) {
            body->_interpPosition = body->_owner->transform.LocalPosition();
            body->_interpRotation = body->_owner->transform.LocalRotation();
            return;
        }
        // 位置は線形補間、回転は球面線形補間
        const VECTOR curPos = body->_owner->transform.LocalPosition();
        body->_interpPosition = VAdd(
            VScale(body->_previousPosition, 1.0f - alpha),
            VScale(curPos, alpha));
        body->_interpRotation = Quaternion::Slerp(
            body->_previousRotation,
            body->_owner->transform.LocalRotation(),
            alpha);
    }, 64);
}

/// <summary>
/// ボディの拘束を適用（回転凍結、スリープ時の速度ゼロ化）
/// </summary>
/// <param name="body">拘束を適用するボディ</param>
void PhysicsManager::ApplyBodyConstraints(PhysicsBody* body) const {
    if (!body) return;
    if (body->_freezeRotation) body->_angularVelocity = VGet(0,0,0);
    if (body->_isSleeping) { body->_velocity = VGet(0,0,0); body->_angularVelocity = VGet(0,0,0); }
}

/// <summary>
/// GameObjectからPhysicsBodyを線形探索（デバッグ用、通常はCachedFindBodyを使用）
/// </summary>
/// <param name="owner">検索対象のGameObject</param>
/// <returns>見つかったPhysicsBody、存在しない場合はnullptr</returns>
PhysicsBody* PhysicsManager::FindBodyByOwner(GameObject* owner) const {
    if (!owner) return nullptr;
    for (auto* body : _bodies) {
        if (body && body->_owner == owner) return body;
    }
    return nullptr;
}

// ============================================================
//  登録
// ============================================================

/// <summary>
/// PhysicsControllerを登録（毎フレームUpdate()が呼ばれる）
/// </summary>
/// <param name="controller">登録するコントローラー</param>
void PhysicsManager::Register(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    if (_asyncEnabled) {
        // 非同期物理ステップがスナップショットを読み取っている間の同時変更を回避。
        WaitForPhysics();
    }
    std::lock_guard lk(_mtx);
    if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
    _controllers.push_back(controller);
}

/// <summary>
/// PhysicsControllerの登録を解除
/// </summary>
/// <param name="controller">解除するコントローラー</param>
void PhysicsManager::Unregister(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    if (_asyncEnabled) {
        WaitForPhysics();
    }
    std::lock_guard lk(_mtx);
    auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
    _controllers.erase(it, _controllers.end());
}

/// <summary>
/// PhysicsBodyを物理シミュレーションに登録
/// </summary>
/// <param name="body">登録するボディ</param>
void PhysicsManager::RegisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    if (_asyncEnabled) {
        // StepSimulation 中に _bodies はロックなしで読まれる; 非同期ステップが実行中でないことを確認。
        WaitForPhysics();
    }
    std::lock_guard lk(_mtx);
    if (std::find(_bodies.begin(), _bodies.end(), body) != _bodies.end()) return;
    _bodies.push_back(body);
    if (body->_owner) {
        body->_previousPosition = body->_owner->transform.LocalPosition();
        body->_previousRotation = body->_owner->transform.LocalRotation();
        Collider* col = ColliderManager::Instance().FindColliderByOwner(body->_owner);
        body->ComputeInertia(col);
    }
}

/// <summary>
/// PhysicsBodyの登録を解除
/// </summary>
/// <param name="body">解除するボディ</param>
void PhysicsManager::UnregisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    if (_asyncEnabled) {
        WaitForPhysics();
    }
    std::lock_guard lk(_mtx);
    auto it = std::remove(_bodies.begin(), _bodies.end(), body);
    _bodies.erase(it, _bodies.end());
}
