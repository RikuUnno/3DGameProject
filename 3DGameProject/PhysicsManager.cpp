#include "PhysicsManager_Internal.h"

// Shutdown 時に呼び出される
// 物理スレッドが停止していることを保証した上で、全データをクリアする。
void PhysicsManager::Shutdown() {
    const bool was = _shuttingDown.exchange(true, std::memory_order_relaxed);
    if (was) return;
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

// 固定フレームレートとサブステップ数の設定
void PhysicsManager::SetFixedDeltaTime(float fixedDeltaTime) noexcept {
    _fixedDeltaTime = (fixedDeltaTime > 1e-4f) ? fixedDeltaTime : (1.0f / 120.0f);
}

// サブステップ数の設定
void PhysicsManager::SetMaxSubSteps(int maxSubSteps) noexcept {
    _maxSubSteps = (maxSubSteps > 1) ? maxSubSteps : 1;
}

// ソルバー反復数の設定
void PhysicsManager::SetSolverIterations(int solverIterations) noexcept {
    _solverIterations = (solverIterations > 1) ? solverIterations : 1;
}

// コンタクト数やアイランドの大きさに応じて、イテレーション数を動的に増減させる
int PhysicsManager::ComputeAdaptiveIterations() const noexcept {
    const int contactCount = static_cast<int>(_solverContacts.size());

    // 最大アイランドのボディ数も考慮する（巨大アイランドほど収束に多くのイテレーションが必要）
    int maxIslandBodies = 0;
    for (const auto& island : _islands) {
        if (!island.allSleeping)
            maxIslandBodies = (std::max)(maxIslandBodies, static_cast<int>(island.bodies.size()));
    }

    // コンタクト数とボディ数の両方からイテレーション数を算出し、大きい方を採用
    const int adaptiveByContact = _minSolverIterations + contactCount / 16;
    const int adaptiveByBodies  = _minSolverIterations + maxIslandBodies / 8;
    int adaptive = (std::max)(adaptiveByContact, adaptiveByBodies);
    adaptive = (std::max)(adaptive, _minSolverIterations);
    adaptive = (std::min)(adaptive, _maxSolverIterations);

    const int base = (std::min)(_solverIterations, _maxSolverIterations);
    return (std::max)(adaptive, base);
}

// Update: 物理シミュレーションのステップを進める。dt は前フレームからの経過時間
void PhysicsManager::Update(float dt) {
    if (IsShuttingDown()) return;

#ifdef _DEBUG // デバッグビルドでは物理更新の区間を計測する
    auto _scopeUpdate = PerformanceMonitor::Instance().Scope("Physics.Update");
#endif

	// dt が負の値になることは通常ないが、万が一そうなった場合は 0 にクランプする（物理ステップを進めない）
    if (dt < 0.0f) dt = 0.0f;
    {
        std::lock_guard lk(_mtx);
        _ctrlSnapshotBuf.assign(_controllers.begin(), _controllers.end());
    }

	// コントローラーの Update を呼び出す（物理ステップの前に状態を更新してもらう）
    for (auto* c : _ctrlSnapshotBuf) {
        if (!c) continue;
        c->Update(dt);
    }

    // フレーム時間が大きいほどサブステップ数を抑える。
    //   - 通常 (dt < fixedDt*2):     _maxSubSteps をそのまま
    //   - やや遅い (< fixedDt*4):    2 にクランプ
    //   - 重い (>= fixedDt*4):       1 にクランプ (spiral of death 防止)
    // 物理 1 ステップが重いシーンで dt が伸びると、次フレームで accumulator が
    // 貯まりさらに多くサブステップを回す → さらに遅れる悪循環を切る。
    int dynamicMaxSubSteps = _maxSubSteps;
    if (dt >= _fixedDeltaTime * 4.0f)      dynamicMaxSubSteps = 1;
    else if (dt >= _fixedDeltaTime * 2.0f) dynamicMaxSubSteps = (std::min)(_maxSubSteps, 2);

    if (_asyncEnabled) {
#ifdef _DEBUG   // デバッグビルドでは物理更新の区間をさらに細かく計測する
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.WaitForPhysics");       WaitForPhysics(); }
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation"); ComputeInterpolation(); }
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.AsyncEnqueue");
          _asyncFuture = ThreadPool::Instance().Enqueue([this, dt, dynamicMaxSubSteps]() { RunAsyncStep(dt, dynamicMaxSubSteps); }); }
#else           // リリースビルドでは計測なしでシンプルに実行する
        WaitForPhysics();
        ComputeInterpolation();
        _asyncFuture = ThreadPool::Instance().Enqueue([this, dt, dynamicMaxSubSteps]() { RunAsyncStep(dt, dynamicMaxSubSteps); });
#endif          // 非同期ステップを開始する前に、前のステップが完了していることを保証するために WaitForPhysics() を呼び出す。
    }   else 
	    {   
        const float maxDt = _fixedDeltaTime * static_cast<float>(dynamicMaxSubSteps);
        _accumulator += (std::min)(dt, maxDt);
        int sub = 0;

        while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < dynamicMaxSubSteps) {
#ifdef _DEBUG   // デバッグビルドでは物理ステップの区間をさらに細かく計測する
            { auto _s = PerformanceMonitor::Instance().Scope("Physics.StepSimulation"); StepSimulation(_fixedDeltaTime); }
#else           // リリースビルドでは計測なしでシンプルに実行する
            StepSimulation(_fixedDeltaTime);
#endif          // サブステップを回すごとに accumulator から fixedDeltaTime を減らしていく
            _accumulator -= _fixedDeltaTime;
            ++sub;
        }
		// 浮動小数点の誤差で accumulator がわずかに負になることがあるので、0 未満になったらクランプする
        if (_accumulator < 0.0f) _accumulator = 0.0f;
#ifdef _DEBUG   // サブステップのループが終わった後に、補間計算の区間を計測する
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation"); ComputeInterpolation(); }
#else           // リリースビルドでは計測なしでシンプルに実行する
        ComputeInterpolation();
#endif          // Update の最後に ComputeInterpolation() を呼び出して、次のフレームで補間されたトランスフォームを計算する
        }
}

// 前の物理ステップが非同期で実行されている場合に、それが完了するまで待機する
void PhysicsManager::WaitForPhysics() {
    if (_asyncFuture.valid()) _asyncFuture.get();
}

// 非同期で物理ステップを実行するための関数。Update() から ThreadPool に渡されて呼び出される。
void PhysicsManager::RunAsyncStep(float dt, int maxSubSteps) {
    const float maxDt = _fixedDeltaTime * static_cast<float>((std::max)(1, maxSubSteps));
    _accumulator += (std::min)(dt, maxDt);
    int sub = 0;
    while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < (std::max)(1, maxSubSteps)) {
        StepSimulation(_fixedDeltaTime);
        _accumulator -= _fixedDeltaTime;
        ++sub;
    }
    if (_accumulator < 0.0f) _accumulator = 0.0f;
}

// キャッシュの構築
// 物理ステップの前に呼び出される。Owner (GameObject) から Body / Collider を O(1) で見つけるためのハッシュマップを構築する。
void PhysicsManager::BuildLookupCaches() {
    _bodyByOwner.clear();
    const size_t bodySize = _bodies.size();
    if (bodySize > 0 && _bodyByOwner.bucket_count() < bodySize * 2)
        _bodyByOwner.reserve(bodySize);
    for (auto* body : _bodies) {
        if (!body || !body->_owner) continue;
        _bodyByOwner.emplace(body->_owner, body);
    }
    _colliderByOwner.clear();
    const auto& colliders = ColliderManager::Instance().GetColliders();
    const size_t colSize = colliders.size();
    if (colSize > 0 && _colliderByOwner.bucket_count() < colSize * 2)
        _colliderByOwner.reserve(colSize);
    for (auto* col : colliders) {
        if (!col || !col->owner) continue;
        _colliderByOwner.emplace(col->owner, col);
    }
}

// Owner (GameObject) から Body / Collider を O(1) で見つけるためのキャッシュを参照する関数。Owner が nullptr なら nullptr を返す。
PhysicsBody* PhysicsManager::CachedFindBody(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _bodyByOwner.find(owner);
    return (it != _bodyByOwner.end()) ? it->second : nullptr;
}

// Owner (GameObject) から Body / Collider を O(1) で見つけるためのキャッシュを参照する関数。Owner が nullptr なら nullptr を返す。
Collider* PhysicsManager::CachedFindCollider(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _colliderByOwner.find(owner);
    return (it != _colliderByOwner.end()) ? it->second : nullptr;
}

// ---- StepSimulation -------------------------------------------------

void PhysicsManager::StepSimulation(float stepDt) {
#ifdef _DEBUG
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildLookupCaches");  BuildLookupCaches(); }
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.IntegrateBodies");    IntegrateBodies(stepDt); }
    { auto _s = PerformanceMonitor::Instance().Scope("Collider.Update");            ColliderManager::Instance().Update(stepDt); }
#else
    BuildLookupCaches();
    IntegrateBodies(stepDt);
    ColliderManager::Instance().Update(stepDt);
#endif

#ifdef _DEBUG
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildSolverContacts");     BuildSolverContacts(stepDt); }
    if (_havokCcdEnabled || _speculativeCcdEnabled) {
        auto _s = PerformanceMonitor::Instance().Scope("Physics.GenerateSpeculativeContacts");
        GenerateSpeculativeContacts(stepDt);
    }
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildIslands");  BuildIslands(); }
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.WarmStart");     WarmStart(); }
#else
    BuildSolverContacts(stepDt);
    if (_havokCcdEnabled || _speculativeCcdEnabled) GenerateSpeculativeContacts(stepDt);
    BuildIslands();
    WarmStart();
#endif

    const int iterations = ComputeAdaptiveIterations();
    const int savedIter  = _solverIterations;
    _solverIterations    = iterations;
#ifdef _DEBUG
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.SolveAllIslands"); SolveAllIslands(stepDt); }
#else
    SolveAllIslands(stepDt);
#endif
    _solverIterations = savedIter;

    if (_splitImpulseEnabled) {
#ifdef _DEBUG
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.SplitImpulseCorrection"); SplitImpulseCorrection(stepDt); }
        // SplitImpulse は速度ベースの反復のため wedge / 多接触では収束しきれず
        // 深いめり込みが残ることがある。仕上げに距離ベースの PositionalCorrection
        // を「深いめり込みのみ・縮小バイアス」で呼んで残留分のみ押し出す。
        // 浅い接触 (kSlop ~ 数 cm) には触れないので壁張り付きの原因になる
        // 過剰押し出しは発生しない。
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.PositionalCorrection"); PositionalCorrection(stepDt, 0.05f, 0.3f); }
#else
        SplitImpulseCorrection(stepDt);
        PositionalCorrection(stepDt, 0.05f, 0.3f);
#endif
    } else {
#ifdef _DEBUG
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.PositionalCorrection"); PositionalCorrection(stepDt); }
#else
        PositionalCorrection(stepDt);
#endif
    }

    _prevSolverContacts.swap(_solverContacts);

#ifdef _DEBUG
    { auto _s = PerformanceMonitor::Instance().Scope("Physics.PropagateIslandSleep"); PropagateIslandSleep(); }
#else
    PropagateIslandSleep();
#endif
}

// ---- 登録 -----------------------------------------------------------

void PhysicsManager::Register(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    if (_asyncEnabled) WaitForPhysics();
    std::lock_guard lk(_mtx);
    if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
    _controllers.push_back(controller);
}

void PhysicsManager::Unregister(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    if (_asyncEnabled) WaitForPhysics();
    std::lock_guard lk(_mtx);
    auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
    _controllers.erase(it, _controllers.end());
}

void PhysicsManager::RegisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    if (_asyncEnabled) WaitForPhysics();
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

void PhysicsManager::UnregisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    if (_asyncEnabled) WaitForPhysics();
    std::lock_guard lk(_mtx);
    auto it = std::remove(_bodies.begin(), _bodies.end(), body);
    _bodies.erase(it, _bodies.end());
}

PhysicsBody* PhysicsManager::FindBodyByOwner(GameObject* owner) const {
    if (!owner) return nullptr;
    for (auto* body : _bodies) {
        if (body && body->_owner == owner) return body;
    }
    return nullptr;
}
