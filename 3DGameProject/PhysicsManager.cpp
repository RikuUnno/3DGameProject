#include "PhysicsManager_Internal.h"

// ---- ライフサイクル -------------------------------------------------

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

void PhysicsManager::SetFixedDeltaTime(float fixedDeltaTime) noexcept {
    _fixedDeltaTime = (fixedDeltaTime > 1e-4f) ? fixedDeltaTime : (1.0f / 120.0f);
}

void PhysicsManager::SetMaxSubSteps(int maxSubSteps) noexcept {
    _maxSubSteps = (maxSubSteps > 1) ? maxSubSteps : 1;
}

void PhysicsManager::SetSolverIterations(int solverIterations) noexcept {
    _solverIterations = (solverIterations > 1) ? solverIterations : 1;
}

int PhysicsManager::ComputeAdaptiveIterations() const noexcept {
    const int contactCount = static_cast<int>(_solverContacts.size());
    int adaptive = _minSolverIterations + contactCount / 8;
    adaptive = (std::max)(adaptive, _minSolverIterations);
    adaptive = (std::min)(adaptive, _maxSolverIterations);
    return (std::max)(adaptive, (std::min)(_solverIterations, _maxSolverIterations));
}

// ---- Update / 非同期 ------------------------------------------------

void PhysicsManager::Update(float dt) {
    if (IsShuttingDown()) return;
    if (dt < 0.0f) dt = 0.0f;

#ifdef _DEBUG
    auto _scopeUpdate = PerformanceMonitor::Instance().Scope("Physics.Update");
#endif

    {
        std::lock_guard lk(_mtx);
        _ctrlSnapshotBuf.assign(_controllers.begin(), _controllers.end());
    }
    for (auto* c : _ctrlSnapshotBuf) {
        if (!c) continue;
        c->Update(dt);
    }

    if (_asyncEnabled) {
#ifdef _DEBUG
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.WaitForPhysics");       WaitForPhysics(); }
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation"); ComputeInterpolation(); }
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.AsyncEnqueue");
          _asyncFuture = ThreadPool::Instance().Enqueue([this, dt]() { RunAsyncStep(dt); }); }
#else
        WaitForPhysics();
        ComputeInterpolation();
        _asyncFuture = ThreadPool::Instance().Enqueue([this, dt]() { RunAsyncStep(dt); });
#endif
    } else {
        const float maxDt = _fixedDeltaTime * static_cast<float>(_maxSubSteps);
        _accumulator += (std::min)(dt, maxDt);
        int sub = 0;
        while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < _maxSubSteps) {
#ifdef _DEBUG
            { auto _s = PerformanceMonitor::Instance().Scope("Physics.StepSimulation"); StepSimulation(_fixedDeltaTime); }
#else
            StepSimulation(_fixedDeltaTime);
#endif
            _accumulator -= _fixedDeltaTime;
            ++sub;
        }
        if (_accumulator < 0.0f) _accumulator = 0.0f;
#ifdef _DEBUG
        { auto _s = PerformanceMonitor::Instance().Scope("Physics.ComputeInterpolation"); ComputeInterpolation(); }
#else
        ComputeInterpolation();
#endif
    }
}

void PhysicsManager::WaitForPhysics() {
    if (_asyncFuture.valid()) _asyncFuture.get();
}

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

// ---- ルックアップキャッシュ -----------------------------------------

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

PhysicsBody* PhysicsManager::CachedFindBody(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _bodyByOwner.find(owner);
    return (it != _bodyByOwner.end()) ? it->second : nullptr;
}

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
#else
        SplitImpulseCorrection(stepDt);
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
