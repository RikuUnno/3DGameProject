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
	const int adaptiveByBodies = _minSolverIterations + maxIslandBodies / 8;
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
		{
			auto _s = PerformanceMonitor::Instance().Scope("Physics.AsyncEnqueue");
			_asyncFuture = ThreadPool::Instance().Enqueue([this, dt, dynamicMaxSubSteps]() { RunAsyncStep(dt, dynamicMaxSubSteps); });
		}

#else           // リリースビルドでは計測なしでシンプルに実行する
		WaitForPhysics();
		ComputeInterpolation();
		_asyncFuture = ThreadPool::Instance().Enqueue([this, dt, dynamicMaxSubSteps]() { RunAsyncStep(dt, dynamicMaxSubSteps); });

#endif          // 非同期ステップを開始する前に、前のステップが完了していることを保証するために WaitForPhysics() を呼び出す。
	}
	else
	{
		// maxDt : サブステップの最大合計時間。これを超える分は切り捨てる（spiral of death 防止）。例えば fixedDeltaTime=1/60, maxSubSteps=5 なら maxDt=1/12。
		const float maxDt = _fixedDeltaTime * static_cast<float>(dynamicMaxSubSteps);
		_accumulator += (std::min)(dt, maxDt);
		int sub = 0;

		while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < dynamicMaxSubSteps) { // _accumulator が fixedDeltaTime 以上ある限り、サブステップを回す。誤差でわずかに足りない場合は +1e-6f でカバーする
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
	if (_asyncFuture.valid()) _asyncFuture.get(); // もし前の非同期ステップがまだ完了していない場合は、get() を呼び出して完了を待つ。すでに完了している場合はすぐに戻る。
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
	
	_bodyByOwner.clear();						// まずはハッシュマップをクリアする。前のフレームの内容が残っていると、Owner が削除された Body / Collider がキャッシュに残ってしまう可能性があるため。
	const size_t bodySize = _bodies.size();		// Body 数を取得する。これをもとにハッシュマップのバケット数を予約することで、挿入時の再ハッシュを減らしてパフォーマンスを向上させる。
	
	// もし Body 数が多い場合は、ハッシュマップのバケット数を事前に予約しておく。LoadFactor を考慮して、必要なバケット数は bodySize * 2 程度。
	if (bodySize > 0 && _bodyByOwner.bucket_count() < bodySize * 2) _bodyByOwner.reserve(bodySize);
	
	// BodyManager から全 Body を取得して、Owner (GameObject) をキー、Body ポインタを値とするハッシュマップを構築する。Owner が nullptr の Body はスキップする。
	for (auto* body : _bodies) {
		if (!body || !body->_owner) continue;
		_bodyByOwner.emplace(body->_owner, body);
	}

	// 同様に、ColliderManager から全 Collider を取得して、Owner (GameObject) をキー、Collider ポインタを値とするハッシュマップを構築する。Owner が nullptr の Collider はスキップする。
	_colliderByOwner.clear();
	const auto& colliders = ColliderManager::Instance().GetColliders();
	const size_t colSize = colliders.size();
	
	// もし Collider 数が多い場合は、ハッシュマップのバケット数を事前に予約しておく。LoadFactor を考慮して、必要なバケット数は colSize * 2 程度。
	if (colSize > 0 && _colliderByOwner.bucket_count() < colSize * 2) _colliderByOwner.reserve(colSize);
	
	// ColliderManager から全 Collider を取得して、Owner (GameObject) をキー、Collider ポインタを値とするハッシュマップを構築する。Owner が nullptr の Collider はスキップする。
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


//StepSimulation: 物理シミュレーションの 1 ステップを実行する。stepDt はサブステップの時間。Update() から呼び出される。
void PhysicsManager::StepSimulation(float stepDt) {
#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildLookupCaches");  BuildLookupCaches(); }
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.IntegrateBodies");    IntegrateBodies(stepDt); }
	{ auto _s = PerformanceMonitor::Instance().Scope("Collider.Update");            ColliderManager::Instance().Update(stepDt); }
#else			// リリースビルドでは計測なしでシンプルに実行する
	BuildLookupCaches();
	IntegrateBodies(stepDt);
	ColliderManager::Instance().Update(stepDt);
#endif

#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildSolverContacts");     BuildSolverContacts(stepDt); }
	// Speculative CCD を有効にしている場合は、BuildSolverContacts と GenerateSpeculativeContacts を分けて計測する。どちらも同じくらい重い可能性があるため。
	if (_havokCcdEnabled || _speculativeCcdEnabled) {
		auto _s = PerformanceMonitor::Instance().Scope("Physics.GenerateSpeculativeContacts");
		GenerateSpeculativeContacts(stepDt);
	}
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.BuildIslands");  BuildIslands(); }
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.WarmStart");     WarmStart(); }
#else			// リリースビルドでは計測なしでシンプルに実行する	
	BuildSolverContacts(stepDt);
	if (_havokCcdEnabled || _speculativeCcdEnabled) GenerateSpeculativeContacts(stepDt);
	BuildIslands();
	WarmStart();
#endif

	const int iterations = ComputeAdaptiveIterations();	// コンタクト数やアイランドの大きさに応じて、イテレーション数を動的に増減させる
	const int savedIter = _solverIterations;			// 現在のイテレーション数を保存しておく。ComputeAdaptiveIterations() で動的に計算されたイテレーション数を一時的に _solverIterations にセットする。
	_solverIterations = iterations;						// ComputeAdaptiveIterations() で計算されたイテレーション数を _solverIterations にセットする。これにより、SolveAllIslands() 内で adaptive なイテレーション数が使用されるようになる。

#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.SolveAllIslands"); SolveAllIslands(stepDt); }
#else			// リリースビルドでは計測なしでシンプルに実行する
	SolveAllIslands(stepDt);
#endif
	_solverIterations = savedIter;						// SolveAllIslands() が終わったら、_solverIterations を元の値に戻す。これにより、次のフレームで ComputeAdaptiveIterations() が呼び出されたときに、正しいベースイテレーション数が使用されるようになる。

	// Split impulse を有効にしている場合は、SolveAllIslands() の後に SplitImpulseCorrection() を呼び出す。これにより、位置補正フェーズで深いめり込みを効果的に解消することができる。
	if (_splitImpulseEnabled) {
#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
		{ auto _s = PerformanceMonitor::Instance().Scope("Physics.SplitImpulseCorrection"); SplitImpulseCorrection(stepDt); }
		{ auto _s = PerformanceMonitor::Instance().Scope("Physics.PositionalCorrection"); PositionalCorrection(stepDt, 0.05f, 0.3f); }
#else			// リリースビルドでは計測なしでシンプルに実行する
		SplitImpulseCorrection(stepDt);
		PositionalCorrection(stepDt, 0.05f, 0.3f);
#endif
	}
	else 
	{
#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
		{ auto _s = PerformanceMonitor::Instance().Scope("Physics.PositionalCorrection"); PositionalCorrection(stepDt); }
#else			// リリースビルドでは計測なしでシンプルに実行する
		PositionalCorrection(stepDt);
#endif
	}

	// PropagateIslandSleep は今フレームの接触 (_solverContacts) を参照して
	// wake/sleep を決めるため、swap より前に呼ぶ。swap 後に呼ぶと
	// 前フレームの古い接触で判定してしまい、めり込みが残っている接触を
	// 見逃してスリープ固定化する。
#ifdef _DEBUG	// デバッグビルドでは物理ステップの区間をさらに細かく計測する
	{ auto _s = PerformanceMonitor::Instance().Scope("Physics.PropagateIslandSleep"); PropagateIslandSleep(); }
#else			// リリースビルドでは計測なしでシンプルに実行する
	PropagateIslandSleep();
#endif

	_prevSolverContacts.swap(_solverContacts);
}

// 登録・解除関数: コントローラーやボディを物理マネージャーに登録・解除するための関数。これらは、物理ステップの前に呼び出されることが想定されている。
// Register() : コントローラーを物理マネージャーに登録する。すでに登録されている場合は何もしない。コントローラーが nullptr の場合も何もしない。
void PhysicsManager::Register(PhysicsController* controller) {
	if (IsShuttingDown() || !controller) return;
	if (_asyncEnabled) WaitForPhysics();
	std::lock_guard lk(_mtx);
	if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
	_controllers.push_back(controller);
}
// Unregister() : コントローラーを物理マネージャーから解除する。登録されていない場合は何もしない。コントローラーが nullptr の場合も何もしない。
void PhysicsManager::Unregister(PhysicsController* controller) {
	if (IsShuttingDown() || !controller) return;
	if (_asyncEnabled) WaitForPhysics();
	std::lock_guard lk(_mtx);
	auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
	_controllers.erase(it, _controllers.end());
}
// RegisterBody() : ボディを物理マネージャーに登録する。すでに登録されている場合は何もしない。ボディが nullptr の場合も何もしない。登録されたボディの Owner (GameObject) が存在する場合は、その位置と回転を前フレームの状態として保存し、コライダーから慣性テンソルを計算して保存する。
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
// UnregisterBody() : ボディを物理マネージャーから解除する。登録されていない場合は何もしない。ボディが nullptr の場合も何もしない。
void PhysicsManager::UnregisterBody(PhysicsBody* body) {
	if (IsShuttingDown() || !body) return;
	if (_asyncEnabled) WaitForPhysics();
	std::lock_guard lk(_mtx);
	auto it = std::remove(_bodies.begin(), _bodies.end(), body);
	_bodies.erase(it, _bodies.end());
}
// FindBodyByOwner() : Owner (GameObject) から Body を見つけるための関数。Owner が nullptr の場合は nullptr を返す。Owner と一致する Body が複数ある場合は、最初に見つかったものを返す。
PhysicsBody* PhysicsManager::FindBodyByOwner(GameObject* owner) const {
	if (!owner) return nullptr;
	for (auto* body : _bodies) {
		if (body && body->_owner == owner) return body;
	}
	return nullptr;
}
