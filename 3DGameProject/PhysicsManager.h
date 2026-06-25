#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <algorithm>

#include "DxLib.h"
#include "Manager.h"

class PhysicsController;
class PhysicsBody;
class GameObject;
class Collider;

// SolverContact - ソルバーで使用する接触情報
struct SolverContact {
	// ホットパス（インパルス計算・適用に使用）: 0-128B = 2 キャッシュライン

	// Body ポインタ・逆質量（インパルス適用に必須）
	PhysicsBody* bodyA = nullptr;       // @ 0   (8B)
	PhysicsBody* bodyB = nullptr;       // @ 8   (8B)
	float invA = 0.0f;                  // @ 16  (4B)
	float invB = 0.0f;                  // @ 20  (4B)

	// 有効逆質量（BuildSolverContacts で事前計算）
	float effectiveInvMassN  = 0.0f;   // @ 24  (4B)
	float effectiveInvMassT1 = 0.0f;   // @ 28  (4B)
	float effectiveInvMassT2 = 0.0f;   // @ 32  (4B)

	// バイアス・摩擦係数
	float normalBias     = 0.0f;       // @ 36  (4B)
	float restitution    = 0.0f;       // @ 40  (4B)
	float friction       = 0.0f;       // @ 44  (4B)
	float staticFriction = 0.0f;       // @ 48  (4B)

	// 累積インパルス（warm-start / クランプ用）
	float normalLambda    = 0.0f;      // @ 52  (4B)
	float frictionLambda1 = 0.0f;      // @ 56  (4B)
	float frictionLambda2 = 0.0f;      // @ 60  (4B)
	//  ↑ ここまで 64B = キャッシュライン 1

	// 法線・腕ベクトル・接線（速度計算・インパルス適用に使用）
	VECTOR normal   = VGet(0,1,0);     // @ 64  (12B)
	VECTOR rA       = VGet(0,0,0);     // @ 76  (12B)
	VECTOR rB       = VGet(0,0,0);     // @ 88  (12B)
	VECTOR tangent1 = VGet(1,0,0);     // @ 100 (12B)
	VECTOR tangent2 = VGet(0,0,1);     // @ 112 (12B)
	bool   speculative = false;        // @ 124 (1B + 3B padding)
	//  ↑ ここまで 128B = キャッシュライン 1-2


	// コールドパス（位置補正・マッチングのみ使用）: 128B 以降

	// Split impulse（位置補正フェーズのみ）
	float splitNormalLambda = 0.0f;    // @ 128
	float splitBias         = 0.0f;    // @ 132
	float penetration       = 0.0f;    // @ 136

	// 転がり抵抗の累積角インパルス（接触法線まわりの回転暴走を抑える）
	VECTOR rollingLambda = VGet(0, 0, 0);

	// アイランド ID
	int islandId = -1;                 // @ 140

	// 接触点・ローカル座標（Warm-start マッチング用）
	VECTOR point  = VGet(0,0,0);       // @ 144
	VECTOR localA = VGet(0,0,0);       // @ 156
	VECTOR localB = VGet(0,0,0);       // @ 168

	// コライダー識別子（マッチング・イベント用）
	Collider* colA = nullptr;          // @ 180
	Collider* colB = nullptr;          // @ 188
	// 合計 196B = 3 キャッシュライン（ホットデータは先頭 2 本に収まる）
};

// ※ SolverContact は 196B で、キャッシュライン 3 本に収まる
// ※ キャッシュラインとは CPU のキャッシュメモリの最小単位で、通常 64B です
// キャッシュラインに収まるデータは連続してアクセスされるため、CPU のキャッシュ効率が向上します
// SolverContact のホットデータ（頻繁にアクセスされるデータ）は先頭の 2 キャッシュライン（128B）に収め
// コールドデータ（あまりアクセスされないデータ）は後ろのキャッシュラインに配置することで、パフォーマンスを最適化しています。


// Island - 物理ボディの接続グラフ（接触でつながったボディ群）
struct PhysicsIsland {
	std::vector<PhysicsBody*> bodies;					// Island に属する PhysicsBody のポインタ
	std::vector<int> contactIndices;					// SolverContact のインデックス
	std::vector<std::vector<int>> constraintBatches;	// 制約ソルバーバッチ（各バッチは並列化可能）
	bool allSleeping = false;							// すべてのボディがスリープ状態か
};

// Island は、物理シミュレーションにおいて、接触や制約によって相互に影響を与える物理ボディの集合を表します
// 各 Island は独立してシミュレーションされるため、パフォーマンスの最適化やスリープ判定の効率化が可能です。


// PhysicsManager - 物理シミュレーション管理
class PhysicsManager : public Manager
{
private:
	// コンストラクタ/デストラクタ
	PhysicsManager() = default;
	virtual ~PhysicsManager() = default;

public:
	// シングルトン取得
	static PhysicsManager& Instance() {
		static PhysicsManager inst;
		return inst;
	}

	// コピー禁止
	PhysicsManager(const PhysicsManager&) = delete;
	PhysicsManager& operator=(const PhysicsManager&) = delete;

	// Manager インターフェース
	void Initialize() override {}		// 初期化は ColliderManager で行うのでここでは何もしない
	void Shutdown() override;			// シャットダウン処理
	void Update(float dt) override;		// 更新処理

	// PhysicsManager のシャットダウン中かどうか（PhysicsController から参照される場合があるので、デストラクタではなくフラグで判定する）
	bool IsShuttingDown() const noexcept { return _shuttingDown.load(std::memory_order_relaxed); }

	// PhysicsController 登録/解除
	void Register(PhysicsController* controller);		// PhysicsController を登録する。Update() で毎フレーム Update(dt) が呼ばれる
	void Unregister(PhysicsController* controller);		// PhysicsController を解除する。Update() で Update(dt) が呼ばれなくなる
	void RegisterBody(PhysicsBody* body);				// PhysicsBody を登録する。Update() で毎フレーム物理シミュレーションが行われる
	void UnregisterBody(PhysicsBody* body);				// PhysicsBody を解除する。Update() で物理シミュレーションが行われなくなる

	// モニター用公開API
	const std::vector<PhysicsBody*>& GetBodies() const noexcept { return _bodies; }		// 登録されている PhysicsBody のリストを取得する
	const std::vector<PhysicsIsland>& GetIslands() const noexcept { return _islands; }	// 登録されている PhysicsIsland のリストを取得する

	// 物理シミュレーションの設定
	void SetGroundPlaneEnabled(bool enabled) noexcept { _groundPlaneEnabled = enabled; }								// 地面平面の有効化/無効化（true なら y = GroundPlaneY() の半空間に押し戻す）
	bool IsGroundPlaneEnabled() const noexcept { return _groundPlaneEnabled; }											// GroundPlaneEnabled の取得
	void SetGroundPlaneY(float y) noexcept { _groundPlaneNormal = VGet(0, 1, 0); _groundPlaneD = y; }					// 地面平面の高さを設定（法線は上向き固定）。y = GroundPlaneY() の半空間に押し戻す
	float GroundPlaneY() const noexcept { return _groundPlaneD; }														// 地面平面の高さを取得
	void SetGroundPlane(const VECTOR& normal, float d) noexcept { _groundPlaneNormal = normal; _groundPlaneD = d; }		// 地面平面の法線と高さを設定
	VECTOR GroundPlaneNormal() const noexcept { return _groundPlaneNormal; }											// 地面平面の法線を取得
	float GroundPlaneD() const noexcept { return _groundPlaneD; }														// 地面平面の高さを取得

	// 固定フレームレートとサブステップ数の設定
	void SetFixedDeltaTime(float fixedDeltaTime) noexcept;																// 固定フレームレートの設定（秒）。1/120秒以下は無効。デフォルトは 1/60 秒
	float FixedDeltaTime() const noexcept { return _fixedDeltaTime; }													// 固定フレームレートの取得（秒）
	void SetMaxSubSteps(int maxSubSteps) noexcept;																		// サブステップ数の設定。1以下は無効。デフォルトは 4
	int MaxSubSteps() const noexcept { return _maxSubSteps; }															// サブステップ数の取得
	void SetSolverIterations(int solverIterations) noexcept;															// ソルバー反復回数の設定。1以下は無効。デフォルトは 10
	int SolverIterations() const noexcept { return _solverIterations; }													// ソルバー反復回数の取得

	// Split impulse（ペネトレーションリカバリーの分離インパルス）の有効化/無効化
	void SetSplitImpulseEnabled(bool enabled) noexcept { _splitImpulseEnabled = enabled; }								// true ならペネトレーションリカバリーの分離インパルスを有効化（ペネトレーションが深い場合に位置補正のみ行い、速度は変化させない）
	bool IsSplitImpulseEnabled() const noexcept { return _splitImpulseEnabled; }										// Split impulse の有効化/無効化の取得

	// Speculative CCD（予測接触検出）の有効化/無効化
	void SetSpeculativeCcdEnabled(bool enabled) noexcept { _speculativeCcdEnabled = enabled; }							// Speculative CCD の有効化/無効化の設定
	bool IsSpeculativeCcdEnabled() const noexcept { return _speculativeCcdEnabled; }									// Speculative CCD の有効化/無効化の取得
	// Havok CCD（スイープAABBによる連続衝突検出）の有効化/無効化
	void SetHavokCcdEnabled(bool enabled) noexcept { _havokCcdEnabled = enabled; }										// Havok CCD の有効化/無効化の設定
	bool IsHavokCcdEnabled() const noexcept { return _havokCcdEnabled; }												// Havok CCD の有効化/無効化の取得

	// ※ Speculative CCD は高速移動でのトンネルを減らすための予測接触検出で、Havok CCD はスイープAABBによる連続衝突検出です。両方有効化することも可能ですが、コストが上がります。

	// InterpolationAlpha の計算（サブステップ補間用）
	// alpha = 0.0f のときは前フレームの位置、alpha = 1.0f のときは現在フレームの位置を使用する。
	void ComputeInterpolation() noexcept;																				// サブステップ補間の計算（Update() 内で呼ばれる）
	float InterpolationAlpha() const noexcept;																			// サブステップ補間のアルファ値の取得（0.0f から 1.0f の範囲）

private:
	// PhysicsController 管理
	void StepSimulation(float stepDt);																					// 物理シミュレーションのステップ実行（固定フレームレートで呼ばれる）
	void IntegrateBodies(float stepDt);																					// 物理ボディの統合（速度・位置の更新）
	void UpdateSleepState(PhysicsBody* body, float stepDt);																// 物理ボディのスリープ状態の更新
	void ApplyBodyConstraints(PhysicsBody* body) const;																	// 物理ボディの制約条件の適用（FreezeRotation など）
	PhysicsBody* FindBodyByOwner(GameObject* owner) const;																// GameObject から PhysicsBody を検索

	// ルックアップキャッシュ（PhysicsBody/Collider の所有者 GameObject からの検索を高速化する）
	void BuildLookupCaches();																							// PhysicsBody/Collider の所有者 GameObject からの検索用キャッシュを構築
	PhysicsBody* CachedFindBody(GameObject* owner) const;																// GameObject から PhysicsBody をキャッシュ検索
	Collider* CachedFindCollider(GameObject* owner) const;																// GameObject から Collider をキャッシュ検索
	std::unordered_map<GameObject*, PhysicsBody*> _bodyByOwner{};														// GameObject* → PhysicsBody* のルックアップキャッシュ
	std::unordered_map<GameObject*, Collider*>    _colliderByOwner{};													// GameObject* → Collider* のルックアップキャッシュ

	// SoA キャッシュ (IntegrateBodies 高速化用)
	// AoS (PhysicsBody*) を連続メモリの SoA に展開し、
	// ParallelFor のキャッシュヒット率を向上させる。
	struct BodySoA {
		std::vector<VECTOR> position;					// ワールド位置
		std::vector<VECTOR> velocity;					// ワールド速度
		std::vector<VECTOR> angularVelocity;			// ワールド角速度
		std::vector<VECTOR> force;						// ワールド外力
		std::vector<VECTOR> torque;						// ワールド外力モーメント
		std::vector<float>  inverseMass;				// 逆質量（0 の場合は無限大）
		std::vector<float>  linearDamping;				// 線形減衰係数（0.0f で減衰なし、1.0f で完全停止）
		std::vector<float>  angularDamping;				// 角減衰係数（0.0f で減衰なし、1.0f で完全停止）
		std::vector<float>  gravityScale;				// 重力スケール（0.0f で重力無効、1.0f で通常重力）
		std::vector<uint8_t> flags;						// BodyFlag を OR したビット集合 (BodyFlag を参照)

		// SoA フラグ用ビット定義 (BitOperation と組み合わせて使う)
		struct BodyFlag {
			static constexpr uint8_t Active		 = 1 << 0; // アクティブボディ
			static constexpr uint8_t Kinematic   = 1 << 1; // キネマティックボディ
			static constexpr uint8_t Sleeping    = 1 << 2; // スリープ中
			static constexpr uint8_t UseGravity  = 1 << 3; // 重力を使用
			static constexpr uint8_t FreezeRot   = 1 << 4; // 回転を固定
			static constexpr uint8_t Ccd         = 1 << 5; // 連続衝突検出
		};

		// SoA のサイズを n にリサイズする
		void Resize(size_t n) {
			position.resize(n); velocity.resize(n); angularVelocity.resize(n);
			force.resize(n); torque.resize(n);
			inverseMass.resize(n); linearDamping.resize(n);
			angularDamping.resize(n); gravityScale.resize(n);
			flags.resize(n);
		}
	};
	BodySoA _bodySoA{};					// SoA キャッシュ
	void GatherBodySoA();				// AoS (PhysicsBody*) から SoA に展開
	void ScatterBodySoA(float stepDt);	// SoA から AoS (PhysicsBody*) に反映

	// ※ SoA (Structure of Arrays) とは、構造体のメンバごとに配列を作ることで、キャッシュ効率を向上させる手法です
	// 例えば PhysicsBody の position, velocity, angularVelocity をそれぞれ別の配列に格納することで、並列処理やキャッシュヒット率を改善できます。
	// ※ AoS (Array of Structs) とは、構造体の配列のことです。例えば PhysicsBody の配列は AoS です。


	// SolverContact の構築とソルバーフェーズ
	// ColliderManager の生 Contact から SolverContact を構築し、
	// 前フレームの累積インパルスを照合してウォームスタート。
	void BuildSolverContacts(float stepDt);																// SolverContact の構築とウォームスタート
	void WarmStart();																					// 前フレームの累積インパルスを SolverContact に適用してウォームスタート
	void SolveIsland(const PhysicsIsland& island, float stepDt);										// アイランドごとのソルバーフェーズ（PGS）
	void SolveRollingFriction(SolverContact& sc);														// 転がり摩擦の解決（接触法線まわりの回転暴走を抑える）
	void SolveAllIslands(float stepDt);																	// すべてのアイランドのソルバーフェーズ（PGS）
	void PositionalCorrection(float stepDt, float depthThreshold = 0.005f, float biasScale = 1.0f);		// ペネトレーションリカバリーの位置補正（Split impulse なし）
	void SplitImpulseCorrection(float stepDt);															// ペネトレーションリカバリーの位置補正（Split impulse あり）
	void GenerateSpeculativeContacts(float stepDt);														// Speculative CCD（予測接触検出）による接触の生成
	void ResolveToiEvents(float stepDt);																// Havok CCD（スイープAABBによる連続衝突検出）による接触の解決
	void PropagateIslandSleep();																		// アイランド内のスリープ状態の伝播（すべてのボディがスリープ状態ならアイランド全体をスリープ状態にする）

	// SolverContact のリストと前フレームのリスト
	std::vector<SolverContact> _solverContacts{};		// SolverContact のリスト（BuildSolverContacts で構築される）
	std::vector<SolverContact> _prevSolverContacts{};	// 前フレーム（マッチング用）

	// Island の構築と分割
	void BuildIslands();											// 接触グラフからアイランドを構築（Union-Find）
	void SplitLargeIsland(int islandIdx, int maxBodiesPerSplit);	// 大きすぎるアイランドを分割（接触グラフの連結成分を再構築）
	void BuildConstraintBatches(PhysicsIsland& island);				// アイランド内の制約ソルバーバッチを構築（グラフ彩色アルゴリズム）
	std::vector<PhysicsIsland> _islands{};							// アイランドのリスト（BuildIslands で構築される）
	std::unordered_map<PhysicsBody*, int> _bodyIslandMap{};			// PhysicsBody* → アイランドインデックスのマップ（BuildIslands で構築される）

	// Island 再利用プール: スロー再構築時、破棄せずバッファを温存
	std::vector<PhysicsIsland> _islandPool{};	// BuildIslands で破棄されるアイランドをプールして再利用する
	int AcquireIsland();						// _islands に 1 スロット追加（プール優先）
	void RecycleAllIslands();					// _islands の中身をプールへ退避し _islands を空に

	// Union-Find データ構造（BuildIslands で使用）
	std::vector<int> _ufParent{};				// Union-Find の親インデックス（BuildIslands で使用）
	std::vector<int> _ufRank{};					// Union-Find のランク（BuildIslands で使用）
	int UFFind(int x) noexcept;					// Union-Find の Find 操作（BuildIslands で使用）
	void UFUnite(int a, int b) noexcept;		// Union-Find の Unite 操作（BuildIslands で使用）

	// ※ Union-Find（Disjoint Set Union, DSU）とは、要素をグループ化し、グループの代表を効率的に管理するデータ構造です。接触グラフの連結成分を求める際に使用されます。


	// BuildIslands の変化検出キャッシュ（前フレームとグラフが同じなら再構築スキップ）
	size_t _prevIslandBodyCount = 0;			// 前フレームのアイランド内ボディ数の合計
	size_t _prevContactHash     = ~size_t(0);	// 初回は必ず再構築

	// Island 分割・バッチングの閾値（大きすぎるアイランドは分割、バッチングは小さすぎると並列化しない）
	static constexpr int kIslandSplitThreshold = 64;			// アイランド分割の閾値（接触点数がこの値を超える場合、SplitLargeIsland で分割する）
	static constexpr int kBatchingThreshold = 32;				// バッチングの閾値（接触点数がこの値を超える場合、グラフ彩色アルゴリズムでバッチを構築する）
	static constexpr int kBatchParallelThreshold = 48;			// 並列実行の閾値（バッチの接触点数がこの値を超える場合、ParallelForBarrier を使用して並列実行する）

private:
	// PhysicsController と PhysicsBody の管理
	std::atomic_bool _shuttingDown{ false };					// PhysicsManager のシャットダウン中かどうか（PhysicsController から参照される場合があるので、デストラクタではなくフラグで判定する）
	mutable std::mutex _mtx;									// PhysicsController と PhysicsBody の登録/解除の排他制御用 mutex
	std::vector<PhysicsController*> _controllers{};				// 登録されている PhysicsController のリスト
	std::vector<PhysicsBody*> _bodies{};						// 登録されている PhysicsBody のリスト

	// 重力ベクトル（デフォルトは y 軸下向き 9.8 m/s^2）
	VECTOR _gravity = VGet(0.0f, -9.8f, 0.0f);

public:
	// 重力ベクトルの設定と取得
	void SetGravity(const VECTOR& g) noexcept { _gravity = g; }	// 重力ベクトルの設定（単位は m/s^2）
	VECTOR GetGravity() const noexcept { return _gravity; }		// 重力ベクトルの取得（単位は m/s^2）

	// Adaptive solver iterations の設定（最小・最大反復回数）
	void SetAdaptiveIterationRange(int minIter, int maxIter) noexcept {
		_minSolverIterations = (minIter >= 1) ? minIter : 1;										// 最小反復回数は 1 以上
		_maxSolverIterations = (maxIter >= _minSolverIterations) ? maxIter : _minSolverIterations;	// 最大反復回数は最小反復回数以上
	}

private:
	// Adaptive solver iterations の最小・最大反復回数
	int _minSolverIterations = 4;						// 最小反復回数（1 以上）
	int _maxSolverIterations = 16;						// 最大反復回数（最小反復回数以上）
	int ComputeAdaptiveIterations() const noexcept;		// 現在のアイランドの状態に応じて反復回数を計算（BuildIslands で使用）

	// 地面平面の設定（デフォルトは y = 0 の水平面）
	bool _groundPlaneEnabled = false;						// 地面平面の有効/無効
	VECTOR _groundPlaneNormal = VGet(0.0f, 1.0f, 0.0f);		// 地面平面の法線ベクトル
	float _groundPlaneD = 0.0f;								// 地面平面の位置（n . x = d）
	bool _splitImpulseEnabled = true;						// スプリットインパルスの有効/無効
	bool _speculativeCcdEnabled = true;						// 予測型 CCD の有効/無効
	bool _havokCcdEnabled = true;							// Havok CCD の有効/無効

	// 物理シミュレーションの固定フレームレートとサブステップ数
	float _fixedDeltaTime = 1.0f / 120.0f;					// 固定フレームレートのデルタタイム（秒）
	int _maxSubSteps = 8;									// 最大サブステップ数
	int _solverIterations = 10;								// ソルバー反復回数
	float _accumulator = 0.0f;								// サブステップ用の時間蓄積

public:
	// 非同期シミュレーションの有効化/無効化
	void SetAsyncEnabled(bool enabled) noexcept { _asyncEnabled = enabled; }	// true なら非同期シミュレーションを有効化（Update() 内で別スレッドで物理シミュレーションを実行する）
	bool IsAsyncEnabled() const noexcept { return _asyncEnabled; }				// 非同期シミュレーションの有効化/無効化の取得
	void WaitForPhysics();														// 非同期シミュレーションの完了を待機（Update() 内で呼ばれる）

	// ※ 非同期シミュレーションとは、物理シミュレーションをメインスレッドとは別のスレッドで実行することで、フレームレートの安定化やパフォーマンス向上を図る手法です。

private:
	// 非同期シミュレーションの状態
	bool _asyncEnabled = false;								// true なら非同期シミュレーションを有効化
	std::future<void> _asyncFuture{};						// 非同期シミュレーションの結果を保持する future
	void RunAsyncStep(float dt, int maxSubSteps);			// 非同期シミュレーションのステップ実行（別スレッドで呼ばれる）


	// 永久バッファ（毎フレームの heap 確保を排除）
	
	// 前フレームの接触情報の照合用
	struct PrevKey {
		Collider* a; Collider* b;															// 前フレームの接触情報の照合用キー（Collider* の組み合わせ）
		bool operator==(const PrevKey& o) const noexcept { return a == o.a && b == o.b; }	// 等価比較
	};
	// PrevKey のハッシュ関数
	struct PrevHash {
		// Collider* の組み合わせをハッシュ化する（前フレームの接触情報の照合用）
		size_t operator()(const PrevKey& k) const noexcept {					
			return (reinterpret_cast<size_t>(k.a) >> 4) ^ (reinterpret_cast<size_t>(k.b) << 1);
		}
	};
	// 前フレームの接触情報の照合用マップ（PrevKey → SolverContact のインデックス）
	struct TaggedContact { SolverContact sc; bool valid = false; };
	// TOI（Time of Impact）イベント情報
	struct ToiEvent {
		PhysicsBody* body = nullptr;	// TOI イベントの対象ボディ
		float toi = 1.0f;				// TOI（Time of Impact）値（0.0f から 1.0f の範囲）
		VECTOR toiPosition{};			// TOI 時点の位置
		VECTOR clampedVelocity{};		// TOI 時点の速度（clamped）
	};

	// ※ 永続バッファとは、毎フレームの heap 確保を排除するために、PhysicsManager 内で保持される一時的なデータ構造です
	// これにより、パフォーマンスの向上とメモリ断片化の防止が期待できます。
	// ※ ToI（Time of Impact）とは、連続衝突検出において、物体が衝突するまでの時間を表す値です
	// 0.0f は衝突直前、1.0f は衝突後を意味します。


	// 永続バッファ
	std::vector<PhysicsController*>                       _ctrlSnapshotBuf;			// PhysicsController のスナップショット（Update() 内で使用）
	std::unordered_multimap<PrevKey, size_t, PrevHash>    _prevContactMapBuf;		// 前フレームの接触情報の照合用マップ（PrevKey → SolverContact のインデックス）
	std::vector<TaggedContact>                            _buildContactResultsBuf;	// BuildSolverContacts で構築される接触情報の一時バッファ
	std::vector<size_t>                                   _islandOrderBuf;			// BuildIslands で構築されるアイランドの順序バッファ
	std::unordered_map<PhysicsBody*, int>                 _bodyIndexBuf;			// BuildIslands で構築される PhysicsBody* → アイランドインデックスのマップ
	std::unordered_map<int, int>                          _rootToIslandBuf;			// BuildIslands で構築される Union-Find のルートインデックス → アイランドインデックスのマップ
	std::vector<VECTOR>                                   _pseudoVelBuf;			// BuildIslands で構築される疑似速度バッファ（スリープ状態の伝播に使用）
	std::vector<VECTOR>                                   _posCorrectionBuf;		// PositionalCorrection で使用される位置補正バッファ
	std::unordered_map<PhysicsBody*, size_t>              _bodyIdxBuf;				// ResolveToiEvents で使用される PhysicsBody* → インデックスのマップ
	std::vector<ToiEvent>                                 _toiEventsBuf;			// ResolveToiEvents で使用される TOI イベントバッファ
	std::unordered_set<PhysicsBody*>                      _specCoveredBodiesBuf;	// GenerateSpeculativeContacts で使用される Speculative CCD によってカバーされたボディの集合
	std::vector<SolverContact>                            _specContactsBuf;			// GenerateSpeculativeContacts で使用される Speculative CCD によって生成された接触情報のバッファ

	//  SplitLargeIsland 永続バッファ（毎呼出の heap 確保を排除）
	std::unordered_map<PhysicsBody*, int> _splitLocalIdxBuf;						// SplitLargeIsland で使用される PhysicsBody* → ローカルインデックスのマップ
	std::vector<std::vector<int>>         _splitAdjBuf;								// SplitLargeIsland で使用される隣接リストバッファ
	std::vector<int>                      _splitColorBuf;							// SplitLargeIsland で使用される色バッファ
	std::vector<int>                      _splitQueueBuf;							// SplitLargeIsland で使用されるキューバッファ
	std::vector<PhysicsBody*>             _splitOrigBodiesBuf;						// SplitLargeIsland で使用される元のボディバッファ
	std::vector<int>                      _splitOrigContactsBuf;					// SplitLargeIsland で使用される元の接触情報バッファ

	//  BuildConstraintBatches 永続バッファ（毎呼出の heap 確保を排除）
	std::unordered_map<PhysicsBody*, std::vector<int>> _batchBodyToContactsBuf;		// BuildConstraintBatches で使用される PhysicsBody* → 接触情報インデックスのマップ
	std::vector<int>                                   _batchContactColorBuf;		// BuildConstraintBatches で使用される接触情報の色バッファ
	std::vector<int>                                   _batchUsedColorEpochBuf;		// BuildConstraintBatches で使用される接触情報の色の使用エポックバッファ

	// BuildIslands の変化検出キャッシュ（前フレームとグラフが同じなら再構築スキップ）
	size_t _prevIslandCount = 0;													// 前フレームのアイランド数
};
