#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"
#include "PhysicsMaterial.h"

class GameObject;
class Collider;

// PhysicsBody
// - GameObject に付与する物理コンポーネント（データ保持）
// - 慣性テンソル（対角近似）を保持し、回転の物理を正しく計算する

// CCD品質レベル: 0=なし、1=低、2=標準、3=高、4=最高
enum class CcdQuality : int {
	Discrete = 0,       // CCD無効: 衝突判定はフレームごとのAABBのみ（高速だが高速移動でトンネルする可能性あり）
	Debris = 1,         // 低品質CCD: 速度クランプのみ（高速移動でトンネルする可能性あり）
	Default = 2,        // 標準CCD: 速度クランプ + TOIバックステップ（高速移動でのトンネルを大幅に減らすが、完全には防止できない）
	Bullet = 3,         // 高品質CCD: 速度クランプ + TOIバックステップ + スイープテスト（高速移動でのトンネルをほぼ完全に防止するが、コストが高い）
	Critical = 4,       // 最高品質CCD: 速度クランプ + TOIバックステップ + スイープテスト + ペネトレーションリカバリー（高速移動でのトンネルを完全に防止し、ペネトレーションも自動修正するが、非常にコストが高い）
};

// 物理マテリアル
class PhysicsBody {
public:
	// デフォルトコンストラクタとデストラクタ
    PhysicsBody() = default;
    virtual ~PhysicsBody() = default;

	// 所有者（Transform参照/Active判定用）
    GameObject* _owner = nullptr;

	// 物理挙動設定
	bool _enabled = true;           // 物理挙動の有効/無効（非アクティブ・プール待機中は false にする想定）
	bool _useGravity = true;        // 重力の影響を受けるか
	bool _isKinematic = false;      // true のときは物理挙動なし、移動は MovePosition/MoveRotation で行う。衝突はするが押し戻されない。
	bool _freezeRotation = false;   // true のときは回転しない（慣性テンソルを無限大にして回転の物理を無効化）。移動は通常通り物理挙動。
	bool _useInterpolation = false; // サブステップ補間の有効化（高速移動での見た目のカクつきを減らす。コストが上がる）
	bool _detectContinuous = false; // 連続衝突検出の有効化（高速移動でのトンネルを減らす。コストが上がる）
	bool _isSleeping = false;       // スリープ状態か（物理挙動停止中。WakeUp() で解除）

	// CCD品質レベル
    CcdQuality _ccdQuality = CcdQuality::Default;

	// ペネトレーションリカバリーの許容深度（単位: ワールド距離）。
    // これを超えるペネトレーションが発生した場合、物理エンジンは自動的に修正を試みる。0の場合はリカバリーなし。
    float _allowedPenetrationDepth = 0.0f;

	// --- 物理パラメータ ---
	float _mass = 1.0f;                         // 質量（kg）。0以下なら質量を計算しない（無限大扱い）。コライダー体積から自動計算する場合は ApplyMaterial() を呼ぶ。
	float _inverseMass = 1.0f;                  // 質量の逆数（kg^-1）。0以下なら質量を計算しない（無限大扱い）。ApplyMaterial() で mass から自動計算されるが、mass と inverseMass の両方を明示的に設定してもよい。
	float _linearDamping = 0.0f;                // 線形減衰（空気抵抗的な減速）
	float _angularDamping = 0.05f;              // 角速度減衰
	float _restitution = 0.0f;                  // 反発係数（0=完全非弾性、1=完全弾性）
	float _friction = 0.5f;                     // 動摩擦係数（0=無摩擦、1=非常に高い摩擦）
	float _gravityScale = 1.0f;                 // 重力の影響の大きさ（1.0fが通常の重力、0で重力なし、2で倍の重力など）
    float _sleepLinearThreshold = 0.02f;       // スリープ判定用の線形速度閾値
    float _sleepAngularThreshold = 0.02f;      // スリープ判定用の角速度閾値
    float _sleepTimeThreshold = 0.3f;          // スリープ判定用の時間閾値
    float _maxLinearSpeed = 100.0f;            // 最大線形速度
    float _maxAngularSpeed = 20.0f;            // 最大角速度

	// --- 物理状態 ---
	VECTOR _velocity = VGet(0, 0, 0);                           // 線形速度（ワールド）
	VECTOR _angularVelocity = VGet(0, 0, 0);                    // 角速度（ワールド、ラジアン/秒）
	VECTOR _force = VGet(0, 0, 0);                              // 外力（ワールド、フレームごとにリセットされる想定）
	VECTOR _torque = VGet(0, 0, 0);                             // 外部トルク（ワールド、フレームごとにリセットされる想定）
	VECTOR _movePositionTarget = VGet(0, 0, 0);                 // MovePosition で目標とする位置（ワールド）。MovePosition を呼ぶと設定され、物理ステップでリセットされる想定。
	Quaternion _moveRotationTarget = Quaternion::Identity();    // MoveRotation で目標とする回転（ワールド）。MoveRotation を呼ぶと設定され、物理ステップでリセットされる想定。
    VECTOR _previousPosition = VGet(0, 0, 0);                   // 前フレームの位置（ワールド）
    Quaternion _previousRotation = Quaternion::Identity();      // 前フレームの回転（ワールド）

	// サブステップ補間用の状態（物理ステップ中に更新される）
	VECTOR _interpPosition = VGet(0, 0, 0);                 // 補間された位置（ワールド）。物理ステップ中に Update() で更新され、描画フレームで使用される想定。
	Quaternion _interpRotation = Quaternion::Identity();    // 補間された回転（ワールド）。物理ステップ中に Update() で更新され、描画フレームで使用される想定。

	// スリープ管理用の状態
	float _sleepTimer = 0.0f;               // 低エネルギー状態が続いた時間（秒）。UpdateSleepState() で更新され、スリープ判定に使用される想定。
	bool _hasMovePositionTarget = false;    // MovePosition で位置目標が設定されているか（物理ステップ中にリセットされる想定）
	bool _hasMoveRotationTarget = false;    // MoveRotation で回転目標が設定されているか（物理ステップ中にリセットされる想定）

	// 慣性エネルギーの指数移動平均（EMA）。UpdateSleepState() で更新され、スリープ判定に使用される想定。
    float _kineticEnergyEMA = 0.0f;

	// 慣性テンソルの逆数（対角近似）。単位は kg^-1 * m^-2。回転の物理を正しく計算するために必要。コライダー体積から自動計算する場合は ApplyMaterial() を呼ぶ。
    float _inverseInertiaLocal[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };

	// 物理マテリアル（friction/restitution/damping を一括管理）。ApplyMaterial() で設定されるが、個別に friction/restitution/damping を設定してもよい。
    PhysicsMaterial _material{};

public:
	// マテリアルの適用
    void ApplyMaterial(const PhysicsMaterial& mat, Collider* collider = nullptr) noexcept;

    // マテリアルの取得
    const PhysicsMaterial& GetMaterial() const noexcept { return _material; }

public:
	// 物理挙動の有効/無効。無効にすると物理挙動が停止する（非アクティブ・プール待機中は false にする想定）。
    bool IsEnabled() const noexcept { return _enabled; }    // 物理挙動が有効か（非アクティブ・プール待機中は false にする想定）
	void SetEnabled(bool enabled) noexcept {                // 物理挙動の有効/無効を設定。無効にすると物理挙動が停止する（非アクティブ・プール待機中は false にする想定）。有効にしたときは WakeUp() してスリープ解除する。
        _enabled = enabled;
        if (enabled) WakeUp();
    }

	// その他のフラグのセッター/ゲッター
	bool IsSleeping() const noexcept { return _isSleeping; }                                                // スリープ状態か
	bool IsDynamic() const noexcept { return _enabled && !_isKinematic && _inverseMass > 0.0f; }            // 動的（物理挙動する）か
	float InverseMass() const noexcept { return (_isKinematic || _mass <= 1e-6f) ? 0.0f : _inverseMass; }   // 質量の逆数（0以下なら質量を計算しない＝無限大扱い）。Kinematic または質量がほぼゼロの場合は 0 を返す。

	// 慣性テンソルの逆数（対角近似）。単位は kg^-1 * m^-2。回転の物理を正しく計算するために必要。Kinematic または質量がほぼゼロ、または回転凍結の場合はゼロベクトルを返す。
    VECTOR InverseInertiaDiag() const noexcept {
        if (_isKinematic || _mass <= 1e-6f || _freezeRotation) return VGet(0, 0, 0);
        return VGet(_inverseInertiaLocal[0], _inverseInertiaLocal[4], _inverseInertiaLocal[8]);
    }

	// 慣性テンソルの逆数（対角近似）を3x3行列形式で返す。回転の物理を正しく計算するために必要。Kinematic または質量がほぼゼロ、または回転凍結の場合はゼロ行列を返す。
    const float* InverseInertiaLocal3x3() const noexcept {
        static const float zero[9] = {};
        if (_isKinematic || _mass <= 1e-6f || _freezeRotation) return zero;
        return _inverseInertiaLocal;
    }

    // ベクトルに慣性テンソルの逆数を適用して、角速度変化に変換する。回転の物理を正しく計算するために必要。Kinematic または質量がほぼゼロ、または回転凍結の場合はゼロベクトルを返す。
    VECTOR ApplyInverseInertia(const VECTOR& v) const noexcept;

	// 慣性テンソルの逆数の平均値を返す。
    // スリープ判定などで慣性エネルギーを計算するときに、単純な質量ベースの閾値ではなく、回転の慣性も考慮したい場合に使用する。Kinematic または質量がほぼゼロ、または回転凍結の場合は 0 を返す。
    float AverageInverseInertia() const noexcept {
        const VECTOR ii = InverseInertiaDiag();
        return (ii.x + ii.y + ii.z) / 3.0f;
    }

	// 質量の設定。
    // 質量がほぼゼロの場合は 0 として扱い、慣性テンソルも無限大（逆数ゼロ）として扱う。質量を変更したときは、Kinematic フラグや慣性テンソルも適切に更新する。
    void SetMass(float mass) noexcept {
        _mass = (mass > 1e-6f) ? mass : 0.0f;
        _inverseMass = (_mass > 1e-6f) ? (1.0f / _mass) : 0.0f;
        if (_mass <= 1e-6f) {
            _isKinematic = true;
            for (int i = 0; i < 9; ++i) _inverseInertiaLocal[i] = 0.0f;
        }
    }

	// コライダーから慣性テンソルを計算して設定する。
    // Collider の形状とサイズに基づいて、慣性テンソルの逆数を対角近似で計算し、_inverseInertiaLocal に設定する。
    // 質量も同時に設定する場合は、ApplyMaterial() を呼ぶ。
    void ComputeInertia(Collider* collider) noexcept;

	// スリープ状態の管理
    void WakeUp() noexcept {
		_isSleeping = false;                                                        // スリープ解除
		_sleepTimer = 0.0f;                                                         // スリープタイマーリセット
		_kineticEnergyEMA = 4.0f * _sleepLinearThreshold * _sleepLinearThreshold;   // スリープ判定用の慣性エネルギー閾値を初期化（線形速度の閾値を基準に設定）。UpdateSleepState() で更新される想定。
    }

	// スリープ状態にする。スリープ状態になると物理挙動が停止する。WakeUp() で解除する。
    void Sleep() noexcept {
		_isSleeping = true;                 // スリープ状態にする
		_sleepTimer = 0.0f;                 // スリープタイマーリセット
		_velocity = VGet(0, 0, 0);          // 速度リセット
		_angularVelocity = VGet(0, 0, 0);   // 角速度リセット
		_force = VGet(0, 0, 0);             // 力リセット
		_torque = VGet(0, 0, 0);            // トルクリセット
		_kineticEnergyEMA = 0.0f;           // 慣性エネルギーEMAリセット
    }

	// 力とトルクの加算
    void ClearAccumulators() noexcept {
        _force = VGet(0, 0, 0);             // 力リセット
        _torque = VGet(0, 0, 0);            // トルクリセット
    }

	// 外力を加える。
    // 加えた力がゼロでない場合は WakeUp() してスリープ解除する。
    void AddForce(const VECTOR& force) noexcept {
        _force = VAdd(_force, force);
        if (force.x != 0.0f || force.y != 0.0f || force.z != 0.0f) WakeUp();
    }

	// 加速度を加える。
    // 内部で質量を考慮して力に変換する。
    void AddAcceleration(const VECTOR& acceleration) noexcept {
        if (InverseMass() <= 0.0f) return;
        AddForce(VScale(acceleration, _mass));
    }

	// 外部トルクを加える。
    void AddImpulse(const VECTOR& impulse) noexcept {
        const float invM = InverseMass();
        if (invM <= 0.0f) return;
        _velocity = VAdd(_velocity, VScale(impulse, invM));
        WakeUp();
    }

	// 加速度インパルスを加える。
    void AddVelocityChange(const VECTOR& deltaVelocity) noexcept {
        _velocity = VAdd(_velocity, deltaVelocity);
        WakeUp();
    }

	// 外部トルクを加える。
    void AddTorque(const VECTOR& torque) noexcept {
        _torque = VAdd(_torque, torque);
        if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f) WakeUp();
    }

	// 角速度変化インパルスを加える。
    void AddAngularImpulse(const VECTOR& angularImpulse) noexcept {
        if (_freezeRotation) return;
        _angularVelocity = VAdd(_angularVelocity, ApplyInverseInertia(angularImpulse));
        WakeUp();
    }

	// MovePosition と MoveRotation は、Kinematic な物体の移動に使用する。
    // これらを呼ぶと、物理ステップで目標位置/回転に向かって移動するようになる。
    // 物理ステップの最後で、目標位置/回転に到達したとみなしてリセットされる想定。
    void MovePosition(const VECTOR& targetPosition) noexcept {
        _movePositionTarget = targetPosition;
		_hasMovePositionTarget = true;   // 目標位置は保存
        WakeUp();
    }

	// MoveRotation は、Kinematic な物体の回転移動に使用する。
    void MoveRotation(const Quaternion& targetRotation) noexcept {
        _moveRotationTarget = targetRotation.Normalized();
		_hasMoveRotationTarget = true;   // 目標回転は正規化して保存
        WakeUp();
    }

	// 物理状態のリセット。主にオブジェクトプールで再利用するときに使用する。
    void Reset() noexcept {
        _enabled = true;                     // 有効化
        _useGravity = true;                  // 重力使用
        _isKinematic = false;                // Kinematic 無効化
        _freezeRotation = false;             // 回転固定無効化
        _useInterpolation = false;           // 補間無効化
        _detectContinuous = false;           // 連続衝突検出無効化
		_isSleeping = false;                 // スリープ解除
		_ccdQuality = CcdQuality::Default;   // CCD品質レベルのデフォルト値
		_allowedPenetrationDepth = 0.0f;     // 物理パラメータのデフォルト値
		_mass = 1.0f;                        // 質量のデフォルト値
		_inverseMass = 1.0f;                 // 質量の逆数のデフォルト値
		_linearDamping = 0.0f;               // 線形減衰のデフォルト値
		_angularDamping = 0.05f;             // 角速度減衰のデフォルト値
		_restitution = 0.0f;                 // 反発係数のデフォルト値
		_friction = 0.5f;                    // 動摩擦係数のデフォルト値
		_gravityScale = 1.0f;                // 重力スケールのデフォルト値
		_sleepLinearThreshold = 0.05f;       // スリープ判定用の線形速度閾値のデフォルト値
		_sleepAngularThreshold = 0.05f;      // スリープ判定用の角速度閾値のデフォルト値
		_sleepTimeThreshold = 0.5f;          // スリープ判定用の時間閾値のデフォルト値
		_maxLinearSpeed = 100.0f;            // 最大線形速度のデフォルト値
		_maxAngularSpeed = 20.0f;            // 最大角速度のデフォルト値
		_velocity = VGet(0, 0, 0);           // 速度リセット
		_angularVelocity = VGet(0, 0, 0);    // 角速度リセット
		_force = VGet(0, 0, 0);              // 力リセット
		_torque = VGet(0, 0, 0);             // トルクリセット
		_movePositionTarget = VGet(0, 0, 0);            // 目標位置は保存 
        _moveRotationTarget = Quaternion::Identity();   // 目標回転は正規化して保存
		_previousPosition = VGet(0, 0, 0);              // 前フレームの位置リセット
		_previousRotation = Quaternion::Identity();     // 前フレームの回転リセット
		_interpPosition = VGet(0, 0, 0);                // 補間位置リセット
		_interpRotation = Quaternion::Identity();       // 補間回転リセット
		_sleepTimer = 0.0f;                  // スリープタイマーリセット
		_hasMovePositionTarget = false;      // 目標位置は保存
		_hasMoveRotationTarget = false;      // 目標回転は保存
		_kineticEnergyEMA = 0.0f;            // 慣性エネルギーEMAリセット
		for (int i = 0; i < 9; ++i) _inverseInertiaLocal[i] = (i % 4 == 0) ? 1.0f : 0.0f;   // 慣性テンソルの逆数は単位行列（対角近似）にリセット
		_material = PhysicsMaterial{};       // 物理マテリアルリセット
    }
};
