#pragma once

#include "DxLib.h"

class GameObject;

// PhysicsBody
// - Collider のように GameObjectへ付与するコンポーネント（データ保持）
// -付けたい時だけ AddPhysicsBody()で生成し、要らない時は RemovePhysicsBody()で外す想定
class PhysicsBody {
public:
	PhysicsBody() = default;
	virtual ~PhysicsBody() = default;

	// 所有者（Transform を動かすため）
	GameObject* _owner = nullptr;

	//物理挙動の有効/スリープ（プール待機中・非アクティブ中は false にする想定）
	bool _enabled = true; 		//物理挙動の有効/スリープ
	bool _useGravity = true;	// 重力の影響を受けるか
	bool _isKinematic = false;	// true: 外部でTransform更新するだけ

	float _mass =1.0f;			// 質量（最低限）
	float _linearDamping =0.0f;	// 線形減衰（最低限）
	float _restitution =0.0f;	//反発（最低限）
	float _friction =0.5f;		// 摩擦（最低限）

	VECTOR _velocity = VGet(0,0,0);			//速度
	VECTOR _angularVelocity = VGet(0,0,0);	//角速度

public:
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept { _enabled = enabled; }

	//使い回し想定のリセット（Poolで OnAcquire/OnRelease を実装する時に便利）
	void Reset() noexcept {
		_enabled = true;
		_useGravity = true;
		_isKinematic = false;
		_mass =1.0f;
		_linearDamping =0.0f;
		_restitution =0.0f;
		_friction =0.5f;
		_velocity = VGet(0,0,0);
		_angularVelocity = VGet(0,0,0);
	}
};
