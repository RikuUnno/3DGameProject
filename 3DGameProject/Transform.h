#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"
#include <vector>

class GameObject;

// Transform
// - 回転の保持は Quaternion を正とする（姿勢・補間に有利）
// - Euler（ラジアン）は「編集/デバッグ表示用」として提供する
// ※ Eulerは表現の一意性がないため、表示値が変化する可能性がある
// - 親子関係を持ち、local/world を分離して dirty でキャッシュ
class Transform {
public:
	Transform();
	~Transform();

	void SetOwner(GameObject* owner) noexcept { _owner = owner; }
	GameObject* Owner() const noexcept { return _owner; }

	// ローカル変換要素の取得・設定
	const VECTOR& LocalPosition() const noexcept { return _localPosition; }

	// Euler は入出力用（内部は Quaternion）
	VECTOR LocalEulerRad() const noexcept; // 現時点の回転をEulerで取得（表示用）
	void SetLocalEulerRad(const VECTOR& eulerRad) noexcept; // Euler入力 -> Quaternionへ反映

	// Quaternion を直接設定（内部表現）
	const Quaternion& LocalRotation() const noexcept { return _localRotation; }
	void SetLocalRotation(const Quaternion& q) noexcept;

	const VECTOR& LocalScale() const noexcept { return _localScale; }

	void SetLocalPosition(const VECTOR& p) noexcept;
	void SetLocalScale(const VECTOR& s) noexcept;

	// 親子関係
	Transform* Parent() const noexcept { return _parent; }
	void SetParent(Transform* parent) noexcept;

	// 行列
	const MATRIX& LocalMatrix() const;
	const MATRIX& WorldMatrix() const;

	VECTOR WorldPosition() const;
	VECTOR WorldScale() const noexcept;           // 親スケールを含んだワールドスケール
	VECTOR TransformPoint(const VECTOR& localPoint) const noexcept;   // ローカル点を親込みでワールドへ変換
	VECTOR TransformVector(const VECTOR& localVector) const noexcept; // ローカル方向ベクトルを親込みでワールドへ変換

	// --- direction helpers (local axes in world space) ---
	VECTOR Forward() const noexcept;
	VECTOR Right() const noexcept;
	VECTOR Up() const noexcept;

	// 状態を dirty にする
	void MarkDirty() noexcept;

private:
	void AddChild(Transform* child) noexcept;
	void RemoveChild(Transform* child) noexcept;
	void PropagateDirtyToChildren() noexcept;

	GameObject* _owner = nullptr;
	VECTOR _localPosition{};
	Quaternion _localRotation{}; // 回転（内部表現）
	VECTOR _localScale{};

	Transform* _parent = nullptr;
	std::vector<Transform*> _children;

	mutable bool _localDirty = true;
	mutable bool _worldDirty = true;
	mutable MATRIX _localMatrix{};
	mutable MATRIX _worldMatrix{};
};
