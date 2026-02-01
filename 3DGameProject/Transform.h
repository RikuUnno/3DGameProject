#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"

// Transform
// - 内部の回転は Quaternion を正とする（補間・合成が安定する）
// - Euler（ラジアン）は「入力/デバッグ表示用」として提供する
// ※ Eulerは表現が一意でないため、表示値が跳ぶ可能性がある
// - 親子関係を見越して local/world を分け、dirtyでキャッシュ
class Transform {
public:
	Transform();

	// ローカル変換成分の取得・設定
	const VECTOR& LocalPosition() const noexcept { return _localPosition; }

	// Euler は入出力用（内部は Quaternion）
	VECTOR LocalEulerRad() const noexcept; //その時点の回転をEulerで取得（表示用）
	void SetLocalEulerRad(const VECTOR& eulerRad) noexcept; // Euler入力 -> Quaternionへ反映

	// Quaternion を直接扱う（内部表現）
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

	// --- direction helpers (local axes in world space) ---
	VECTOR Forward() const noexcept;
	VECTOR Right() const noexcept;
	VECTOR Up() const noexcept;

	// 状態を dirty にする
	void MarkDirty() noexcept;

private:
	VECTOR _localPosition{};
	Quaternion _localRotation{}; // 回転（内部表現）
	VECTOR _localScale{};

	Transform* _parent = nullptr;

	mutable bool _localDirty = true;
	mutable bool _worldDirty = true;
	mutable MATRIX _localMatrix{};
	mutable MATRIX _worldMatrix{};
};
