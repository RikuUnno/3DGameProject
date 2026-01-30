#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"

// Transform
// - 見やすさ重視で Euler（ラジアン）を保持
// - 行列生成には Quaternion を使用（ジンバル問題を緩和しやすくするための足場）
// - 親子関係を見越して local/world を分け、dirtyでキャッシュ
class Transform {
public:
	Transform();

	// ローカル変換成分の取得・設定
	const VECTOR& LocalPosition() const noexcept { return _localPosition; }
	const VECTOR& LocalEulerRad() const noexcept { return _localEulerRad; }
	const VECTOR& LocalScale() const noexcept { return _localScale; }

	void SetLocalPosition(const VECTOR& p) noexcept;
	void SetLocalEulerRad(const VECTOR& eulerRad) noexcept;
	void SetLocalScale(const VECTOR& s) noexcept;

	// 親子関係
	Transform* Parent() const noexcept { return _parent; }
	void SetParent(Transform* parent) noexcept;

	// 行列
	const MATRIX& LocalMatrix() const;
	const MATRIX& WorldMatrix() const;

	// ワールド変換成分の取得
	VECTOR WorldPosition() const;

	// 状態を dirty にする
	void MarkDirty() noexcept;

private:
	// ローカル変換成分
	VECTOR _localPosition{};
	VECTOR _localEulerRad{}; // pitch(x), yaw(y), roll(z)
	VECTOR _localScale{};

	// 親子関係
	Transform* _parent = nullptr;

	// キャッシュ
	mutable bool _localDirty = true;	// ローカル行列が最新でない
	mutable bool _worldDirty = true;	// ワールド行列が最新でない
	mutable MATRIX _localMatrix{};		// ローカル行列キャッシュ
	mutable MATRIX _worldMatrix{};		// ワールド行列キャッシュ
};
