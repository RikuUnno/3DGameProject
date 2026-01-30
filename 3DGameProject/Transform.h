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

	// ---- local TRS ----
	const VECTOR& LocalPosition() const noexcept { return _localPosition; }
	const VECTOR& LocalEulerRad() const noexcept { return _localEulerRad; }
	const VECTOR& LocalScale() const noexcept { return _localScale; }

	void SetLocalPosition(const VECTOR& p) noexcept;
	void SetLocalEulerRad(const VECTOR& eulerRad) noexcept;
	void SetLocalScale(const VECTOR& s) noexcept;

	// ---- hierarchy ----
	Transform* Parent() const noexcept { return _parent; }
	void SetParent(Transform* parent) noexcept;

	// ---- matrices ----
	const MATRIX& LocalMatrix() const;
	const MATRIX& WorldMatrix() const;

	VECTOR WorldPosition() const;

	void MarkDirty() noexcept;

private:
	static MATRIX MakeScale(const VECTOR& s);
	static MATRIX MakeTranslation(const VECTOR& p);
	static MATRIX Mul(const MATRIX& a, const MATRIX& b);

	// local TRS
	VECTOR _localPosition{};
	VECTOR _localEulerRad{}; // pitch(x), yaw(y), roll(z)
	VECTOR _localScale{};

	Transform* _parent = nullptr;

	mutable bool _localDirty = true;
	mutable bool _worldDirty = true;
	mutable MATRIX _localMatrix{};
	mutable MATRIX _worldMatrix{};
};
