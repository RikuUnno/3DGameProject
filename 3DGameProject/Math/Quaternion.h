#pragma once

// Euler（見やすい）を主に扱いつつ、内部計算に Quaternion を使えるようにするための最小クラス。
// - DxLib の数値型（VECTOR/MATRIX）と併用する。
// - 補間(Slerp等)や合成(乗算)など、3Dで必要になりやすい演算を提供する。

#include "DxLib.h"

class Quaternion {
public:
	// 成分
	float x = 0.0f; // i
	float y = 0.0f; // j
	float z = 0.0f; // k
	float w = 1.0f; //1

	// コンストラクタ
	Quaternion() = default;
	Quaternion(float _x, float _y, float _z, float _w) noexcept;

	static Quaternion Identity() noexcept;

	// --- 基本演算 ---
	float LengthSq() const noexcept;			// 二乗長さ
	float Length() const noexcept;				// 長さ
	Quaternion Normalized() const noexcept;		// 正規化
	Quaternion Conjugate() const noexcept;		// 共役
	Quaternion Inverse() const noexcept;		// 逆元

	// 内積、乗算
	static float Dot(const Quaternion& a, const Quaternion& b) noexcept;			// 内積
	static Quaternion Multiply(const Quaternion& a, const Quaternion& b) noexcept;  // 乗算 a*b

	Quaternion operator*(const Quaternion& rhs) const noexcept { return Multiply(*this, rhs); }

	// --- creation ---
	// オイラー角（ラジアン）からクォータニオンを生成
	// pitch: X, yaw: Y, roll: Z
	// 回転順序: Z(roll) * Y(yaw) * X(pitch)
	static Quaternion FromEulerRad(float pitch, float yaw, float roll) noexcept;

	// 軸回転（axisは正規化されている前提。未正規化でも内部で正規化する）
	static Quaternion FromAxisAngleRad(const VECTOR& axis, float angleRad) noexcept;

	// --- interpolation ---
	// 正規化線形補間（高速、tが小さい/誤差許容の用途）
	static Quaternion Nlerp(const Quaternion& a, const Quaternion& b, float t) noexcept;
	// 球面線形補間（回転として自然）
	static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t) noexcept;

	// --- conversion ---
	MATRIX ToRotationMatrix() const noexcept;

	// --- utility ---
	// ベクトルを回転させる（q * v * q^-1）
	VECTOR RotateVector(const VECTOR& v) const noexcept;

	//代表的な用途（デバッグ表示・Euler入力との橋渡し）
	// - FromEulerRad と同じ回転順序（Z*Y*X）でEulerを返す
	// - Euler表現は一意でないため、値が跳ぶ可能性がある
	VECTOR ToEulerRad() const noexcept;
};
