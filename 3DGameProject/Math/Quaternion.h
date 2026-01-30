#pragma once

// Euler（見やすい）を主に扱いつつ、内部計算に Quaternion を使えるようにするための最小クラス。
// - DxLib の数値型（VECTOR/MATRIX）と併用する。
// - 今後の補間(Slerp等)追加を見越して、実装は cpp に分離する。

#include "DxLib.h"

class Quaternion {
public:
	// 成分
	float x =0.0f; // i
	float y =0.0f; // j
	float z =0.0f; // k
	float w =1.0f; //1

	// コンストラクタ
	Quaternion() = default;
	Quaternion(float _x, float _y, float _z, float _w) noexcept;

	static Quaternion Identity() noexcept;

	// オイラー角（ラジアン）からクォータニオンを生成
	// pitch: X, yaw: Y, roll: Z
	// 回転順序: Z(roll) * Y(yaw) * X(pitch)
	static Quaternion FromEulerRad(float pitch, float yaw, float roll) noexcept;

	// 正規化クォータニオンを取得
	Quaternion Normalized() const noexcept;

	// 回転行列を取得
	MATRIX ToRotationMatrix() const noexcept;
};
