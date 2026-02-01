#include "Quaternion.h"

#include <cmath>
#include <algorithm>

Quaternion::Quaternion(float _x, float _y, float _z, float _w) noexcept
	: x(_x), y(_y), z(_z), w(_w) {
}

Quaternion Quaternion::Identity() noexcept {
	return Quaternion{};
}

float Quaternion::LengthSq() const noexcept {
	return x * x + y * y + z * z + w * w;
}

float Quaternion::Length() const noexcept {
	return std::sqrt(LengthSq());
}

float Quaternion::Dot(const Quaternion& a, const Quaternion& b) noexcept {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

Quaternion Quaternion::Conjugate() const noexcept {
	return Quaternion{ -x, -y, -z, w };
}

Quaternion Quaternion::Inverse() const noexcept {
	const float len2 = LengthSq();
	if (len2 <= 0.0f) return Identity();
	const float inv = 1.0f / len2;
	const Quaternion c = Conjugate();
	return Quaternion{ c.x * inv, c.y * inv, c.z * inv, c.w * inv };
}

Quaternion Quaternion::Normalized() const noexcept {
	const float len2 = LengthSq();
	if (len2 <= 0.0f) return Identity();
	const float inv = 1.0f / std::sqrt(len2);
	return Quaternion{ x * inv, y * inv, z * inv, w * inv };
}

Quaternion Quaternion::Multiply(const Quaternion& a, const Quaternion& b) noexcept {
	// Hamilton product
	return Quaternion{
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
	};
}

Quaternion Quaternion::FromEulerRad(float pitch, float yaw, float roll) noexcept {
	const float hx = pitch * 0.5f;
	const float hy = yaw * 0.5f;
	const float hz = roll * 0.5f;

	const float cx = std::cos(hx);
	const float sx = std::sin(hx);
	const float cy = std::cos(hy);
	const float sy = std::sin(hy);
	const float cz = std::cos(hz);
	const float sz = std::sin(hz);

	Quaternion q;
	// 回転順序: Z(roll) * Y(yaw) * X(pitch)
	q.w = cz * cy * cx + sz * sy * sx;
	q.x = cz * cy * sx - sz * sy * cx;
	q.y = cz * sy * cx + sz * cy * sx;
	q.z = sz * cy * cx - cz * sy * sx;
	return q.Normalized();
}

Quaternion Quaternion::FromAxisAngleRad(const VECTOR& axis, float angleRad) noexcept {
	VECTOR n = axis;
	const float len = VSize(n);
	if (len <= 0.0f) return Identity();
	n = VScale(n, 1.0f / len);

	const float half = angleRad * 0.5f;
	const float s = std::sin(half);
	const float c = std::cos(half);
	return Quaternion{ n.x * s, n.y * s, n.z * s, c }.Normalized();
}

Quaternion Quaternion::Nlerp(const Quaternion& a, const Quaternion& b, float t) noexcept {
	const float tt = std::clamp(t, 0.0f, 1.0f);
	Quaternion bb = b;
	//反対側を選ばないようにする
	if (Dot(a, b) < 0.0f) {
		bb.x = -bb.x;
		bb.y = -bb.y;
		bb.z = -bb.z;
		bb.w = -bb.w;
	}
	Quaternion r{
		a.x + (bb.x - a.x) * tt,
		a.y + (bb.y - a.y) * tt,
		a.z + (bb.z - a.z) * tt,
		a.w + (bb.w - a.w) * tt
	};
	return r.Normalized();
}

Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t) noexcept {
	const float tt = std::clamp(t, 0.0f, 1.0f);

	Quaternion bb = b;
	float cosTheta = Dot(a, b);

	//反対側を選ばない
	if (cosTheta < 0.0f) {
		cosTheta = -cosTheta;
		bb.x = -bb.x;
		bb.y = -bb.y;
		bb.z = -bb.z;
		bb.w = -bb.w;
	}

	//角度が小さい場合は Nlerpで十分（数値的にも安定）
	constexpr float kEps = 1e-5f;
	if (1.0f - cosTheta < kEps) {
		return Nlerp(a, bb, tt);
	}

	cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
	const float theta = std::acos(cosTheta);
	const float sinTheta = std::sin(theta);

	const float w0 = std::sin((1.0f - tt) * theta) / sinTheta;
	const float w1 = std::sin(tt * theta) / sinTheta;

	Quaternion r{
		a.x * w0 + bb.x * w1,
		a.y * w0 + bb.y * w1,
		a.z * w0 + bb.z * w1,
		a.w * w0 + bb.w * w1
	};
	return r.Normalized();
}

VECTOR Quaternion::RotateVector(const VECTOR& v) const noexcept {
	// v を純虚クォータニオンとして扱う
	const Quaternion q = Normalized();
	const Quaternion p{ v.x, v.y, v.z,0.0f };
	const Quaternion r = Multiply(Multiply(q, p), q.Inverse());
	return VGet(r.x, r.y, r.z);
}

MATRIX Quaternion::ToRotationMatrix() const noexcept {
	const Quaternion q = Normalized();
	const float xx = q.x * q.x;
	const float yy = q.y * q.y;
	const float zz = q.z * q.z;
	const float xy = q.x * q.y;
	const float xz = q.x * q.z;
	const float yz = q.y * q.z;
	const float wx = q.w * q.x;
	const float wy = q.w * q.y;
	const float wz = q.w * q.z;

	MATRIX m = MGetIdent();
	m.m[0][0] = 1.0f - 2.0f * (yy + zz);
	m.m[0][1] = 2.0f * (xy + wz);
	m.m[0][2] = 2.0f * (xz - wy);

	m.m[1][0] = 2.0f * (xy - wz);
	m.m[1][1] = 1.0f - 2.0f * (xx + zz);
	m.m[1][2] = 2.0f * (yz + wx);

	m.m[2][0] = 2.0f * (xz + wy);
	m.m[2][1] = 2.0f * (yz - wx);
	m.m[2][2] = 1.0f - 2.0f * (xx + yy);
	return m;
}

VECTOR Quaternion::ToEulerRad() const noexcept {
	// 回転順序: Z(roll) * Y(yaw) * X(pitch)
	//参考: 一般的なyaw-pitch-roll抽出（ただし実装系によって符号/軸が異なる）
	// 本プロジェクトの FromEulerRad と対になるように近い形で返す。
	const Quaternion q = Normalized();

	// pitch (X)
	const float sinp =2.0f * (q.w * q.x + q.y * q.z);
	const float cosp =1.0f -2.0f * (q.x * q.x + q.y * q.y);
	float pitch = std::atan2(sinp, cosp);

	// yaw (Y)
	float siny =2.0f * (q.w * q.y - q.z * q.x);
	siny = std::clamp(siny, -1.0f,1.0f);
	float yaw = std::asin(siny);

	// roll (Z)
	const float sinr =2.0f * (q.w * q.z + q.x * q.y);
	const float cosr =1.0f -2.0f * (q.y * q.y + q.z * q.z);
	float roll = std::atan2(sinr, cosr);

	return VGet(pitch, yaw, roll);
}
