#include "Quaternion.h"

#include <cmath>

Quaternion::Quaternion(float _x, float _y, float _z, float _w) noexcept
	: x(_x), y(_y), z(_z), w(_w) {
}

Quaternion Quaternion::Identity() noexcept {
	return Quaternion{};
}

Quaternion Quaternion::FromEulerRad(float pitch, float yaw, float roll) noexcept {
	const float hx = pitch *0.5f;
	const float hy = yaw *0.5f;
	const float hz = roll *0.5f;

	const float cx = std::cos(hx);
	const float sx = std::sin(hx);
	const float cy = std::cos(hy);
	const float sy = std::sin(hy);
	const float cz = std::cos(hz);
	const float sz = std::sin(hz);

	Quaternion q;
	// âÒì]èáèò: Z(roll) * Y(yaw) * X(pitch)
	q.w = cz * cy * cx + sz * sy * sx;
	q.x = cz * cy * sx - sz * sy * cx;
	q.y = cz * sy * cx + sz * cy * sx;
	q.z = sz * cy * cx - cz * sy * sx;
	return q.Normalized();
}

Quaternion Quaternion::Normalized() const noexcept {
	const float len2 = x * x + y * y + z * z + w * w;
	if (len2 <=0.0f) return Identity();
	const float inv =1.0f / std::sqrt(len2);
	return Quaternion{ x * inv, y * inv, z * inv, w * inv };
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
	m.m[0][0] =1.0f -2.0f * (yy + zz);
	m.m[0][1] =2.0f * (xy + wz);
	m.m[0][2] =2.0f * (xz - wy);

	m.m[1][0] =2.0f * (xy - wz);
	m.m[1][1] =1.0f -2.0f * (xx + zz);
	m.m[1][2] =2.0f * (yz + wx);

	m.m[2][0] =2.0f * (xz + wy);
	m.m[2][1] =2.0f * (yz - wx);
	m.m[2][2] =1.0f -2.0f * (xx + yy);
	return m;
}
