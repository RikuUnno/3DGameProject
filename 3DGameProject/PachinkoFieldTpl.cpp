#include "PachinkoFieldTpl.h"

#include <algorithm>
#include <cstdlib>

#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"
#include "SceneManager.h"

// コンストラクタ/デストラクタ
PachinkoFieldTpl::PachinkoFieldTpl() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	transform.SetParent(nullptr);
	transform.SetLocalPosition(VGet(0.0f, 0.0f, 0.0f));
	transform.SetLocalEulerRad(VGet(0.0f, 0.0f, 0.0f));
	transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
	_boxCollider = std::make_unique<BoxCollider>();
	_boxCollider->owner = this;
	_physicsBody._owner = this;
	_physicsBody.Reset();
}

PachinkoFieldTpl::~PachinkoFieldTpl() = default;

// ライフサイクル
void PachinkoFieldTpl::OnDestroy() {
	ReleaseFromManagers_();
}

// プール/再利用フック
void PachinkoFieldTpl::OnAcquire(const VariantMap& params) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	transform.SetParent(nullptr);
	transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
	_physicsBody.Reset();
	_physicsBody._owner = this;

	_halfExtents = DefaultHalfExtents_();
	_drawColor = DefaultColor_();
	_materialName = DefaultMaterialName_();
	ApplyParams_(params);

	RebuildCollider_();
	if (_boxCollider) {
		_boxCollider->owner = this;
		_boxCollider->isTrigger = ParseBool_(params, "trigger", false);
		_boxCollider->layer = layerMask::ENVIRONMENT;
		_boxCollider->mask = mask::ALL;
		_boxCollider->enableCCD = false;
		_boxCollider->SetDebugColor(_drawColor);
		_boxCollider->UpdateShape();
	}

	_physicsBody._useGravity = false;
	_physicsBody._isKinematic = true;
	_physicsBody._freezeRotation = true;
	_physicsBody._linearDamping = 0.0f;
	_physicsBody._angularDamping = 0.0f;
	_physicsBody._restitution = 0.0f;
	_physicsBody._friction = 0.0f;
	_physicsBody._material = PhysicsMaterial::FromName(_materialName.c_str());
	_physicsBody._material.friction = 0.0f;
	_physicsBody._material.staticFriction = 0.0f;
	_physicsBody._material.restitution = 0.0f;
	_physicsBody._material.linearDamping = 0.0f;
	_physicsBody._material.angularDamping = 0.0f;
	_physicsBody.SetMass(0.0f);
	_physicsBody.ComputeInertia(_boxCollider.get());

	if (_boxCollider) ColliderManager::Instance().RegisterCollider(_boxCollider.get());
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
	_registered = true;
}

// プールに返却される直前の後片付け
void PachinkoFieldTpl::OnRelease() {
	ReleaseFromManagers_();
	SetActive(false);
	transform.SetParent(nullptr);
}

// 更新（毎フレーム呼ばれる）
void PachinkoFieldTpl::Draw() {
	if (_boxCollider) {
		_boxCollider->DrawDebug();
	}
}

// プールに返却される直前の後片付け
void PachinkoFieldTpl::ReleaseFromManagers_() {
	if (!_registered) return;
	if (_boxCollider) ColliderManager::Instance().UnregisterCollider(_boxCollider.get());
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
	_registered = false;
}

// コライダーの再構築
void PachinkoFieldTpl::RebuildCollider_() {
	if (!_boxCollider) _boxCollider = std::make_unique<BoxCollider>();
	_boxCollider->owner = this;
	_boxCollider->_box.center = VGet(0.0f, 0.0f, 0.0f);
	_boxCollider->_box.halfExtents = _halfExtents;
	_boxCollider->UpdateShape();
}

// パラメータマップから設定を適用
void PachinkoFieldTpl::ApplyParams_(const VariantMap& params) {
	const float px = ParseFloat_(params, "px", 0.0f);
	const float py = ParseFloat_(params, "py", 0.0f);
	const float pz = ParseFloat_(params, "pz", 0.0f);
	transform.SetLocalPosition(VGet(px, py, pz));

	const float pitch = ParseFloat_(params, "pitch", ParseFloat_(params, "rx", 0.0f));
	const float yaw   = ParseFloat_(params, "yaw",   ParseFloat_(params, "ry", 0.0f));
	const float roll  = ParseFloat_(params, "roll",  ParseFloat_(params, "rz", 0.0f));
	transform.SetLocalEulerRad(VGet(pitch, yaw, roll));

	const float sx = ParseFloat_(params, "sx", 1.0f);
	const float sy = ParseFloat_(params, "sy", 1.0f);
	const float sz = ParseFloat_(params, "sz", 1.0f);
	transform.SetLocalScale(VGet(sx, sy, sz));

	_halfExtents = VGet(
		ParseFloat_(params, "hx", _halfExtents.x),
		ParseFloat_(params, "hy", _halfExtents.y),
		ParseFloat_(params, "hz", _halfExtents.z)
	);

	const int color = ParseInt_(params, "color", 0);
	if (color != 0) _drawColor = static_cast<unsigned int>(color);
	_materialName = ParseString_(params, "material", _materialName);
}


// パラメータマップから値を取得するユーティリティ関数群
float PachinkoFieldTpl::ParseFloat_(const VariantMap& params, const char* key, float defaultValue) {						// Float 型の値をパラメータマップから取得。見つからなければ defaultValue を返す
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return static_cast<float>(std::atof(it->second.c_str()));
}
int PachinkoFieldTpl::ParseInt_(const VariantMap& params, const char* key, int defaultValue) {								// Int 型の値をパラメータマップから取得。見つからなければ defaultValue を返す
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return std::atoi(it->second.c_str());
}
bool PachinkoFieldTpl::ParseBool_(const VariantMap& params, const char* key, bool defaultValue) {							// Bool 型の値をパラメータマップから取得。見つからなければ defaultValue を返す
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	const std::string& s = it->second;
	return s == "1" || s == "true" || s == "TRUE" || s == "True";
}
std::string PachinkoFieldTpl::ParseString_(const VariantMap& params, const char* key, const std::string& defaultValue) {	// String 型の値をパラメータマップから取得。見つからなければ defaultValue を返す
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return it->second;
}