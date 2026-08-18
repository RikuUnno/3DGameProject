#include "PhysicsDebugClass.h"

#include <cstdlib>

#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "ColliderManager.h"
#include "LayerMask.h"
#include "PhysicsMaterial.h"
#include "PhysicsManager.h"
#include "SceneManager.h"
#include "SphereCollider.h"
#include "DxLib.h"

namespace {
	// 0 以下や極小値を避けたい設定値用の clamp
	inline float ClampPositive(float value, float fallback) noexcept {
		return (value > 1e-4f) ? value : fallback;
	}
}

// 生成時は既定形状を用意しておき、OnAcquire で用途ごとに上書きする
PhysicsDebugClass::PhysicsDebugClass() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	_physicsBody._owner = this;
	_physicsBody.Reset();
	CreateCollider_(DefaultShapeType());
	ApplyVisualDefaults_();
}

PhysicsDebugClass::~PhysicsDebugClass() {
	OnDestroy();
}

void PhysicsDebugClass::Awake() {
}

void PhysicsDebugClass::Start() {
}

void PhysicsDebugClass::Update(float /*dt*/) {
}

// 現在の形状に応じたデバッグ描画を行う
void PhysicsDebugClass::Draw() {
	if (Collider* collider = GetCollider()) {
		collider->DrawDebug();
	}
}

void PhysicsDebugClass::End() {
}

// Scene 終了や完全破棄時の後片付け
void PhysicsDebugClass::OnDestroy() {
	ReleaseCollider_();
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

// Pool から取得された直後の初期化
void PhysicsDebugClass::OnAcquire(const VariantMap& params) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	transform.SetParent(nullptr);
	transform.SetLocalPosition(VGet(0.0f, 0.0f, 0.0f));
	transform.SetLocalRotation(Quaternion::Identity());
	transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._linearDamping = 0.03f;
	_physicsBody._angularDamping = 0.05f;
	_physicsBody._friction = 0.7f;
	_physicsBody._restitution = 0.1f;
	_physicsBody._material.friction = 0.7f;
	_physicsBody._material.staticFriction = 0.84f;
	_physicsBody._material.restitution = 0.1f;
	_physicsBody._material.linearDamping = 0.03f;
	_physicsBody._material.angularDamping = 0.05f;
	_physicsBody._maxLinearSpeed = 80.0f;
	_physicsBody._maxAngularSpeed = 20.0f;
	isStatic = false;
	_materialName.clear();
	CreateCollider_(DefaultShapeType());
	ConfigureFromParams_(params);
	EnsureColliderRegistered_();
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
}

// Pool へ返却する時は Collider / PhysicsBody の登録を外す
void PhysicsDebugClass::OnRelease() {
	ReleaseCollider_();
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
	transform.SetParent(nullptr);
	SetActive(false);
}

// 現在有効な形状に対応する collider を返す
Collider* PhysicsDebugClass::GetCollider() const noexcept {
	switch (_shapeType) {
	case ShapeType::Sphere:
		return _sphereCollider.get();
	case ShapeType::Capsule:
		return _capsuleCollider.get();
	case ShapeType::Box:
	default:
		return _boxCollider.get();
	}
}

void PhysicsDebugClass::ReleaseCollider_() {
	if (_registeredToColliderManager) {
		if (Collider* collider = GetCollider()) {
			ColliderManager::Instance().UnregisterCollider(collider);
		}
		_registeredToColliderManager = false;
	}
}

void PhysicsDebugClass::EnsureColliderRegistered_() {
	if (_registeredToColliderManager) return;
	if (Collider* collider = GetCollider()) {
		ColliderManager::Instance().RegisterCollider(collider);
		_registeredToColliderManager = true;
	}
}

// VariantMap からシーン側指定の形状・物理パラメータを反映する
void PhysicsDebugClass::ConfigureFromParams_(const VariantMap& params) {
	const std::string defaultShape =
		DefaultShapeType() == ShapeType::Sphere ? "sphere" :
		DefaultShapeType() == ShapeType::Capsule ? "capsule" :
		"box";
	const std::string shape = ParseString_(params, "shape", defaultShape);
	if (shape == "sphere") {
		CreateCollider_(ShapeType::Sphere);
	}
	else if (shape == "capsule") {
		CreateCollider_(ShapeType::Capsule);
	}
	else {
		CreateCollider_(ShapeType::Box);
	}

	const bool isTrigger = ParseBool_(params, "trigger", false);
	const bool isStaticObject = ParseBool_(params, "static", false);
	const float px = ParseFloat_(params, "px", 0.0f);
	const float py = ParseFloat_(params, "py", 0.0f);
	const float pz = ParseFloat_(params, "pz", 0.0f);
	transform.SetLocalPosition(VGet(px, py, pz));

	// 回転パラメータの適用（ラジアン）
	// - pitch/yaw/roll を優先
	// - 省略時は rx/ry/rz を代替キーとして使用
	const float pitch = ParseFloat_(params, "pitch", ParseFloat_(params, "rx", 0.0f));
	const float yaw   = ParseFloat_(params, "yaw",   ParseFloat_(params, "ry", 0.0f));
	const float roll  = ParseFloat_(params, "roll",  ParseFloat_(params, "rz", 0.0f));
	transform.SetLocalEulerRad(VGet(pitch, yaw, roll));

	const float sx = ParseFloat_(params, "sx", 1.0f);
	const float sy = ParseFloat_(params, "sy", 1.0f);
	const float sz = ParseFloat_(params, "sz", 1.0f);
	transform.SetLocalScale(VGet(sx, sy, sz));

	if (_shapeType == ShapeType::Box) {
		const float hx = ClampPositive(ParseFloat_(params, "hx", 0.5f), 0.5f);
		const float hy = ClampPositive(ParseFloat_(params, "hy", 0.5f), 0.5f);
		const float hz = ClampPositive(ParseFloat_(params, "hz", 0.5f), 0.5f);
		_boxCollider->_box.center = VGet(0.0f, 0.0f, 0.0f);
		_boxCollider->_box.halfExtents = VGet(hx, hy, hz);
		_drawHalfExtents = VGet(hx, hy, hz);
	}
	else if (_shapeType == ShapeType::Sphere) {
		const float radius = ClampPositive(ParseFloat_(params, "radius", 0.5f), 0.5f);
		_sphereCollider->_sphere.center = VGet(0.0f, 0.0f, 0.0f);
		_sphereCollider->_sphere.radius = radius;
		_drawRadius = radius;
	}
	else {
		const float radius = ClampPositive(ParseFloat_(params, "radius", 0.45f), 0.45f);
		const float halfHeight = ClampPositive(ParseFloat_(params, "halfHeight", 0.7f), 0.7f);
		_capsuleCollider->_cap.center = VGet(0.0f, 0.0f, 0.0f);
		_capsuleCollider->_cap.bottom = VGet(0.0f, -halfHeight, 0.0f);
		_capsuleCollider->_cap.top = VGet(0.0f, halfHeight, 0.0f);
		_capsuleCollider->_cap.radius = radius;
		_drawRadius = radius;
		_drawHeight = halfHeight * 2.0f;
	}

	if (Collider* collider = GetCollider()) {
		collider->isTrigger = isTrigger;
		collider->layer = isStaticObject ? layerMask::ENVIRONMENT : layerMask::DEFAULT;
		collider->mask = mask::ALL;
		collider->enableCCD = ParseBool_(params, "ccd", true);
		collider->ccdDistanceThreshold = ClampPositive(ParseFloat_(params, "ccdThreshold", 8.0f), 8.0f);
		collider->UpdateShape();
	}

	isStatic = isStaticObject;
	if (isStaticObject) {
		_physicsBody.SetMass(0.0f);
		_physicsBody._isKinematic = true;
		_physicsBody._useGravity = false;
		_physicsBody._linearDamping = 0.0f;
		_physicsBody._angularDamping = 0.0f;
	}
	else {
		_physicsBody._isKinematic = false;
		_physicsBody.SetMass(ClampPositive(ParseFloat_(params, "mass", 1.0f), 1.0f));
		_physicsBody._useGravity = ParseBool_(params, "gravity", true);
		_physicsBody._linearDamping = ParseFloat_(params, "linearDamping", 0.03f);
		_physicsBody._angularDamping = ParseFloat_(params, "angularDamping", 0.05f);
	}

	// マテリアル適用: "material" パラメータがあればプリセットから一括設定。
	// その後の個別パラメータ（friction, restitution 等）で上書き可能。
	const std::string materialName = ParseString_(params, "material", "");
	_materialName = materialName;
	if (!materialName.empty()) {
		PhysicsMaterial mat = PhysicsMaterial::FromName(materialName.c_str());
		_physicsBody.ApplyMaterial(mat, GetCollider());
	}

	// 個別パラメータ上書き（material 適用後でも明示指定があれば優先）
	if (params.count("restitution")) {
		_physicsBody._restitution = ParseFloat_(params, "restitution", _physicsBody._restitution);
		_physicsBody._material.restitution = _physicsBody._restitution;
	}
	else if (materialName.empty()) {
		_physicsBody._restitution = 0.1f;
		_physicsBody._material.restitution = _physicsBody._restitution;
	}
	if (params.count("friction")) {
		_physicsBody._friction = ParseFloat_(params, "friction", _physicsBody._friction);
		_physicsBody._material.friction = _physicsBody._friction;
		_physicsBody._material.staticFriction = _physicsBody._friction * 1.2f;
	}
	else if (materialName.empty()) {
		_physicsBody._friction = 0.7f;
		_physicsBody._material.friction = _physicsBody._friction;
		_physicsBody._material.staticFriction = _physicsBody._friction * 1.2f;
	}
	if (params.count("linearDamping")) {
		_physicsBody._linearDamping = ParseFloat_(params, "linearDamping", _physicsBody._linearDamping);
		_physicsBody._material.linearDamping = _physicsBody._linearDamping;
	}
	if (params.count("angularDamping")) {
		_physicsBody._angularDamping = ParseFloat_(params, "angularDamping", _physicsBody._angularDamping);
		_physicsBody._material.angularDamping = _physicsBody._angularDamping;
	}
	_physicsBody._freezeRotation = ParseBool_(params, "freezeRotation", false);
	_physicsBody._detectContinuous = ParseBool_(params, "ccd", false);
	_physicsBody._velocity = VGet(
		ParseFloat_(params, "vx", 0.0f),
		ParseFloat_(params, "vy", 0.0f),
		ParseFloat_(params, "vz", 0.0f)
	);
	_physicsBody._angularVelocity = VGet(
		ParseFloat_(params, "avx", 0.0f),
		ParseFloat_(params, "avy", 0.0f),
		ParseFloat_(params, "avz", 0.0f)
	);
	_physicsBody._maxLinearSpeed = ClampPositive(ParseFloat_(params, "maxLinearSpeed", 80.0f), 80.0f);
	_physicsBody._maxAngularSpeed = ClampPositive(ParseFloat_(params, "maxAngularSpeed", 20.0f), 20.0f);

	_drawColor = static_cast<unsigned int>(ParseInt_(params, "color", 0));
	ApplyVisualDefaults_();
}

// 必要な collider を lazy に生成し、使用形状だけ切り替える
void PhysicsDebugClass::CreateCollider_(ShapeType shapeType) {
	if (_shapeType == shapeType && GetCollider() != nullptr) {
		return;
	}

	ReleaseCollider_();
	_shapeType = shapeType;

	if (!_boxCollider) {
		_boxCollider = std::make_unique<BoxCollider>();
		_boxCollider->owner = this;
	}
	if (!_sphereCollider) {
		_sphereCollider = std::make_unique<SphereCollider>();
		_sphereCollider->owner = this;
	}
	if (!_capsuleCollider) {
		_capsuleCollider = std::make_unique<CapsuleCollider>();
		_capsuleCollider->owner = this;
	}
}

// static 物は青系、明示色がある場合はそれを優先する
void PhysicsDebugClass::ApplyVisualDefaults_() {
	if (Collider* collider = GetCollider()) {
		if (_drawColor != 0) {
			collider->SetDebugColor(_drawColor);
		}
		else if (!_materialName.empty()) {
			// 素材名から色を自動決定
			unsigned int matColor = 0;
			if (_materialName == "wood")        matColor = GetColor(180, 140, 80);
			else if (_materialName == "metal")   matColor = GetColor(180, 185, 200);
			else if (_materialName == "rubber")  matColor = GetColor(220, 80, 80);
			else if (_materialName == "ice")     matColor = GetColor(160, 220, 255);
			else if (_materialName == "stone")   matColor = GetColor(160, 160, 150);
			else if (_materialName == "bouncy")  matColor = GetColor(255, 200, 60);
			if (matColor != 0) {
				collider->SetDebugColor(matColor);
			} else if (isStatic) {
				collider->SetDebugColor(GetColor(120, 220, 255));
			} else {
				collider->ClearDebugColor();
			}
		}
		else if (isStatic) {
			collider->SetDebugColor(GetColor(120, 220, 255));
		}
		else {
			collider->ClearDebugColor();
		}
	}
}

float PhysicsDebugClass::ParseFloat_(const VariantMap& params, const char* key, float defaultValue) {
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return static_cast<float>(std::atof(it->second.c_str()));
}

int PhysicsDebugClass::ParseInt_(const VariantMap& params, const char* key, int defaultValue) {
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return std::atoi(it->second.c_str());
}

bool PhysicsDebugClass::ParseBool_(const VariantMap& params, const char* key, bool defaultValue) {
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return it->second == "1" || it->second == "true" || it->second == "TRUE";
}

std::string PhysicsDebugClass::ParseString_(const VariantMap& params, const char* key, const std::string& defaultValue) {
	auto it = params.find(key);
	if (it == params.end()) return defaultValue;
	return it->second;
}