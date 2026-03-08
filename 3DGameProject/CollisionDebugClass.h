#pragma once

#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "ColliderManager.h"
#include "DxLib.h"
#include "GameObject.h"
#include "LayerMask.h"
#include "SceneManager.h"
#include "SphereCollider.h"

class CollisionDebugClass : public GameObject {
public:
	enum class ShapeType {
		Box,
		Sphere,
		Capsule,
	};

	enum class MotionType {
		Static,
		PingPongX,
		CircleXZ,
	};

	static std::string StaticPoolKey() { return "CollisionDebugClass"; }
	static const std::string& LastEventText() noexcept { return s_lastEventText; }
	static void ResetEventText() { s_lastEventText = "まだイベントは発生していません"; }

	CollisionDebugClass() {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		ResetState_();
		CreateCollider_(ShapeType::Box);
	}
	virtual ~CollisionDebugClass() override {
		OnDestroy();
	}

	void Awake() override {}
	void Start() override {}
	void End() override {}

	void OnDestroy() override {
		ReleaseCollider_();
	}

	void OnAcquire(const VariantMap& params) override {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		SetActive(true);
		transform.SetParent(nullptr);
		ResetState_();
		ConfigureFromParams_(params);
		EnsureColliderRegistered_();
	}

	void OnRelease() override {
		ReleaseCollider_();
		transform.SetParent(nullptr);
		SetActive(false);
	}

	void Update(float dt) override {
		_time += dt;
		VECTOR p = _basePosition;
		const float a = _speed * _time + _phase;
		switch (_motionType) {
		case MotionType::PingPongX:
			p.x += std::sin(a) * _motionRadius;
			break;
		case MotionType::CircleXZ:
			p.x += std::cos(a) * _motionRadius;
			p.z += std::sin(a) * _motionRadius;
			break;
		case MotionType::Static:
		default:
			break;
		}
		transform.SetLocalPosition(p);

		VECTOR e = _baseEuler;
		e.y += _spinSpeed * _time;
		transform.SetLocalEulerRad(e);
	}

	void Draw() override {
		if (Collider* c = GetCollider()) {
			c->DrawDebug();
		}
	}

	void OnCollisionEnter(Collider* self, Collider* other) override {
		if (!self) return;
		++_collisionDepth;
		SetLastEvent_(_name + " : CollisionEnter -> " + OtherName_(other));
		ApplyStateColor_();
	}

	void OnCollisionStay(Collider* self, Collider* other) override {
		if (!self) return;
		SetLastEvent_(_name + " : CollisionStay -> " + OtherName_(other));
		ApplyStateColor_();
	}

	void OnCollisionExit(Collider* self, Collider* other) override {
		if (!self) return;
		if (_collisionDepth > 0) --_collisionDepth;
		SetLastEvent_(_name + " : CollisionExit -> " + OtherName_(other));
		ApplyStateColor_();
	}

	void OnTriggerEnter(Collider* self, Collider* other) override {
		if (!self) return;
		++_triggerDepth;
		SetLastEvent_(_name + " : TriggerEnter -> " + OtherName_(other));
		ApplyStateColor_();
	}

	void OnTriggerStay(Collider* self, Collider* other) override {
		if (!self) return;
		SetLastEvent_(_name + " : TriggerStay -> " + OtherName_(other));
		ApplyStateColor_();
	}

	void OnTriggerExit(Collider* self, Collider* other) override {
		if (!self) return;
		if (_triggerDepth > 0) --_triggerDepth;
		SetLastEvent_(_name + " : TriggerExit -> " + OtherName_(other));
		ApplyStateColor_();
	}

private:
	void ResetState_() {
		_shapeType = ShapeType::Box;
		_motionType = MotionType::Static;
		_registeredToColliderManager = false;
		_name = "CollisionObject";
		_basePosition = VGet(0.0f, 0.0f, 0.0f);
		_baseEuler = VGet(0.0f, 0.0f, 0.0f);
		_motionRadius = 2.5f;
		_speed = 1.0f;
		_phase = 0.0f;
		_spinSpeed = 0.0f;
		_isTriggerState = false;
		_collisionDepth = 0;
		_triggerDepth = 0;
		_baseColor = GetColor(220, 220, 220);
		_collisionColor = GetColor(255, 90, 90);
		_triggerColor = GetColor(90, 140, 255);
		_time = 0.0f;
		isStatic = false;
	}

	Collider* GetCollider() const noexcept {
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

	void CreateCollider_(ShapeType shapeType) {
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

	void EnsureColliderRegistered_() {
		if (_registeredToColliderManager) return;
		if (Collider* c = GetCollider()) {
			ColliderManager::Instance().RegisterCollider(c);
			_registeredToColliderManager = true;
		}
	}

	void ReleaseCollider_() {
		if (_registeredToColliderManager) {
			if (Collider* c = GetCollider()) {
				ColliderManager::Instance().UnregisterCollider(c);
			}
			_registeredToColliderManager = false;
		}
	}

	void ConfigureFromParams_(const VariantMap& params) {
		_name = ParseString_(params, "name", _name);
		const std::string shape = ParseString_(params, "shape", "box");
		if (shape == "sphere") CreateCollider_(ShapeType::Sphere);
		else if (shape == "capsule") CreateCollider_(ShapeType::Capsule);
		else CreateCollider_(ShapeType::Box);

		const std::string motion = ParseString_(params, "motion", "static");
		if (motion == "pingpong") _motionType = MotionType::PingPongX;
		else if (motion == "circle") _motionType = MotionType::CircleXZ;
		else _motionType = MotionType::Static;

		_basePosition = VGet(ParseFloat_(params, "px", 0.0f), ParseFloat_(params, "py", 0.0f), ParseFloat_(params, "pz", 0.0f));
		_baseEuler = VGet(ParseFloat_(params, "pitch", 0.0f), ParseFloat_(params, "yaw", 0.0f), ParseFloat_(params, "roll", 0.0f));
		transform.SetLocalPosition(_basePosition);
		transform.SetLocalEulerRad(_baseEuler);

		_motionRadius = ParseFloat_(params, "motionRadius", _motionRadius);
		_speed = ParseFloat_(params, "speed", _speed);
		_phase = ParseFloat_(params, "phase", _phase);
		_spinSpeed = ParseFloat_(params, "spinSpeed", _spinSpeed);
		_isTriggerState = ParseBool_(params, "trigger", false);
		isStatic = ParseBool_(params, "static", false);
		_baseColor = static_cast<unsigned int>(ParseInt_(params, "color", static_cast<int>(_baseColor)));
		_collisionColor = static_cast<unsigned int>(ParseInt_(params, "collisionColor", static_cast<int>(_collisionColor)));
		_triggerColor = static_cast<unsigned int>(ParseInt_(params, "triggerColor", static_cast<int>(_triggerColor)));

		if (_shapeType == ShapeType::Box) {
			_boxCollider->_box.center = VGet(0, 0, 0);
			_boxCollider->_box.halfExtents = VGet(ParseFloat_(params, "hx", 0.7f), ParseFloat_(params, "hy", 0.7f), ParseFloat_(params, "hz", 0.7f));
		}
		else if (_shapeType == ShapeType::Sphere) {
			_sphereCollider->_sphere.center = VGet(0, 0, 0);
			_sphereCollider->_sphere.radius = ParseFloat_(params, "radius", 0.6f);
		}
		else {
			const float radius = ParseFloat_(params, "radius", 0.45f);
			const float halfHeight = ParseFloat_(params, "halfHeight", 0.9f);
			_capsuleCollider->_cap.center = VGet(0, 0, 0);
			_capsuleCollider->_cap.bottom = VGet(0, -halfHeight, 0);
			_capsuleCollider->_cap.top = VGet(0, halfHeight, 0);
			_capsuleCollider->_cap.radius = radius;
		}

		if (Collider* c = GetCollider()) {
			c->isTrigger = _isTriggerState;
			c->layer = ParseInt_(params, "layer", _isTriggerState ? layerMask::TRIGGER : layerMask::DEFAULT);
			c->mask = ParseInt_(params, "mask", mask::ALL);
			c->sendEventsToOwner = true;
			c->bubbleEventsToParentOwner = false;
			ApplyStateColor_();
			c->UpdateShape();
		}
	}

	void ApplyStateColor_() {
		if (Collider* c = GetCollider()) {
			if (_triggerDepth > 0) c->SetDebugColor(_triggerColor);
			else if (_collisionDepth > 0) c->SetDebugColor(_collisionColor);
			else c->SetDebugColor(_baseColor);
		}
	}

	std::string OtherName_(Collider* other) const {
		if (!other || !other->owner) return "Unknown";
		if (auto* dbg = dynamic_cast<CollisionDebugClass*>(other->owner)) return dbg->_name;
		return "Other";
	}

	static void SetLastEvent_(const std::string& text) {
		s_lastEventText = text;
	}

	static float ParseFloat_(const VariantMap& params, const char* key, float defaultValue) {
		auto it = params.find(key);
		if (it == params.end()) return defaultValue;
		return static_cast<float>(std::atof(it->second.c_str()));
	}

	static int ParseInt_(const VariantMap& params, const char* key, int defaultValue) {
		auto it = params.find(key);
		if (it == params.end()) return defaultValue;
		return std::atoi(it->second.c_str());
	}

	static bool ParseBool_(const VariantMap& params, const char* key, bool defaultValue) {
		auto it = params.find(key);
		if (it == params.end()) return defaultValue;
		return it->second == "1" || it->second == "true" || it->second == "TRUE";
	}

	static std::string ParseString_(const VariantMap& params, const char* key, const std::string& defaultValue) {
		auto it = params.find(key);
		if (it == params.end()) return defaultValue;
		return it->second;
	}

private:
	inline static std::string s_lastEventText = "まだイベントは発生していません";
	std::unique_ptr<SphereCollider> _sphereCollider;
	std::unique_ptr<BoxCollider> _boxCollider;
	std::unique_ptr<CapsuleCollider> _capsuleCollider;
	bool _registeredToColliderManager = false;
	ShapeType _shapeType = ShapeType::Box;
	MotionType _motionType = MotionType::Static;
	std::string _name = "CollisionObject";
	VECTOR _basePosition = VGet(0, 0, 0);
	VECTOR _baseEuler = VGet(0, 0, 0);
	float _motionRadius = 2.5f;
	float _speed = 1.0f;
	float _phase = 0.0f;
	float _spinSpeed = 0.0f;
	bool _isTriggerState = false;
	int _collisionDepth = 0;
	int _triggerDepth = 0;
	unsigned int _baseColor = 0;
	unsigned int _collisionColor = 0;
	unsigned int _triggerColor = 0;
	float _time = 0.0f;
};