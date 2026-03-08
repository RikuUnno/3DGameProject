#pragma once

#include <cstdlib>
#include <string>

#include "DxLib.h"
#include "GameObject.h"
#include "SceneManager.h"

class ObjectPoolDebugClass : public GameObject {
public:
	static std::string StaticPoolKey() { return "ObjectPoolDebugClass"; }

	ObjectPoolDebugClass() {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		ResetState_();
	}
	virtual ~ObjectPoolDebugClass() override = default;

	void Awake() override {}
	void Start() override {}
	void End() override {}
	void OnDestroy() override {}

	void OnAcquire(const VariantMap& params) override {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		SetActive(true);
		transform.SetParent(nullptr);
		ResetState_();
		ConfigureFromParams_(params);
		transform.SetLocalPosition(_position);
	}

	void OnRelease() override {
		SetActive(false);
		transform.SetParent(nullptr);
	}

	void Update(float dt) override {
		_age += dt;
		_position = VAdd(_position, VScale(_velocity, dt));
		_velocity.y += _gravity * dt;
		transform.SetLocalPosition(_position);
	}

	void Draw() override {
		DrawSphere3D(transform.LocalPosition(), _radius, 12, _color, _color, TRUE);
	}

	bool IsExpired() const noexcept { return _age >= _lifeSec; }
	float AgeSec() const noexcept { return _age; }
	float LifeSec() const noexcept { return _lifeSec; }

private:
	void ResetState_() {
		_position = VGet(0.0f, 0.0f, 0.0f);
		_velocity = VGet(0.0f, 0.0f, 0.0f);
		_radius = 0.35f;
		_lifeSec = 3.0f;
		_age = 0.0f;
		_gravity = -2.5f;
		_color = GetColor(255, 220, 120);
	}

	void ConfigureFromParams_(const VariantMap& params) {
		_position = VGet(
			ParseFloat_(params, "px", 0.0f),
			ParseFloat_(params, "py", 0.0f),
			ParseFloat_(params, "pz", 0.0f)
		);
		_velocity = VGet(
			ParseFloat_(params, "vx", 0.0f),
			ParseFloat_(params, "vy", 0.0f),
			ParseFloat_(params, "vz", 0.0f)
		);
		_radius = ParseFloat_(params, "radius", 0.35f);
		_lifeSec = ParseFloat_(params, "life", 3.0f);
		_gravity = ParseFloat_(params, "gravity", -2.5f);
		const int color = ParseInt_(params, "color", 0);
		if (color != 0) {
			_color = static_cast<unsigned int>(color);
		}
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

private:
	VECTOR _position = VGet(0.0f, 0.0f, 0.0f);
	VECTOR _velocity = VGet(0.0f, 0.0f, 0.0f);
	float _radius = 0.35f;
	float _lifeSec = 3.0f;
	float _age = 0.0f;
	float _gravity = -2.5f;
	unsigned int _color = 0;
};