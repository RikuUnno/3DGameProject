#pragma once

#include <cstdlib>
#include <string>

#include "DxLib.h"
#include "GameObject.h"
#include "SceneManager.h"

class TransformDebugClass : public GameObject {
public:
	static std::string StaticPoolKey() { return "TransformDebugClass"; }

	TransformDebugClass() {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		ResetState_();
	}
	virtual ~TransformDebugClass() override = default;

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
	}

	void OnRelease() override {
		transform.SetParent(nullptr);
		SetActive(false);
	}

	void Update(float dt) override {
		if (_selfSpinRadSec != 0.0f) {
			VECTOR e = transform.LocalEulerRad();
			e.y += _selfSpinRadSec * dt;
			transform.SetLocalEulerRad(e);
		}
	}

	void Draw() override {
		const VECTOR c = transform.WorldPosition();
		const VECTOR rx = transform.Right();
		const VECTOR uy = transform.Up();
		const VECTOR fz = transform.Forward();
		DrawSphere3D(c, _radius, 12, _color, _color, TRUE);
		DrawLine3D(c, VAdd(c, VScale(rx, _axisLength)), GetColor(255, 80, 80));
		DrawLine3D(c, VAdd(c, VScale(uy, _axisLength)), GetColor(80, 255, 80));
		DrawLine3D(c, VAdd(c, VScale(fz, _axisLength)), GetColor(80, 80, 255));
	}

private:
	void ResetState_() {
		transform.SetLocalPosition(VGet(0.0f, 0.0f, 0.0f));
		transform.SetLocalEulerRad(VGet(0.0f, 0.0f, 0.0f));
		transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
		_radius = 0.45f;
		_axisLength = 1.0f;
		_selfSpinRadSec = 0.0f;
		_color = GetColor(220, 220, 220);
	}

	void ConfigureFromParams_(const VariantMap& params) {
		transform.SetLocalPosition(VGet(
			ParseFloat_(params, "px", 0.0f),
			ParseFloat_(params, "py", 0.0f),
			ParseFloat_(params, "pz", 0.0f)
		));
		transform.SetLocalEulerRad(VGet(
			ParseFloat_(params, "pitch", 0.0f),
			ParseFloat_(params, "yaw", 0.0f),
			ParseFloat_(params, "roll", 0.0f)
		));
		_radius = ParseFloat_(params, "radius", 0.45f);
		_axisLength = ParseFloat_(params, "axis", 1.0f);
		_selfSpinRadSec = ParseFloat_(params, "spin", 0.0f);
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
	float _radius = 0.45f;
	float _axisLength = 1.0f;
	float _selfSpinRadSec = 0.0f;
	unsigned int _color = 0;
};