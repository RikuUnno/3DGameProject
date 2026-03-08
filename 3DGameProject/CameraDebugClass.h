#pragma once

#include <cmath>
#include <cstdlib>
#include <string>

#include "DxLib.h"
#include "GameObject.h"
#include "SceneManager.h"

// CameraDebugClass
// - CameraScene で使うカメラ確認用のデモオブジェクト
// - 単純な軌道運動を行い、追従/注視/周回カメラの確認対象にする
class CameraDebugClass : public GameObject {
public:
	enum class MotionType {
		Static,
		CircleXZ,
		PingPongX,
		BobY,
	};

	static std::string StaticPoolKey() { return "CameraDebugClass"; }

	CameraDebugClass() {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		ResetState_();
	}
	virtual ~CameraDebugClass() override = default;

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
		transform.SetLocalPosition(_basePosition);
	}

	void OnRelease() override {
		SetActive(false);
		transform.SetParent(nullptr);
	}

	void Update(float dt) override {
		_time += dt;
		const float a = _speed * _time + _phase;
		VECTOR p = _basePosition;
		switch (_motionType) {
		case MotionType::CircleXZ:
			p.x += std::cos(a) * _motionRadius;
			p.z += std::sin(a) * _motionRadius;
			break;
		case MotionType::PingPongX:
			p.x += std::sin(a) * _motionRadius;
			break;
		case MotionType::BobY:
			p.y += std::sin(a) * _motionRadius;
			break;
		case MotionType::Static:
		default:
			break;
		}
		transform.SetLocalPosition(p);
	}

	void Draw() override {
		const VECTOR c = transform.LocalPosition();
		DrawSphere3D(c, _drawRadius, 12, _drawColor, _drawColor, TRUE);
		DrawLine3D(VAdd(c, VGet(-_drawRadius * 1.5f, 0, 0)), VAdd(c, VGet(_drawRadius * 1.5f, 0, 0)), _drawColor);
		DrawLine3D(VAdd(c, VGet(0, -_drawRadius * 1.5f, 0)), VAdd(c, VGet(0, _drawRadius * 1.5f, 0)), _drawColor);
		DrawLine3D(VAdd(c, VGet(0, 0, -_drawRadius * 1.5f)), VAdd(c, VGet(0, 0, _drawRadius * 1.5f)), _drawColor);
	}

private:
	void ResetState_() {
		_motionType = MotionType::Static;
		_basePosition = VGet(0.0f, 0.0f, 0.0f);
		_motionRadius = 2.0f;
		_speed = 1.0f;
		_phase = 0.0f;
		_drawRadius = 0.35f;
		_drawColor = GetColor(255, 220, 120);
		_time = 0.0f;
	}

	void ConfigureFromParams_(const VariantMap& params) {
		_basePosition = VGet(
			ParseFloat_(params, "px", 0.0f),
			ParseFloat_(params, "py", 0.0f),
			ParseFloat_(params, "pz", 0.0f)
		);
		_motionRadius = ParseFloat_(params, "motionRadius", 2.0f);
		_speed = ParseFloat_(params, "speed", 1.0f);
		_phase = ParseFloat_(params, "phase", 0.0f);
		_drawRadius = ParseFloat_(params, "drawRadius", 0.35f);
		const int color = ParseInt_(params, "color", 0);
		if (color != 0) {
			_drawColor = static_cast<unsigned int>(color);
		}

		const std::string motion = ParseString_(params, "motion", "static");
		if (motion == "circle") _motionType = MotionType::CircleXZ;
		else if (motion == "pingpong") _motionType = MotionType::PingPongX;
		else if (motion == "bob") _motionType = MotionType::BobY;
		else _motionType = MotionType::Static;
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

	static std::string ParseString_(const VariantMap& params, const char* key, const std::string& defaultValue) {
		auto it = params.find(key);
		if (it == params.end()) return defaultValue;
		return it->second;
	}

private:
	MotionType _motionType = MotionType::Static;
	VECTOR _basePosition = VGet(0.0f, 0.0f, 0.0f);
	float _motionRadius = 2.0f;
	float _speed = 1.0f;
	float _phase = 0.0f;
	float _drawRadius = 0.35f;
	unsigned int _drawColor = 0;
	float _time = 0.0f;
};