#pragma once

#include <memory>
#include <string>

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTpl.h"
#include "SceneTransition.h"
#include "TitleScene.h"
#include "TransformDebugClass.h"

class TransformScene : public SceneTpl<TransformScene> {
public:
	static std::string StaticName() { return "TransformScene"; }

	void Start() override {
		auto& cameraManager = CameraManager::Instance();
		const int sceneId = SceneManager::Instance().CurrentSceneId();

		static bool s_registered = false;
		if (!s_registered) {
			ObjectFactory::Instance().RegisterCreator(TransformDebugClass::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<TransformDebugClass>(); });
			ObjectManager::Instance().RegisterPool(TransformDebugClass::StaticPoolKey(), 16);
			s_registered = true;
		}

		_parent = SpawnTransformObject_({
			{"px", "0.0"}, {"py", "1.2"}, {"pz", "0.0"},
			{"radius", "0.55"},
			{"axis", "1.5"},
			{"color", std::to_string(GetColor(255, 255, 255))}
		});
		_child = SpawnTransformObject_({
			{"px", std::to_string(_childLocalOffset.x)}, {"py", std::to_string(_childLocalOffset.y)}, {"pz", std::to_string(_childLocalOffset.z)},
			{"radius", "0.38"},
			{"axis", "1.0"},
			{"color", std::to_string(GetColor(255, 220, 120))}
		});
		_worldMarker = SpawnTransformObject_({
			{"px", "0.0"}, {"py", "1.2"}, {"pz", "0.0"},
			{"radius", "0.25"},
			{"axis", "0.8"},
			{"color", std::to_string(GetColor(120, 220, 255))}
		});

		if (_child && _parent) {
			_child->transform.SetParent(&_parent->transform);
			_child->transform.SetLocalPosition(_childLocalOffset);
		}
		if (_worldMarker) {
			_worldMarker->transform.SetLocalPosition(VGet(0.0f, 1.2f, 0.0f));
		}

		if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
			_cameraId = _cameraController.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 4.5f, -14.0f), VGet(0.12f, 0.0f, 0.0f));
		}
		cameraManager.SetRender(_cameraId);
	}

	void End() override {
		_parent = nullptr;
		_child = nullptr;
		_worldMarker = nullptr;
	}

	void Update(float dtSec) override {
		ObjectManager::Instance().UpdateAll(dtSec);

		_cameraController.SetCamera(_cameraId);
		_cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

		_driveTime += dtSec;
		UpdateParentMotion_();

		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F) && _child) {
			if (_child->transform.Parent()) {
				_child->transform.SetParent(nullptr);
				_isAttached = false;
			}
			else if (_parent) {
				_child->transform.SetParent(&_parent->transform);
				_child->transform.SetLocalPosition(_childLocalOffset);
				_isAttached = true;
			}
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
			SceneManager::Instance().RequestChange(std::make_unique<TransformScene>());
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {
			SceneTransition::Params p;
			p.mode = SceneTransition::Mode::MaskImage;
			p.durationSec = 0.4f;
			p.maskGraphPath = "Data/Transition/mask.png";
			p.pixelShaderPath = "Data/Transition/mask_transition.pso";
			SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p, 0.5f);
		}
	}

	void Draw() override {
		DrawGridFloor_(0.0f, 12, 1.0f);
		ObjectManager::Instance().DrawAll();

		DrawString(10, 10, "TransformScene - F:着脱切替 R:リセット T:タイトル", GetColor(255, 255, 255));
		DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
		DrawString(10, 50, "白:親  黄:子  水色:ワールド基準マーカー", GetColor(255, 220, 140));
		DrawString(10, 70, _isAttached ? "状態: 親子付け中(子は親回転に追従)" : "状態: 切り離し中(子はワールド側で独立)", GetColor(180, 255, 180));

		if (_child) {
			const VECTOR lp = _child->transform.LocalPosition();
			const VECTOR wp = _child->transform.WorldPosition();
			DrawFormatString(10, 100, GetColor(220, 220, 220), "子 Local  : (%.2f, %.2f, %.2f)", lp.x, lp.y, lp.z);
			DrawFormatString(10, 120, GetColor(220, 220, 220), "子 World  : (%.2f, %.2f, %.2f)", wp.x, wp.y, wp.z);
		}
		if (_parent) {
			const VECTOR pp = _parent->transform.WorldPosition();
			DrawFormatString(10, 140, GetColor(220, 220, 220), "親 World  : (%.2f, %.2f, %.2f)", pp.x, pp.y, pp.z);
		}
	}

private:
	void UpdateParentMotion_() {
		if (!_parent) return;
		const VECTOR parentPos = VGet(std::sin(_driveTime * 0.6f) * 1.2f, 1.2f, std::cos(_driveTime * 0.4f) * 0.8f);
		_parent->transform.SetLocalPosition(parentPos);
		_parent->transform.SetLocalEulerRad(VGet(0.0f, _driveTime * 1.1f, 0.0f));
	}

	static TransformDebugClass* SpawnTransformObject_(const VariantMap& params) {
		return dynamic_cast<TransformDebugClass*>(ObjectManager::Instance().Spawn(TransformDebugClass::StaticPoolKey(), params));
	}

	static void DrawGridFloor_(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -halfCells * step), VGet(x, y, halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-halfCells * step, y, z), VGet(halfCells * step, y, z), colGrid);
		}
		DrawLine3D(VGet(0, y, 0), VGet(2, y, 0), GetColor(255, 80, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y + 2, 0), GetColor(80, 255, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y, 2), GetColor(80, 80, 255));
	}

private:
	CameraController _cameraController;
	CameraController::CameraId _cameraId = 0;
	TransformDebugClass* _parent = nullptr;
	TransformDebugClass* _child = nullptr;
	TransformDebugClass* _worldMarker = nullptr;
	VECTOR _childLocalOffset = VGet(2.0f, 0.6f, 0.0f);
	float _driveTime = 0.0f;
	bool _isAttached = true;
};