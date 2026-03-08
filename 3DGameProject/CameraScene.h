#pragma once

#include <memory>
#include <string>

#include "SceneTpl.h"
#include "CameraController.h"
#include "CameraDebugClass.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "TitleScene.h"

// CameraScene
// - カメラ挙動の確認用シーン
// - Free / Follow / Orbit / Top の切替とブレンドを試せる
class CameraScene : public SceneTpl<CameraScene> {
public:
	static std::string StaticName() { return "CameraScene"; }

	void Start() override {
		auto& cameraManager = CameraManager::Instance();
		const int sceneId = SceneManager::Instance().CurrentSceneId();

		static bool s_registered = false;
		if (!s_registered) {
			ObjectFactory::Instance().RegisterCreator(CameraDebugClass::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<CameraDebugClass>(); });
			ObjectManager::Instance().RegisterPool(CameraDebugClass::StaticPoolKey(), 16);
			s_registered = true;
		}

		_focusTarget = SpawnDemo_({
			{"motion", "circle"},
			{"px", "0.0"}, {"py", "1.5"}, {"pz", "0.0"},
			{"motionRadius", "3.0"}, {"speed", "0.9"},
			{"drawRadius", "0.75"},
			{"color", std::to_string(GetColor(255, 220, 120))}
		});
		_pathMarker = SpawnDemo_({
			{"motion", "bob"},
			{"px", "-6.0"}, {"py", "2.0"}, {"pz", "6.0"},
			{"motionRadius", "1.2"}, {"speed", "1.6"},
			{"drawRadius", "0.28"},
			{"color", std::to_string(GetColor(120, 220, 255))}
		});
		_staticMarker = SpawnDemo_({
			{"motion", "static"},
			{"px", "6.0"}, {"py", "1.5"}, {"pz", "-4.0"},
			{"drawRadius", "0.3"},
			{"color", std::to_string(GetColor(180, 255, 180))}
		});

		if (_freeCamId == 0 || cameraManager.Get(_freeCamId) == nullptr) {
			_freeCamId = _freeCamCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 4.5f, -14.0f), VGet(0.12f, 0.0f, 0.0f));
		}
		if (_followCamId == 0 || cameraManager.Get(_followCamId) == nullptr) {
			_followCamId = _followCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(-6.0f, 3.0f, -8.0f), VGet(0.0f, 0.0f, 0.0f));
		}
		if (_orbitCamId == 0 || cameraManager.Get(_orbitCamId) == nullptr) {
			_orbitCamId = _orbitCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(8.0f, 4.0f, 0.0f), VGet(0.0f, 0.0f, 0.0f));
		}
		if (_topCamId == 0 || cameraManager.Get(_topCamId) == nullptr) {
			_topCamId = _topCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 14.0f, 0.0f), VGet(DX_PI_F * 0.5f, 0.0f, 0.0f));
		}

		_currentCamId = _freeCamId;
		cameraManager.SetRender(_currentCamId);
	}

	void Update(float dtSec) override {
		ObjectManager::Instance().UpdateAll(dtSec);
		UpdateDemoCameras_(dtSec);

		_freeCamCtrl.SetCamera(_freeCamId);
		_freeCamCtrl.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {
			_useBlend = !_useBlend;
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
			SwitchRenderCamera_(_freeCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {
			SwitchRenderCamera_(_followCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_3)) {
			SwitchRenderCamera_(_orbitCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_4)) {
			SwitchRenderCamera_(_topCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
			SceneManager::Instance().RequestChange(std::make_unique<CameraScene>());
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {
			SceneTransition::Params p;
			p.mode = SceneTransition::Mode::MaskImage;
			p.durationSec = 0.4;
			p.maskGraphPath = "Data/Transition/mask.png";
			p.pixelShaderPath = "Data/Transition/mask_transition.pso";
			SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p, 0.5f);
		}
	}

	void Draw() override {
		DrawGridFloor_(0.0f, 12, 1.0f);
		ObjectManager::Instance().DrawAll();

		DrawString(10, 10, "CameraScene - T:タイトル R:リセット", GetColor(255, 255, 255));
		DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
		DrawString(10, 50, "1:フリー  説明: 自由移動して全体を確認", GetColor(255, 220, 140));
		DrawString(10, 70, "2:追従    説明: メイン対象を後方から追う", GetColor(255, 220, 140));
		DrawString(10, 90, "3:周回    説明: メイン対象の周囲を回る", GetColor(255, 220, 140));
		DrawString(10, 110, "4:俯瞰    説明: 上空から全体配置を見る", GetColor(255, 220, 140));
		DrawString(10, 130, "B : ブレンド ON/OFF", GetColor(180, 255, 180));
		DrawString(10, 150, "中央付近を動く球体がメインのカメラ対象", GetColor(255, 220, 120));
	}

private:
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

	CameraDebugClass* SpawnDemo_(const VariantMap& params) {
		return dynamic_cast<CameraDebugClass*>(ObjectManager::Instance().Spawn(CameraDebugClass::StaticPoolKey(), params));
	}

	void SwitchRenderCamera_(CameraController::CameraId targetId) {
		_currentCamId = targetId;
		if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
		else CameraManager::Instance().SetRender(_currentCamId);
	}

	void UpdateDemoCameras_(float dtSec) {
		auto& cameraManager = CameraManager::Instance();
		const VECTOR target = _focusTarget ? _focusTarget->transform.LocalPosition() : VGet(0.0f, 1.5f, 0.0f);
		const VECTOR staticRef = _staticMarker ? _staticMarker->transform.LocalPosition() : VGet(6.0f, 1.5f, -4.0f);

		if (Camera* cam = cameraManager.Get(_followCamId)) {
			const VECTOR eye = VAdd(target, VGet(-6.0f, 2.8f, -6.0f));
			cam->LookAt(eye, target, VGet(0, 1, 0));
		}
		if (Camera* cam = cameraManager.Get(_orbitCamId)) {
			_orbitAngle += dtSec * 0.8f;
			const VECTOR eye = VAdd(target, VGet(std::cos(_orbitAngle) * 8.0f, 3.8f, std::sin(_orbitAngle) * 8.0f));
			cam->LookAt(eye, target, VGet(0, 1, 0));
		}
		if (Camera* cam = cameraManager.Get(_topCamId)) {
			const VECTOR eye = VAdd(target, VGet(0.0f, 14.0f, 0.0f));
			cam->LookAt(eye, VScale(VAdd(target, staticRef), 0.5f), VGet(0, 0, 1));
		}
		if (Camera* cam = cameraManager.Get(_freeCamId)) {
			cam->MarkDirty();
		}
	}

private:
	CameraController _freeCamCtrl;
	CameraController _followCamCtrl;
	CameraController _orbitCamCtrl;
	CameraController _topCamCtrl;
	CameraController::CameraId _freeCamId = 0;
	CameraController::CameraId _followCamId = 0;
	CameraController::CameraId _orbitCamId = 0;
	CameraController::CameraId _topCamId = 0;
	CameraController::CameraId _currentCamId = 0;
	CameraDebugClass* _focusTarget = nullptr;
	CameraDebugClass* _pathMarker = nullptr;
	CameraDebugClass* _staticMarker = nullptr;
	bool _useBlend = true;
	float _blendSec = 0.5f;
	float _orbitAngle = 0.0f;
};