#pragma once

#include <deque>
#include <memory>
#include <string>

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "ObjectPoolDebugClass.h"
#include "SceneManager.h"
#include "SceneTpl.h"
#include "SceneTransition.h"
#include "TitleScene.h"

class ObjectPoolScene : public SceneTpl<ObjectPoolScene> {
public:
	static std::string StaticName() { return "ObjectPoolScene"; }

	void Start() override {
		auto& cameraManager = CameraManager::Instance();
		const int sceneId = SceneManager::Instance().CurrentSceneId();

		static bool s_registered = false;
		if (!s_registered) {
			ObjectFactory::Instance().RegisterCreator(ObjectPoolDebugClass::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<ObjectPoolDebugClass>(); });
			ObjectManager::Instance().RegisterPool(ObjectPoolDebugClass::StaticPoolKey(), 32);
			s_registered = true;
		}

		_liveBalls.clear();
		_spawnAccumSec = 0.0f;
		_spawnSerial = 0;

		if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
			_cameraId = _cameraController.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 4.5f, -16.0f), VGet(0.10f, 0.0f, 0.0f));
		}
		cameraManager.SetRender(_cameraId);
	}

	void End() override {
		for (auto* ball : _liveBalls) {
			if (ball) ObjectManager::Instance().Release(ball);
		}
		_liveBalls.clear();
	}

	void Update(float dtSec) override {
		ObjectManager::Instance().UpdateAll(dtSec);
		ReleaseExpiredBalls_();

		_cameraController.SetCamera(_cameraId);
		_cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

		_spawnAccumSec += dtSec;
		while (_spawnAccumSec >= _autoSpawnIntervalSec) {
			_spawnAccumSec -= _autoSpawnIntervalSec;
			SpawnBall_();
		}

		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F)) {
			for (int i = 0; i < 6; ++i) {
				SpawnBall_();
			}
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
			SceneManager::Instance().RequestChange(std::make_unique<ObjectPoolScene>());
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
		DrawGridFloor_(0.0f, 14, 1.0f);
		ObjectManager::Instance().DrawAll();

		DrawString(10, 10, "ObjectPoolScene - R:リセット T:タイトル", GetColor(255, 255, 255));
		DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
		DrawString(10, 50, "一定間隔で球を生成し、数秒後にプールへ戻す", GetColor(255, 220, 140));
		DrawString(10, 70, "F : 球をまとめて追加生成", GetColor(255, 220, 140));
		DrawString(10, 90, "右下の ObjectManager Debug で 取得数 / 生成数 / 未使用ストック を確認", GetColor(180, 255, 180));
		DrawFormatString(10, 110, GetColor(220, 220, 220), "現在の球数: %d", (int)_liveBalls.size());
	}

private:
	void SpawnBall_() {
		const float lane = static_cast<float>((_spawnSerial % 7) - 3);
		const float x = lane * 0.7f;
		const float z = 2.5f + static_cast<float>((_spawnSerial % 3) * 0.7f);
		const float vx = lane * 0.15f;
		const float vy = 4.8f + static_cast<float>((_spawnSerial % 4) * 0.35f);
		const float vz = 1.8f + static_cast<float>((_spawnSerial % 5) * 0.15f);
		const unsigned int color = (_spawnSerial % 2 == 0) ? GetColor(255, 220, 120) : GetColor(120, 220, 255);

		auto* ball = dynamic_cast<ObjectPoolDebugClass*>(ObjectManager::Instance().Spawn(
			ObjectPoolDebugClass::StaticPoolKey(),
			{
				{"px", std::to_string(x)},
				{"py", "1.0"},
				{"pz", std::to_string(z)},
				{"vx", std::to_string(vx)},
				{"vy", std::to_string(vy)},
				{"vz", std::to_string(vz)},
				{"life", "3.0"},
				{"radius", "0.35"},
				{"gravity", "-2.8"},
				{"color", std::to_string(color)}
			}
		));
		if (ball) {
			_liveBalls.push_back(ball);
		}
		++_spawnSerial;
	}

	void ReleaseExpiredBalls_() {
		for (auto it = _liveBalls.begin(); it != _liveBalls.end();) {
			auto* ball = *it;
			if (!ball) {
				it = _liveBalls.erase(it);
				continue;
			}
			if (!ball->IsActive() || ball->IsExpired()) {
				ObjectManager::Instance().Release(ball);
				it = _liveBalls.erase(it);
				continue;
			}
			++it;
		}
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
	std::deque<ObjectPoolDebugClass*> _liveBalls;
	float _spawnAccumSec = 0.0f;
	int _spawnSerial = 0;
	static constexpr float _autoSpawnIntervalSec = 0.35f;
};