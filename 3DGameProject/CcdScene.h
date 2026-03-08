#pragma once

#include <array>
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
#include "PhysicsDebugClass.h"
#include "SceneManager.h"
#include "SceneTpl.h"
#include "SceneTransition.h"
#include "TitleScene.h"

class CcdScene : public SceneTpl<CcdScene> {
public:
	static std::string StaticName() { return "CcdScene"; }

	void Start() override {
		auto& cameraManager = CameraManager::Instance();
		const int sceneId = SceneManager::Instance().CurrentSceneId();

		RegisterPools_();
		ClearProjectiles_();
		SpawnSetup_();
		_spawnAccumSec = 0.0f;

		if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
			_cameraId = _cameraController.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 22.0f, -18.0f), VGet(0.72f, 0.0f, 0.0f));
		}
		cameraManager.SetRender(_cameraId);
	}

	void End() override {
		ClearProjectiles_();
	}

	void Update(float dtSec) override {
		ObjectManager::Instance().UpdateAll(dtSec);
		ReleaseOldProjectiles_();

		_cameraController.SetCamera(_cameraId);
		_cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

		_spawnAccumSec += dtSec;
		if (_autoFire && _spawnAccumSec >= _autoFireIntervalSec) {
			_spawnAccumSec = 0.0f;
			FireAllLanes_();
		}

		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F)) FireAllLanes_();
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) FireLane_(0);
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) FireLane_(1);
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_3)) FireLane_(2);
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_4)) FireLane_(3);
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_A)) _autoFire = !_autoFire;
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) SceneManager::Instance().RequestChange(std::make_unique<CcdScene>());
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
		ObjectManager::Instance().DrawAll();

		DrawString(10, 10, "CcdScene - A:自動発射 F:全発射 R:リセット T:タイトル", GetColor(255, 255, 255));
		DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
		DrawString(10, 50, "1:CCD OFF  2:CCD ON  3:閾値低め  4:閾値高め", GetColor(255, 220, 140));
		DrawString(10, 70, _autoFire ? "自動発射 : ON" : "自動発射 : OFF", GetColor(180, 255, 180));
		DrawString(10, 90, "薄い壁に高速球を撃ち、すり抜け差を比較", GetColor(220, 220, 220));
	}

private:
	struct Lane {
		float z = 0.0f;
		const char* label = "";
		bool ccd = true;
		float threshold = 8.0f;
		unsigned int color = 0;
	};

	void RegisterPools_() {
		static bool s_registered = false;
		if (s_registered) return;
		ObjectFactory::Instance().RegisterCreator(PhysicsDebugBox::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugBox>(); });
		ObjectFactory::Instance().RegisterCreator(PhysicsDebugSphere::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugSphere>(); });
		ObjectManager::Instance().RegisterPool(PhysicsDebugBox::StaticPoolKey(), 96);
		ObjectManager::Instance().RegisterPool(PhysicsDebugSphere::StaticPoolKey(), 96);
		s_registered = true;
	}

	void SpawnSetup_() {
		SpawnStaticBox_(0.0f, -1.0f, 0.0f, 26.0f, 1.0f, 18.0f, GetColor(120, 220, 255));
		SpawnStaticBox_(0.0f, 3.0f, 18.0f, 26.0f, 4.0f, 1.0f, 0);
		SpawnStaticBox_(0.0f, 3.0f, -18.0f, 26.0f, 4.0f, 1.0f, 0);
		for (const auto& lane : _lanes) {
			SpawnStaticBox_(0.0f, 1.2f, lane.z, 0.02f, 1.4f, 2.4f, GetColor(255, 210, 120));
			SpawnStaticBox_(10.0f, 1.2f, lane.z, 0.8f, 1.6f, 2.6f, GetColor(120, 220, 255));
		}
	}

	void SpawnStaticBox_(float px, float py, float pz, float hx, float hy, float hz, unsigned int color) {
		VariantMap params = {
			{"static", "true"},
			{"px", std::to_string(px)}, {"py", std::to_string(py)}, {"pz", std::to_string(pz)},
			{"hx", std::to_string(hx)}, {"hy", std::to_string(hy)}, {"hz", std::to_string(hz)}
		};
		if (color != 0) {
			params["color"] = std::to_string(color);
		}
		ObjectManager::Instance().Spawn(PhysicsDebugBox::StaticPoolKey(), params);
	}

	void FireAllLanes_() {
		for (int i = 0; i < static_cast<int>(_lanes.size()); ++i) {
			FireLane_(i);
		}
	}

	void FireLane_(int index) {
		if (index < 0 || index >= static_cast<int>(_lanes.size())) return;
		const auto& lane = _lanes[index];
		auto* obj = dynamic_cast<PhysicsDebugClass*>(ObjectManager::Instance().Spawn(
			PhysicsDebugSphere::StaticPoolKey(),
			{
				{"px", "-12.0"}, {"py", "1.1"}, {"pz", std::to_string(lane.z)},
				{"radius", "0.20"},
				{"mass", "0.7"},
				{"friction", "0.05"},
				{"restitution", "0.05"},
				{"gravity", "false"},
				{"ccd", lane.ccd ? "true" : "false"},
				{"ccdThreshold", std::to_string(lane.threshold)},
				{"maxLinearSpeed", "240.0"},
				{"color", std::to_string(lane.color)}
			}
		));
		if (!obj) return;
		obj->GetPhysicsBody()->_velocity = VGet(120.0f, 0.0f, 0.0f);
		_trackers[index].push_back(obj);
		ReleaseOldestIfNeeded_(_trackers[index]);
	}

	void ReleaseOldestIfNeeded_(std::deque<PhysicsDebugClass*>& list) {
		while (list.size() > _maxPerLane) {
			auto* oldest = list.front();
			list.pop_front();
			if (oldest) ObjectManager::Instance().Release(oldest);
		}
	}

	void ReleaseOldProjectiles_() {
		for (auto& list : _trackers) {
			for (auto it = list.begin(); it != list.end();) {
				auto* obj = *it;
				if (!obj || !obj->IsActive()) {
					it = list.erase(it);
					continue;
				}
				const VECTOR p = obj->transform.LocalPosition();
				if (p.x > 18.0f || p.y < -3.0f) {
					ObjectManager::Instance().Release(obj);
					it = list.erase(it);
					continue;
				}
				++it;
			}
		}
	}

	void ClearProjectiles_() {
		for (auto& list : _trackers) {
			list.clear();
		}
	}

private:
	CameraController _cameraController;
	CameraController::CameraId _cameraId = 0;
	std::array<std::deque<PhysicsDebugClass*>, 4> _trackers{};
	float _spawnAccumSec = 0.0f;
	bool _autoFire = true;
	static constexpr float _autoFireIntervalSec = 1.2f;
	static constexpr size_t _maxPerLane = 8;
	const std::array<Lane, 4> _lanes{{
		{ -6.0f, "CCD OFF", false, 9999.0f, GetColor(255, 120, 120) },
		{ -2.0f, "CCD ON", true, 8.0f, GetColor(120, 255, 120) },
		{  2.0f, "閾値低め", false, 3.0f, GetColor(120, 220, 255) },
		{  6.0f, "閾値高め", false, 150.0f, GetColor(255, 220, 120) },
	}};
};