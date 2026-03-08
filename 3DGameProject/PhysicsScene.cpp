#include "PhysicsScene.h"

#include <memory>
#include <string>
#include <deque>

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "TitleScene.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "PhysicsDebugClass.h"
#include "SceneManager.h"
#include "SceneTransition.h"

namespace {
	CameraController _cameraController;
	CameraController::CameraId _cameraId = 0;
	bool _registered = false;
	bool _isSpawningArena = false;

	constexpr size_t _maxDynamicBoxCount = 20;
	constexpr size_t _maxDynamicSphereCount = 30;
	constexpr size_t _maxDynamicCapsuleCount = 20;

	std::deque<PhysicsDebugClass*> _dynamicBoxes;
	std::deque<PhysicsDebugClass*> _dynamicSpheres;
	std::deque<PhysicsDebugClass*> _dynamicCapsules;

	void ReleaseOldestIfNeeded_(std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
		while (objects.size() >= maxCount && !objects.empty()) {
			PhysicsDebugClass* oldest = objects.front();
			objects.pop_front();
			if (!oldest) continue;
			ObjectManager::Instance().Release(oldest);
		}
	}

	void RegisterDynamicObject_(PhysicsDebugClass* obj, std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
		if (!obj) return;
		if (_isSpawningArena) return;
		ReleaseOldestIfNeeded_(objects, maxCount);
		objects.push_back(obj);
	}

	void ClearDynamicTracking_() {
		_dynamicBoxes.clear();
		_dynamicSpheres.clear();
		_dynamicCapsules.clear();
	}

	PhysicsDebugClass* SpawnPhysicsObject(const std::string& key, const VariantMap& params) {
		return dynamic_cast<PhysicsDebugClass*>(ObjectManager::Instance().Spawn(key, params));
	}

	PhysicsDebugClass* SpawnPhysicsBox(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugBox::StaticPoolKey(), params);
		RegisterDynamicObject_(obj, _dynamicBoxes, _maxDynamicBoxCount);
		return obj;
	}

	PhysicsDebugClass* SpawnPhysicsSphere(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugSphere::StaticPoolKey(), params);
		RegisterDynamicObject_(obj, _dynamicSpheres, _maxDynamicSphereCount);
		return obj;
	}

	PhysicsDebugClass* SpawnPhysicsCapsule(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugCapsule::StaticPoolKey(), params);
		RegisterDynamicObject_(obj, _dynamicCapsules, _maxDynamicCapsuleCount);
		return obj;
	}

	// 体験用の床・壁・斜面・積みオブジェクト群を配置する
	void SpawnArena() {
		_isSpawningArena = true;

		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0"}, {"py", "-1.0"}, {"pz", "0"},
			{"hx", "18.0"}, {"hy", "1.0"}, {"hz", "18.0"},
			{"color", std::to_string(GetColor(120, 220, 255))}
		});

		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0"}, {"py", "4.0"}, {"pz", "18.0"},
			{"hx", "18.0"}, {"hy", "5.0"}, {"hz", "1.0"}
		});
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0"}, {"py", "4.0"}, {"pz", "-18.0"},
			{"hx", "18.0"}, {"hy", "5.0"}, {"hz", "1.0"}
		});
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "18.0"}, {"py", "4.0"}, {"pz", "0"},
			{"hx", "1.0"}, {"hy", "5.0"}, {"hz", "18.0"}
		});
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-18.0"}, {"py", "4.0"}, {"pz", "0"},
			{"hx", "1.0"}, {"hy", "5.0"}, {"hz", "18.0"}
		});

		auto* ramp = SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-8.0"}, {"py", "0.2"}, {"pz", "-6.0"},
			{"hx", "3.0"}, {"hy", "0.4"}, {"hz", "6.0"},
			{"friction", "0.0"},
			{"restitution", "0.0"},
			{"color", std::to_string(GetColor(255, 210, 120))}
		});
		if (ramp) {
			ramp->transform.SetLocalEulerRad(VGet(0.0f, 0.35f, -0.45f));
		}

		for (int y = 0; y < 4; ++y) {
			for (int x = 0; x < 4; ++x) {
				SpawnPhysicsBox({
					{"px", std::to_string(-2.5f + x * 1.1f)},
					{"py", std::to_string(0.6f + y * 1.05f)},
					{"pz", "4.0"},
					{"hx", "0.45"}, {"hy", "0.45"}, {"hz", "0.45"},
					{"mass", "1.2"}, {"friction", "0.8"}, {"restitution", "0.05"}
				});
			}
		}

		for (int i = 0; i < 6; ++i) {
			SpawnPhysicsSphere({
				{"px", std::to_string(4.0f + i * 0.9f)},
				{"py", std::to_string(1.0f + i * 1.1f)},
				{"pz", "2.0"},
				{"radius", "0.45"},
				{"mass", "0.8"}, {"friction", "0.35"}, {"restitution", "0.55"},
				{"ccd", "true"}
			});
		}

		_isSpawningArena = false;
	}

	// カメラ前方に任意形状の物体を落とす
	void SpawnDropObject(int type) {
		auto* cam = CameraManager::Instance().Get(_cameraId);
		if (!cam) return;

		const VECTOR forward = cam->transform.Forward();
		const VECTOR eye = cam->transform.LocalPosition();
		const VECTOR spawnPos = VAdd(eye, VScale(forward, 3.0f));
		VariantMap params;
		params["px"] = std::to_string(spawnPos.x);
		params["py"] = std::to_string(spawnPos.y);
		params["pz"] = std::to_string(spawnPos.z);
		params["mass"] = "1.0";
		params["friction"] = "0.55";
		params["restitution"] = "0.18";
		params["ccd"] = "true";

		PhysicsDebugClass* obj = nullptr;
		if (type == 1) {
			params["hx"] = "0.5";
			params["hy"] = "0.5";
			params["hz"] = "0.5";
			obj = SpawnPhysicsBox(params);
		}
		else if (type == 2) {
			params["radius"] = "0.45";
			params["restitution"] = "0.6";
			obj = SpawnPhysicsSphere(params);
		}
		else {
			params["radius"] = "0.38";
			params["halfHeight"] = "0.75";
			obj = SpawnPhysicsCapsule(params);
		}

		if (obj) {
			obj->GetPhysicsBody()->_velocity = VScale(forward, 2.0f);
		}
	}

	// 前方へ高速な球を射出して CCD や反発を確認しやすくする
	void FireProjectile() {
		auto* cam = CameraManager::Instance().Get(_cameraId);
		if (!cam) return;

		const VECTOR forward = cam->transform.Forward();
		const VECTOR eye = cam->transform.LocalPosition();
		const VECTOR spawnPos = VAdd(eye, VScale(forward, 2.0f));
		auto* obj = SpawnPhysicsSphere({
			{"px", std::to_string(spawnPos.x)},
			{"py", std::to_string(spawnPos.y)},
			{"pz", std::to_string(spawnPos.z)},
			{"radius", "0.35"},
			{"mass", "0.7"},
			{"friction", "0.25"},
			{"restitution", "0.65"},
			{"ccd", "true"},
			{"ccdThreshold", "12.0"},
			{"color", std::to_string(GetColor(255, 140, 140))}
		});
		if (obj) {
			obj->GetPhysicsBody()->AddImpulse(VScale(forward, 22.0f));
		}
	}
}

void PhysicsScene::Start() {
	auto& cameraManager = CameraManager::Instance();
	const int sceneId = SceneManager::Instance().CurrentSceneId();
	ClearDynamicTracking_();

	if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
		_cameraId = _cameraController.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 6.0f, -18.0f), VGet(0.18f, 0.0f, 0.0f));
	}
	cameraManager.SetRender(_cameraId);

	if (!_registered) {
		ObjectFactory::Instance().RegisterCreator(PhysicsDebugBox::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugBox>(); });
		ObjectFactory::Instance().RegisterCreator(PhysicsDebugSphere::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugSphere>(); });
		ObjectFactory::Instance().RegisterCreator(PhysicsDebugCapsule::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugCapsule>(); });
		ObjectManager::Instance().RegisterPool(PhysicsDebugBox::StaticPoolKey(), 128);
		ObjectManager::Instance().RegisterPool(PhysicsDebugSphere::StaticPoolKey(), 128);
		ObjectManager::Instance().RegisterPool(PhysicsDebugCapsule::StaticPoolKey(), 128);
		_registered = true;
	}

	SpawnArena();
}

void PhysicsScene::Update(float dtSec) {
	ObjectManager::Instance().UpdateAll(dtSec);

	_cameraController.SetCamera(_cameraId);
	_cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
		SpawnDropObject(1);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {
		SpawnDropObject(2);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_3)) {
		SpawnDropObject(3);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F)) {
		FireProjectile();
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
		SceneManager::Instance().RequestChange(std::make_unique<PhysicsScene>());
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {
		SceneTransition::Params params;
		params.mode = SceneTransition::Mode::MaskImage;
		params.durationSec = 0.4;
		params.maskGraphPath = "Data/Transition/mask.png";
		params.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<TitleScene>(), params, 0.5f);
	}
}

void PhysicsScene::Draw() {
	ObjectManager::Instance().DrawAll();

	DrawString(10, 10, "PhysicsScene - R:リセット T:タイトル", GetColor(255, 255, 255));
	DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
	DrawString(10, 50, "1:箱 2:球 3:カプセル を前方に落とす", GetColor(255, 220, 140));
	DrawString(10, 70, "F : 球体を前方へ発射", GetColor(255, 180, 180));
	DrawString(10, 90, "球:30 箱:20 カプセル:20 / 古いものからプールへ戻す", GetColor(180, 255, 180));
}