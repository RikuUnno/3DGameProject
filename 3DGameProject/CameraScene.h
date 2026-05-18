#pragma once

#include <memory>
#include <string>
#include <cstdlib>
#include <array>

#include "SceneTpl.h"
#include "GameObject.h"
#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "TitleScene.h"

// CameraModelObject - モデル表示用のシンプルな GameObject
class CameraModelObject : public GameObject {
public:
	static std::string StaticPoolKey() { return "CameraModelObject"; }

	bool IsModelLoaded() const noexcept { return _modelHandle >= 0; }
	const std::string& LastTriedPath() const noexcept { return _lastTriedPath; }

	// モデルの読み込みと描画のみを行うシンプルなオブジェクト
	~CameraModelObject() override {
		if (_modelHandle >= 0) {
			MV1DeleteModel(_modelHandle);
			_modelHandle = -1;
		}
	}

	// 再利用時の初期化
	void OnAcquire(const VariantMap& params) override {
		_ownerSceneId = SceneManager::Instance().CurrentSceneId();
		SetActive(true);
		isStatic = true;
		transform.SetParent(nullptr);
		ConfigureFromParams_(params);
		LoadModelIfNeeded_();
	}

	// 解放時のクリーンアップ
	void OnRelease() override {
		transform.SetParent(nullptr);
		SetActive(false);
	}

	// ?フレームの描画
	void Draw() override {
		if (_modelHandle < 0) return;
		MV1SetPosition(_modelHandle, transform.WorldPosition());		// ワールド座標で配置
		MV1SetRotationXYZ(_modelHandle, transform.LocalEulerRad());		// ローカル回転をそのまま適用（親がいない前提）
		MV1SetScale(_modelHandle, transform.LocalScale());				// ローカルスケールをそのまま適用（親がいない前提）
		MV1DrawModel(_modelHandle);										// 描画
	}

private:
	// パラメータから状態を設定するヘルパー
	void ConfigureFromParams_(const VariantMap& params) {
		auto f = [&](const char* key, float def) {
			auto it = params.find(key);
			return (it == params.end()) ? def : static_cast<float>(std::atof(it->second.c_str()));
		};
		auto s = [&](const char* key, const char* def) {
			auto it = params.find(key);
			return (it == params.end()) ? std::string(def) : it->second;
		};

		_modelPath = s("modelPath", "");
		transform.SetLocalPosition(VGet(f("px", 0.0f), f("py", 0.0f), f("pz", 0.0f)));
		transform.SetLocalEulerRad(VGet(f("pitch", 0.0f), f("yaw", 0.0f), f("roll", 0.0f)));
		const float sc = f("scale", 1.0f);
		transform.SetLocalScale(VGet(f("sx", sc), f("sy", sc), f("sz", sc)));
	}

	// モデルの読み込み（必要な場合のみ）
	void LoadModelIfNeeded_() {
		if (_modelPath.empty()) return;
		if (_loadedPath == _modelPath && _modelHandle >= 0) return;
		if (_modelHandle >= 0) {
			MV1DeleteModel(_modelHandle);
			_modelHandle = -1;
		}

		// 実行フォルダ差異に強くするため、候補パスを順に試す。
		const std::array<std::string, 7> candidates = {
			_modelPath,
			std::string("./") + _modelPath,
			std::string("../") + _modelPath,
			std::string("3DGameProject/") + _modelPath,
			std::string("../3DGameProject/") + _modelPath,
			std::string("../../3DGameProject/") + _modelPath,
			std::string("C:/Users/rinsa/source/repos/3DGameProject/3DGameProject/") + _modelPath,
		};

		for (const auto& p : candidates) {
			_lastTriedPath = p;
			_modelHandle = MV1LoadModel(p.c_str());
			if (_modelHandle >= 0) {
				_loadedPath = _modelPath;
				return;
			}
		}
	}

private:
	int _modelHandle = -1;		// モデルハンドル（-1は未ロード）
	std::string _modelPath;		// ロードするモデルのパス
	std::string _loadedPath;	// 現在ロードされているモデルのパス（同じなら再ロードしないためのキャッシュ）
	std::string _lastTriedPath;
};

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
			ObjectFactory::Instance().RegisterCreator(CameraModelObject::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<CameraModelObject>(); });
			ObjectManager::Instance().RegisterPool(CameraModelObject::StaticPoolKey(), 4);
			s_registered = true;
		}

		// 旧デモ用のマーカーオブジェクトは生成しない

		// CameraModelObject はシーン所有にして、
		// シーン遷移直後のプール再利用タイミングの影響を受けないようにする。
		_ownedCircusModel = std::make_unique<CameraModelObject>();
		VariantMap circusParams{
			{"modelPath", "models/chicken-gun-fruzer-circus/source/circus.mv1"},	// モデルパス
			{"px", "13.5"}, {"py", "10.0"}, {"pz", "5.0"},							// 位置
			{"scale", "0.003"}														// スケール（モデルが大きいので小さめに）
		};
		_ownedCircusModel->OnAcquire(circusParams);
		_circusModel = _ownedCircusModel.get();

		// カメラ生成（既に生成されている場合は再利用）
		if (_freeCamId == 0 || cameraManager.Get(_freeCamId) == nullptr) {
			_freeCamId = _freeCamCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 5.0f, -10.0f), VGet(0.0f, 0.0f, 0.0f));
		}
		if (_followCamId == 0 || cameraManager.Get(_followCamId) == nullptr) {
			_followCamId = _followCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 0.0f, 0.0f), VGet(0.0f, 0.0f, 0.0f));
		}
		if (_orbitCamId == 0 || cameraManager.Get(_orbitCamId) == nullptr) {
			_orbitCamId = _orbitCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 3.0f, -10.0f), VGet(0.0f, 0.0f, 0.0f));
		}
		if (_topCamId == 0 || cameraManager.Get(_topCamId) == nullptr) {
			_topCamId = _topCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 7.0f, -14.0f), VGet(0.0f, 0.0f, 0.0f));
		}

		_cameraTime = 0.0f;
		_currentCamId = _freeCamId;
		cameraManager.SetRender(_currentCamId);
	}

	void Update(float dtSec) override {
		UpdateDemoCameras_(dtSec);
		UpdateCircusModelInput_(dtSec);

		// カメラ1はフリーカメラ
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

	// 描画
	void Draw() override {
		if (_ownedCircusModel && _ownedCircusModel->IsActive()) {
			_ownedCircusModel->Draw();
		}

		DrawString(10, 10, "CameraScene - T:タイトル R:リセット", GetColor(255, 255, 255));
		DrawString(10, 30, "1:フリーカメラ", GetColor(255, 220, 140));
		DrawString(10, 50, "2:位置(0,0,0)で視線回転(5秒/1周)", GetColor(255, 220, 140));
		DrawString(10, 70, "3:Offset10で原点を見る外周カメラ", GetColor(255, 220, 140));
		DrawString(10, 90, "4:俯瞰で原点周回しカメラ3を見る", GetColor(255, 220, 140));
		DrawString(10, 110, "B : ブレンド ON/OFF", GetColor(180, 255, 180));
		DrawString(10, 130, "CircusModel Move: Arrow(XZ) / PgUp PgDn(Y)", GetColor(180, 255, 180));
		if (_circusModel && !_circusModel->IsModelLoaded()) {
			DrawString(10, 150, "Circus model load failed. Check path: models/circus.mv1", GetColor(255, 120, 120));
		}
	}

private:
	// カメラモデルオブジェクトの位置をキーボード入力で動かす（デモ用）
	void UpdateCircusModelInput_(float dtSec) {
		if (!_circusModel) return;
		VECTOR p = _circusModel->transform.LocalPosition();
		const float spd = 4.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_LEFT))  p.x -= spd * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_RIGHT)) p.x += spd * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_UP))    p.z += spd * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_DOWN))  p.z -= spd * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_PGUP))  p.y += spd * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_PGDN))  p.y -= spd * dtSec;
		_circusModel->transform.SetLocalPosition(p);
	}

	// 描画カメラの切替（ブレンドあり/なし）
	void SwitchRenderCamera_(CameraController::CameraId targetId) {
		_currentCamId = targetId;
		if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
		else CameraManager::Instance().SetRender(_currentCamId);
	}

	// デモ用カメラの更新
	void UpdateDemoCameras_(float dtSec) {
		auto& cameraManager = CameraManager::Instance();
		const VECTOR origin = VGet(0.0f, 0.0f, 0.0f);

		const float period = 5.0f;
		const float omega = (2.0f * DX_PI_F) / period;
		_cameraTime += dtSec;

		// カメラ2: カメラ位置を原点(0,0,0)に固定し、視線を5秒で1周回転
		if (Camera* cam = cameraManager.Get(_followCamId)) {
			const float a = omega * _cameraTime;
			const VECTOR eye = origin;
			const VECTOR at = VGet(std::cos(a), 0.0f, std::sin(a));
			cam->LookAt(eye, at, VGet(0, 1, 0));
		}

		// カメラ3: Offset=10で原点を見る外周回転
		VECTOR cam3Eye = VGet(0.0f, 3.0f, -10.0f);
		if (Camera* cam = cameraManager.Get(_orbitCamId)) {
			const float a = omega * _cameraTime + DX_PI_F * 0.5f;
			cam3Eye = VGet(std::cos(a) * 10.0f, 3.0f, std::sin(a) * 10.0f);
			cam->LookAt(cam3Eye, origin, VGet(0, 1, 0));
		}

		// カメラ4: 俯瞰で原点を軸に周回し、カメラ3を見る
		if (Camera* cam = cameraManager.Get(_topCamId)) {
			const float a = omega * _cameraTime + DX_PI_F;
			const VECTOR eye = VGet(std::cos(a) * 16.0f, 18.0f, std::sin(a) * 16.0f);
			cam->LookAt(eye, cam3Eye, VGet(0, 1, 0));
		}

		// カメラ1（フリー）はコントローラ更新に任せる
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
	std::unique_ptr<CameraModelObject> _ownedCircusModel;
	CameraModelObject* _circusModel = nullptr;
	bool _useBlend = true;
	float _blendSec = 0.5f;
	float _cameraTime = 0.0f;
};
