#include "PachinkoGameStage.h"

#include "PachinkoBall.h"
#include "PachinkoNail.h"
#include "PachinkoField.h"
#include "PachinkoGameMenu.h"
#include "PachinkoSensor.h"

#include <random>

#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneTransition.h"
#include "SceneManager.h"
#include "CameraManager.h"
#include "KeyInput.h"
#include "DxLib.h"

namespace {
	// シーン遷移開始
	void StartTransition(std::unique_ptr<IScene> next) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::move(next), p, 0.5f);
	}
	// 床グリッド描画
	void DrawGridFloor(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -(float)halfCells * step), VGet(x, y, (float)halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-(float)halfCells * step, y, z), VGet((float)halfCells * step, y, z), colGrid);
		}
	}

	float RandRange_(float minValue, float maxValue) {
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> dist(minValue, maxValue);
		return dist(rng);
	}

	GameObject* SpawnMetalBall_() {
		const float x = RandRange_(-2.4f, 2.4f);
		const float z = 1.0f + RandRange_(-0.08f, 0.08f);

		const float vx = RandRange_(-0.25f, 0.25f);
		const float vy = RandRange_(-0.05f, 0.05f);
		const float vz = RandRange_(-0.20f, 0.20f);

		const float avx = RandRange_(-0.8f, 0.8f);
		const float avy = RandRange_(-0.8f, 0.8f);
		const float avz = RandRange_(-0.8f, 0.8f);

		return ObjectManager::Instance().Spawn(PachinkoBall::StaticPoolKey(), {
			{"px", std::to_string(x)},
			{"py", "13.0"},
			{"pz", std::to_string(z)},
			{"vx", std::to_string(vx)},
			{"vy", std::to_string(vy)},
			{"vz", std::to_string(vz)},
			{"avx", std::to_string(avx)},
			{"avy", std::to_string(avy)},
			{"avz", std::to_string(avz)},
			{"freezeRotation", "0"}
		});
	}
	// PachinkoField_Front / Back / Side のプール登録を確実に行う
	void EnsurePachinkoFieldRegistered_() {
		auto& factory = ObjectFactory::Instance();
		auto& objMgr = ObjectManager::Instance();

		if (!factory.IsRegistered(PachinkoField_Front::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoField_Front::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoField_Front>(); });
		}
		if (!factory.IsRegistered(PachinkoField_Back::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoField_Back::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoField_Back>(); });
		}
		if (!factory.IsRegistered(PachinkoField_Side::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoField_Side::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoField_Side>(); });
		}
		if (!factory.IsRegistered(PachinkoBall::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoBall::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoBall>(); });
		}
		if (!factory.IsRegistered(PachinkoNail::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoNail::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoNail>(); });
		}
		if (!factory.IsRegistered(PachinkoSensor::StaticPoolKey())) {
			factory.RegisterCreator(PachinkoSensor::StaticPoolKey(),
				[](const VariantMap&) { return std::make_unique<PachinkoSensor>(); });
		}

		objMgr.RegisterPool(PachinkoField_Front::StaticPoolKey(), 8);
		objMgr.RegisterPool(PachinkoField_Back::StaticPoolKey(), 8);
		objMgr.RegisterPool(PachinkoField_Side::StaticPoolKey(), 16);
		objMgr.RegisterPool(PachinkoBall::StaticPoolKey(), 128);
		objMgr.RegisterPool(PachinkoNail::StaticPoolKey(), 256);
		objMgr.RegisterPool(PachinkoSensor::StaticPoolKey(), 16);
	}

	void SpawnNails_() {
		constexpr int rows = 9;
		constexpr int cols = 11;
		const float startY = 3.2f;
		const float stepY = 0.85f;
		const float stepX = 0.68f;
		constexpr float kNailRotX = DX_PI_F * 0.5f; // 既存姿勢から +90度
		constexpr float kNailHalfHeight = 0.90f;    // Front/Back に届く長さへ延長
		for (int r = 0; r < rows; ++r) {
			const float y = startY + stepY * static_cast<float>(r);
			const float xOffset = (r % 2 == 0) ? 0.0f : (stepX * 0.5f);
			for (int c = 0; c < cols; ++c) {
				const float x = (static_cast<float>(c) - (cols - 1) * 0.5f) * stepX + xOffset;
				ObjectManager::Instance().Spawn(PachinkoNail::StaticPoolKey(), {
					{"px", std::to_string(x)},
					{"py", std::to_string(y)},
					{"pz", "1.0"},
					{"rx", std::to_string(kNailRotX)},
					{"halfHeight", std::to_string(kNailHalfHeight)}
				});
			}
		}
	}
} // namespace

// センサーを配置してメンバーリストに追加
void PachinkoGame_StageScene::SpawnSensors_() {
	// フィールド底部に3つの入賞ポケットを並べる
	// X 位置: 左(-2.8) / 中央(0.0) / 右(+2.8)
	// Y: フィールド最下部 (y = 0.15 = 床の直上)
	// Z: フィールド中央 (z = 1.0)
	struct SensorDef { float x; int score; const char* name; };
	static constexpr SensorDef kDefs[] = {
		{ -2.8f, 100, "Left"   },
		{  0.0f, 300, "Center" },
		{  2.8f, 100, "Right"  },
	};
	static constexpr unsigned int kColors[] = {
		0, // Left  : 赤
		0, // Center: 金
		0, // Right  : 赤
	};
	const unsigned int colors[] = {
		GetColor(255, 80, 80),
		GetColor(255, 220, 0),
		GetColor(255, 80, 80),
	};

	for (int i = 0; i < 3; ++i) {
		const SensorDef& def = kDefs[i];
		GameObject* obj = ObjectManager::Instance().Spawn(PachinkoSensor::StaticPoolKey(), {
			{"px",    std::to_string(def.x)},
			{"py",    "0.15"},
			{"pz",    "1.0"},
			{"hx",    "1.2"},
			{"hy",    "0.15"},
			{"hz",    "1.0"},
			{"color", std::to_string(colors[i])},
			{"name",  def.name},
		});

		if (auto* sensor = static_cast<PachinkoSensor*>(obj)) {
			const int score = def.score;
			sensor->onHit = [this, score](Collider*) {
				_totalScore += score;
			};
			_sensors.push_back(sensor);
		}
	}
}

// メイン開始
void PachinkoGame_StageScene::Start() {
	_returningToMenu = false;
	_freeCameraMode = false;
	_ballCount = 0;
	_activeBallCount = 0;
	_liveBalls.clear();
	_ballTracks.clear();
	_sensors.clear();
	_totalScore = 0;

	EnsurePachinkoFieldRegistered_();

	// 縦長の長方形パチンコ枠
	// Back: 白い実壁
	ObjectManager::Instance().Spawn(PachinkoField_Back::StaticPoolKey(), {
		{"px", "0.0"}, {"py", "7.0"}, {"pz", "0.0"},
		{"hx", "4.2"}, {"hy", "7.0"}, {"hz", "0.08"},
		{"color", std::to_string(GetColor(255, 255, 255))},
		{"material", "frictionless"}
	});

	// Front: 透明板イメージ（AABB線のみ）
	ObjectManager::Instance().Spawn(PachinkoField_Front::StaticPoolKey(), {
		{"px", "0.0"}, {"py", "7.0"}, {"pz", "2.0"},
		{"hx", "4.2"}, {"hy", "7.0"}, {"hz", "0.08"},
		{"color", std::to_string(GetColor(120, 200, 255))},
		{"material", "frictionless"}
	});

	// Side(L/R): 透明板イメージ（AABB線のみ）
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
		{"px", "-4.28"}, {"py", "7.0"}, {"pz", "1.0"},
		{"hx", "0.08"}, {"hy", "7.0"}, {"hz", "1.9"},
		{"color", std::to_string(GetColor(120, 200, 255))},
		{"material", "frictionless"}
	});
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
		{"px", "4.28"}, {"py", "7.0"}, {"pz", "1.0"},
		{"hx", "0.08"}, {"hy", "7.0"}, {"hz", "1.9"},
		{"color", std::to_string(GetColor(120, 200, 255))},
		{"material", "frictionless"}
	});

	// パチンコ釘（鉄カプセル）を配置
	SpawnNails_();

	// 入賞判定センサーを配置
	SpawnSensors_();

	// 初期の鉄球を1つ配置
	if (GameObject* ball = SpawnMetalBall_()) {
		_liveBalls.push_back(ball);
		BallTrack t; t.ball = ball; t.lastSnapPos = ball->transform.WorldPosition();
		_ballTracks.push_back(t);
		++_ballCount;
	}

	// フロントオブジェクトの後ろ側(内側)に固定カメラを配置
	CameraManager& camMgr = CameraManager::Instance();
	const int sceneId = SceneManager::Instance().CurrentSceneId();
	_cameraId = camMgr.CreateCamera(sceneId);
	if (Camera* cam = camMgr.Get(_cameraId)) { // カメラの初期位置を固定カメラ位置に設定
		cam->transform.SetLocalPosition(_fixedCameraEye);
		cam->LookAt(_fixedCameraEye, _fixedCameraTarget, VGet(0.0f, 1.0f, 0.0f));
	}
	camMgr.SetActive(_cameraId);
	camMgr.SetRender(_cameraId);
	_cameraController.SetCamera(_cameraId);
}

// メイン更新
void PachinkoGame_StageScene::Update(float dtSec) {
	if (_returningToMenu) return;

	ObjectManager::Instance().UpdateAll(dtSec);

	// 古い弾 or 画面下(y<-40)に落ちた弾 or スタック弾を Pool に返却
	constexpr size_t kMaxLiveBalls = 96;
	auto& om = ObjectManager::Instance();

	auto it  = _liveBalls.begin();
	auto tit = _ballTracks.begin();
	while (it != _liveBalls.end()) {
		GameObject* ball = *it;
		bool remove = false;

		if (!ball || !ball->IsActive() || ball->transform.WorldPosition().y < -40.0f) {
			remove = true;
		} else {
			// スタック判定（位置変化量チェック）
			BallTrack& track = *tit;
			track.accumSec += dtSec;
			if (track.accumSec >= kStuckSnapInterval) {
				track.accumSec -= kStuckSnapInterval;
				const VECTOR cur = ball->transform.WorldPosition();
				const VECTOR diff = VSub(cur, track.lastSnapPos);
				const float dist = VSize(diff);
				if (dist < kStuckMinDisplacement) {
					++track.stuckCount;
					if (track.stuckCount >= kStuckCountThreshold) {
						// スタック確定 → 強制 Release
						remove = true;
					}
				} else {
					track.stuckCount = 0;
				}
				track.lastSnapPos = cur;
			}
		}

		if (remove) {
			if (ball && ball->IsActive()) om.Release(ball);
			it  = _liveBalls.erase(it);
			tit = _ballTracks.erase(tit);
		} else {
			++it;
			++tit;
		}
	}

	while (_liveBalls.size() > kMaxLiveBalls) {
		GameObject* old = _liveBalls.front();
		_liveBalls.pop_front();
		_ballTracks.pop_front();
		if (old && old->IsActive()) om.Release(old);
	}
	_activeBallCount = static_cast<int>(_liveBalls.size());

#ifdef _DEBUG	// チートモード切替
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F6)) {
		_isCheatMode = !_isCheatMode;
	}
#endif // _DEBUG

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F1)) {
		_freeCameraMode = !_freeCameraMode;
		if (!_freeCameraMode) {
			if (Camera* cam = CameraManager::Instance().Get(_cameraId)) {
				cam->transform.SetLocalPosition(_fixedCameraEye);
				cam->LookAt(_fixedCameraEye, _fixedCameraTarget, VGet(0.0f, 1.0f, 0.0f));
			}
		}
	}

	if (_freeCameraMode) {
		// 右ドラッグで視点回転、WASD/EQで移動、ホイールで前後移動
		_cameraController.UpdateFreeMoveMouse(8.0f, 1.8f, 1.5f, dtSec);
	}

#ifdef _DEBUG
	// チートモード中に 1 キーで鉄球生成
	if (_isCheatMode && KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
		if (GameObject* ball = SpawnMetalBall_()) {
			_liveBalls.push_back(ball);
			BallTrack t; t.ball = ball; t.lastSnapPos = ball->transform.WorldPosition();
			_ballTracks.push_back(t);
			++_ballCount;
			_activeBallCount = static_cast<int>(_liveBalls.size());
		}
	}
#endif

	// 鉄球追加（スペースキー）
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		if (GameObject* ball = SpawnMetalBall_()) {
			_liveBalls.push_back(ball);
			BallTrack t; t.ball = ball; t.lastSnapPos = ball->transform.WorldPosition();
			_ballTracks.push_back(t);
			++_ballCount;
			_activeBallCount = static_cast<int>(_liveBalls.size());
		}
	}

	// メニューへ戻る（Escキー）
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
		_returningToMenu = true;
		StartTransition(std::make_unique<PachinkoGame_MenuScene>());
	}
}

// メイン描画
void PachinkoGame_StageScene::Draw() {
	// 床グリッド描画
	DrawGridFloor(0.0f, 30, 1.0f);

	// オブジェクト描画
	ObjectManager::Instance().DrawAll();

	// UI描画
	DrawString(10, 10, "パチンコステージ", GetColor(255, 255, 120));
	DrawString(10, 30, "前面/側面: 透明(AABB表示)", GetColor(180, 220, 255));
	DrawString(10, 50, "後面: 白い壁", GetColor(255, 255, 255));
	DrawFormatString(10, 70, GetColor(180, 255, 180), "F1: フリー移動カメラ [%s]", _freeCameraMode ? "ON" : "OFF");
	DrawString(10, 90, "Space: 鉄球を追加", GetColor(180, 255, 180));
	DrawString(10, 110, "Esc: メニューへ戻る", GetColor(180, 255, 180));
	DrawFormatString(10, 130, GetColor(220, 220, 220), "アクティブ鉄球数: %d", _activeBallCount);

	// センサー入賞情報
	DrawFormatString(10, 150, GetColor(255, 220, 50), "スコア: %d", _totalScore);
	for (int i = 0; i < static_cast<int>(_sensors.size()); ++i) {
		const PachinkoSensor* s = _sensors[i];
		DrawFormatString(10, 170 + i * 18, GetColor(180, 255, 180),
			"[%s] 入賞: %d 回", s->GetSensorName().c_str(), s->GetHitCount());
	}

	// カメラ座標と視線方向の表示
	if (Camera* cam = CameraManager::Instance().Get(_cameraId)) {
		const VECTOR p = cam->transform.WorldPosition();
		const VECTOR f = cam->transform.Forward();
		if (_freeCameraMode) {
			DrawFormatString(10, 160, GetColor(255, 255, 255), "カメラ座標 XYZ: (%.3f, %.3f, %.3f)", p.x, p.y, p.z);
			DrawFormatString(10, 180, GetColor(255, 255, 255), "視線方向 XYZ: (%.3f, %.3f, %.3f)", f.x, f.y, f.z);
			DrawString(10, 205, "操作: 右ドラッグ回転 / WASD前後左右 / EQ上下 / ホイール前後", GetColor(200, 200, 255));
		}
	}

#ifdef _DEBUG
	// チートモード表示
	if (_isCheatMode) {
		DrawString(10, 230, "チートモード ON (1キーで鉄球生成)", GetColor(255, 100, 100));
	}
#endif // _DEBUG
}

// メイン終了
void PachinkoGame_StageScene::End() {
	if (_cameraId != 0) {
		CameraManager::Instance().DestroyCamera(_cameraId);
		_cameraId = 0;
	}
	_liveBalls.clear();
	_ballTracks.clear();
	_sensors.clear();
	ObjectManager::Instance().ReleaseBySceneId(SceneManager::Instance().CurrentSceneId());
}
