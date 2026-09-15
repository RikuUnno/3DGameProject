#include "PachinkoGameStage.h"

#include "PachinkoBall.h"
#include "PachinkoNail.h"
#include "PachinkoField.h"
#include "PachinkoGameMenu.h"
#include "PachinkoSensor.h"

#include <cmath>
#include <random>

#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneTransition.h"
#include "SceneManager.h"
#include "CameraManager.h"
#include "KeyInput.h"
#include "DxLib.h"

// PachinkoGameStage の自動登録
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

	// ランダム範囲生成（minValue <= x < maxValue）
	float RandRange(float minValue, float maxValue) {
		static std::mt19937 rng{ std::random_device{}() };
		std::uniform_real_distribution<float> dist(minValue, maxValue);
		return dist(rng);
	}

	// PachinkoField_Front / Back / Side のプール登録を確実に行う
	void EnsurePachinkoFieldRegistered() {
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

	// 釘を配置する
	void SpawnNails() {
		constexpr float kNailRotX = DX_PI_F * 0.5f; // 既存姿勢から +90度
		constexpr float kNailHalfHeight = 1.6f;    // Front/Back に届く長さへ延長
		auto spawnNail = [&](float x, float y) {
			ObjectManager::Instance().Spawn(PachinkoNail::StaticPoolKey(), {
				{"px", std::to_string(x)},
				{"py", std::to_string(y)},
				{"pz", "1.0"},
				{"rx", std::to_string(kNailRotX)},
				{"halfHeight", std::to_string(kNailHalfHeight)}
			});
		};

		// 縦に一括で釘を配置するヘルパー関数
		auto spawnNailsVertical = [&](float x, float yStart, float yEnd, float interval) {
			for (float y = yStart; y >= yEnd; y -= interval) {
				spawnNail(x, y);
			}
		};

		// 縦に一括で釘を配置するヘルパー関数（個数指定版）
		auto spawnNailsVerticalCount = [&](float x, float yStart, int count, float interval) {
			for (int i = 0; i < count; ++i) {
				spawnNail(x, yStart - static_cast<float>(i) * interval);
			}
		};

		// XYの2点間に釘を配置するヘルパー関数（間隔指定版）
		auto spawnNailsBetween = [&](float x1, float y1, float x2, float y2, float interval) {
			const float dx = x2 - x1;
			const float dy = y2 - y1;
			const float distance = std::sqrt(dx * dx + dy * dy);
			const int count = static_cast<int>(distance / interval) + 1;

			for (int i = 0; i < count; ++i) {
				const float t = static_cast<float>(i) / static_cast<float>(count - 1);
				const float x = x1 + dx * t;
				const float y = y1 + dy * t;
				spawnNail(x, y);
			}
		};

		// XYの2点間に釘を配置するヘルパー関数（個数指定版）
		auto spawnNailsBetweenCount = [&](float x1, float y1, float x2, float y2, int count) {
			const float dx = x2 - x1;
			const float dy = y2 - y1;

			for (int i = 0; i < count; ++i) {
				const float t = static_cast<float>(i) / static_cast<float>(count - 1);
				const float x = x1 + dx * t;
				const float y = y1 + dy * t;
				spawnNail(x, y);
			}
		};

		constexpr float kPi = DX_PI_F;

		// ボール生成位置（0, 12, 0）のちょっと下に釘を一本配置
		spawnNail(0.0f, 11.0f);

		// 中央液晶部を囲む連釘（半円 - 隙間なし）
		for (int i = 0; i <= 64; ++i) {  // 釘の数を17→65に大幅増加
			const float angle = kPi * static_cast<float>(i) / 64.0f;  // 0～πの範囲で均等配置
			const float x = 1.4f * std::cos(angle);
			const float y = 9.2f + 0.8f * std::sin(angle);
			spawnNail(x, y);
		}

		spawnNailsVerticalCount(3.4f, 8.0f, 8, 0.8f); // 左側の縦釘一列目
		spawnNailsVerticalCount(-3.4f, 8.0f, 8, 0.8f); // 右側の縦釘一列目

		spawnNailsVerticalCount(2.8f, 7.0f, 5, 0.6f); // 左側の縦釘二列目
		spawnNailsVerticalCount(-2.8f, 7.0f, 5, 0.6f); // 右側の縦釘二列目
		spawnNailsVerticalCount(2.3f, 7.7f, 7, 0.8f); // 左側の縦釘三列目
		spawnNailsVerticalCount(-2.3f, 7.7f, 7, 0.8f); // 右側の縦釘三列目

		spawnNailsBetween(1.8f, 5.0f, 0.9f, 4.5f, 0.4f); // へそを目指す斜めの釘（左）
		spawnNailsBetween(-1.8f, 5.0f, -0.9f, 4.5f, 0.4f); // へそを目指す斜めの釘（右）
		spawnNailsBetween(1.6f, 4.0f, 0.7f, 3.5f, 0.4f); // へそを目指す斜めの釘の下の釘（左）
		spawnNailsBetween(-1.6f, 4.0f, -0.8f, 3.5f, 0.4f); // へそを目指す斜めの釘の下の釘（右）

		spawnNail(0.25f, 4.2f); // へそ中央の釘 （左）
		spawnNail(-0.25f, 4.2f); // へそ中央の釘（右）
	}
} // namespace

// 金属玉をランダム位置・速度で生成（上部から落下）
GameObject* PachinkoGame_StageScene::SpawnMetalBall() {
	// 固定位置 (0, 12, 0) からスポーン
	const float x = 0.0f;
	const float z = 1.0f;
	const float startY = 12.0f;

	// いろんな方向へのランダムな力（より大きな力）
	const float launchX = RandRange(-100.0f, 100.0f);    // 左右に大きくランダム
	const float launchY = RandRange(3.0f, 8.0f);     // 上方向にランダム
	const float launchZ = RandRange(-3.0f, 3.0f);    // 前後にランダム

	// 回転要素（ランダム性を保持）
	const float avx = RandRange(-3.0f, 3.0f);        // 回転
	const float avy = RandRange(-3.0f, 3.0f);
	const float avz = RandRange(-3.0f, 3.0f);

	GameObject* ball = ObjectManager::Instance().Spawn(PachinkoBall::StaticPoolKey(), {
		{"px", std::to_string(x)}, {"py", std::to_string(startY)}, {"pz", std::to_string(z)},		// Position（固定上部）
		{"vx", "0.0"}, {"vy", "0.0"}, {"vz", "0.0"},												// 初期速度ゼロ
		{"avx", std::to_string(avx)}, {"avy", std::to_string(avy)}, {"avz", std::to_string(avz)},	// 回転（ランダム）
		{"freezeRotation", "0"}
	});

	// 生成成功時にカウンターを増やす
	if (ball) {
		++_spawnedBallCount;
	}

	// PachinkoBall に Launch() を呼んで初速を与える
	if (auto* pachinkoBall = dynamic_cast<PachinkoBall*>(ball)) {
		pachinkoBall->Launch(VGet(launchX, launchY, launchZ));
	}
	return ball;
}

// センサーを配置してメンバーリストに追加
void PachinkoGame_StageScene::SpawnSensors() {
	// センサー配置用のヘルパー関数
	auto spawnSensor = [&](float x, float y, float hx, float hy, int score, const char* name, unsigned int color) {
		GameObject* obj = ObjectManager::Instance().Spawn(PachinkoSensor::StaticPoolKey(), {
			{"px", std::to_string(x)},
			{"py", std::to_string(y)},
			{"pz", "1.0"},
			{"hx", std::to_string(hx)},
			{"hy", std::to_string(hy)},
			{"hz", "1.5"},
			{"color", std::to_string(color)},
			{"name", name},
		});

		if (auto* sensor = static_cast<PachinkoSensor*>(obj)) {
			sensor->onHit = [this, score, name](Collider* other) {
				_totalScore += score;
				
				// BottomLineセンサーの場合、ボールを削除（持ち球には戻さない）
				if (strcmp(name, "Bottom_Line") == 0 && other && other->owner) {
					GameObject* ball = other->owner;
					
					// ボールをアクティブリストから削除
					auto it = std::find(_liveBalls.begin(), _liveBalls.end(), ball);
					if (it != _liveBalls.end()) {
						// 対応するBallTrackも削除
						auto tit = _ballTracks.begin() + std::distance(_liveBalls.begin(), it);
						_liveBalls.erase(it);
						_ballTracks.erase(tit);
						
						// ボールをリリース
						if (ball->IsActive()) {
							ObjectManager::Instance().Release(ball);
						}
						
						// カウンターを更新
						++_deletedBallCount;
						_activeBallCount = static_cast<int>(_liveBalls.size());
					}
				}
			};
			_sensors.push_back(sensor);
		}
	};

	// XYの2点間にセンサーを配置するヘルパー関数（間隔指定版）
	auto spawnSensorsBetween = [&](float x1, float y1, float x2, float y2,
		float hx, float hy, int score, const char* name, unsigned int color, float interval) {
		const float dx = x2 - x1;
		const float dy = y2 - y1;
		const float distance = std::sqrt(dx * dx + dy * dy);
		const int count = static_cast<int>(distance / interval) + 1;

		for (int i = 0; i < count; ++i) {
			const float t = static_cast<float>(i) / static_cast<float>(count - 1);
			const float x = x1 + dx * t;
			const float y = y1 + dy * t;
			spawnSensor(x, y, hx, hy, score, name, color);
		}
	};

	// XYの2点間にセンサーを配置するヘルパー関数（個数指定版）
	auto spawnSensorsBetweenCount = [&](float x1, float y1, float x2, float y2,
		float hx, float hy, int score, const char* name, unsigned int color, int count) {
		const float dx = x2 - x1;
		const float dy = y2 - y1;

		for (int i = 0; i < count; ++i) {
			const float t = static_cast<float>(i) / static_cast<float>(count - 1);
			const float x = x1 + dx * t;
			const float y = y1 + dy * t;
			spawnSensor(x, y, hx, hy, score, name, color);
		}
	};

	// --- センサー配置例 ---
	// 電チュウ(入賞センサー)
	spawnSensor(0.0f, 4.1f, 0.15f, 0.04f, 10, "CenterSensor", GetColor(255, 255, 0)); // 黄色

	// 下部の連続センサー（例：ボールが集まる場所）
	spawnSensor(0.0f, -0.0f, 6.0f, 0.5f, 0, "Bottom_Line", GetColor(255, 255, 0)); // 白色
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

	EnsurePachinkoFieldRegistered();

	// -- 新構成案 -- 
	// 前面
	ObjectManager::Instance().Spawn(PachinkoField_Front::StaticPoolKey(), {
	  {"px", "0.0"}, {"py", "6.5"}, {"pz", "2.5"},
	  {"hx", "4.0"}, {"hy", "7.5"}, {"hz", "0.08"},
	  {"color", std::to_string(GetColor(255, 255, 255))}, // 前面は白色
	  {"material", "frictionless"}
		});
	

	// 背面
	ObjectManager::Instance().Spawn(PachinkoField_Back::StaticPoolKey(), {
	  {"px", "0.0"}, {"py", "6.5"}, {"pz", "-1.1"},
	  {"hx", "4.0"}, {"hy", "7.5"}, {"hz", "0.08"},
	  {"color", std::to_string(GetColor(128, 128, 128))}, // 背面は灰色
	  {"material", "frictionless"}
		});

	// 六角形のパチンコ台を作る為、壁を六角形に配置する
	// 右上
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
	  {"px", "-2.5"}, {"py", "12.0"}, {"pz", "1.0"},	// Position
	  {"hx", "0.1"}, {"hy", "2.8"}, {"hz", "2.0"},		// Scale
	  {"rz", "-1.0472"},								// 回転角度（ラジアン）
	  {"color", std::to_string(GetColor(255, 0, 0))},	// 赤色
	  {"material", "frictionless"}						// 素材(摩擦なし)
		});

	// 左上
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
	  {"px", "2.5"}, {"py", "12.0"}, {"pz", "1.0"},
	  {"hx", "0.1"}, {"hy", "2.8"}, {"hz", "2.0"},
	  {"rz", "1.0472"},
	  {"material", "frictionless"}
		});

	// 右中
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
	  {"px", "-4.0"}, {"py", "4.8"}, {"pz", "1.0"},
	  {"hx", "0.1"}, {"hy", "6.5"}, {"hz", "2.0"},
	  {"color", std::to_string(GetColor(255, 0, 0))}, // 赤色
	  {"material", "frictionless"}
		});

	// 左中
	ObjectManager::Instance().Spawn(PachinkoField_Side::StaticPoolKey(), {
	  {"px", "4.0"}, {"py", "4.8"}, {"pz", "1.0"},
	  {"hx", "0.1"}, {"hy", "6.5"}, {"hz", "2.0"},
	  {"material", "frictionless"}
		});

	// 液晶に当たる内側の壁は、釘を敷き詰めて円形にし対応する、
	// 釘（円形）
	SpawnNails();

	// sensor を配置してメンバーリストに追加
	SpawnSensors();
	// --------------

	// フロントオブジェクトの後ろ側(内側)に固定カメラを配置
	CameraManager& camMgr = CameraManager::Instance();
	const int sceneId = SceneManager::Instance().CurrentSceneId();
	_cameraId = camMgr.CreateCamera(sceneId);
	if (Camera* cam = camMgr.Get(_cameraId)) { // カメラの初期位置を固定カメラ位置に設定
		cam->transform.SetLocalPosition(_fixedCameraEye);
		cam->LookAt(_fixedCameraEye, _fixedCameraTarget, VGet(0.0f, 1.0f, 0.0f));

		// カメラ位置に光源を配置（ポイントライト）
		SetLightPosition(_fixedCameraEye);
		SetLightAmbColor(GetColorF(0.8f, 0.8f, 0.8f, 1.0f));
		SetLightDifColor(GetColorF(1.0f, 1.0f, 1.0f, 1.0f));
		SetLightSpcColor(GetColorF(1.0f, 1.0f, 1.0f, 1.0f));
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
			// 削除理由を判定
			bool isStuck = false;
			bool isOutOfBounds = false;
			
			if (ball && ball->IsActive()) {
				// 画面下に落下した場合
				if (ball->transform.WorldPosition().y < -40.0f) {
					isOutOfBounds = true;
				}
				
				// スタック判定（位置変化量チェック）
				BallTrack& track = *tit;
				if (track.stuckCount >= kStuckCountThreshold) {
					isStuck = true;
				}
			}
			
			if (ball && ball->IsActive()) om.Release(ball);
			it  = _liveBalls.erase(it);
			tit = _ballTracks.erase(tit);
			++_deletedBallCount;  // 削除カウンターを増やす（総数）
			
			// 途中で引っかかった（スタックした）場合のみ持ち球に返す
			if (isStuck && !isOutOfBounds) {
				++_remainingBalls;
			}
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
		if (GameObject* ball = SpawnMetalBall()) {
			_liveBalls.push_back(ball);
			BallTrack t; t.ball = ball; t.lastSnapPos = ball->transform.WorldPosition();
			_ballTracks.push_back(t);
			++_ballCount;
			_activeBallCount = static_cast<int>(_liveBalls.size());
		}
	}
#endif

	// 鉄球追加（スペースキー押下中は持ち球を消費して自動生成）
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_SPACE)) {
		_ballSpawnTimer += dtSec;
		if (_ballSpawnTimer >= _ballSpawnIntervalSec) {
			_ballSpawnTimer -= _ballSpawnIntervalSec;
			if (_remainingBalls > 0) {  // 持ち球がある場合のみ生成
				if (GameObject* ball = SpawnMetalBall()) {
					_liveBalls.push_back(ball);
					BallTrack t; t.ball = ball; t.lastSnapPos = ball->transform.WorldPosition();
					_ballTracks.push_back(t);
					++_ballCount;
					_activeBallCount = static_cast<int>(_liveBalls.size());
					--_remainingBalls;  // 持ち球を消費
				}
			}
		}
	} else {
		_ballSpawnTimer = 0.0f;  // スペースキーが離されたらタイマーをリセット
	}

	// Rキーで125球の貸出し
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
		_remainingBalls += 125;  // 持ち球を125増やす
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
	DrawString(10, 110, "R: 125球貸出", GetColor(255, 255, 100));
	DrawString(10, 130, "Esc: メニューへ戻る", GetColor(180, 255, 180));
	DrawFormatString(10, 150, GetColor(220, 220, 220), "アクティブ鉄球数: %d", _activeBallCount);

	// 右上に持ち球を表示
	DrawFormatString(1150 - 200, 10, GetColor(255, 255, 0), "持ち球: %d", _remainingBalls);

	// ボール管理情報（左上）
	DrawFormatString(10, 170, GetColor(255, 200, 100), "生成数: %d", _spawnedBallCount);
	DrawFormatString(10, 190, GetColor(255, 150, 100), "削除数: %d", _deletedBallCount);
	DrawFormatString(10, 210, GetColor(200, 255, 200), "有効球数: %d", _spawnedBallCount - _deletedBallCount);

	// センサー入賞情報
	DrawFormatString(10, 230, GetColor(255, 220, 50), "スコア: %d", _totalScore);
	for (int i = 0; i < static_cast<int>(_sensors.size()); ++i) {
		const PachinkoSensor* s = _sensors[i];
		DrawFormatString(10, 250 + i * 18, GetColor(180, 255, 180),
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
