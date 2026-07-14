#include "MiniGame_1_MenuScene.h"

#include <cmath>
#include <algorithm>

#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "CameraManager.h"

namespace {
	inline void DrawGridFloor(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -(float)halfCells * step), VGet(x, y, (float)halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-(float)halfCells * step, y, z), VGet((float)halfCells * step, y, z), colGrid);
		}
		// XYZ軸
		DrawLine3D(VGet(0, y, 0), VGet(3, y, 0), GetColor(255, 80, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y + 3, 0), GetColor(80, 255, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y, 3), GetColor(80, 80, 255));
	}
}

void MiniGame_1_MenuScene::Start()
{
	// 戦闘機の生成・配置
	_aircraft = std::make_unique<FighterAircraft>();
	_aircraft->transform.SetLocalPosition(VGet(0.0f, 5.0f, 0.0f));
	_aircraft->Start();

	// カメラ生成
	const int sceneId = SceneManager::Instance().CurrentSceneId();
	_camId = _camCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 8.0f, -20.0f));
	CameraManager::Instance().SetRender(_camId);
	_camYaw = 0.0f;
	_camPitch = -0.25f;
	_mouseInit = false;
}

void MiniGame_1_MenuScene::Update(float dtSec)
{
	if (_aircraft) {
		_aircraft->Update(dtSec);
	}

	UpdateThirdPersonCamera_(dtSec);
}

void MiniGame_1_MenuScene::UpdateThirdPersonCamera_(float dtSec)
{
	if (!_aircraft) return;

	auto* cam = CameraManager::Instance().Get(_camId);
	if (!cam) return;

	auto& key = KeyInput::Instance();

	// 右クリック中はマウスでカメラ回転
	if (GetMouseInput() & MOUSE_INPUT_RIGHT) {
		int mx = 0, my = 0;
		GetMousePoint(&mx, &my);
		if (_mouseInit) {
			const int dx = mx - _prevMouseX;
			const int dy = my - _prevMouseY;
			_camYaw   += dx * 0.005f;
			_camPitch += dy * 0.004f;
			// ピッチ制限
			_camPitch = (std::max)(-1.2f, (std::min)(0.0f, _camPitch));
		}
		_prevMouseX = mx;
		_prevMouseY = my;
		_mouseInit = true;
	} else {
		_mouseInit = false;
	}

	// スクロールホイールで距離調整
	_camDistance -= GetMouseWheelRotVol() * 1.5f;
	_camDistance = (std::max)(5.0f, (std::min)(60.0f, _camDistance));

	// 機体の位置に追従
	const VECTOR target = _aircraft->transform.WorldPosition();

	// カメラ位置を計算（球面座標）
	const float cosP = std::cos(_camPitch);
	const float sinP = std::sin(_camPitch);
	const float cosY = std::cos(_camYaw);
	const float sinY = std::sin(_camYaw);
	const VECTOR offset = VGet(
		sinY * cosP * _camDistance,
		-sinP * _camDistance + _camHeight,
		cosY * cosP * _camDistance
	);
	const VECTOR camPos = VAdd(target, offset);

	cam->transform.SetLocalPosition(camPos);
	cam->LookAt(camPos, target, VGet(0.0f, 1.0f, 0.0f));
}

void MiniGame_1_MenuScene::End()
{
	if (_aircraft) {
		_aircraft->End();
		_aircraft.reset();
	}
}

void MiniGame_1_MenuScene::Draw()
{
	DrawGridFloor(0.0f, 20, 2.0f);

	if (_aircraft) {
		_aircraft->Draw();
	}

	DrawString(10, 10, "MiniGame_1 - 戦闘機飛行", GetColor(255, 255, 120));
	DrawString(10, 30, "WASD: 移動  Space: 上昇  LShift: 下降", GetColor(255, 255, 255));
	DrawString(10, 50, "右クリック+ドラッグ: カメラ回転  ホイール: 距離調整", GetColor(200, 200, 255));
	DrawString(10, 70, "Esc: メニューへ戻る", GetColor(180, 255, 180));
}
