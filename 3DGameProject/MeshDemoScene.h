#pragma once

// MeshDemoScene
// - MeshCollider のデモシーン
// - data/models/MeshDemoCity.mv1 をステージとして読み込み、メッシュコライダーを適用
// - デバッグカメラ（右ドラッグ+WASDQE）でステージを見学
// - L で MeshCollider のワイヤ描画 ON/OFF
// - R でリセット、T でタイトルへ戻る

#include <memory>
#include <string>
#include <cmath>

#include "DxLib.h"
#include "SceneTpl.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "TitleScene.h"

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "KeyInput.h"

#include "GameObject.h"
#include "MeshCollider.h"
#include "ColliderManager.h"
#include "LayerMask.h"

class MeshDemoScene : public SceneTpl<MeshDemoScene> {
public:
    static std::string StaticName() { return "MeshDemoScene"; }

    void Start() override {
        auto& camMgr = CameraManager::Instance();
        const int sceneId = SceneManager::Instance().CurrentSceneId();

        // --- ステージ（MV1）の読み込みと MeshCollider 構築 ---
        // 実行フォルダの差異に強くするため、複数候補から読み込みを試す
        {
            const char* candidates[] = {
                "models/MeshDemoCity/MeshDemoCity.mv1",
                "./models/MeshDemoCity/MeshDemoCity.mv1",
                "../models/MeshDemoCity/MeshDemoCity.mv1",
                "3DGameProject/models/MeshDemoCity/MeshDemoCity.mv1",
                "../3DGameProject/models/MeshDemoCity/MeshDemoCity.mv1",
                "../../3DGameProject/models/MeshDemoCity/MeshDemoCity.mv1",
                "data/models/MeshDemoCity/MeshDemoCity.mv1",
                "C:/Users/rinsa/source/repos/3DGameProject/3DGameProject/models/MeshDemoCity/MeshDemoCity.mv1",
            };
            _stageHandle = -1;
            _loadedPath.clear();
            for (const char* p : candidates) {
                _stageHandle = MV1LoadModel(p);
                if (_stageHandle >= 0) { _loadedPath = p; break; }
            }
        }
        _stage = std::make_unique<GameObject>();
        _stage->isStatic = true;
        _stage->transform.SetLocalPosition(VGet(0.0f, -20.0f, 0.0f));
        // ステージを 1 倍に拡大。Transform のスケールと MV1 のスケールを揃えることで、
        // 描画と MeshCollider（owner の WorldScale を反映して頂点をワールド化）が一致する。
        _stage->transform.SetLocalScale(VGet(_stageScale, _stageScale, _stageScale));

        _stageCollider = std::make_unique<MeshCollider>();
        _stageCollider->owner    = _stage.get();
        _stageCollider->isStatic = true;
        _stageCollider->layer    = layerMask::DEFAULT;
        _stageCollider->mask     = mask::ALL;
        if (_stageHandle >= 0) {
            _stageCollider->BuildFromMV1(_stageHandle);
        }
        ColliderManager::Instance().RegisterCollider(_stageCollider.get());

        // --- カメラ ---
        if (_debugCamId == 0 || camMgr.Get(_debugCamId) == nullptr) {
            _debugCamId = _debugCamCtrl.SpawnAuto(sceneId, CameraTag::Debug,
                VGet(0.0f, 30.0f, -60.0f), VGet(0.30f, 0.0f, 0.0f));
        }
        _currentCamId = _debugCamId;
        camMgr.SetRender(_currentCamId);
    }

    void End() override {
        // 登録解除（GameObject/Collider のデストラクタ前に必ず外す）
        if (_stageCollider) {
            ColliderManager::Instance().UnregisterCollider(_stageCollider.get());
        }
        _stageCollider.reset();
        _stage.reset();

        if (_stageHandle >= 0) {
            MV1DeleteModel(_stageHandle);
            _stageHandle = -1;
        }
    }

    void Update(float dtSec) override {
        UpdateCamera_(dtSec);

        auto& key = KeyInput::Instance();
        if (key.IsKeyInputTrigger(KEY_INPUT_L)) {
            _drawMeshWire = !_drawMeshWire;
        }

        // 矢印キーでステージ移動（XZ 平面）、PageUp/Down で Y、+/- で拡縮
        bool transformChanged = false;
        VECTOR pos = _stage->transform.LocalPosition();
        const float moveSpeed = 80.0f; // 単位/秒（ステージが巨大なので大きめ）
        if (key.IsKeyInputHeld(KEY_INPUT_LEFT))  { pos.x -= moveSpeed * dtSec; transformChanged = true; }
        if (key.IsKeyInputHeld(KEY_INPUT_RIGHT)) { pos.x += moveSpeed * dtSec; transformChanged = true; }
        if (key.IsKeyInputHeld(KEY_INPUT_UP))    { pos.z += moveSpeed * dtSec; transformChanged = true; }
        if (key.IsKeyInputHeld(KEY_INPUT_DOWN))  { pos.z -= moveSpeed * dtSec; transformChanged = true; }
        if (key.IsKeyInputHeld(KEY_INPUT_PGUP))  { pos.y += moveSpeed * dtSec; transformChanged = true; }
        if (key.IsKeyInputHeld(KEY_INPUT_PGDN))  { pos.y -= moveSpeed * dtSec; transformChanged = true; }
        if (transformChanged) {
            _stage->transform.SetLocalPosition(pos);
        }

        // 拡縮 : Z で拡大、X で縮小（テンキーの +/- も対応）
        bool scaleChanged = false;
        const float scaleRate = 1.5f; // 1秒あたりの倍率変化
        if (key.IsKeyInputHeld(KEY_INPUT_Z) || key.IsKeyInputHeld(KEY_INPUT_ADD)) {
            _stageScale *= std::pow(scaleRate, dtSec);
            scaleChanged = true;
        }
        if (key.IsKeyInputHeld(KEY_INPUT_X) || key.IsKeyInputHeld(KEY_INPUT_SUBTRACT)) {
            _stageScale /= std::pow(scaleRate, dtSec);
            scaleChanged = true;
        }
        if (scaleChanged) {
            if (_stageScale < 0.01f) _stageScale = 0.01f;
            if (_stageScale > 1000.0f) _stageScale = 1000.0f;
            _stage->transform.SetLocalScale(VGet(_stageScale, _stageScale, _stageScale));
        }

        // Transform が変わったらコライダーを再構築（owner の WorldScale/Position を反映）
        if ((transformChanged || scaleChanged) && _stageCollider && _stageHandle >= 0) {
            _stageCollider->BuildFromMV1(_stageHandle);
        }

        if (key.IsKeyInputTrigger(KEY_INPUT_R)) {
            SceneManager::Instance().RequestChange(std::make_unique<MeshDemoScene>());
        }
        if (key.IsKeyInputTrigger(KEY_INPUT_T)) {
            SceneTransition::Params p;
            p.mode = SceneTransition::Mode::MaskImage;
            p.durationSec = 0.4;
            p.maskGraphPath = "Data/Transition/mask.png";
            p.pixelShaderPath = "Data/Transition/mask_transition.pso";
            SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p, 0.5f);
        }
    }

    void Draw() override {
        // ステージモデルを描画
        if (_stageHandle >= 0) {
            MV1SetPosition(_stageHandle, _stage->transform.LocalPosition());
            MV1SetScale(_stageHandle, VGet(_stageScale, _stageScale, _stageScale));
            MV1DrawModel(_stageHandle);
        }

        // メッシュコライダーのワイヤ描画（デバッグカメラ位置周辺のみ：巨大ステージ対策）
        if (_drawMeshWire && _stageCollider) {
            auto* cam = CameraManager::Instance().Get(_debugCamId);
            const VECTOR pc = cam ? cam->transform.LocalPosition() : VGet(0, 0, 0);
            const float r = 60.0f;
            AABB q;
            q.min = VGet(pc.x - r, pc.y - r, pc.z - r);
            q.max = VGet(pc.x + r, pc.y + r, pc.z + r);
            q.center = pc;
            _stageCollider->SetDebugColor(GetColor(0, 255, 0));
            _stageCollider->DrawDebugInAABB(q);
        }

        // UI
        DrawString(10, 10, "MeshDemoScene  R:Reset  T:Title", GetColor(255, 255, 255));
        DrawString(10, 30, "[Camera] Right-Drag+WASDQE = Free Move", GetColor(180, 220, 255));
        DrawString(10, 50, "[Stage]  Arrows:Move XZ  PgUp/PgDn:Move Y  Z/X(+/-):Scale", GetColor(255, 220, 180));
        DrawString(10, 70, "[Debug]  L:Mesh wire toggle", GetColor(200, 255, 200));

        char buf[160];
        // 読み込み状況
        const bool mv1Loaded = (_stageHandle >= 0);
        const size_t tri = _stageCollider ? _stageCollider->TriangleCount() : 0;
        const bool colliderBuilt = (tri > 0);
        sprintf_s(buf, sizeof(buf), "MV1 model     : %s  (handle=%d)",
            mv1Loaded ? "LOADED" : "FAILED", _stageHandle);
        DrawString(10, 100, buf, mv1Loaded ? GetColor(120, 255, 120) : GetColor(255, 120, 120));
        sprintf_s(buf, sizeof(buf), "MeshCollider  : %s  (triangles=%zu)",
            colliderBuilt ? "BUILT" : "EMPTY", tri);
        DrawString(10, 120, buf, colliderBuilt ? GetColor(120, 255, 120) : GetColor(255, 120, 120));

        // ステージ Transform 状態
        const VECTOR pos = _stage ? _stage->transform.LocalPosition() : VGet(0, 0, 0);
        sprintf_s(buf, sizeof(buf), "Stage pos     : (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);
        DrawString(10, 140, buf, GetColor(220, 220, 220));
        sprintf_s(buf, sizeof(buf), "Stage scale   : %.3f", _stageScale);
        DrawString(10, 160, buf, GetColor(220, 220, 220));

        // 読み込んだパス（失敗時は最後に試した候補）
        sprintf_s(buf, sizeof(buf), "Path : %s",
            _loadedPath.empty() ? "(not loaded)" : _loadedPath.c_str());
        DrawString(10, 180, buf, mv1Loaded ? GetColor(180, 220, 255) : GetColor(255, 180, 180));
    }

private:
    void UpdateCamera_(float dtSec) {
        // Debug カメラは自由視点
        _debugCamCtrl.SetCamera(_debugCamId);
        _debugCamCtrl.UpdateFreeMoveMouse(20.0f, 0.45f, 12.0f, dtSec);
    }

private:
    // ステージ
    int _stageHandle = -1;
    std::string _loadedPath;
    float _stageScale = 1.0f;
    std::unique_ptr<GameObject>   _stage;
    std::unique_ptr<MeshCollider> _stageCollider;

    // カメラ
    CameraController _debugCamCtrl;
    CameraController::CameraId _debugCamId = 0;
    CameraController::CameraId _currentCamId = 0;

    // デバッグ表示
    bool _drawMeshWire = false;
};
