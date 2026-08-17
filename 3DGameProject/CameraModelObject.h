#pragma once

#include <array>
#include <string>

#include "GameObject.h"
#include "SceneManager.h"
#include "DxLib.h"

// CameraScene で使うモデル表示用 GameObject
class CameraModelObject : public GameObject
{
public:
	// プール用のキー文字列
    static std::string StaticPoolKey() { return "CameraModelObject"; }
    
	// コンストラクタ・デストラクタ
    CameraModelObject() {
        _ownerSceneId = SceneManager::Instance().CurrentSceneId();
	}
    virtual ~CameraModelObject() override;

	// GameObject のライフサイクル
    bool IsModelLoaded() const noexcept { return _modelHandle >= 0; }
    const std::string& LastTriedPath() const noexcept { return _lastTriedPath; }

	// プールから取得/返却される時の初期化・後片付け
    void OnAcquire(const VariantMap& params) override;
    void OnRelease() override;
    void Draw() override;

private:
	// VariantMap から安全に値を読み出すヘルパー
    void ConfigureFromParams_(const VariantMap& params);
    void LoadModelIfNeeded_();

private:
	// モデルのロード状態
	int         _modelHandle = -1;  // モデルハンドル（ロード済みなら >= 0）
    std::string _modelPath;         // モデルのパス
    std::string _loadedPath;        // 最後にロードしたモデルのパス
    std::string _lastTriedPath;     // 最後にロードを試みたモデルのパス
};
