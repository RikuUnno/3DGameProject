#pragma once

#include <unordered_map>
#include <memory>
#include <cstdint>

#include "Camera.h"
#include "CameraPool.h"
#include "Pool.h"
#include "Manager.h"

class CameraManager : public Manager {
public:
	// カメラIDの型定義
	using CameraId = std::uint32_t;

	// シングルトンインスタンス取得
	static CameraManager& Instance() noexcept;

	// カメラ作成/破棄
	CameraId CreateCamera(int ownerSceneId);	// ownerSceneId をセットして管理下に置く。返り値はカメラID
	bool DestroyCamera(CameraId id);			// カメラ破棄
	void ReleaseBySceneId(int sceneId);			// シーンID指定でカメラ一括破棄

	// カメラ取得
	Camera* Get(CameraId id);					// カメラIDでカメラを取得。存在しないIDなら nullptr を返す
	const Camera* Get(CameraId id) const;		// カメラIDでカメラを取得（const 版）。存在しないIDなら nullptr を返す

	// アクティブカメラ設定/レンダリングカメラ設定
	CameraId ActiveCameraId() const noexcept { return _activeId; }	// アクティブカメラIDを取得。設定されていない場合は0
	CameraId RenderCameraId() const noexcept { return _renderId; }	// レンダリングカメラIDを取得。設定されていない場合は0

	// アクティブカメラ設定/レンダリングカメラ設定
	bool SetActive(CameraId id);	// Blend中であっても即座に切り替える（Blendはキャンセルされる）
	bool SetRender(CameraId id);	// Blend中であっても即座に切り替える（Blendはキャンセルされる）

	// アクティブカメラ取得/レンダリングカメラ取得
	Camera* Active();	// アクティブカメラを取得。設定されていない場合は nullptr を返す
	Camera* Render();	// レンダリングカメラを取得。設定されていない場合は nullptr を返す

	// レンダリングカメラを別のカメラへ Blend で切り替える
	bool BlendRenderTo(CameraId targetId, float durationSec);	// Blend中であっても即座に切り替える（Blendはキャンセルされる）
	bool IsBlending() const noexcept { return _blend.active; }	// Blend 中かどうか
	void Update(float dtSec) override;							// Blend 更新

	// 描画前に呼び出す想定
	// Render カメラの情報を DxLib の描画用カメラに反映する
	void ApplyRenderCameraToDxLib(int screenW, int screenH);

private:
	// 非コピー・非ムーブ
	CameraManager() = default;
	~CameraManager() = default;
	CameraManager(const CameraManager&) = delete;				// コピーコンストラクタ削除
	CameraManager& operator=(const CameraManager&) = delete;	// コピー代入演算子削除
	CameraManager(CameraManager&&) = delete;					// ムーブコンストラクタ削除
	CameraManager& operator=(CameraManager&&) = delete;			// ムーブ代入演算子削除

	// 内部データ
	CameraId _nextId = 1;	// 次に割り当てるカメラID。0 は無効IDとして予約しておく
	CameraId _activeId = 0;	// アクティブカメラID。設定されていない場合は0
	CameraId _renderId = 0;	// レンダリングカメラID。設定されていない場合は0

	// カメラ管理用のプールとマップ
	CameraPool _pool{ 32 };										// カメラプール（最大32台まで）
	std::unordered_map<CameraId, Pool::UniquePtr> _cameras;		// カメラIDからカメラオブジェクトへのマップ

	// Blend 状態
	struct BlendState {
		bool active = false;	// Blend がアクティブかどうか
		CameraId fromId = 0;	// Blend 開始時のレンダリングカメラID
		CameraId toId = 0;		// Blend 目標のレンダリングカメラID
		float duration = 0.0f;	// Blend の総時間（秒）
		float t =0.0f;			// Blend の経過時間（秒）

		std::unique_ptr<Camera> scratch; // Blend 用の一時的なカメラオブジェクト（Blend 中は from/to カメラの補間結果をセットしておく）
	} _blend; // Blend 状態
};

// CameraManager は, 複数のCameraオブジェクトを管理する。
// CameraManager は、アクティブカメラ（Active）とレンダリングカメラ（Render）を区別して管理する
// アクティブカメラはゲームロジックやオブジェクトの更新で参照されるカメラ
// レンダリングカメラは実際の描画に使用されるカメラ