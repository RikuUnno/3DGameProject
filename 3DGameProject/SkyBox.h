#pragma once

#include "DxLib.h"
#include <array>
#include <string>

// SkyBox
// - 6 枚のテクスチャ (right / left / top / bottom / front / back) を
//   カメラを中心に立方体として貼り付ける天球表現
// - Draw() はカメラ位置を毎フレーム渡して呼ぶこと。
//   Z バッファは書き込まないので、必ず他の3D描画より前に呼ぶこと。
class SkyBox {
public:
    // 6面の方向（DxLib の左手座標、+Z が前）
    enum class Face : int {
        Right = 0,  // +X
        Left,       // -X
        Top,        // +Y
        Bottom,     // -Y
        Front,      // +Z
        Back,       // -Z
        Count
    };

	// コンストラクタ / デストラクタ
    SkyBox() = default;
    virtual ~SkyBox();

	// コピー禁止
    SkyBox(const SkyBox&) = delete;
    SkyBox& operator=(const SkyBox&) = delete;

    // 6 面のテクスチャをまとめて読み込む。
    // dir に "skybox/" のような相対パスを与え、basenames[0..5] を結合する。
    // 拡張子 (".png" など) も basenames に含めて指定する。
    bool Load(const std::string& dir, const std::array<std::string, 6>& basenames);

    // 個別に 1 面読み込む（テスト・差し替え用）
    bool LoadFace(Face face, const std::string& filePath);

    // 全テクスチャを解放
    void Reset();

    // 立方体のサイズ（半辺）。near/far の範囲内に収まる値を渡す。
    void SetSize(float halfExtent) noexcept { _halfExtent = halfExtent; }
    float Size() const noexcept { return _halfExtent; }

    // 描画。cameraPos を中心に常に追従させる。
    // Z バッファ書き込みを無効にして描画する（中身は復元する）。
    void Draw(const VECTOR& cameraPos) const;

	// 読み込み状態を返す。全ての面が有効なテクスチャを持っている場合に true。
    bool IsLoaded() const noexcept;

private:
	std::array<int, static_cast<int>(Face::Count)> _handles{ -1, -1, -1, -1, -1, -1 };  // 各面のテクスチャハンドル。-1 の場合は未読み込み / 読み込み失敗
	float _halfExtent = 500.0f;                                                         // 立方体の半辺の長さ
};
