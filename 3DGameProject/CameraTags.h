#pragma once

#include <cstdint>

// CameraTag: カメラの用途分類（最小）
// - ゲーム用 / デバッグ用 / シネマ用…など
// - 必要になったらビットフラグ化や Layerへ拡張する
enum class CameraTag : std::uint8_t {
	Game =0,
	Debug =1,
	Cinematic =2,
};
