#pragma once

#include "Time.h"
#include "DxLib.h"
#include <algorithm>

// DxLibのキー数を想定
class KeyInput {
public:
	static constexpr int KEY_COUNT = 256;

	// シングルトン取得（画面遷移で状態を保持するため）
	static KeyInput& Instance() noexcept;

	KeyInput();
	~KeyInput();

	// コピー禁止（シングルトン運用）
	KeyInput(const KeyInput&) = delete;
	KeyInput& operator=(const KeyInput&) = delete;
	KeyInput(KeyInput&&) = delete;
	KeyInput& operator=(KeyInput&&) = delete;

	// 初期化（コンストラクタから呼ぶ）
	void Initialize();

	// 押下判定（押された瞬間に true を返す）
	bool IsKeyInputTrigger(int keyCode);

	// 押している間 true を返す（連続）
	bool IsKeyInputHeld(int keyCode);

	// 離された瞬間に true を返す
	bool IsKeyInputReleased(int keyCode);

	// 一定間隔で断続的に true を返す（パルス／リピート）
	bool IsKeyInputRepeated(int keyCode);

	// キーごとの繰り返し間隔をセット（秒）
	void SetInputRepeatedTime(int keyCode, double setTime);

	// 入力のオン／オフ
	void BeginKeyInput(); // 入力有効化
	void EndKeyInput();   // 入力無効化

private:
	// DxLib の GetHitKeyStateAll は char[256] を要求
	char _currentKey[KEY_COUNT];
	char _previousKey[KEY_COUNT];

	// リピート設定（秒）とタイマー（秒）
	double _repeatedTime[KEY_COUNT];
	double _repeatedTimer[KEY_COUNT];

	bool _isKeyInputOn{};
};
