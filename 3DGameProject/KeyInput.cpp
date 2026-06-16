#include "KeyInput.h"
#include <cstring>
#include <cassert>
#include <cstdio>

// シングルトン取得（画面遷移で状態を保持するため）
KeyInput& KeyInput::Instance() noexcept {
	static KeyInput inst;
	return inst;
}

// コンストラクタ/デストラクタ
KeyInput::KeyInput()
{
	Initialize();													// 初期化
	GetHitKeyStateAll(_currentKey);									// 現在のキー状態を取得して保存
	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey));	// 前回の状態も同様に初期化
}

KeyInput::~KeyInput()
{}

// 初期化（コンストラクタから呼ぶ）
void KeyInput::Initialize()
{
	_isKeyInputOn = false;
	std::fill(std::begin(_currentKey), std::end(_currentKey), 0);			// すべてのキーの状態を0（押されていない）に初期化
	std::fill(std::begin(_previousKey), std::end(_previousKey), 0);			// 前回の状態も同様に初期化
	std::fill(std::begin(_repeatedTime), std::end(_repeatedTime), 1.0);		// デフォルトのリピート間隔は1秒
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer), 0.0);	// タイマーは0に初期化
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);	// リピートフラグはすべてオフに初期化
}

// 毎フレームの入力更新
void KeyInput::Update(float dtSec)
{
	if (!_isKeyInputOn) return;
	if (dtSec < 0.0f) dtSec = 0.0f;	// 負の値はありえないが、万が一そうなったときにタイマーが減るのを防止

	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey)); // 前回の状態を保存
	GetHitKeyStateAll(_currentKey);

	// キーごとのリピート判定
	for (int keyCode = 0; keyCode < KEY_COUNT; ++keyCode) {
		_repeatedFlag[keyCode] = false;

		// 押されていない、または前回押されていない場合はリピート判定をリセット
		if (_currentKey[keyCode] == 0) {
			_repeatedTimer[keyCode] = 0.0;
			continue;
		}
		// 前回押されていない場合はリピート判定をリセット
		if (_previousKey[keyCode] == 0) {
			_repeatedTimer[keyCode] = 0.0;
			continue;
		}
		// リピート間隔が0以下なら常にリピート判定をオン
		if (_repeatedTime[keyCode] <= 0.0) {
			_repeatedFlag[keyCode] = true;
			continue;
		}
		// 押されていて前回も押されている場合はタイマーを進める
		_repeatedTimer[keyCode] += dtSec;
		if (_repeatedTimer[keyCode] >= _repeatedTime[keyCode]) {
			_repeatedFlag[keyCode] = true;
			do {
				_repeatedTimer[keyCode] -= _repeatedTime[keyCode];
			} while (_repeatedTimer[keyCode] >= _repeatedTime[keyCode]);
		}
	}
}

// 押された瞬間に true を返す
bool KeyInput::IsKeyInputTrigger(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = (_currentKey[keyCode] !=0) && (_previousKey[keyCode] ==0);

	return flag;
}

// 押している間 true を返す（連続）
bool KeyInput::IsKeyInputHeld(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;
	return (_currentKey[keyCode] !=0);
}

// 離された瞬間に true を返す
bool KeyInput::IsKeyInputReleased(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = (_previousKey[keyCode] !=0) && (_currentKey[keyCode] ==0);

	return flag;
}

// 一定間隔で断続的に true を返す（パルス／リピート）
bool KeyInput::IsKeyInputRepeated(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = _repeatedFlag[keyCode];

	return flag;
}

// キーごとの繰り返し間隔をセット（秒）
void KeyInput::SetInputRepeatedTime(int keyCode, double setTime)
{
	if (keyCode <0 || keyCode >= KEY_COUNT) return;
	if (setTime <0.0) setTime =0.0;
	_repeatedTime[keyCode] = setTime;
}

// 入力のオン
void KeyInput::BeginKeyInput()
{
	_isKeyInputOn = true;													// 入力を有効化
	GetHitKeyStateAll(_currentKey);											// 現在のキー状態を取得して保存
	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey));			// 前回の状態も同様に初期化
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer), 0.0);	// タイマーは0に初期化
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);	// リピートフラグはすべてオフに初期化
}

// 入力のオフ
void KeyInput::EndKeyInput()
{
	_isKeyInputOn = false;
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer), 0.0);	// タイマーは0に初期化
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);	// リピートフラグはすべてオフに初期化
}