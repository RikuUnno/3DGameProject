#include "KeyInput.h"
#include <cstring>
#include <cassert>
#include <cstdio>

KeyInput& KeyInput::Instance() noexcept {
	static KeyInput inst;
	return inst;
}

KeyInput::KeyInput()
{
	Initialize();
	GetHitKeyStateAll(_currentKey);
	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey));
}

KeyInput::~KeyInput()
{
}

void KeyInput::Initialize()
{
	_isKeyInputOn = false;
	std::fill(std::begin(_currentKey), std::end(_currentKey),0);
	std::fill(std::begin(_previousKey), std::end(_previousKey),0);
	std::fill(std::begin(_repeatedTime), std::end(_repeatedTime),1.0);
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer),0.0);
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);
}

void KeyInput::Update(float dtSec)
{
	if (!_isKeyInputOn) return;
	if (dtSec < 0.0f) dtSec = 0.0f;

	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey));
	GetHitKeyStateAll(_currentKey);

	for (int keyCode = 0; keyCode < KEY_COUNT; ++keyCode) {
		_repeatedFlag[keyCode] = false;

		if (_currentKey[keyCode] == 0) {
			_repeatedTimer[keyCode] = 0.0;
			continue;
		}

		if (_previousKey[keyCode] == 0) {
			_repeatedTimer[keyCode] = 0.0;
			continue;
		}

		if (_repeatedTime[keyCode] <= 0.0) {
			_repeatedFlag[keyCode] = true;
			continue;
		}

		_repeatedTimer[keyCode] += dtSec;
		if (_repeatedTimer[keyCode] >= _repeatedTime[keyCode]) {
			_repeatedFlag[keyCode] = true;
			do {
				_repeatedTimer[keyCode] -= _repeatedTime[keyCode];
			} while (_repeatedTimer[keyCode] >= _repeatedTime[keyCode]);
		}
	}
}

bool KeyInput::IsKeyInputTrigger(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = (_currentKey[keyCode] !=0) && (_previousKey[keyCode] ==0);

#ifdef _DEBUG
	DrawFormatString(0,90, GetColor(255,255,255), "TriggerFlag[%d]: %d", keyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputHeld(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;
	return (_currentKey[keyCode] !=0);
}

bool KeyInput::IsKeyInputReleased(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = (_previousKey[keyCode] !=0) && (_currentKey[keyCode] ==0);

#ifdef _DEBUG
	DrawFormatString(0,75, GetColor(255,255,255), "ReleasedFlag[%d]: %d", keyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputRepeated(int keyCode) const
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	const bool flag = _repeatedFlag[keyCode];

#ifdef _DEBUG
	DrawFormatString(0,90, GetColor(255,255,255), "RepeatedTimer[%d]: %.4f", keyCode, _repeatedTimer[keyCode]);
	DrawFormatString(0,105, GetColor(255,255,255), "RepeatedFlag[%d]: %d", keyCode, flag);
#endif

	return flag;
}

void KeyInput::SetInputRepeatedTime(int keyCode, double setTime)
{
	if (keyCode <0 || keyCode >= KEY_COUNT) return;
	if (setTime <0.0) setTime =0.0;
	_repeatedTime[keyCode] = setTime;
}

void KeyInput::BeginKeyInput()
{
	_isKeyInputOn = true;
	GetHitKeyStateAll(_currentKey);
	std::memcpy(_previousKey, _currentKey, sizeof(_currentKey));
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer),0.0);
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);
}

void KeyInput::EndKeyInput()
{
	_isKeyInputOn = false;
	std::fill(std::begin(_repeatedTimer), std::end(_repeatedTimer),0.0);
	std::fill(std::begin(_repeatedFlag), std::end(_repeatedFlag), false);
}