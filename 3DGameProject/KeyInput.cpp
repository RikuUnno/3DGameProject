#include "KeyInput.h"
#include "DxLib.h"
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
	GetHitKeyStateAll(_previousKey);
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
}

bool KeyInput::IsKeyInputTrigger(int keyCode)
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	GetHitKeyStateAll(_currentKey);
	const bool flag = (_currentKey[keyCode] !=0) && (_currentKey[keyCode] != _previousKey[keyCode]);
	_previousKey[keyCode] = _currentKey[keyCode];

#ifdef _DEBUG
	DrawFormatString(0,90, GetColor(255,255,255), "TriggerFlag[%d]: %d", keyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputHeld(int keyCode)
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;
	return (CheckHitKey(keyCode) !=0);
}

bool KeyInput::IsKeyInputReleased(int keyCode)
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	GetHitKeyStateAll(_currentKey);
	const bool flag = (_previousKey[keyCode] !=0) && (_currentKey[keyCode] ==0);
	_previousKey[keyCode] = _currentKey[keyCode];

#ifdef _DEBUG
	DrawFormatString(0,75, GetColor(255,255,255), "ReleasedFlag[%d]: %d", keyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputRepeated(int keyCode)
{
	if (!_isKeyInputOn) return false;
	if (keyCode <0 || keyCode >= KEY_COUNT) return false;

	bool flag = false;
	GetHitKeyStateAll(_currentKey);

	if (_currentKey[keyCode] !=0)
	{
		_repeatedTimer[keyCode] += Time::Instance().GetDeltaTime();

		if (_repeatedTime[keyCode] >0.0 && _repeatedTimer[keyCode] >= _repeatedTime[keyCode])
		{
			flag = true;
			_repeatedTimer[keyCode] =0.0;
		}
	}
	else
	{
		_repeatedTimer[keyCode] =0.0;
	}

#ifdef _DEBUG
	DrawFormatString(0,75, GetColor(255,255,255), "DeltaTime: %.4f", Time::Instance().GetDeltaTime());
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
}

void KeyInput::EndKeyInput()
{
	_isKeyInputOn = false;
}