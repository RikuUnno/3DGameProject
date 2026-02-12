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

bool KeyInput::IsKeyInputTrigger(int KeyCode)
{
	if (!_isKeyInputOn) return false;
	if (KeyCode <0 || KeyCode >= KEY_COUNT) return false;

	GetHitKeyStateAll(_currentKey);
	const bool flag = (_currentKey[KeyCode] !=0) && (_currentKey[KeyCode] != _previousKey[KeyCode]);
	_previousKey[KeyCode] = _currentKey[KeyCode];

#ifdef _DEBUG
	DrawFormatString(0,90, GetColor(255,255,255), "TriggerFlag[%d]: %d", KeyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputHeld(int KeyCode)
{
	if (!_isKeyInputOn) return false;
	if (KeyCode <0 || KeyCode >= KEY_COUNT) return false;
	return (CheckHitKey(KeyCode) !=0);
}

bool KeyInput::IsKeyInputReleased(int KeyCode)
{
	if (!_isKeyInputOn) return false;
	if (KeyCode <0 || KeyCode >= KEY_COUNT) return false;

	GetHitKeyStateAll(_currentKey);
	const bool flag = (_previousKey[KeyCode] !=0) && (_currentKey[KeyCode] ==0);
	_previousKey[KeyCode] = _currentKey[KeyCode];

#ifdef _DEBUG
	DrawFormatString(0,75, GetColor(255,255,255), "ReleasedFlag[%d]: %d", KeyCode, flag);
#endif

	return flag;
}

bool KeyInput::IsKeyInputRepeated(int KeyCode)
{
	if (!_isKeyInputOn) return false;
	if (KeyCode <0 || KeyCode >= KEY_COUNT) return false;

	bool flag = false;
	GetHitKeyStateAll(_currentKey);

	if (_currentKey[KeyCode] !=0)
	{
		_repeatedTimer[KeyCode] += Time::Instance().GetDeltaTime();

		if (_repeatedTime[KeyCode] >0.0 && _repeatedTimer[KeyCode] >= _repeatedTime[KeyCode])
		{
			flag = true;
			_repeatedTimer[KeyCode] =0.0;
		}
	}
	else
	{
		_repeatedTimer[KeyCode] =0.0;
	}

#ifdef _DEBUG
	DrawFormatString(0,75, GetColor(255,255,255), "DeltaTime: %.4f", Time::Instance().GetDeltaTime());
	DrawFormatString(0,90, GetColor(255,255,255), "RepeatedTimer[%d]: %.4f", KeyCode, _repeatedTimer[KeyCode]);
	DrawFormatString(0,105, GetColor(255,255,255), "RepeatedFlag[%d]: %d", KeyCode, flag);
#endif

	return flag;
}

void KeyInput::SetInputRepeatedTime(int KeyCode, double SetTime)
{
	if (KeyCode <0 || KeyCode >= KEY_COUNT) return;
	if (SetTime <0.0) SetTime =0.0;
	_repeatedTime[KeyCode] = SetTime;
}

void KeyInput::BeginKeyInput()
{
	_isKeyInputOn = true;
}

void KeyInput::EndKeyInput()
{
	_isKeyInputOn = false;
}