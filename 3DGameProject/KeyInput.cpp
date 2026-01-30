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
    // 起動時のキー状態を取り込んでおく
    GetHitKeyStateAll(m_previousKey);
}

KeyInput::~KeyInput()
{
}

void KeyInput::Initialize()
{
    IsKeyInputON = false;
    std::fill(std::begin(m_currntKey), std::end(m_currntKey), 0);
    std::fill(std::begin(m_previousKey), std::end(m_previousKey), 0);
    std::fill(std::begin(m_repeatedTime), std::end(m_repeatedTime), 1.0); // デフォルト1秒
    std::fill(std::begin(m_repeatedTimer), std::end(m_repeatedTimer), 0.0);
}

bool KeyInput::IsKeyInputTrigger(int KeyCode)
{
    if (!IsKeyInputON) return false;
    if (KeyCode < 0 || KeyCode >= KEY_COUNT) return false;

    bool flag = false;
    // 現在の押下情報を更新
    GetHitKeyStateAll(m_currntKey);

    // 前フレームと比べて変化あり && 今回押されている（押された瞬間）
    flag = (m_currntKey[KeyCode] != 0) && (m_currntKey[KeyCode] != m_previousKey[KeyCode]);

    // 次フレーム用に保存
    m_previousKey[KeyCode] = m_currntKey[KeyCode];

#ifdef _DEBUG
    DrawFormatString(0, 90, GetColor(255,255,255), "TriggerFlag[%d]: %d", KeyCode, flag);
#endif

    return flag;
}

bool KeyInput::IsKeyInputHeld(int KeyCode)
{
    if (!IsKeyInputON) return false;
    if (KeyCode < 0 || KeyCode >= KEY_COUNT) return false;

    // DxLib の CheckHitKey を使う
    return (CheckHitKey(KeyCode) != 0);
}

bool KeyInput::IsKeyInputReleased(int KeyCode)
{
    if (!IsKeyInputON) return false;
    if (KeyCode < 0 || KeyCode >= KEY_COUNT) return false;

    bool flag = false;
    GetHitKeyStateAll(m_currntKey);

    // 前フレームが押されていて今回離れているなら離された瞬間
    flag = (m_previousKey[KeyCode] != 0) && (m_currntKey[KeyCode] == 0);

    // 次フレームのために更新
    m_previousKey[KeyCode] = m_currntKey[KeyCode];

#ifdef _DEBUG
    DrawFormatString(0, 75, GetColor(255,255,255), "ReleasedFlag[%d]: %d", KeyCode, flag);
#endif

    return flag;
}

bool KeyInput::IsKeyInputRepeated(int KeyCode)
{
    if (!IsKeyInputON) return false;
    if (KeyCode < 0 || KeyCode >= KEY_COUNT) return false;

    bool flag = false;
    GetHitKeyStateAll(m_currntKey);

    // キーが押されている場合、経過時間を加算して閾値超えで true を返す
    if (m_currntKey[KeyCode] != 0)
    {
        m_repeatedTimer[KeyCode] += Time::Instance().GetDeltaTime();

        if (m_repeatedTime[KeyCode] > 0.0 && m_repeatedTimer[KeyCode] >= m_repeatedTime[KeyCode])
        {
            flag = true;
            m_repeatedTimer[KeyCode] = 0.0;
        }
    }
    else
    {
        // キーが離されたらタイマーを初期化
        m_repeatedTimer[KeyCode] = 0.0;
    }

#ifdef _DEBUG
    DrawFormatString(0, 75, GetColor(255,255,255), "DeltaTime: %.4f", Time::Instance().GetDeltaTime());
    DrawFormatString(0, 90, GetColor(255,255,255), "RepeatedTimer[%d]: %.4f", KeyCode, m_repeatedTimer[KeyCode]);
    DrawFormatString(0, 105, GetColor(255,255,255), "RepeatedFlag[%d]: %d", KeyCode, flag);
#endif

    return flag;
}

void KeyInput::SetInputRepeatedTime(int KeyCode, double SetTime)
{
    if (KeyCode < 0 || KeyCode >= KEY_COUNT) return;
    if (SetTime < 0.0) SetTime = 0.0;
    m_repeatedTime[KeyCode] = SetTime;
}

void KeyInput::BeginKeyInput()
{
    IsKeyInputON = true;
}

void KeyInput::EndKeyInput()
{
    IsKeyInputON = false;
}