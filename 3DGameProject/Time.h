#pragma once

#include <chrono>
#include <mutex>
#include <atomic>

class Time
{
public:
	// シングルトンインスタンスの取得
	static Time& Instance() noexcept;

	// 時間の更新
	void Update() noexcept;

	// 経過時間の取得
	double GetDeltaTime() const noexcept; // 前回更新からの経過時間を秒で取得
	double GetTotalTime() const noexcept; // プログラム開始からの経過時間を秒で取得
	double GetWallTimeSeconds() const noexcept; // システムの壁時計時間を秒で取得

	// 時間のリセット
	void Reset() noexcept;

private:
	// コンストラクタ
	Time() noexcept;

	// 開始時刻と前回更新時刻
	std::chrono::steady_clock::time_point _start;
	std::chrono::steady_clock::time_point _last;

	// mutex
	mutable std::mutex _mtx;

	// 経過時間
	std::atomic<double> _deltaSec{ 0.0 };
	// 総経過時間
	std::atomic<double> _totalSec{ 0.0 };
};