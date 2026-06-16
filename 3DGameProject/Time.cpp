#include "Time.h"

//　シングルトンインスタンスの取得
Time& Time::Instance() noexcept {
	static Time instance;
	return instance;
}

// コンストラクタ（基準時刻を記録）
Time::Time() noexcept {
	_start = std::chrono::steady_clock::now();
	_last = _start;
	_deltaSec =0.0;
	_totalSec =0.0;
}

// 毎フレーム呼んで時間を更新（スレッド安全）
void Time::Update() noexcept {
	std::lock_guard<std::mutex> lock(_mtx);
	auto now = std::chrono::steady_clock::now();
	_deltaSec = std::chrono::duration<double>(now - _last).count();
	_totalSec = std::chrono::duration<double>(now - _start).count();
	_last = now;
}

// 前回更新からの経過秒を返す
double Time::GetDeltaTime() const noexcept {
	std::lock_guard<std::mutex> lock(_mtx);
	return _deltaSec;
}

// プログラム開始からの総経過秒を返す
double Time::GetTotalTime() const noexcept {
	std::lock_guard<std::mutex> lock(_mtx);
	return _totalSec;
}

// システムの現在時刻を秒で返す
// OSの壁時計時間を基準とする
double Time::GetWallTimeSeconds() const noexcept {
	auto now = std::chrono::system_clock::now();
	return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// 時間をリセットする
void Time::Reset() noexcept {
	std::lock_guard<std::mutex> lock(_mtx);		// スレッド安全にリセット
	_start = std::chrono::steady_clock::now();	// 現在の時刻を基準にしてスタートとラストをリセット
	_last = _start;								// 経過時間もリセット
	_deltaSec = 0.0;							// 総経過時間もリセット
	_totalSec = 0.0;							// これで次の Update() から新しい基準で時間が計測されるようになる
}