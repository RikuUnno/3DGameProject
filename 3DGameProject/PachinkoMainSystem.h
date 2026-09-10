#pragma once

#include <string>

// 保留の状態を管理する構造体
struct State
{
	const int NONE = 0;			// 無効の状態
	const int MISS = 1;			// ハズレの状態	
	const int CHALLENGE = 2;	// チャージの状態
	const int BIGBONUS = 3;		// 当たりの状態
};

// ボーナスの状態を管理する構造体
struct BonusState
{
	const int NONE = 0;			// 無効の状態
	const int BIG = 1;			// 通常当たり状態
	const int RUSH = 2;			// RUSHの状態(RUSH)
};

// PachinkoGameの入賞時の抽選等を行うMainSystemClass
class MainSystem
{
	
public:
	// コンストラクタ デストラクタ
	MainSystem();
	virtual ~MainSystem();

	// -- 進行関係 --

	void Initialize();	// MainSystemの初期化処理
	void Update();		// MainSystemの更新処理
	void ShutDown();	// MainSystemの終了処理

			
	// -- 保留関係 -- 
public:
	// チェッカーからの入賞時に呼ばれる関数
	void OnHitChecker();	// 保留数を増やす 


private:

	// CheckMaxRetainedSpins: もしRetainedSpinの残留数がMaxRetainedSpin以下ならisMaxRetainedSpinをfalseにより多ければtrueにする
	void CheckMaxRetainedSpins() { RetainedSpin <= MaxRetainedSpin ? isMaxRetainedSpins = false : isMaxRetainedSpins = true; }


private: 
	// -- 抽選関係 --

	// Lottery: 抽選処理
	void Lottery();	// 抽選処理

	// 抽選確定後の処理
	void AfterLottery();

	
private: // 変数

	int RetainedSpin;						// 保留数
	const int MaxRetainedSpin = 4;			// 保留数の上限
	bool isMaxRetainedSpins = false;		// 保留上限か？

};