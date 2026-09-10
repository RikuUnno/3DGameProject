#include "PachinkoMainSystem.h"

// コンストラクタ
MainSystem::MainSystem() {
	// MainSystem の初期化処理
}
// デストラクタ
MainSystem::~MainSystem() {

}

// -- 進行関係 --

void MainSystem::Initialize() {
	// MainSystem の初期化処理
	RetainedSpin = 0; // 保留数を初期化
	isMaxRetainedSpins = false; // 保留上限フラグを初期化
}

void MainSystem::Update() {
	// MainSystem の更新処理
}

void MainSystem::ShutDown() {
	// MainSystem の終了処理
}


// --- 保留処理 --- 

// OnHitChecker: チェッカーからの入賞時に呼ばれる関数
void MainSystem::OnHitChecker() {
	if (isMaxRetainedSpins) { return; } // 保留上限に達している場合は何もしない
	
	// 保留数を増やす処理
	RetainedSpin++;
	CheckMaxRetainedSpins();
}


// --- 抽選処理 ---

// Lottery: 抽選処理
void MainSystem::Lottery() {
	// 確率抽選

	




	// 抽選後の処理に移行
	AfterLottery();
}

// AfterLottery: 抽選確定後の処理
void MainSystem::AfterLottery() {
	// 抽選結果に応じた処理
}
