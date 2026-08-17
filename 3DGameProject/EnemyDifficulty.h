#pragma once

// 敵AIの難易度定義
// ・倒しやすさ　= 旋回速度（турnSpeed）が遅いほど後ろを取りやすく、倒しやすい
// ・撒きやすさ　= 索敵範囲（detectionRange）・視野角（detectionHalfAngleDeg）が狭いほど
//                視野角から外れやすく、撒きやすい
// 新しい難易度を追加する場合は EnemyDifficulty に列挙子を足し、
// GetEnemyDifficultyParams() に対応する case を追加するだけで拡張できる。
enum class EnemyDifficulty
{
    Easy,	// 簡単に倒せる・すぐ撒ける
    Normal,	// 普通に倒せる・ある程度撒きにくい
    Hard,	// 倒しにくい・撒きにくい
    // 今後追加する場合はここに列挙子を足す
};

// 難易度ごとのAIパラメータ一式
struct EnemyDifficultyParams
{
    // --- 索敵（視野角）関連：撒きやすさに直結 ---
    float detectionRange     = 200.0f;	// 索敵範囲（この距離を超えると発見できない）。Normal＝200m
    float detectionHalfAngleDeg = 75.0f;	// 視野角の半角（度）。機首前方±この角度＝視野角は2倍。Normal＝150度
    float searchTurnSpeed    = 0.8f;	// 索敵中（未発見時）の旋回速度（ラジアン/秒）

    // --- 追尾・戦闘関連：倒しやすさに直結 ---
    float turnSpeed  = 1.2f;	// 発見中の旋回速度（ラジアン/秒）。遅いほど後ろを取りやすい＝倒しやすい
    float minSpeed   = 6.0f;	// 最低飛行速度
    float maxSpeed   = 28.0f;	// 最大飛行速度
    float fireRange  = 60.0f;	// 射撃を開始する距離

    // --- 体力・武器関連 ---
    float maxHp         = 50.0f;	// 最大HP
    float gunRpm        = 360.0f;	// 発射レート（発射/分）
    int   gunAmmo       = 30;		// 1マガジンの装弾数
    float gunReloadTime = 2.2f;		// リロード時間（秒）
    float gunBulletSpeed = 90.0f;	// 弾の速度
    float gunBulletLife  = 3.0f;	// 弾の寿命（秒）
    float gunBulletDamage = 6.0f;	// 弾のダメージ量
};

// 難易度に応じたパラメータを取得する
// 新しい難易度を追加する際はここに case を追加すればよい
inline EnemyDifficultyParams GetEnemyDifficultyParams(EnemyDifficulty difficulty) noexcept
{
    switch (difficulty) {
    case EnemyDifficulty::Easy: {
        EnemyDifficultyParams p;
        p.detectionRange        = 100.0f;	// 索敵範囲が狭い（100m）＝撒きやすい
        p.detectionHalfAngleDeg = 60.0f;	// 視野角が狭い（合計120度）＝撒きやすい
        p.searchTurnSpeed       = 0.6f;
        p.turnSpeed  = 0.7f;				// 旋回が遅い＝後ろを取りやすい＝倒しやすい
        p.minSpeed   = 5.0f;
        p.maxSpeed   = 20.0f;
        p.fireRange  = 45.0f;
        p.maxHp          = 30.0f;
        p.gunRpm         = 260.0f;
        p.gunAmmo        = 20;
        p.gunReloadTime  = 2.6f;
        p.gunBulletSpeed = 80.0f;
        p.gunBulletLife  = 2.5f;
        p.gunBulletDamage = 4.0f;
        return p;
    }
    case EnemyDifficulty::Hard: {
        EnemyDifficultyParams p;
        p.detectionRange        = 300.0f;	// 索敵範囲が広い（300m）＝撒きにくい
        p.detectionHalfAngleDeg = 90.0f;	// 視野角が広い（合計180度）＝撒きにくい
        p.searchTurnSpeed       = 1.1f;
        p.turnSpeed  = 1.8f;				// 旋回が速い＝後ろを取りにくい＝倒しにくい
        p.minSpeed   = 8.0f;
        p.maxSpeed   = 34.0f;
        p.fireRange  = 75.0f;
        p.maxHp          = 75.0f;
        p.gunRpm         = 480.0f;
        p.gunAmmo        = 40;
        p.gunReloadTime  = 1.8f;
        p.gunBulletSpeed = 110.0f;
        p.gunBulletLife  = 3.5f;
        p.gunBulletDamage = 9.0f;
        return p;
    }
    case EnemyDifficulty::Normal:
    default: {
        EnemyDifficultyParams p; // デフォルト値がそのまま Normal 相当
        return p;
    }
    }
}
