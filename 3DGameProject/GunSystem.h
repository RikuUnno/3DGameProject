#pragma once

#include "Bullet.h"
#include "DxLib.h"
#include <vector>
#include <memory>
#include <functional>

// 戦闘機の機銃システム
// ・発射レート（rpm）・装弾数・リロード時間 を変数で制御
// ・指定弾数を打ち切るとリロードへ移行、リロード完了後に再装填
// ・発射位置は外部から砲口 Transform（位置・方向）として渡す
class GunSystem
{
public:
	GunSystem(float rpm = 600.0f,	// 発射レート（発射/分）
		int   ammoCount = 30,		// 1マガジンの装弾数
		float reloadTime = 1.5f,	// リロード時間（秒）
		float bulletSpeed = 120.0f, // 弾の速度（単位/秒）
		float bulletLife = 3.0f);	// 弾の寿命（秒）

	virtual ~GunSystem() = default;

	// コピー禁止
	GunSystem(const GunSystem&) = delete;
	GunSystem& operator=(const GunSystem&) = delete;

	// 毎フレーム呼ぶ
	// muzzlePos : 砲口ワールド座標
	// muzzleDir : 砲口方向（正規化済み推奨）
	// trigger   : 発射ボタンが押されているか
	void Update(float dtSec, const VECTOR& muzzlePos, const VECTOR& muzzleDir, bool trigger);

	// 全弾の描画（DrawDebug）
	void Draw();

	// セッター
	void SetRPM			(float rpm) { _rpm = rpm; }				// 発射レート（発射/分）
	void SetAmmoCount	(int count) { _maxAmmo = count; }		// 1マガジンの装弾数
	void SetReloadTime	(float sec) { _reloadTime = sec; }		// リロード時間（秒）
	void SetBulletSpeed	(float speed) { _bulletSpeed = speed; }	// 弾の速度（単位/秒）
	void SetBulletLife	(float sec) { _bulletLife = sec; }		// 弾の寿命（秒）

	// ゲッター
	float GetRPM()				const { return _rpm; }			// 発射レート（発射/分）
	int   GetCurrentAmmo()		const { return _currentAmmo; }	//	現在の残弾数
	int   GetMaxAmmo()			const { return _maxAmmo; }		// 1マガジンの装弾数
	float GetReloadTime()		const { return _reloadTime; }	// リロード時間（秒）
	bool  IsReloading()			const { return _reloading; }	// リロード中か
	float GetReloadProgress()	const; // 0.0〜1.0				// リロード進行度（0.0=開始直後, 1.0=完了）

private:
	// 弾を発射する
	void Fire_(const VECTOR& muzzlePos, const VECTOR& muzzleDir);
	void CleanupDeadBullets_();

private:
	// パラメータ
	float _rpm          = 600.0f;  // 発射レート（発射/分）
	int   _maxAmmo      = 30;      // 1マガジンの最大装弾数
	float _reloadTime   = 1.5f;    // リロード時間（秒）
	float _bulletSpeed  = 120.0f;  // 弾の速度
	float _bulletLife   = 3.0f;    // 弾の寿命（秒）

	// 状態
	int   _currentAmmo  = 30;      // 残弾数
	float _fireTimer    = 0.0f;    // 次弾発射までの残り時間
	bool  _reloading    = false;   // リロード中フラグ
	float _reloadTimer  = 0.0f;    // リロード残り時間

	// 弾管理
	std::vector<std::unique_ptr<Bullet>> _bullets;
};
