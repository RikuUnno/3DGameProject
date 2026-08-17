#include "GunSystem.h"
#include "GunSystem.h"

#include <algorithm>
#include <cmath>

// コンストラクタ
GunSystem::GunSystem(float rpm, int ammoCount, float reloadTime,
					 float bulletSpeed, float bulletLife,
					 int shooterLayer, float bulletDamage)
	: _rpm(rpm)
	, _maxAmmo(ammoCount)
	, _currentAmmo(ammoCount)
	, _reloadTime(reloadTime)
	, _bulletSpeed(bulletSpeed)
	, _bulletLife(bulletLife)
	, _shooterLayer(shooterLayer)
	, _bulletDamage(bulletDamage)
	, _fireTimer(0.0f)
	, _reloading(false)
	, _reloadTimer(0.0f)
{
}

void GunSystem::Update(float dtSec, const VECTOR& muzzlePos, const VECTOR& muzzleDir, bool trigger)
{
	// リロード処理
	if (_reloading) {
		_reloadTimer -= dtSec;
		if (_reloadTimer <= 0.0f) {
			_reloadTimer  = 0.0f;
			_reloading    = false;
			_currentAmmo  = _maxAmmo; // 装填完了
		}
	}

	// 発射処理
	if (!_reloading && trigger && _currentAmmo > 0) {
		_fireTimer -= dtSec;
		if (_fireTimer <= 0.0f) {
			Fire_(muzzlePos, muzzleDir);
			--_currentAmmo;

			// 次弾発射間隔をリセット（rpm → 秒/発）
			const float interval = (_rpm > 0.0f) ? (60.0f / _rpm) : 999.0f;
			_fireTimer = interval;

			// 装弾数が尽きたらリロード開始
			if (_currentAmmo <= 0) {
				_reloading   = true;
				_reloadTimer = _reloadTime;
				_fireTimer   = 0.0f;
			}
		}
	} else if (!trigger) {
		// トリガーを離したら発射タイマーをリセット（連射ラグなし）
		_fireTimer = 0.0f;
	}

	// 弾の更新
	for (auto& b : _bullets) {
		if (b && b->IsAlive()) b->Update(dtSec);
	}
	CleanupDeadBullets_();
}

// 弾の描画
void GunSystem::Draw()
{
	for (auto& b : _bullets) {
		if (b && b->IsAlive()) b->Draw();
	}
}

// リロード進行度（0.0=開始直後, 1.0=完了）を取得
float GunSystem::GetReloadProgress() const
{
	if (!_reloading || _reloadTime <= 0.0f) return 1.0f;
	return 1.0f - (_reloadTimer / _reloadTime);
}

// 弾を発射する
void GunSystem::Fire_(const VECTOR& muzzlePos, const VECTOR& muzzleDir)
{
	auto bullet = std::make_unique<Bullet>();
	bullet->Fire(muzzlePos, muzzleDir, _bulletSpeed, _bulletLife, _shooterLayer, _bulletDamage);
	_bullets.push_back(std::move(bullet));
}

// 生存していない弾を削除する
void GunSystem::CleanupDeadBullets_()
{
	_bullets.erase(
		std::remove_if(_bullets.begin(), _bullets.end(),
			[](const std::unique_ptr<Bullet>& b) {
				return !b || !b->IsAlive();
			}),
		_bullets.end());
}
