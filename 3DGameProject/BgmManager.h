#pragma once

#include <string>
#include <memory>

#include "Manager.h"
#include "BgmPool.h"

// BgmManager
// - BGMの管理（3D不要、ループ再生）
// - 通常は "1曲だけ再生" を想定
class BgmManager : public Manager {
public:
	static BgmManager& Instance() noexcept;

	// 指定パスのBGMをループ再生（同じパスなら鳴らし直し）
	bool PlayLoop(const std::string& filePath, bool restart = true);
	void Stop();
	bool IsPlaying() const;

	// ボリューム等が必要ならここに追加

	void Update() override {}

private:
	BgmManager() = default;
	~BgmManager() = default;
	BgmManager(const BgmManager&) = delete;
	BgmManager& operator=(const BgmManager&) = delete;

	BgmPool _pool{4};
	BgmPool::TypedUniquePtr _current;
};
