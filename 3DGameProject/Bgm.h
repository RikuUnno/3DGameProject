#pragma once

#include <string>

// Bgm
// - BGM1曲分
// -3Dは不要、ループ再生をサポート
class Bgm {
public:
	Bgm() = default;
	~Bgm();

	Bgm(const Bgm&) = delete;
	Bgm& operator=(const Bgm&) = delete;

	bool Load(const std::string& filePath);
	void Reset();

	//ループ再生
	void PlayLoop(bool top = true);
	void Stop();
	bool IsPlaying() const;

	//ループ区間設定（秒指定）
	// - DxLibはサンプル単位が基本だが、最小実装として「0=先頭から最後までループ」に寄せる
	// - 必要になったら SetLoopSamplePosSoundMem を使う版を追加する
	void SetLoopEnable(bool enable) noexcept { _loop = enable; }
	bool LoopEnabled() const noexcept { return _loop; }

	const std::string& Path() const noexcept { return _path; }
	int Handle() const noexcept { return _handle; }

private:
	std::string _path;
	int _handle = -1;
	bool _loop = true;
};
