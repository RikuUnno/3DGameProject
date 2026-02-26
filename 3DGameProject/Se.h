#pragma once

#include <string>

class Transform;

class Se {
public:
	Se() = default;
	~Se();

	Se(const Se&) = delete;
	Se& operator=(const Se&) = delete;

	//2D sound load
	bool Load(const std::string& filePath);
	//3D sound load (3Dフラグ付きで作成)
	bool Load3D(const std::string& filePath);

	void Reset();

	// --- play ---
	void Play(bool top = true);
	void PlayPan(float pan, bool top = true);
	void PlayAt(const Transform& t, bool top = true); // 簡易pan版(後方互換)

	//3D: 次の再生のみに使う位置/距離を設定して再生
	void Play3DAt(const Transform& emitter, float radius, bool top = true);

	void Stop();
	bool IsPlaying() const;

	const std::string& Path() const noexcept { return _path; }
	int Handle() const noexcept { return _handle; }

	void SetPan(float pan) noexcept;
	float Pan() const noexcept { return _pan; }

private:
	static float ClampPan_(float pan) noexcept;
	static int PanToDxLib_(float pan) noexcept;

	std::string _path;
	int _handle = -1;
	float _pan =0.0f;
	bool _is3D = false;
};
