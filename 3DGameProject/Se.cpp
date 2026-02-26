#include "Se.h"

#include "DxLib.h"
#include <algorithm>

#include "Transform.h"

Se::~Se() {
	Reset();
}

bool Se::Load(const std::string& filePath) {
	Reset();

	_path = filePath;
	_is3D = false;
	_handle = LoadSoundMem(filePath.c_str());
	return _handle != -1;
}

bool Se::Load3D(const std::string& filePath) {
	Reset();

	_path = filePath;
	_is3D = true;

	// 次の LoadSoundMemで作るサウンドを3D化する
	SetCreate3DSoundFlag(TRUE);
	_handle = LoadSoundMem(filePath.c_str());
	SetCreate3DSoundFlag(FALSE);

	return _handle != -1;
}

void Se::Reset() {
	Stop();
	if (_handle != -1) {
		DeleteSoundMem(_handle);
		_handle = -1;
	}
	_path.clear();
	_pan =0.0f;
	_is3D = false;
}

void Se::Play(bool top) {
	PlayPan(_pan, top);
}

float Se::ClampPan_(float pan) noexcept {
	return std::clamp(pan, -1.0f,1.0f);
}

int Se::PanToDxLib_(float pan) noexcept {
	const float p = ClampPan_(pan);
	return (int)(p *10000.0f);
}

void Se::SetPan(float pan) noexcept {
	_pan = ClampPan_(pan);
}

void Se::PlayPan(float pan, bool top) {
	if (_handle == -1) return;
	SetPan(pan);

	//3Dサウンドでも pan は存在するが、基本は3D位置で定位させる想定。
	ChangeNextPlayPanSoundMem(PanToDxLib_(_pan), _handle);
	PlaySoundMem(_handle, DX_PLAYTYPE_BACK, top ? TRUE : FALSE);
}

void Se::PlayAt(const Transform& t, bool top) {
	const VECTOR pos = t.WorldPosition();
	constexpr float kPanScale =0.2f;
	const float pan = pos.x * kPanScale;
	PlayPan(pan, top);
}

void Se::Play3DAt(const Transform& emitter, float radius, bool top) {
	if (_handle == -1) return;
	if (!_is3D) {
		// 誤用対策:3Dとしてロードされていないなら、2D再生へフォールバック
		PlayAt(emitter, top);
		return;
	}

	const VECTOR p = emitter.WorldPosition();
	SetNextPlay3DPositionSoundMem(p, _handle);
	SetNextPlay3DRadiusSoundMem(radius, _handle);
	PlaySoundMem(_handle, DX_PLAYTYPE_BACK, top ? TRUE : FALSE);
}

void Se::Stop() {
	if (_handle == -1) return;
	StopSoundMem(_handle);
}

bool Se::IsPlaying() const {
	if (_handle == -1) return false;
	return CheckSoundMem(_handle) ==1;
}
