#include "Bgm.h"

#include "DxLib.h"

Bgm::~Bgm() {
	Reset();
}

bool Bgm::Load(const std::string& filePath) {
	Reset();
	_path = filePath;
	_handle = LoadSoundMem(filePath.c_str());
	return _handle != -1;
}

void Bgm::Reset() {
	Stop();
	if (_handle != -1) {
		DeleteSoundMem(_handle);
		_handle = -1;
	}
	_path.clear();
	_loop = true;
}

void Bgm::PlayLoop(bool top) {
	if (_handle == -1) return;
	//ループは PlaySoundMem の再生タイプで指定
	const int playType = _loop ? DX_PLAYTYPE_LOOP : DX_PLAYTYPE_BACK;
	PlaySoundMem(_handle, playType, top ? TRUE : FALSE);
}

void Bgm::Stop() {
	if (_handle == -1) return;
	StopSoundMem(_handle);
}

bool Bgm::IsPlaying() const {
	if (_handle == -1) return false;
	return CheckSoundMem(_handle) ==1;
}
