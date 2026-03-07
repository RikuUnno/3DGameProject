#include "SeManager.h"

#include <algorithm>

#include "Transform.h"
#include "Se.h"
#include "DxLib.h"

SeManager& SeManager::Instance() noexcept {
	static SeManager inst;
	return inst;
}

void SeManager::ApplyListener_() {
	if (!_listener) return;

	const VECTOR pos = _listener->WorldPosition();
	// forward/up は Transform のローカル軸ヘルパからワールド方向で返る
	const VECTOR forward = _listener->Forward();
	const VECTOR up = _listener->Up();
	const VECTOR frontPos = VAdd(pos, forward);

	Set3DSoundListenerPosAndFrontPosAndUpVec(pos, frontPos, up);
}

void SeManager::PlayInternal_(
	const std::string& filePath,
	bool want3D,
	const std::function<void(Se*)>& playFn
) {
	auto se = _pool.AcquireSe();
	if (!se) return;

	//同一インスタンスでも別ロード/別パスならロードし直す
	const bool needReload = (se->Path() != filePath);
	if (needReload) {
		bool ok = false;
		if (want3D) ok = se->Load3D(filePath);
		else ok = se->Load(filePath);
		if (!ok) return;
	}

	// リスナーを更新してから再生（3D/通常どちらでもよい）
	ApplyListener_();

	if (playFn) playFn(se.get());

	_playing.push_back(std::move(se));
}

void SeManager::Play(const std::string& filePath, bool top) {
	PlayInternal_(filePath, false, [top](Se* se) { se->Play(top); });
}

void SeManager::PlayPan(const std::string& filePath, float pan, bool top) {
	PlayInternal_(filePath, false, [pan, top](Se* se) { se->PlayPan(pan, top); });
}

void SeManager::PlayAt(const std::string& filePath, const Transform& t, bool top) {
	PlayInternal_(filePath, false, [&t, top](Se* se) { se->PlayAt(t, top); });
}

void SeManager::Play3DAt(const std::string& filePath, const Transform& emitter, float radius, bool top) {
	PlayInternal_(filePath, true, [&emitter, radius, top](Se* se) { se->Play3DAt(emitter, radius, top); });
}

void SeManager::Update(float /*dt*/) {
	// 毎フレーム listener を反映（移動に追随させる）
	ApplyListener_();

	auto it = _playing.begin();
	while (it != _playing.end()) {
		Se* se = it->get();
		if (!se) {
			it = _playing.erase(it);
			continue;
		}
		if (!se->IsPlaying()) {
			it = _playing.erase(it);
			continue;
		}
		++it;
	}
}

void SeManager::StopAll() {
	for (auto& se : _playing) {
		if (se) se->Stop();
	}
	_playing.clear();
}
