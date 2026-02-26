#include "BgmManager.h"

BgmManager& BgmManager::Instance() noexcept {
	static BgmManager inst;
	return inst;
}

bool BgmManager::PlayLoop(const std::string& filePath, bool restart) {
	//Šù‚É“¯‚¶BGM‚ªƒ[ƒh‚³‚ê‚Ä‚¢‚ÄÄ‹N“®•s—v‚È‚ç‰½‚à‚µ‚È‚¢
	if (_current && _current->Path() == filePath) {
		if (restart) {
			_current->PlayLoop(true);
		}
		return true;
	}

	// •Ê‹È‚È‚çŒ»Ý‚ð”jŠü(=pool‚Ö•Ô‹p)
	_current.reset();

	auto bgm = _pool.AcquireBgm();
	if (!bgm) return false;

	if (!bgm->Load(filePath)) {
		return false; // unique_ptr”jŠü‚Åpool‚Ö
	}
	bgm->SetLoopEnable(true);
	bgm->PlayLoop(true);

	_current = std::move(bgm);
	return true;
}

void BgmManager::Stop() {
	if (_current) {
		_current->Stop();
		_current.reset();
	}
}

bool BgmManager::IsPlaying() const {
	return _current ? _current->IsPlaying() : false;
}
