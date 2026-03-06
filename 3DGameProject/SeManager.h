#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>

#include "Manager.h"
#include "SePool.h"

// 前方宣言
class Transform;

// SeManager
// - 効果音(SE)の管理
// - 役割:
// - 再利用プール（SePool）
// -複数同時再生
// -追加のたびに再利用（同じSEを複数重ねて再生するためインスタンスを都度確保）
class SeManager : public Manager {
public:
	static SeManager& Instance() noexcept;

	//2D
	// SEを再生（必要ならプールインスタンスを確保して再利用）
	// - filePath: wav/ogg等（DxLibが解釈できる想定）
	// - top: TRUEで先頭再生（同じSEを切って鳴らし直す用途）
	void Play(const std::string& filePath, bool top = true);

	// パン・ポジション指定再生
	void PlayPan(const std::string& filePath, float pan, bool top = true);
	void PlayAt(const std::string& filePath, const Transform& t, bool top = true);

	//3D
	// - radius:音が聞こえる距離（ゲーム側のスケールに合わせる）
	void Play3DAt(const std::string& filePath, const Transform& emitter, float radius, bool top = true);

	// listener（聞く側）
	// - 通常は RenderCamera の Transform を渡す
	void SetListener(const Transform* listener) noexcept { _listener = listener; }
	const Transform* Listener() const noexcept { return _listener; }

	// 再生終了したインスタンスをプールへ戻す
	void Update() override;
	void Update(float dt) override { (void)dt; Update(); }

	// 全停止（再生中をすべて止め、プールへ戻す）
	void StopAll();

	// キャッシュ系（sound handle）の使い回しをしたくなった場合の拡張ポイント
	// 現状は Se が個別に LoadSoundMemする（最小実装）

	void SetPoolMaxSize(size_t n) { _pool.SetMaxSize(n); }
	size_t PoolFreeSize() const { return _pool.Size(); }

private:
	SeManager() = default;
	~SeManager() = default;
	SeManager(const SeManager&) = delete;
	SeManager& operator=(const SeManager&) = delete;

	void PlayInternal_(
		const std::string& filePath,
		bool want3D,
		const std::function<void(class Se*)>& playFn
	);

	void ApplyListener_();

	// 再生中インスタンス
	std::vector<SePool::TypedUniquePtr> _playing;
	SePool _pool{64};

	const Transform* _listener = nullptr;
};
