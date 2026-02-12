#pragma once
#include <vector>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

class GameObject;

class ObjectPool {
public:
	using Creator = std::function<std::unique_ptr<GameObject>()>;   // オブジェクト生成関数
	using Deleter = std::function<void(GameObject*)>;               // カスタムデリータ
	using UniquePtr = std::unique_ptr<GameObject, Deleter>;         // カスタムデリータ付きユニークポインタ

	ObjectPool() = default; // デフォルトコンストラクタ
	explicit ObjectPool(Creator creator, size_t maxSize = 64); // コンストラクタ

	// Acquire: プールにあれば取り出し、なければ creator で生成
	UniquePtr Acquire();

	// Release: 生ポインタを pool に戻す（Deleter を介して呼ばれる）
	void Release(GameObject* obj);

	// プール内（未使用）の数
	size_t Size() const;
	void SetMaxSize(size_t maxSize);

	// freeList を全破棄（使用中のオブジェクトには触れない）
	void Clear();

	// 指定秒以上「未使用」のストックを削除（使用中のオブジェクトには触れない）
	// 戻り値: 削除した個数
	size_t TrimUnused(double maxIdleSeconds, double nowSeconds);

private:
	struct FreeEntry {
		GameObject* obj{};
		double lastReleasedSec{}; // freeList に戻った時刻（秒）
	};

	Creator _creator; // オブジェクト生成関数
	std::vector<FreeEntry> _freeList; // プール内の利用可能オブジェクトリスト
	size_t _maxSize =64; // プールの最大サイズ
	mutable std::mutex _mtx; // スレッド安全用ミューテックス
};
