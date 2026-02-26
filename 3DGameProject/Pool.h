#pragma once

#include <memory>
#include <functional>
#include <cstddef>

class Pool {
public:
	// 汎用的なプールインターフェース
	using Deleter = std::function<void(void*)>;
	// カスタムデリータ付きユニークポインタ（void* を使用して型非依存にする）
	using UniquePtr = std::unique_ptr<void, Deleter>;

	// デストラクタは仮想にしておく（継承前提）
	virtual ~Pool() = default;

	// Acquire: プールからオブジェクトを取得する。利用後は UniquePtr のデリータを通じて Release される想定。
	virtual UniquePtr Acquire() =0;
	// Release: プールへオブジェクトを返却する。Acquire で返された UniquePtr のデリータから呼び出される想定。
	virtual void Release(void* obj) =0;

	// プール内の利用可能オブジェクト数を返す
	virtual size_t Size() const =0;
	// プールの最大サイズを設定する
	virtual void SetMaxSize(size_t maxSize) =0;
	// freeList を全破棄する（使用中のオブジェクトには触れない）
	virtual void Clear() =0;
	// 指定秒以上「未使用」のストックを削除する（使用中のオブジェクトには触れない）。戻り値: 削除した個数
	virtual size_t TrimUnused(double maxIdleSeconds, double nowSeconds) =0;
};
