// ソース管理系のPoolクラスのテンプレート
#pragma once

class PoolTpl
{
private:
	// 派生クラスでは、オブジェクトの生成/破棄を管理するためのインターフェースを定義する想定
	PoolTpl() = default;
	virtual ~PoolTpl() = default;

	// コピー禁止
	PoolTpl(const PoolTpl&) = delete;
	PoolTpl& operator=(const PoolTpl&) = delete;

public:
	// 派生クラスで必要な共通インターフェースを定義する場合はここに追加
	
	// オブジェクトの生成/破棄
	virtual void Create() = 0;	// オブジェクトの生成処理（Poolの管理対象オブジェクトが必要なタイミングで呼ぶ想定）
	virtual void Destroy() = 0;	// オブジェクトの破棄処理（Poolの管理対象オブジェクトが不要になったタイミングで呼ぶ想定）

	// オブジェクトの取得/解放
	virtual void Acquire() = 0;	// オブジェクトの取得処理（Poolの管理対象オブジェクトが必要なタイミングで呼ぶ想定）
	virtual void Release() = 0;	// オブジェクトの解放処理（Poolの管理対象オブジェクトが不要になったタイミングで呼ぶ想定）

	// 更新
	virtual void Update() = 0;	// 毎フレームの更新処理（必要に応じて呼ぶ想定）

	// その他、必要に応じて共通のインターフェースを追加
	
};