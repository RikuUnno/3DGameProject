// ソース管理系のManagerクラスのテンプレート
#pragma once

class ManagerTpl {
private:
	// 派生クラスにはシングルトンアクセス用の GetInstance() を実装させる想定
	ManagerTpl() = default;
	virtual ~ManagerTpl() = default;

	// コピー禁止
	ManagerTpl(const ManagerTpl&) = delete;
	ManagerTpl& operator=(const ManagerTpl&) = delete;

public:
	// 派生クラスで必要な共通インターフェースを定義する場合はここに追加

	// 登録/削除
	virtual void Register() = 0;	// 登録処理（Managerの管理対象オブジェクトが生成されるタイミングで呼ぶ想定）
	virtual void Unregister() = 0;	// 登録/削除は、Managerの管理対象オブジェクトが生成/破棄されるタイミングで呼ぶ想定

	// 初期化/終了
	virtual void Initialize() = 0;	// 初期化処理（mainの最初で呼ぶ）
	virtual void Shutdown() = 0;	// 終了処理（mainの最後で呼ぶ）

	// 更新
	virtual void Update() = 0;		// 毎フレームの更新処理

};