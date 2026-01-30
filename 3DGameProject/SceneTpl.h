#pragma once
#include <string>
#include <typeinfo>
#include <memory>

// 共通インタフェース（SceneManager はこれを扱う）
class IScene {
public:
	virtual ~IScene() = default;        // 仮想デストラクタ必須
	virtual void Awake() {}             // 初期化処理
	virtual void Start() {}             // 開始処理
	virtual void Update() {}   // 更新処理
	virtual void Draw() {}              // 描画処理
	virtual void OnSuspend() {}         // 一時停止処理
	virtual void OnResume() {}          // 再開処理
	virtual void OnDestroy() {}         // 破棄処理
	virtual std::string Name() const { return {}; } // シーン名取得
};

// CRTP ベースクラス：シーン実装はこれを継承する
// 例: class TitleScene : public SceneTpl<TitleScene> { ... };
template<typename Derived>
class SceneTpl : public IScene {
public:
	virtual ~SceneTpl() = default;

	// デフォルト Name() は Derived::StaticName() があれば使う、
	// なければ typeid の名前を返す
	virtual std::string Name() const override {
		if constexpr (std::is_same_v<void, decltype(Derived::StaticName())>) {
			return typeid(Derived).name();
		}
		else {
			return Derived::StaticName();
		}
	}
};
