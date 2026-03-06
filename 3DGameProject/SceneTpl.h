#pragma once
#include <string>
#include <typeinfo>
#include <memory>

// 共通インターフェース（SceneManager はこれを扱う）
class IScene {
public:
	virtual ~IScene() = default;        // 仮想デストラクタ必須
	virtual void Awake() {}             // 生成直後
	virtual void Start() {}             // 開始直後
	virtual void Update(float /*dt*/) {} // 可変FPS対応の主更新入口
	virtual void Draw() {}              // 描画処理
	virtual void End() {}               // 終了直前
	virtual void OnSuspend() {}         // 一時停止直前
	virtual void OnResume() {}          // 再開直後
	virtual void OnDestroy() {}         // 破棄直前
	virtual std::string Name() const { return {}; } // シーン名取得
};

// CRTP ベースクラス：シーン名を簡単に実装可能
// 例: class TitleScene : public SceneTpl<TitleScene> { ... };
template<typename Derived>
class SceneTpl : public IScene {
public:
	virtual ~SceneTpl() = default;

	// デフォルト Name() は Derived::StaticName() があれば使い、
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
