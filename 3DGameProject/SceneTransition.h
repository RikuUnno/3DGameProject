#pragma once

#include <memory>
#include <string>

class IScene;

// SceneTransition
// - SceneManager の RequestChange と組み合わせて使う
// -旧画面/新画面をそれぞれオフスクリーンに描画して合成する
// - 合成方法
// - MaskImage: 白黒画像(マスク)を使ったトランジション（ピクセルシェーダ）
// - Shader: 任意のピクセルシェーダ（tだけ渡す）
class SceneTransition {
public:
	// トランジションの種類
	enum class Mode {
		MaskImage,	// 白黒画像をマスクにして合成する
		Shader,		// 任意のピクセルシェーダで合成する
	};

	// トランジション開始時のパラメータ
	struct Params {
		Mode mode = Mode::MaskImage; // トランジションの種類
		double durationSec = 0.5;	 // トランジションの継続時間（秒）

		// MaskImage
		std::string maskGraphPath; // 白黒画像

		// Shader
		std::string pixelShaderPath; // .pso 等
	};

	static SceneTransition& Instance() noexcept;

	// 遷移開始
	// - nextScene は遷移中に生成して描画する
	// - cutoverT:0..1 のうち、どの時点で SceneManager::ChangeScene を実行するか
	void Start(std::unique_ptr<IScene> nextScene, const Params& p, float cutoverT = 0.5f);

	// 遷移中か
	bool IsActive() const noexcept { return _active; }

	// 毎フレーム更新
	void Update(double dtSec);

	// 描画
	// - active中は通常の SceneManager::Draw を呼ばず、こちらを呼ぶ
	void Draw();

	// 終了（強制）
	void Cancel();

	// 現在の遷移時間（秒）を取得/設定
	double DurationSec() const noexcept { return _p.durationSec; }							// 遷移時間を取得
	void SetDurationSec(double sec) noexcept { _p.durationSec = (sec < 0.0) ? 0.0 : sec; }	// 遷移時間を設定（0未満は0扱い）

private:
	// 非公開コンストラクタ・デストラクタ（シングルトン）
	SceneTransition() = default;
	~SceneTransition();

	// 内部処理
	void EnsureTargets_();							// オフスクリーン描画用のグラフィックを確保
	void ReleaseResources_();						// 確保したリソースを解放
	void ApplyShaderCommon_(int psHandle, float t);	// シェーダ共通のパラメータをセット（tは0..1の遷移進行度）
	void DrawFullScreenGraph_(int graph);			// 指定グラフィックをフルスクリーンに描画（アルファブレンド）

private:
	bool _active = false;		// 遷移中フラグ
	Params _p{};				// 遷移パラメータ（開始時にコピーして保持）
	float _t = 0.0f;			// 経過時間（秒）
	float _cutoverT = 0.5f;		// シーン切替タイミング（0..1）
	bool _cutoverDone = false;	// シーン切替実行済みフラグ

	std::unique_ptr<IScene> _nextScene;	// 遷移先シーン（遷移中に生成して描画する）

	int _w = 0;	// 画面サイズ（オフスクリーン描画用グラフィックのサイズ）
	int _h = 0;	// 画面サイズ（オフスクリーン描画用グラフィックのサイズ）

	int _rtOld = -1;		// オフスクリーン描画用グラフィック（遷移元シーン）
	int _rtNew = -1;		// オフスクリーン描画用グラフィック（遷移先シーン）
	int _maskGraph = -1;	// マスク画像（MaskImageモードで使用）
	int _psHandle = -1;		// ピクセルシェーダ（Shaderモードで使用）
};
