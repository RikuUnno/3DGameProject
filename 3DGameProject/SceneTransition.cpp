#include "SceneTransition.h"

#include "DxLib.h"

#include "SceneManager.h"

namespace {
	void DrawFullScreenQuadToShader(int w, int h) {
		VERTEX2DSHADER v[4]{};

		v[0].pos = VGet(0.0f,0.0f,0.0f);
		v[1].pos = VGet((float)w,0.0f,0.0f);
		v[2].pos = VGet(0.0f,(float)h,0.0f);
		v[3].pos = VGet((float)w,(float)h,0.0f);

		v[0].rhw = v[1].rhw = v[2].rhw = v[3].rhw =1.0f;

		// dif は BYTE(0-255) の構造体想定
		v[0].dif = {255,255,255,255 };
		v[1].dif = {255,255,255,255 };
		v[2].dif = {255,255,255,255 };
		v[3].dif = {255,255,255,255 };

		v[0].u =0.0f; v[0].v =0.0f;
		v[1].u =1.0f; v[1].v =0.0f;
		v[2].u =0.0f; v[2].v =1.0f;
		v[3].u =1.0f; v[3].v =1.0f;

		VERTEX2DSHADER tri[6] = { v[0],v[1],v[2], v[2],v[1],v[3] };
		DrawPolygon2DToShader(tri,2);
	}
}

SceneTransition& SceneTransition::Instance() noexcept {
	static SceneTransition inst;
	return inst;
}

SceneTransition::~SceneTransition() {
	ReleaseResources_();
}

void SceneTransition::ReleaseResources_() {
	if (_rtOld != -1) {
		DeleteGraph(_rtOld);
		_rtOld = -1;
	}
	if (_rtNew != -1) {
		DeleteGraph(_rtNew);
		_rtNew = -1;
	}
	if (_maskGraph != -1) {
		DeleteGraph(_maskGraph);
		_maskGraph = -1;
	}
	if (_psHandle != -1) {
		DeleteShader(_psHandle);
		_psHandle = -1;
	}
}

void SceneTransition::EnsureTargets_() {
	int w =0, h =0, bpp =0;
	GetScreenState(&w, &h, &bpp);
	if (w <=0 || h <=0) return;

	if (_w == w && _h == h && _rtOld != -1 && _rtNew != -1) return;

	ReleaseResources_();
	_w = w;
	_h = h;

	_rtOld = MakeScreen(_w, _h, TRUE);
	_rtNew = MakeScreen(_w, _h, TRUE);
}

void SceneTransition::Start(std::unique_ptr<IScene> nextScene, const Params& p, float cutoverT) {
	Cancel();

	_p = p;
	_cutoverT = (cutoverT <0.0f) ?0.0f : (cutoverT >1.0f ?1.0f : cutoverT);
	_t =0.0f;
	_cutoverDone = false;
	_nextScene = std::move(nextScene);
	_active = true;

	EnsureTargets_();

	if (!_p.maskGraphPath.empty()) {
		_maskGraph = LoadGraph(_p.maskGraphPath.c_str());
	}
	if (!_p.pixelShaderPath.empty()) {
		_psHandle = LoadPixelShader(_p.pixelShaderPath.c_str());
	}

	// 次シーンを事前初期化（描画できる状態にする）
	if (_nextScene) {
		_nextScene->Awake();
		_nextScene->Start();
	}
}

void SceneTransition::Cancel() {
	_active = false;
	_nextScene.reset();
	_p = Params{};
	_t =0.0f;
	_cutoverDone = false;
	ReleaseResources_();
}

void SceneTransition::Update(double dtSec) {
	if (!_active) return;
	if (_p.durationSec <=0.0001) {
		_t =1.0f;
	} else {
		_t += (float)(dtSec / _p.durationSec);
		if (_t >1.0f) _t =1.0f;
	}

	// SceneManager の実体切替（オブジェクト解放などをこのタイミングで行う）
	if (!_cutoverDone && _t >= _cutoverT) {
		_cutoverDone = true;
		SceneManager::Instance().ChangeScene(std::move(_nextScene));
	}

	if (_t >=1.0f) {
		_active = false;
		//ここで _nextScene は既に move 済み
	}
}

void SceneTransition::DrawFullScreenGraph_(int graph) {
	if (graph == -1) return;
	DrawExtendGraph(0,0, _w, _h, graph, FALSE);
}

void SceneTransition::ApplyShaderCommon_(int psHandle, float t) {
	if (psHandle == -1) return;
	FLOAT4 c{ t,0,0,0 };
	SetPSConstF(0, c);
	SetUsePixelShader(psHandle);
}

void SceneTransition::Draw() {
	if (!_active) {
		// 通常描画
		SceneManager::Instance().Draw();
		return;
	}

	EnsureTargets_();
	if (_rtOld == -1 || _rtNew == -1) {
		SceneManager::Instance().Draw();
		return;
	}

	// old: 現在 SceneManager が持っているシーン
	SetDrawScreen(_rtOld);
	ClearDrawScreen();
	SceneManager::Instance().Draw();

	// new: 切替前は _nextScene を描画、切替後は SceneManager の新シーンを描画
	SetDrawScreen(_rtNew);
	ClearDrawScreen();
	if (!_cutoverDone && _nextScene) {
		_nextScene->Draw();
	} else {
		SceneManager::Instance().Draw();
	}

	// backbuffer
	SetDrawScreen(DX_SCREEN_BACK);

	const bool canShader = (_psHandle != -1);
	if ((_p.mode == Mode::MaskImage) && canShader && (_maskGraph != -1)) {
		// shader: tex0=old, tex1=new, tex2=mask
		SetRenderTargetToShader(DX_SCREEN_BACK, -1);
		SetUseTextureToShader(0, _rtOld);
		SetUseTextureToShader(1, _rtNew);
		SetUseTextureToShader(2, _maskGraph);
		ApplyShaderCommon_(_psHandle, _t);
		DrawFullScreenQuadToShader(_w, _h);
		SetUsePixelShader(-1);
		return;
	}

	if ((_p.mode == Mode::Shader) && canShader) {
		// 任意シェーダ（tex0 old / tex1 new）
		SetRenderTargetToShader(DX_SCREEN_BACK, -1);
		SetUseTextureToShader(0, _rtOld);
		SetUseTextureToShader(1, _rtNew);
		ApplyShaderCommon_(_psHandle, _t);
		DrawFullScreenQuadToShader(_w, _h);
		SetUsePixelShader(-1);
		return;
	}

	// shader無し: 単純クロスフェード
	DrawFullScreenGraph_(_rtOld);
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, (int)(_t *255));
	DrawFullScreenGraph_(_rtNew);
	SetDrawBlendMode(DX_BLENDMODE_NOBLEND,0);
}
