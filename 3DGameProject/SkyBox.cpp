#include "SkyBox.h"

namespace {
    // VERTEX3D を作るヘルパ
	inline VERTEX3D MakeVertex(float x, float y, float z, float u, float v) noexcept {  // constexpr にできるといいけど、DxLib の型は構造体で constexpr コンストラクタがないので仕方ない
		VERTEX3D vert{};                                // 0 で初期化
		vert.pos = VGet(x, y, z);                       // 座標
		vert.norm = VGet(0.0f, 0.0f, 0.0f);             // 法線はライティングなしなので適当でいい
		vert.dif = GetColorU8(255, 255, 255, 255);      // 拡散反射色は白（テクスチャの色をそのまま出す）
		vert.spc = GetColorU8(0, 0, 0, 0);              // 鏡面反射色は黒（ライティングなしなので関係ない）
		vert.u = u;                                     // テクスチャ座標
		vert.v = v;                                     // テクスチャ座標は呼び出し側で指定する
		vert.su = 0.0f;                                 // スペキュラマップは使わないので su/sv は 0 でいい
		vert.sv = 0.0f;                                 // 頂点カラーは白（テクスチャの色をそのまま出す）
		return vert;                                    // DxLib の描画関数は頂点配列を受け取るので、ここでまとめて作っておくと便利
    }
}

SkyBox::~SkyBox() {
	Reset();    // テクスチャを解放
}

// 6 面のテクスチャをまとめて読み込む。dir に "skybox/" のような相対パスを与え、basenames[0..5] を結合する。拡張子 (".png" など) も basenames に含めて指定する。
bool SkyBox::Load(const std::string& dir, const std::array<std::string, 6>& basenames) {
    Reset();
    bool ok = true;
    for (int i = 0; i < static_cast<int>(Face::Count); ++i) {
        const std::string path = dir + basenames[i];
        const int h = LoadGraph(path.c_str());
        _handles[i] = h;
        if (h == -1) ok = false;
    }
    return ok;
}

// 個別に 1 面読み込む（テスト・差し替え用）
bool SkyBox::LoadFace(Face face, const std::string& filePath) {
    const int idx = static_cast<int>(face);
    if (_handles[idx] != -1) {
        DeleteGraph(_handles[idx]);
        _handles[idx] = -1;
    }
    _handles[idx] = LoadGraph(filePath.c_str());
    return _handles[idx] != -1;
}

// 全テクスチャを解放
void SkyBox::Reset() {
    for (int& h : _handles) {
        if (h != -1) {
            DeleteGraph(h);
            h = -1;
        }
    }
}

// 読み込み状態を返す。全ての面が有効なテクスチャを持っている場合に true。
bool SkyBox::IsLoaded() const noexcept {
    for (int h : _handles) {
        if (h == -1) return false;
    }
    return true;
}

// 描画。cameraPos を中心に常に追従させる。Z バッファ書き込みを無効にして描画する（中身は復元する）。
void SkyBox::Draw(const VECTOR& cameraPos) const {
    // skybox は Z 書き込みなし／カリングなし／ライティングなし
    SetWriteZBuffer3D(FALSE);
    SetUseZBuffer3D(FALSE);
    SetUseBackCulling(DX_CULLING_NONE);
    SetUseLighting(FALSE);

    const float s = _halfExtent;
    const float cx = cameraPos.x;
    const float cy = cameraPos.y;
    const float cz = cameraPos.z;

    // 立方体の 8 頂点（カメラ中心）
    // (±s, ±s, ±s) を符号で表す
    const VECTOR p000 = VGet(cx - s, cy - s, cz - s); // -X -Y -Z
    const VECTOR p100 = VGet(cx + s, cy - s, cz - s); // +X -Y -Z
    const VECTOR p010 = VGet(cx - s, cy + s, cz - s); // -X +Y -Z
    const VECTOR p110 = VGet(cx + s, cy + s, cz - s); // +X +Y -Z
    const VECTOR p001 = VGet(cx - s, cy - s, cz + s); // -X -Y +Z
    const VECTOR p101 = VGet(cx + s, cy - s, cz + s); // +X -Y +Z
    const VECTOR p011 = VGet(cx - s, cy + s, cz + s); // -X +Y +Z
    const VECTOR p111 = VGet(cx + s, cy + s, cz + s); // +X +Y +Z

    // 1 面 (4 頂点) を 2 三角形で描画するヘルパ
    // 引数: 左上, 右上, 右下, 左下（テクスチャを正しい向きで貼った時の四隅）
    auto drawFace = [](int handle, const VECTOR& tl, const VECTOR& tr,
                                    const VECTOR& br, const VECTOR& bl) {
        if (handle == -1) return;
        VERTEX3D verts[6];
        // tri 1: tl, tr, br
        verts[0] = MakeVertex(tl.x, tl.y, tl.z, 0.0f, 0.0f);
        verts[1] = MakeVertex(tr.x, tr.y, tr.z, 1.0f, 0.0f);
        verts[2] = MakeVertex(br.x, br.y, br.z, 1.0f, 1.0f);
        // tri 2: tl, br, bl
        verts[3] = MakeVertex(tl.x, tl.y, tl.z, 0.0f, 0.0f);
        verts[4] = MakeVertex(br.x, br.y, br.z, 1.0f, 1.0f);
        verts[5] = MakeVertex(bl.x, bl.y, bl.z, 0.0f, 1.0f);
        DrawPolygon3D(verts, 2, handle, FALSE);
    };

    // 各面を「内側から見たとき」の左上→右上→右下→左下 で渡す
    // Right (+X): 内側から +X を見る  左=-Z 右=+Z 上=+Y
    drawFace(_handles[(int)Face::Right],  p110, p111, p101, p100);
    // Left (-X): 内側から -X を見る  左=+Z 右=-Z 上=+Y
    drawFace(_handles[(int)Face::Left],   p011, p010, p000, p001);
    // Top (+Y): 180度回転
    drawFace(_handles[(int)Face::Top],    p110, p010, p011, p111);
    // Bottom (-Y): 内側から -Y を見る
    drawFace(_handles[(int)Face::Bottom], p000, p100, p101, p001);
    // Front (+Z): 内側から +Z を見る
    drawFace(_handles[(int)Face::Front],  p111, p011, p001, p101);
    // Back (-Z): 内側から -Z を見る
    drawFace(_handles[(int)Face::Back],   p010, p110, p100, p000);

    // 描画状態を復元（呼び出し側の通常設定に戻す）
    SetUseLighting(TRUE);
    SetUseBackCulling(TRUE);
    SetUseZBuffer3D(TRUE);
    SetWriteZBuffer3D(TRUE);
}
