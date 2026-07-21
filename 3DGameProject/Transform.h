#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"
#include <vector>

class GameObject;

// Transform
// - 回転の保持は Quaternion を正とする（姿勢・補間に有利）
// - Euler（ラジアン）は「編集/デバッグ表示用」として提供する
// ※ Eulerは表現の一意性がないため、表示値が変化する可能性がある
// - 親子関係を持ち、local/world を分離して dirty でキャッシュ
class Transform {
public:
	// コンストラクタ・デストラクタ
	Transform();
	~Transform();

	// 所有者の取得・設定
	void SetOwner(GameObject* owner) noexcept { _owner = owner; }
	GameObject* Owner() const noexcept { return _owner; }

	// ローカル変換要素の取得・設定
	const VECTOR& LocalPosition() const noexcept { return _localPosition; }

	// Euler は入出力用（内部は Quaternion）
	VECTOR LocalEulerRad() const noexcept; // 現時点の回転をEulerで取得（表示用）
	void SetLocalEulerRad(const VECTOR& eulerRad) noexcept; // Euler入力 -> Quaternionへ反映

	// Quaternion を直接設定（内部表現）
	const Quaternion& LocalRotation() const noexcept { return _localRotation; }
	void SetLocalRotation(const Quaternion& q) noexcept;

	// スケールは VECTOR で保持
	const VECTOR& LocalScale() const noexcept { return _localScale; }

	// ローカル変換要素の設定
	void SetLocalPosition(const VECTOR& p) noexcept;
	void SetLocalScale(const VECTOR& s) noexcept;

	// Unity の Transform.Translate に相当する移動
	void Translate(const VECTOR& delta) noexcept;		// ローカル軸方向に delta だけ移動（親回転込み）
	void TranslateWorld(const VECTOR& delta) noexcept;	// ワールド軸方向に delta だけ移動（親回転を無視）

	// 親子関係
	Transform* Parent() const noexcept { return _parent; }
	void SetParent(Transform* parent) noexcept;

	// local/world 行列の取得
	const MATRIX& LocalMatrix() const;
	const MATRIX& WorldMatrix() const;

	// ワールド変換要素の取得（親込み）
	VECTOR WorldPosition() const;
	VECTOR WorldScale() const noexcept;           // 親スケールを含んだワールドスケール
	Quaternion WorldRotation() const noexcept;    // 親回転を含んだワールド回転（Quaternion）
	VECTOR TransformPoint(const VECTOR& localPoint) const noexcept;   // ローカル点を親込みでワールドへ変換
	VECTOR TransformVector(const VECTOR& localVector) const noexcept; // ローカル方向ベクトルを親込みでワールドへ変換

	// ワールド軸での方向ベクトル取得（親回転込み）
	VECTOR WForward() const noexcept;
	VECTOR WRight()   const noexcept;
	VECTOR WUp()      const noexcept;

	// エイリアス（W なし = ワールド軸と同義。Unity の transform.forward 相当）
	VECTOR Forward() const noexcept { return WForward(); }
	VECTOR Right()   const noexcept { return WRight(); }
	VECTOR Up()      const noexcept { return WUp(); }

	// 状態を dirty にする
	void MarkDirty() noexcept;
	// dirty: 行列キャッシュが古くなった状態

private:
	// 親子関係の管理
	void AddChild(Transform* child) noexcept;		// 親の子リストに追加
	void RemoveChild(Transform* child) noexcept;	// 親の子リストから削除
	void PropagateDirtyToChildren() noexcept;		// dirty を子へ伝播

	// 所有者の GameObject
	GameObject* _owner = nullptr;	// 所有者の GameObject（Transform は GameObject に必ず1つ存在する）
	VECTOR _localPosition{};		// 位置
	Quaternion _localRotation{};	// 回転（内部表現）
	VECTOR _localScale{};			// スケール

	// 親子関係
	Transform* _parent = nullptr;		// 親の Transform
	std::vector<Transform*> _children;	// 子の Transform リスト

	// 行列キャッシュ
	mutable bool _localDirty = true;	// ローカル行列が古くなった状態
	mutable bool _worldDirty = true;	// ワールド行列が古くなった状態
	mutable MATRIX _localMatrix{};		// ローカル行列キャッシュ
	mutable MATRIX _worldMatrix{};		// ワールド行列キャッシュ
};
