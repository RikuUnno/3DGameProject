#pragma once

#include <memory>

#include "GameObject.h"
#include "PhysicsBody.h"

class CapsuleCollider;
class Collider;

// Player
// - カプセルコライダー + PhysicsBody で構成される操作キャラクター
// - WASD で水平移動、スペースでジャンプ
// - 接地判定は床法線との接触（OnCollisionStay）で行う
// - カメラ基準の移動方向に対応するため、移動の前方/右方向は外部から渡す
class Player : public GameObject {
public:
    Player();
    ~Player() override;

    // ColliderManager / PhysicsManager への登録・解除
    void Spawn(const VECTOR& position);
    void Despawn();

    void Update(float dt) override;
    void Draw() override;

    // 接地法線を受け取って接地状態を更新する
    void OnCollisionStay(Collider* self, Collider* other) override;
    void OnCollisionEnter(Collider* self, Collider* other) override;
    // 入力の基準となる水平方向（カメラの向きから与える）。正規化は内部で行う。
    void SetMoveBasis(const VECTOR& forward, const VECTOR& right) noexcept;

    // アクセサ
    PhysicsBody* GetPhysicsBody() noexcept { return &_physicsBody; }
    Collider* GetCollider() const noexcept;
    bool IsGrounded() const noexcept { return _grounded; }
    float Radius() const noexcept { return _radius; }
    float Height() const noexcept { return _height; }

    // 移動・ジャンプのパラメータ
    float moveSpeed = 8.0f;     // 水平移動速度（m/s）
    float jumpSpeed = 7.0f;     // ジャンプ初速（m/s）

private:
    void RegisterToManagers_();
    void UnregisterFromManagers_();

private:
    PhysicsBody _physicsBody{};
    std::unique_ptr<CapsuleCollider> _capsule;

    // カプセル寸法
    float _radius = 0.4f;
    float _height = 1.8f; // 全高（カプセル両端の球を含む）

    // 入力基準（水平面に投影済み・正規化済み）
    VECTOR _moveForward = VGet(0, 0, 1);
    VECTOR _moveRight   = VGet(1, 0, 0);

    // 接地状態
    bool  _grounded = false;
    bool  _touching = false;      // このフレーム、非トリガーと接触しているか
    float _groundedTimer = 0.0f;  // 最後に接地してからの経過時間（コヨーテタイム用）

    bool _registered = false;
};
