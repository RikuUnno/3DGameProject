#include "Collider.h"
#include "ColliderManager.h"

// 仮想デストラクタの実装。
// Collider が破棄される際、ColliderManager に登録されたままだと
// ダングリングポインタが残り、後続フレームの走査で vtable 参照が
// 不正アドレスとなりアクセス違反を引き起こす。
// ここで必ず登録解除する。
Collider::~Collider() {
    // ColliderManager 自体が破棄シーケンスに入っている場合は何もしない
    // (UnregisterCollider 側でも _shuttingDown を見ているが二重防御)
    if (!ColliderManager::Instance().IsShuttingDown()) {
        ColliderManager::Instance().UnregisterCollider(this);
    }
}