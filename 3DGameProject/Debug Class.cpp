#include "Debug Class.h"

#include "ColliderManager.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "DxLib.h"
#include "SceneManager.h"

namespace {
	class DebugSphereCollider final : public SphereCollider {
	public:
		explicit DebugSphereCollider(DebugSphereObject* ownerObj) : ownerObj_(ownerObj) {}
		void OnCollisionEnter(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(true); }
		void OnCollisionStay(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(true); }
		void OnCollisionExit(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(false); }
		void OnTriggerEnter(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(true); }
		void OnTriggerStay(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(true); }
		void OnTriggerExit(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(false); }
	private:
		DebugSphereObject* ownerObj_{};
	};

	class DebugBoxCollider final : public BoxCollider {
	public:
		explicit DebugBoxCollider(DebugBoxObject* ownerObj) : ownerObj_(ownerObj) {}
		void OnCollisionEnter(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(true); }
		void OnCollisionStay(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(true); }
		void OnCollisionExit(Collider*) override { if (ownerObj_) ownerObj_->SetColliding(false); }
		void OnTriggerEnter(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(true); }
		void OnTriggerStay(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(true); }
		void OnTriggerExit(Collider*) override { if (ownerObj_) ownerObj_->SetTriggering(false); }
	private:
		DebugBoxObject* ownerObj_{};
	};
}

SphereCollider* DebugSphereObject::GetCollider() const noexcept {
	return dynamic_cast<SphereCollider*>(collider_.get());
}

BoxCollider* DebugBoxObject::GetCollider() const noexcept {
	return dynamic_cast<BoxCollider*>(collider_.get());
}

namespace DebugClass {
	void DrawSimple3DDebug() {
		const int half =10;
		const float step =1.0f;
		const unsigned int colGrid = GetColor(60,60,60);
		for (int i = -half; i <= half; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x,0.0f, -half * step), VGet(x,0.0f, half * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-half * step,0.0f, z), VGet(half * step,0.0f, z), colGrid);
		}

		DrawLine3D(VGet(0,0,0), VGet(2,0,0), GetColor(255,80,80));
		DrawLine3D(VGet(0,0,0), VGet(0,2,0), GetColor(80,255,80));
		DrawLine3D(VGet(0,0,0), VGet(0,0,2), GetColor(80,80,255));
	}
}

DebugSphereObject::DebugSphereObject(float radius, int layerBits, int maskBits) {
	// 現在シーンに属する（ColliderManagerのscene filter対策）
	ownerSceneId = SceneManager::Instance().CurrentSceneId();

	collider_ = std::make_unique<DebugSphereCollider>(this);
	collider_->owner = this;
	auto* s = dynamic_cast<SphereCollider*>(collider_.get());
	if (s) {
		s->sphere_.radius = radius;
	}
	collider_->layer = layerBits;
	collider_->mask = maskBits;
	ColliderManager::GetInstance().RegisterCollider(collider_.get());
}

DebugSphereObject::~DebugSphereObject() {
	ColliderManager::GetInstance().UnregisterCollider(collider_.get());
}

void DebugSphereObject::Draw() {
	auto* s = dynamic_cast<SphereCollider*>(collider_.get());
	const float r = s ? s->sphere_.radius :0.5f;

	// 状態で色分け（検証用）
	const unsigned int col = IsColliding() ? GetColor(255,80,80)
		: (IsTriggering() ? GetColor(255,220,80) : GetColor(80,200,200));

	DrawSphere3D(transform.WorldPosition(), r,20, col, col, FALSE);
	DrawSphere3D(transform.WorldPosition(),0.05f,8, col, col, TRUE);
}

DebugBoxObject::DebugBoxObject(const VECTOR& halfExtents, int layerBits, int maskBits) {
	// 現在シーンに属する（ColliderManagerのscene filter対策）
	ownerSceneId = SceneManager::Instance().CurrentSceneId();

	collider_ = std::make_unique<DebugBoxCollider>(this);
	collider_->owner = this;
	auto* b = dynamic_cast<BoxCollider*>(collider_.get());
	if (b) {
		b->box_.halfExtents = halfExtents;
	}
	collider_->layer = layerBits;
	collider_->mask = maskBits;
	ColliderManager::GetInstance().RegisterCollider(collider_.get());
}

DebugBoxObject::~DebugBoxObject() {
	ColliderManager::GetInstance().UnregisterCollider(collider_.get());
}

void DebugBoxObject::Draw() {
	auto* boxCol = dynamic_cast<BoxCollider*>(collider_.get());
	if (!boxCol) {
		const VECTOR c = transform.WorldPosition();
		DrawSphere3D(c,0.07f,8, GetColor(255,80,80), GetColor(255,80,80), TRUE);
		return;
	}

	const VECTOR c = boxCol->box_.center;
	const VECTOR ax = boxCol->box_.axisX;
	const VECTOR ay = boxCol->box_.axisY;
	const VECTOR az = boxCol->box_.axisZ;
	const VECTOR he = boxCol->box_.halfExtents;

	const unsigned int col = IsColliding() ? GetColor(255,80,80)
		: (IsTriggering() ? GetColor(255,220,80) : GetColor(80,200,200));

	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = c;
		p = VAdd(p, VScale(ax, he.x * sx));
		p = VAdd(p, VScale(ay, he.y * sy));
		p = VAdd(p, VScale(az, he.z * sz));
		return p;
	};

	const VECTOR p000 = Corner(-1,-1,-1);
	const VECTOR p001 = Corner(-1,-1,1);
	const VECTOR p010 = Corner(-1,1,-1);
	const VECTOR p011 = Corner(-1,1,1);
	const VECTOR p100 = Corner(1,-1,-1);
	const VECTOR p101 = Corner(1,-1,1);
	const VECTOR p110 = Corner(1,1,-1);
	const VECTOR p111 = Corner(1,1,1);

	// wireframe edges
	DrawLine3D(p000, p001, col);
	DrawLine3D(p001, p011, col);
	DrawLine3D(p011, p010, col);
	DrawLine3D(p010, p000, col);

	DrawLine3D(p100, p101, col);
	DrawLine3D(p101, p111, col);
	DrawLine3D(p111, p110, col);
	DrawLine3D(p110, p100, col);

	DrawLine3D(p000, p100, col);
	DrawLine3D(p001, p101, col);
	DrawLine3D(p010, p110, col);
	DrawLine3D(p011, p111, col);

	DrawSphere3D(c,0.06f,8, col, col, TRUE);
}
