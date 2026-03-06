#pragma once

// Manager
// - 各種 Manager の最小ベース
// - シングルトン本体（Instanceなど）は各派生に残す

class Manager {
public:
	virtual ~Manager() = default;

	// 任意。使わなければ空でよい
	virtual void Initialize() {}
	virtual void Shutdown() {}
	virtual void Update() {}
	virtual void Update(float /*dt*/) { Update(); }
};
