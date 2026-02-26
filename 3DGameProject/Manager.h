#pragma once

// Manager
// - 共通 Manager 基底（継承ベース）
// - シングルトン本体（Instance等）は各派生に残す

class Manager {
public:
	virtual ~Manager() = default;

	// 任意：使わないなら空実装でもよい
	virtual void Initialize() {}
	virtual void Shutdown() {}
	virtual void Update() {}
};
