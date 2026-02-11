#pragma once
// layerに指定した値とmaskに指定した値をAND演算して、0以外なら当たり判定を行う
// 例えば、layerがPLAYERでmaskがENEMYの場合、PLAYERとENEMYの当たり判定を行う

// レイヤー(Layer)の定義
struct layerMask
{
	static const int DEFAULT = 1 << 0;		// デフォルトレイヤー
	static const int TRIGGER = 1 << 1;		// トリガーレイヤー(当たり判定はするが物理的な反応はしない)
	static const int PLAYER = 1 << 2;		// プレイヤーレイヤー
	static const int ENEMY = 1 << 3;		// エネミーレイヤー
	static const int ENVIRONMENT = 1 << 4;	// 環境レイヤー(地形など)
	// etc...
};

// マスク(Mask)の定義
struct mask
{
	static const int ALL = layerMask::DEFAULT | layerMask::PLAYER |layerMask::ENEMY | layerMask::ENVIRONMENT | layerMask::TRIGGER;	// 全レイヤー
	static const int PLAYER = layerMask::ENEMY | layerMask::ENVIRONMENT | layerMask::TRIGGER;										// プレイヤーが当たるレイヤー
	static const int ENEMY = layerMask::PLAYER | layerMask::ENVIRONMENT | layerMask::TRIGGER;										// エネミーが当たるレイヤー
	static const int ENVIRONMENT = layerMask::PLAYER | layerMask::ENEMY;															// 環境が当たるレイヤー
	static const int TRIGGER = layerMask::PLAYER | layerMask::ENEMY;																// トリガーが当たるレイヤー
	// etc...
};