#pragma once
// layerに指定した値とmaskに指定した値をAND演算して、0以外なら当たり判定を行う
// 例えば、layerがPLAYERでmaskがENEMYの場合、PLAYERとENEMYの当たり判定を行う

// レイヤー(Layer)の定義
struct layerMask
{
	static const int DEFAULT = 1 << 0;		// デフォルトレイヤー
	static const int TRIGGER = 1 << 1;		// トリガーレイヤー(判定はするが物理的な反応はしない)
	static const int PLAYER = 1 << 2;		// プレイヤーレイヤー
	static const int ENEMY = 1 << 3;		// エネミーレイヤー
	static const int ENVIRONMENT = 1 << 4;	// 環境レイヤー(壁など)
	static const int GROUND = 1 << 5;		// 地面レイヤー(床専用)
	static const int BALL = 1 << 6;			// 鉄球レイヤー(パチンコ玉専用)
	static const int SENSOR = 1 << 7;		// センサーレイヤー(パチンコセンサー専用)
	// etc...
};

// マスク(Mask)の定義
struct mask
{
	static const int ALL = layerMask::DEFAULT | layerMask::PLAYER | layerMask::ENEMY | layerMask::ENVIRONMENT | layerMask::TRIGGER | layerMask::GROUND | layerMask::BALL | layerMask::SENSOR;	// 全レイヤー
	static const int PLAYER = layerMask::PLAYER;									// プレイヤーのみ
	static const int ENEMY = layerMask::ENEMY;										// エネミーのみ
	static const int ENVIRONMENT = layerMask::ENVIRONMENT;							// 環境のみ
	static const int TRIGGER = layerMask::TRIGGER;									// トリガーのみ
	static const int GROUND = layerMask::GROUND;									// 地面のみ
	static const int BALL = layerMask::BALL;										// 鉄球のみ
	static const int SENSOR = layerMask::SENSOR;									// センサーのみ
	// etc...
};