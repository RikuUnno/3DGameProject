# PerformanceMonitor 詳細ログ機能

## 概要
PerformanceMonitorに詳細ログ機能を追加しました。TXTファイルに以下の情報を保存します：

- **シーン情報**: 現在のシーン名
- **パフォーマンス概要**: FPS、フレーム時間、CPU使用率、GPU使用率
- **メモリ使用量**: WorkingSet、VirtualMemory、GPU専用/共有メモリ
- **システム情報**: プロセッサ数、物理メモリ総量、利用可能メモリ
- **ThreadPool状態**: ワーカー数、キューサイズ
- **スレッド詳細**: 各スレッドのCPU使用率、カーネル/ユーザー時間
- **スレッド状態**: 各スレッドの現在実行中の処理
- **プロファイリング**: 全セクションの実行時間、呼び出し回数、平均時間

## 使い方

### 1. 詳細ログの有効化
```cpp
// メインループの初期化時
PerformanceMonitor::Instance().EnableDetailedLogging(true);
```

### 2. 自動ログ保存
詳細ログが有効な場合、フレーム時間が50ms以上（約20FPS以下）になると、
1秒に1回自動的にログファイルが生成されます。

ファイル名: `PerformanceLog_YYYYMMDD_HHMMSS.txt`

### 3. 手動ログ保存
```cpp
// デフォルトファイル名で保存
PerformanceMonitor::Instance().SaveDetailedLog();

// カスタムファイル名で保存
PerformanceMonitor::Instance().SaveDetailedLog("MyCustomLog.txt");
```

### 4. シーン名の自動設定
SceneManagerが自動的に現在のシーン名を設定します。
独自に設定したい場合：
```cpp
PerformanceMonitor::Instance().SetCurrentSceneName("BattleScene");
```

## ログファイル例

```
========================================
[DETAILED LOG] Frame: 1024
========================================
Timestamp: 2024/3/15 14:30:45

--- Scene Information ---
Current Scene: GameplayScene

--- Performance Summary ---
FPS: 7.34245
Frame Time: 136.253 ms
CPU Usage: 16.356 %
GPU Usage: 45.2 %

--- Memory Usage ---
Working Set: 512.5 MB
Virtual Memory: 768.3 MB
GPU Dedicated: 1024.0 MB
GPU Shared: 256.0 MB

--- System Information ---
Processor Count: 16
Total Physical Memory: 32768.0 MB
Available Physical Memory: 16384.0 MB
Memory Load: 50 %

--- ThreadPool Status ---
Worker Count: 15
Queue Size: 0

--- Thread Details ---
[Main] TID=12345 CPU=5.2% Kernel=123456us User=234567us
[Worker 0] TID=12346 CPU=3.1% Kernel=98765us User=123456us
...

--- Thread States ---
TID=12345 State=[Physics.Update] LastUpdate=1234567890us
TID=12346 State=[Collider.SpatialPartitioning] LastUpdate=1234567891us
...

--- Profiling Sections (All) ---
Physics.Update: Time=130951us Calls=1 Avg=130951us
Physics.StepSimulation: Time=127836us Calls=4 Avg=31959us
Collider.SpatialPartitioning: Time=26734us Calls=4 Avg=6683us
...
```

## 画面表示は変更なし
画面上の表示は今まで通りです。詳細情報はTXTファイルにのみ保存されます。

## 注意事項
- 詳細ログは無効化されていれば、パフォーマンスへの影響はありません
- 自動保存は遅いフレーム検出時のみ（50ms以上）
- ファイルは追記モードで保存されるため、古いログは手動で削除してください
