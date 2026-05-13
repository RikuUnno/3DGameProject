# パフォーマンス最適化 - コミットサマリ

## 変更ファイル
- `ColliderManager.cpp` - 衝突検出システムの最適化
- `PerformanceMonitor.h/cpp` - 詳細ログ機能追加
- `SceneManager.cpp` - シーン名自動設定
- `PerformanceMonitor_Usage.md` - 使用方法ドキュメント

## 最適化内容

### 1. ColliderManager最適化
#### GridCellCompute高速化
- 不要な`reserve()`呼び出しを削減（既にcapacityがある場合はスキップ）
- grain sizeを32→64に増やしてスレッド同期コストを削減

#### GridMerge並列化
- 100個未満: シングルスレッド（ロックオーバーヘッド回避）
- 100個以上: バッチ並列処理（32個ずつ処理してロック回数を1/32に削減）

#### BroadPhase高速化
- `unordered_set`による重複削除 → ソート+線形走査に変更
- キャッシュヒット率向上、ハッシュ計算コスト削減

#### NarrowPhase調整
- grain sizeを8→16に増やしてバリア待機コストを削減

### 2. PerformanceMonitor詳細ログ機能
- TXTファイルへの詳細パフォーマンスログ出力
- シーン名、CPU/GPU使用率、メモリ使用量、スレッド状態、全プロファイリングセクションを記録
- 遅いフレーム検出時の自動保存機能（50ms以上のフレーム）
- SceneManagerとの連携によるシーン名自動設定

## パフォーマンス改善結果

| 指標 | Before | After | 改善率 |
|------|--------|-------|--------|
| Frame Time | 145.7ms | 46.7ms | **67.9%削減** |
| FPS | 7.85 | 21.4 | **173%向上** |
| SpatialPartitioning | 33.2ms | 4.0ms | **87.9%削減** |
| GridMerge | 5.5ms | 0.9ms | **83.6%削減** |
| BroadPhase | 5.4ms | 1.0ms | **81.5%削減** |
| ThreadPool.Barrier.SpinWait | 10.9ms | 0.4ms | **96.3%削減** |

## 詳細ログ使用例

```cpp
// 有効化
PerformanceMonitor::Instance().EnableDetailedLogging(true);

// 手動保存
PerformanceMonitor::Instance().SaveDetailedLog();
```

## 推奨コミットメッセージ

```
feat: 衝突検出システムの大幅な最適化とパフォーマンスログ機能追加

- ColliderManager: SpatialPartitioning, GridMerge, BroadPhaseを最適化
  * GridMerge並列化でロック回数を1/32に削減 (5.5ms→0.9ms)
  * BroadPhaseをソート+線形走査に変更 (5.4ms→1.0ms)
  * grain size調整でスレッド同期コストを削減 (Barrier 10.9ms→0.4ms)
- PerformanceMonitor: 詳細ログ機能追加
  * CPU/GPU/メモリ使用率、スレッド状態、全セクション詳細を記録
  * 遅いフレーム自動検出・保存機能
  * SceneManagerとの連携でシーン名自動設定

パフォーマンス: フレーム時間67.9%削減 (145.7ms→46.7ms), FPS 173%向上 (7.85→21.4)
```
