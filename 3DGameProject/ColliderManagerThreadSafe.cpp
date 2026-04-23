// ColliderManagerThreadSafe.cpp
// ================================================================
//  Week 1-2: Narrow Phase 並列化 — 補足メモ
// ================================================================
//
// 実装の詳細は ColliderManager.cpp の SpatialPartitioning() 内
// "Step 2: ナロウフェーズ判定 (Week 1-2: 並列化)" セクションを参照。
//
// 並列化の方針:
//   Phase A (Parallel):
//     - candidates 配列を ThreadPool::ParallelForBarrier で並列処理
//     - 各スレッドは thread_local の _tlNarrowHit / _tlContactOut を使用
//     - CheckXxx は EmitContact() を通じて per-index バッファへ書き込み
//     - 共有状態への書き込みなし → mutex 不要
//
//   Phase B (Serial):
//     - ヒットしたペアを _currPairs に登録
//     - 接触点を _contacts へ移動 (std::move)
//     - ResolvePushOut はシリアル (Transform 書き換えのため)
//
// スレッドローカル変数:
//   thread_local bool                   ColliderManager::_tlNarrowHit
//   thread_local std::vector<Contact>*  ColliderManager::_tlContactOut
//   (ColliderManager.cpp の先頭で定義)

