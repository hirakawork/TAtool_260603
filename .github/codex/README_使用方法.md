# 尻尾・コード TA ツール 使用方法

## ファイル

- `tail_code_ta_tool.py` が Maya 用ツール本体です。
- Maya 2026 / Python 3.11 / PySide6 を想定しています。

## 起動方法

Maya の Script Editor で以下を実行します。

```python
import sys
tool_dir = r"C:\Users\hirat\OneDrive\デスクトップ\codex"
if tool_dir not in sys.path:
    sys.path.append(tool_dir)

import tail_code_ta_tool
tail_code_ta_tool.show()
```

## 実装済み機能

- ジョイントチェーンの登録、ラベル管理、表示/非表示切り替え
- Root から Tip への自動並び替え
- ワールド座標取得
- 各ジョイントの回転角度（Rotate X / Y / Z）表示
- 隣接ジョイント間の回転角度差分表示
- 曲がり角によるスコア算出
- 曲がり角が大きいジョイントの検出とリスト表示
- 現在フレームの曲がり折れ線グラフ表示
- 曲がりヒートマップ表示
- Maya シーン内へのチェーン情報保存と復元
- `MNodeMessage.addAttributeChangedCallback` による監視更新
- 指定フレーム範囲の一括スキャン
- スキャン中のプログレスバーとキャンセル
- グラフクリックによるフレーム移動
- CSV 書き出し
- Tail Tweaker の作成、削除、Bake
- 曲がりが大きい箇所への Auto Tweaker
- 推奨補正を表示する Smart Suggest

## 曲がり評価について

表の「曲がり」は、対象ジョイントの前後ジョイントのワールド座標から求めた曲がり角です。
現在フレームの折れ線グラフとヒートマップは、この「曲がり」を元に表示します。
UI のしきい値を超えたジョイントは、曲がりが大きい箇所として赤く強調表示します。

「回転 X / Y / Z」と「差分 X / Y / Z」は、原因確認用の補助情報として表示しています。

## Tail Tweaker について

Tail Tweaker は、選択ジョイントに一時的な補助コントローラを追加する機能です。
作成されたノードは `TailTweaker_ジョイント名` という名前になり、対象ジョイントへ orientConstraint で接続されます。

- `Create Tweaker`: 選択中のジョイント、または問題ジョイントリストで選択中のジョイントに Tweaker を作成します。
- `Delete Tweaker`: 選択中の Tweaker、または選択ジョイントに接続された Tweaker を削除します。
- `Bake Tweaker`: 現在フレームの見た目の回転を対象ジョイントへ反映し、Tweaker を削除します。
- `Auto Tweaker`: 選択チェーン内で曲がりがしきい値を超えた最大箇所に Tweaker を作成します。
- `Smart Suggest`: 曲がりが大きい箇所と、調整の目安になる回転軸・角度を表示します。

既存リグの構造はシーンごとに異なるため、現版の Bake は対象ジョイントへ反映します。
元リグコントローラへ直接 Bake する場合は、リグ命名規則や接続ルールに合わせた追加対応が必要です。

## 補足

ビューポート表示は、Maya のカーブノードを使った安定版のプレビューとして実装しています。
VP2 の専用 Draw Override 化が必要な場合は、Maya プラグイン形式に分離して追加実装できます。
