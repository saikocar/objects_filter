# Object Filter Node

このパッケージは、ROS 2上で動作するオブジェクトフィルタリングノードです。指定したラベルを持つ物体や、視野の右側（x > 0.0）に存在する物体を除外し、フィルタ済みのオブジェクト群を出力します。

## 機能概要

* 指定したラベル（分類）に一致する物体を除外
* カメラ座標系でx > 0に位置する物体を除外
* `/autoware_auto_perception_msgs/msg/DetectedObjects`型のメッセージを処理

## サブスクライブするトピック

| トピック名         | 型                                                     | 説明       |
| ------------- | ----------------------------------------------------- | -------- |
| `input_topic` | `autoware_auto_perception_msgs::msg::DetectedObjects` | 入力される物体群 |

## パブリッシュするトピック

| トピック名          | 型                                                     | 説明         |
| -------------- | ----------------------------------------------------- | ---------- |
| `output_topic` | `autoware_auto_perception_msgs::msg::DetectedObjects` | フィルタ済みの物体群 |

## パラメータ

パラメータは`config/config.yaml`で管理されています。

| パラメータ名            | 型        | デフォルト値              | 説明             |
| ----------------- | -------- | ------------------- | -------------- |
| `input_topic`     | `string` | `/objects_on_route` | 入力トピック名        |
| `output_topic`    | `string` | `/objects_filtered` | 出力トピック名        |
| `excluded_labels` | `int[]`  | `[]`（空のリスト）         | 除外対象のラベル一覧（数値） |

## 使用例

```bash
ros2 launch objects_filter objects_filter.launch.py
```

## ラベルに関する補足

`excluded_labels`は、`DetectedObject.classification.front().label`で取得されるラベル番号を指定します。ラベルの番号と対応するクラスは、使用する`ObjectClassification.msg`に準拠します。

前段階でyoloの検出結果を使っており、yoloは電信柱等も検出していますがUNKNOWNにまとめられていることに注意してください。

```
uint8 UNKNOWN = 0
uint8 CAR = 1
uint8 TRUCK = 2
uint8 BUS = 3
uint8 TRAILER = 4
uint8 MOTORCYCLE = 5
uint8 BICYCLE = 6
uint8 PEDESTRIAN = 7
```

## ビルド方法

```bash
colcon build --packages-select objects_filter
```

`--symlink-install`等のオプションは適宜使用してください。

## ライセンス

このパッケージはApache 2.0ライセンスの下で公開されています。

```
© 2025 [minamidani(確認中)]
```
