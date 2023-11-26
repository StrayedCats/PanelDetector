# 開発ガイド

ここでは、開発に関する簡単な流れを説明します。

他のプロジェクトでも同様の流れを取れるようにします。

<br>

## 使用する環境

使用する環境はつぎの通りです。

### ハードウェア

- VSCodeが動くくらいの性能のPC

### ソフトウェア

- Ubuntu22.04 LTS (ROS 2 Humbleのrosdepが解決できる環境)
- ROS 2 Humble

<br>

## 開発の流れ

開発の流れはつぎの通りです。

1. タスクの割当てがある。（issue・Projectsのチケット）
    - このとき、タスクの割当ては、`~~_plugins` フォルダに対して行う。
    - テストコードも作成しておく
2. ブランチを切る。
3. `~~_plugins` フォルダに対して、ソースコードを作成する。
4. `~~_plugins` フォルダに対して、ドキュメントを作成する。
4. `~~_plugins` フォルダに対して、テストコードを作成する。（ない場合）
6. テストが通るようになったら、PRを出してレビューを受ける。
7. レビューが通ったら、マージする。

途中でパラメータの追加などの

<br>


## ブランチの命名規則

ブランチの命名規則はつぎの通りです。

- `feat/<機能名>` : 機能追加
- `fix/<機能名>` : 機能修正
- `refactor/<機能名>` : 機能のリファクタリング
- `docs/<機能名>` : ドキュメントの追加・修正
- `test/<機能名>` : テストの追加・修正

<br>

## フォルダ構成

ここでは、PanelDetectorのフォルダ構成を例に説明します。

フォルダ構成はつぎの通りです。

<img width="542" alt="dirs" src="https://github.com/StrayedCats/PanelDetector/assets/67567093/c3a038e1-d480-4bdb-a836-3bde7ec2d456">

ファイルとして作成するのは、 `panel_detector_plugins` 内のヘッダーファイルとソースコードファイルのみ（橙色で囲っている部分）ですが、その登録先（青で囲っている部分）については新規作成した内容に応じて適宜修正していきます。

ヘッダーファイルについては、事前に作成されている`_base.hpp`を継承します。

テスト内容は、`_plugin.cpp`(ここでは`publish_center.cpp`）をテストします。

<br>

publish_centerの場合、新しく追加するファイルはつぎの通りです。

- `panel_detector/panel_detector_plugins/src/publish_center.cpp` : ソースコード
- `panel_detector/panel_detector_plugins/include/panel_detector_plugins/publish_center.hpp` : ヘッダファイル
  - `panel_detector_base.hpp` を継承します。

プラグインを追加するため、次のファイルを追加で記述します。（記述しないとコンパイルエラーになります。）

- `panel_detector/panel_detector_plugins/panel_detector_plugins.xml`
- `panel_detector/panel_detector_plugins/CMakeLists.txt`

必要に応じて次のファイルを追加で記述します。

- `panel_detector/panel_detector_plugins/package.xml`
- `panel_detector/build_depends.repos`


<br>

## テストについて

TODO

<br>

## ドキュメントについて

ドキュメントは、チーム外にも共有されます。

初めてでもわかりやすい説明を心がけましょう。

<br>



