
ボディモデル
============

.. contents::
   :local:
   :depth: 1


ボディモデルとボディアイテム
----------------------------

コンピュータ上でロボットに関わる各種計算を行うためには、ロボットや環境物体の形状や物理特性等をデータ化したモデルが必要となります。Choreonoidではこのモデルを「ボディモデル」と呼んでいます。これは実際にはChoreonoidが利用している「Bodyライブラリ」にてC++のクラスとして定義されている "Body" クラスに対応しています。

このボディモデルにChoreonoidのGUIからアクセスするためのプロジェクトアイテムとして、「ボディアイテム」型が定義されており、Choreonoid上で行うロボットに関する操作の多くはこのアイテムを介して行うことになります。

ボディアイテムは通常、ロボットや環境物体のモデルを記述したファイルである :doc:`modelfile/index` を読み込むことで生成します。

.. _bodymodel_samplemodels:

サンプルモデル
--------------

Choreonoidには標準で以下のようなサンプルモデルが付属していて、shareディレクトリの "model" ディレクトリ以下に格納されています。（shareディレクトリの場所については :doc:`../install/directories` を参照してください。）まずはこれらのモデルを読み込んでモデルの操作方法を確認するとよいでしょう。

.. tabularcolumns:: |p{2.0cm}|p{3.5cm}|p{9.0cm}|

.. list-table::
 :widths: 15,25,60
 :header-rows: 1

 * - モデル
   - ファイル
   - 概要
 * - box1
   - misc/box1.body
   - 箱状物体のサンプル
 * - floor
   - misc/floor.body
   - シンプルな床のモデル
 * - SR1
   - SR1/SR1.body
   - シンプルな二足歩行ロボットモデルのサンプル
 * - GR001
   - GR001/GR001.body
   - HPI社製小型二足歩行ロボット"GR001"のモデル
 * - PA10
   - PA10/PA10.body
   - 三菱重工業株式会社製マニピュレータ "PA10" のモデル
 * - Tank
   - tank/tank.body
   - 戦車型モデルのサンプル

.. _loading_model:

ボディモデルの読み込み
----------------------

ボディモデルを記述したファイルの読み込みは、メインメニューの「ファイル」-「読み込み」-「ボディ」から行います。このメニューを実行すると以下のようなファイル読み込みダイアログが表示されます。

.. image:: images/dialog-to-load-body.png
    :scale: 80%

このダイアログで読み込みたいモデルのファイルを選択します。その際、ダイアログの左側にある「サイドバー」から関連するディレクトリに移動することができますので、必要に応じてこちらも活用してください。

サンプルモデルのひとつである "PA10" モデルを読み込んでみましょう。この場合、まずサイドバーからshareディレクトリを選択するとよいです。上の図ではサイドバーの "choreonoid-1.8" がshareディレクトリに対応していますので、まずこれを選択します。するとshareディレクトリの内容が右側の領域に表示されますので、ここから "model" を選択します。そこで表示されるディレクトリの中からさらに "PA10" を選択すると、"PA10.body" というファイルがあるかと思うので、これをダブルクリックするか、選択して「読み込み」ボタンを押してください。

ファイルが正常に読み込まれると、アイテムツリービュー上に以下のようにモデル名が表示されます。

.. image:: images/pa10item_checked.png

この時、メッセージビューには以下のようなメッセージが出力されます。 ::

 ボディ "/usr/local/share/choreonoid-1.6/PA10/PA10.body" を読み込み中
 -> 完了!

.. 英訳指示： 上のメッセージは以下のメッセージとしてください。
   Loading Body "/home/nakaoka/choreonoid/build/share/choreonoid-2.3/model/PA10/PA10.body"
   -> ok!

もしモデルファイルに問題があれば、代わりにエラーメッセージが出力されますので、ファイルの内容を確認してください。

読み込まれたボディモデルはアイテムツリービュー上で「ボディアイテム」と呼ばれます。

.. note:: アイテムツリービュー上でボディアイテムを選択してCtrl + Rを押すと、モデルファイルの再読み込みを行います。この操作では再読込されたモデルがすぐにGUI上に反映されるので、モデルファイルの編集時などに使うと便利です。


シーンビュー上での表示
----------------------

ボディアイテムとして読み込まれたモデルは、シーンビュー上で以下のように3Dグラフィックスで表示されます。

.. image:: images/pa10scene.png

この表示を行うかどうかはユーザが切り替えることができます。以下の図のようにボディアイテムのチェックを外した状態にすると

.. image:: images/pa10item.png

シーンビュー上からモデルが消えるかと思います。

再度チェックを入れることで、シーンビュー上での表示も元に戻りますので、必要に応じて表示／非表示を切り替えるようにしてください。

.. _model_structure:

モデルの構造
------------

ロボット工学では一般的にモデルを構成する個々のパーツ（剛体）を「リンク」と呼んでいます。モデルとしては単一のリンクからなるものと複数のリンクからなるもの（マルチリンクモデル）があります。

単一の剛体とみなせるモデルについては、通常リンクがひとつとなります。例えば箱型のサンプルモデルである "box1" はそのようなモデルです。

ロボットは通常マルチリンクモデルとなります。マルチリンクモデルにおいて、リンクは「関節」によって接続され、関節を動かすことで様々な姿勢をとることが可能です。

モデルの構造は「リンク／デバイスビュー」を用いて確認することができます。リンク／デバイスビューの表示は現在選択されているボディアイテムに対してなされるので、まずは構造を確認したいモデルのボディアイテムをアイテムツリービュー上で選択してください。例えばPA10のボディアイテムを選択すると、リンク／デバイスビューに以下のように表示されるかと思います。

.. 英訳指示： リンク／デバイスビューはLink/Device Viewとしてください。

.. image:: images/linkview_pa10links.png

PA10モデルについては、ここに表示されているように、"BASE" から "HAND_R" までの10個のリンクで構成されています。さらに、"J1" から "HAND_R" については、関節も兼ねるリンクとなっています。"ID" に表示されているのは関節の識別番号で、0〜8までの9つの関節があることが分かります。この表示では関節の接続関係まではよく分かりませんが、リンク／デバイスビュー上部のコンボボックスを「リンク一覧」から「リンクツリー」に変更すると以下のような表示となり、関節の接続関係も確認することができます。

.. image:: images/linkview_pa10linktree.png

このツリーにおける親子関係は、親と子の間に関節が存在することを示しています。この表示により、PA10モデルは "BASE" から "J7" まで直列にリンクが接続され、最後の "HAND_L" と "HAND_R" はどちらも "J7" に接続されたリンクであることが分かります。また、この例の "BASE" のように、ツリーのルートとなるリンクを「ルートリンク」と呼びます。

.. note:: マルチリンクモデルの構造は基本的にはこのようにツリーで表現することができます。ただし、モデルに閉ループ構造がある場合はツリーだけでは表現できません。Choreonoidでは現在のところ閉ループ構造のモデルには完全には対応していませんので注意が必要です。


ボディアイテムのプロパティ
--------------------------

ボディアイテムを選択状態にすると、プロパティビューに選択したボディアイテムのプロパティ一覧が表示されます。PA10の場合、以下のように表示されるかと思います。

.. image:: images/pa10properties.png

ボディアイテム特有のプロパティとしては以下のようなものがあり、これらによってモデルの概要を確認したり、編集方法を変更したりすることが可能となっています。

.. tabularcolumns:: |p{3.5cm}|p{11.5cm}|

.. list-table::
 :widths: 25,75
 :header-rows: 1

 * - プロパティ
   - 意味
 * - モデル名
   - モデル自体の名前です。アイテム名はモデルの個々の実態を区別するために任意に設定可能ですが、モデル名はモデルの種類を特定するためのもので、同一モデルに対しては常に同じとなります。
 * - リンク数
   - モデルが有するリンクの数です。
 * - 関節数
   - モデルが有する関節の数です。
 * - デバイス数
   - モデルが有するデバイス（各種センサ、ライト等）の数です。
 * - ルートリンク
   - ルートリンクの名前です。
 * - ベースリンク
   - 姿勢変更操作においてベースとなるリンクを表しています。
 * - 質量
   - モデルの全質量です。
 * - 重心
   - 重心位置です。
 * - モデルタイプ
   - 動的（動くもの）、静的（動かないもの）のいずれかになります。ルートが固定されていても、関節が動く場合は「動的」になります。
 * - ルート固定
   - ルートが固定されているかどうかを示しています。
 * - 干渉検出
   - 他の物体との干渉検出を行うかどうかを示しています。
 * - 自己干渉検出
   - 自己干渉検出を行うかどうかを示しています。
 * - 配置ロック
   - Trueの場合はシーンビュー上でルートの位置をドラッグできなくなります。
 * - シーン感知
   - Falseの場合はシーンビュー上でマウスの操作に反応しなくなります。
 * - 透明度
   - 透明度を設定できます。
 * - 表示リンクの選択
   - Trueの場合は、「リンク／デバイスビュー」上で選択されているリンクのみシーンビュー上に表示されるようになります。
 * - 多重化数
   - モデルが複数個に複製されて表示されている場合は、その数を示します。
 * - 存在
   - モデルが存在しないものとして扱われている間はFalseとなります。
 * - ファイル
   - モデルのファイル名です。



.. 英訳指示： 上の表おけるプロパティ名は上から順番に以下の対応付けとしてください。
   Model name
   Num links
   Num joints
   Num devices
   Root link
   Base link
   Mass
   Center of mass
   Model type
   Root fixed
   Collision detection
   Self-collision detection
   Lock location
   Scene sensitive
   Transparency
   Visible link selection
   Multiplexing number
   Existence
   File

.. 英訳指示： 上の表の用語は以下の対応としてください。
   動的 -> Dynamic, 静的 -> Static, リンク／デバイスビュー -> Link/Device View
