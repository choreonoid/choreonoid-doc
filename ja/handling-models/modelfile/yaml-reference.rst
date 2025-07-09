
Bodyファイル リファレンスマニュアル
===================================

.. contents::
   :local:
   :depth: 2

概要
----

Choreonoid標準のBody形式モデルファイル（Bodyファイル）のリファレンスマニュアルです。

モデルファイルの規約
~~~~~~~~~~~~~~~~~~~
モデルファイルひとつにつき、ロボットや環境のモデル１体を記述するようにします。
また、ファイルの拡張子は通常のYAML形式ファイル(".yaml")と区別するため".body" をつけるようにします。

.. _body-file-reference-key-style:

キーの記述形式について
~~~~~~~~~~~~~~~~~~~~~

Choreonoidで使用するYAMLファイルにおけるキーの記述形式について、従来「ロワーキャメルケース」を用いていましたが、これを「スネークケース」に順次切り替えています。ただし切り替えは完全ではなく、一部のキーはまだキャメルケースとなっています。このためChoreonoidで使用するYAMLファイルではキャメルケースとスネークケースが混在している場合があります。これはBodyファイルでも同様です。

本マニュアルではスネークケースに移行済みのキーについてはスネークケースで表記しています。従来キャメルケースで定義されていたキーについては、互換性維持のため最新版でも古い形式のキーを読み込めるようにしています。ただしこの措置は今後廃止される可能性もありますので、新たにファイルを作成する際は新しい形式を利用するようにしてください。

角度の単位について
~~~~~~~~~~~~~~~~~

基本的には度数法（degree）になります。詳細は :ref:`body-file-reference-header` の "angle_unit" の項目を参照ください。


YAML文法
--------

YAMLの文法については `プログラマーのための YAML 入門 (初級編)  <http://magazine.rubyist.net/?0009-YAML>`_
などを参照してください。

.. 英訳指示： 上記の参照は `The Official YAML Web Site <https://yaml.org>`_ に変更してください。

ノード一覧
----------

ノードとしては以下のようなものが定義されており、これらのインスタンスを組み上げていくことにより、モデルを作成します。
実モデル定義部においてこれらのノードのインスタンスを組み合わせて階層構造を作ることで、モデルを作成していきます。

リンク構造、動力学/機構パラメータを定義するノードとして、以下のノードが定義されています。

* :ref:`body-file-reference-link-node`
* :ref:`body-file-reference-rigid-body-node`
* :ref:`body-file-reference-transform-node`

リンクの形状、表示を定義するノードとして、以下のノードが定義されています。

* :ref:`body-file-reference-shape-node`
* :ref:`body-file-reference-geometry-node`

 * :ref:`body-file-reference-box-node`
 * :ref:`body-file-reference-sphere-node`
 * :ref:`body-file-reference-cylinder-node`
 * :ref:`body-file-reference-capsule-node`
 * :ref:`body-file-reference-cone-node`
 * :ref:`body-file-reference-extrusion-node`
 * :ref:`body-file-reference-elevation-grid-node`

* :ref:`body-file-reference-appearance-node`
* :ref:`body-file-reference-material-node`
* :ref:`body-file-reference-resource-node`

各種センサ・デバイスを定義するノードとして以下のノードが定義されています。

* :ref:`body-file-reference-acceleration-sensor-node`
* :ref:`body-file-reference-rate-gyro-sensor-node`
* :ref:`body-file-reference-force-sensor-node`
* :ref:`body-file-reference-camera-node`
* :ref:`body-file-reference-range-sensor-node`
* :ref:`body-file-reference-spot-light-node`

閉リンク機構を定義するノードとして以下のノードが定義されています。

* :ref:`body-file-reference-extra-joint-node`

ノードをグループ化するためのノードとして以下のノードが定義されています。

* :ref:`body-file-reference-group-node`

以下では各ノードの詳細を説明します。

.. _body-file-reference-header:

ヘッダ
------

ファイルの先頭に置き、モデルファイルのフォーマットを指定します。

.. list-table:: ヘッダのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - format
   - "ChoreonoidBody"を指定。
 * - format_version
   - モデルファイルのフォーマットのバージョンを指定。現在のバージョンは2.0。
 * - angle_unit
   - モデルファイルにおける関節角度の単位を指定する項目。"degree"または"radian"を指定。デフォルトはdegree。※ format_versionが2.0のときはradianは指定できません。
 * - name
   - モデルの名前を指定。
 * - root_link
   - ルートリンク名を指定。


リンク構造、動力学/機構パラメータを定義するノード
-------------------------------------------------

.. _body-file-reference-link-node:

Linkノード
~~~~~~~~~~

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Linkノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Link
 * - name
   - リンクの名称。モデル内で重複しない任意の文字列を指定可能
 * - parent
   - 親リンク。親リンクの名前（nameに記述した文字列）で指定する。ルートリンクの場合は使用しない
 * - translation
   - 本リンクローカルフレームの親リンクからの相対位置。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる
 * - rotation
   - 本リンクローカルフレームの親リンクからの相対姿勢。姿勢は回転軸と回転角度に対応する4つの数値で表現 (Axis-Angle形式）。ルートリンクの場合はモデル読み込み時のデフォルト位置として使われる
 * - joint_id
   - 関節ID値。0以上の整数値を指定する。モデル内で重複しない任意の値を指定可能。リンクが関節でない場合 （ルートリンクやjoint_typeがfixedの場合）や、ID値によるアクセスを必要としない場合は、指定しなくてもよい
 * - joint_type
   - 関節タイプ。 **fixed** (固定）、 **free** (非固定。ルートリンクにのみ指定可）、 **revolute** (回転関節）、 **prismatic** (直動関節）、 **pseudo_continuous_track** (簡易無限軌道）、 のどれかを指定
 * - joint_axis
   - 関節軸。3次元ベクトルの3要素のリストとして関節軸の向きを指定する。値は単位ベクトルとする。関節軸がリンクのローカル座標におけるX, Y, Z、及びそれらの逆方向のいずれかに一致する場合は、対応する軸の文字(X, Y, Z,-X,-Y,-Z）によって指定することも可能。
 * - joint_angle
   - 関節の初期角度。
 * - joint_displacement
   - 関節の初期角度。radianで指定。joint_angleよりも優先される。
 * - joint_range
   - 関節可動範囲。最小値、最大値の2つの値をリストとして列挙する。値をunlimitedと記述することで、可動範囲の制限を無くすことも可能。最小値と最大値の絶対値が同じでそれぞれ符号がマイナス、プラスとなる場合は 、その絶対値をひとつだけ（スカラ値として）記述してもよい
 * - max_joint_velocity
   - 関節の回転・移動速度の範囲をスカラ値(>=0)で指定。この値のマイナス、プラスの範囲に設定される。joint_typeがrevoluteのときは最大角速度、それ以外のときは最大速度(m/sec)
 * - joint_velocity_range
   - 関節の回転・移動速度の範囲。最小値、最大値の2つの値をリストとして列挙する。max_joint_velocityより優先される。
 * - rotor_inertia
   - ロータ慣性モーメント。default値=0.0。
 * - gear_ratio
   - ギア比。default値=1.0。
     等価ロータ慣性モーメントはgear_ratio*gear_ratio*rotor_inertiaで設定される。
 * - center_of_mass
   - 重心位置。リンクローカル座標で指定
 * - mass
   - 質量[kg]
 * - inertia
   - 慣性モーメント。慣性テンソルの9要素をリストとして列挙。慣性テンソルの対称性より、上三角部分の6要素のみを列挙してもよい。
 * - import
   - エイリアスをつけたノードをこの場所に読み込む。 import: \*defined_alias
 * - elements
   - リンクの構成要素となる子ノードを記述


.. note::
	最初に記述するLinkノードはモデルのルートノードとみなされます。

.. note::
	剛体パラメータ(center_of_mass, mass, inertia)は次に述べるRigidBodyノードで記述することも可能です。その場合elementsを用いてRigidBodyノードをLinkノードの子ノードとして配置します。

.. _body-file-reference-rigid-body-node:

RigidBodyノード
~~~~~~~~~~~~~~~

RigidBodyノードはリンクの剛体パラメータを定義します。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RigidBodyノードの項目
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - RigidBody
 * - center_of_mass
   - 重心位置。リンクローカル座標で指定
 * - mass
   - 質量[kg]
 * - inertia
   - 慣性モーメント。慣性テンソルの9要素をリストとして列挙。慣性テンソルの対称性より、上三角部分の6要素のみを列挙してもよい。
 * - elements
   - 子ノードでリンクの形状やセンサーなどを記述。

.. _body-file-reference-transform-node:

Transformノード
~~~~~~~~~~~~~~~

配下のノードを平行移動・回転・拡大縮小します。

.. list-table:: Transformノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Transform
 * - translation
   - 位置のオフセット
 * - rotation
   - 姿勢のオフセット
 * - scale
   - サイズの拡大・縮小
 * - elements
   - 変換を受ける子ノードを記述。


リンク形状・見た目を定義するノード
----------------------------------

.. _body-file-reference-shape-node:

Shapeノード
~~~~~~~~~~~

.. list-table:: Shapeノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Shape
 * - geometry
   - リンクの形状を :ref:`body-file-reference-geometry-node` のいずれかで記述
 * - appearance
   - リンクの色やテクスチャを :ref:`body-file-reference-appearance-node` として記述

.. _body-file-reference-geometry-node:

幾何形状ノード
~~~~~~~~~~~~~~

幾何形状の記述には、以下のBox、Shpere、Cyinder、Capsule、Cone、Extrusion、ElevationGrid、IndexedFaceSetのいずれかのノードを使用することができます。

.. _body-file-reference-box-node:

Boxノード
'''''''''

Boxノードは直方体を記述する幾何形状ノードです。

.. list-table:: Boxノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Boxを指定
 * - size
   - 直方体の縦横奥行きの長さ

.. _body-file-reference-sphere-node:

Sphereノード
''''''''''''

Sphereノードは球を記述する幾何形状ノードです。

.. list-table:: Sphereノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Sphere
 * - radius
   - 球の半径

.. _body-file-reference-cylinder-node:

Cylinderノード
''''''''''''''

Cylinderノードは円柱を記述する幾何形状ノードです。

.. list-table:: Cylinderノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Cylinder
 * - radius
   - 半径
 * - height
   - 高さ
 * - bottom
   - true:底面あり(default)  false:底面なし
 * - top
   - true:上面あり(default)  false:上面なし

.. _body-file-reference-capsule-node:

Capsuleノード
''''''''''''''

Capsuleノードはカプセル（円柱＋球２つ）を記述する幾何形状ノードです。

.. list-table:: Capsuleノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Capsule
 * - radius
   - 半径
 * - height
   - 高さ

.. _body-file-reference-cone-node:

Coneノード
''''''''''

Coneノードは円錐を記述する幾何形状ノードです。

.. list-table:: Coneノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Cone
 * - radius
   - 底面の半径
 * - height
   - 高さ
 * - bottom
   - true:底面あり(default)  false:底面なし

.. _body-file-reference-extrusion-node:

Extrusionノード
'''''''''''''''

Extrusionノードは押し出し形状を記述する幾何形状ノードです。

.. list-table:: Extrusionノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Extrusion
 * - cross_section
   - | 押し出す断面の形状を頂点の座標で指定(x-z平面)。
     | cross_section: [ x0, z0, x1, z1, x2, z2, ・・・, xn, zn ]
     | のようにx座標,z座標を並べる。改行・スペースを入れて良い。
     | cross_section: [ x0, z0,
     |                 x1, z1,
     |                  ：
 * - spine
   - | cross_sectionで指定した断面を沿わせて動かす区分的直線を端点の座標で指定。
     | spine: [ x0, y0, z0, x1, y1, z1, ・・・, xn, yn, zn ]
 * - orientation
   - spineの各点におけるcross_sectionの回転をaxis-angle形式のパラメータ(x, y, z, θ)を並べて指定。
     1組のみ指定した場合は全spineで同じ回転が使われる。spineの個数より少ない場合は不足分が回転無しになり、spineの個数より多い場合は無視される。
 * - scale
   - cross_sectionで指定した断面のspineの各点における拡大率。x軸方向の拡大率、z軸方向の拡大率をspineの個数分並べて指定。1組のみ指定した場合は全spineで同じ拡大率になる。spineの個数より指定が少ない場合、未指定分は0倍に拡大され1点になる。spineの個数より多く指定された分は無視される。
 * - crease_angle
   - 光源と法線ベクトルの角度によってシェーディングを変えるための閾値。crease_angle未満のときはスムーズシェーディングされる。デフォルトは0。
 * - begin_cap
   - true:開始端側の断面あり(default) false:開始端側の断面なし
 * - end_cap
   - true:終端側の断面あり(default) false:終端側の断面なし

※参照: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#Extrusion


.. _body-file-reference-elevation-grid-node:

ElevationGridノード
'''''''''''''''''''

ElevationGridノードはグリッドの格子点ごとに高さを与えた地形状の形状を記述する幾何形状ノードです。

.. list-table:: ElevationGridノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - ElevationGrid
 * - x_dimension
   - x軸方向のグリッドの数
 * - z_dimension
   - z軸方向のグリッドの数
 * - x_spacing
   - x軸方向のグリッド間隔
 * - z_spacing
   - z軸方向のグリッド間隔
 * - ccw
   - true: 頂点の順序が反時計回り false: 頂点の順序が時計回り
 * - crease_angle
   - 光源と法線ベクトルの角度によってシェーディングを変えるための閾値。crease_angle未満のときはスムーズシェーディングされる。デフォルトは0。
 * - height
   - 各格子点上の高さを配列で指定。格子点の個数(x_dimension*z_dimension)分の要素が必要。

.. TODO: tex_coord キーに関する記述を追加

※参照: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#ElevationGrid


.. _body-file-reference-IndexedFaceSet-node:

IndexedFaceSetノード
''''''''''''''''''''''''

IndexedFaceSetノードは、リストされた頂点から面（ポリゴン）を作成することによって形状を記述する幾何形状ノードです。

.. list-table:: IndexedFaceSetノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - IndexedFaceSet
 * - vertices
   - | 頂点の座標を指定。 vertices: [ x0, y0, z0, x1, y1, z1, ・・・, xn, yn, zn ]
     | のようにx座標,y座標、z座標を並べる。
 * - faces
   - | verticesで指定した座標に0からNまでインデックスを付けてポリゴン面を指定。インデックス「-1」は、現在の面が終了したことを示す。
     | faces: [ 0, 1, 2, 3, -1, 3, 2, 4, 5, -1、  ・・・ ] のようにインデックスを並べる。頂点の順序は反時計回り。
 * - tex_coords
   - | textureを貼る場合に使用する。テクスチャを頂点にマッピングするための2次元座標を指定。
     | tex_coords: [ s0, t0, s1, t1, ・・・, sm, tm ]
     | のように、テクスチャの左下を(0.0, 0.0), 右上を(1.0, 1.0)としたときの座標を並べる。
 * - tex_coord_indices
   - | facesと同様に、各頂点のテクスチャ座標を選択するために使用する。facesフィールドと同じ数のインデックスを含み、同じ位置に面の終了記号である「-1」を含まなければならない。
     | 指定しない場合は、facesが使用される。
 * - crease_angle
   - 光源と法線ベクトルの角度によってシェーディングを変えるための閾値。crease_angle未満のときはスムーズシェーディングされる。デフォルトは0。

.. TODO: normalsキーに関する記述を追加
   TODO: normal_indicesキーに関する記述を追加

※参照: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#IndexedFaceSet


.. _body-file-reference-appearance-node:

Appearanceノード
~~~~~~~~~~~~~~~~

.. list-table:: Appearanceノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - material
   - 物体表面の材質を :ref:`body-file-reference-material-node` として記述
 * - texture
   - 物体表面のテクスチャを :ref:`body-file-reference-texture-node` として記述
 * - texture_transform
   - テクスチャの平行移動・回転・拡大縮小を :ref:`body-file-reference-textureTransform-node` として記述

.. _body-file-reference-material-node:

Materialノード
~~~~~~~~~~~~~~

.. list-table:: materialノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - ambient
   - 環境光の反射率(0.0〜1.0)
 * - diffuse
   - RGBごとの拡散反射率(物体の色) (RGBそれぞれ0.0〜1.0のリスト)
 * - emissive
   - 物体自体から発光する色 (RGBそれぞれ0.0〜1.0のリスト)
 * - specular_exponent
   - 鏡面反射の鋭さを制御するパラメータ。値が大きいとハイライトが小さく鋭くなり、金属や磨かれた表面のような見た目になる。0以上の値を設定。デフォルトでは25となる。値が100程度になると金属っぽくなる。
 * - shininess
   - 鏡面反射の鋭さを制御する古いパラメータ。0〜1の範囲で指定。このパラメータは今後使用せず、specular_exponentを使用する。
 * - specular
   - 鏡面反射率(光のハイライトの色) (RGBそれぞれ0.0〜1.0のリスト)
 * - transparency
   - 透過度(0:透明 〜 1:不透明)

.. _body-file-reference-texture-node:

Textureノード
~~~~~~~~~~~~~~

.. list-table:: textureノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - url
   - テクスチャファイルのパス
 * - repeat_s
   - テクスチャを水平方向に繰り返し表示することを指定
 * - repeat_t
   - テクスチャを垂直方向に繰り返し表示することを指定
   
.. _body-file-reference-textureTransform-node:

TextureTransformノード
~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: textureTransformノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - translation
   - 位置のオフセット
 * - rotation
   - 姿勢のオフセット
 * - scale
   - サイズの拡大・縮小
 * - center
   - rotation,scaleの中心点

※参照: http://tecfa.unige.ch/guides/vrml/vrml97/spec/part1/nodesRef.html#TextureTransform

.. _body-file-reference-resource-node:

Resourceノード
~~~~~~~~~~~~~~

リンクの形状にCADやモデリングツールで作成したメッシュを読み込みます。

.. list-table:: Resourceノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Resource
 * - uri
   - リンク形状のメッシュファイルのパス
 * - node
   - メッシュファイル内の特定のノードのみを読み込む場合にノード名を指定

.. _body-file-reference-devices:

各種センサ・デバイスを定義するノード
------------------------------------

Deviceノード
~~~~~~~~~~~~

各種デバイスで共通の設定項目を示します。

.. list-table:: Deviceノードの共通フィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - name
   - デバイスの名前
 * - id
   - デバイスのID
 * - translation
   - ローカル座標系の位置を、親ノード座標系からのオフセット値で指定。
 * - rotation
   - ローカル座標系の姿勢を、親ノード座標系からのオフセット値で指定([x, y, z, θ]  ベクトル[x, y, z]の周りにθ回転)。

.. note::
  各種センサノードはそのセンサが取り付けられているLinkノードの下に取り付けます。 例えば、サンプルモデルの腰部(WAIST)に加速度センサを取り付けている場合は、次のように記述します。

.. code-block:: yaml

    links:
      - 
        name: WAIST
        elements:
          -
            type: AccelerationSensor
            id: 0

.. _body-file-reference-acceleration-sensor-node:

AccelerationSensorノード
~~~~~~~~~~~~~~~~~~~~~~~~

AccelerationSensorノードは、3軸加速度センサを定義します。

.. list-table:: AccelerationSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - フィールド
   - 内容
 * - type
   - AccelerationSensor
 * - max_acceleration
   - 計測可能な最大加速度。3次元ベクトルの3要素のリストとして指定する。

.. _body-file-reference-rate-gyro-sensor-node:

RateGyroSensorノード
~~~~~~~~~~~~~~~~~~~~

RateGyroSensorノードは、3軸角速度センサを定義します。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: RateGyroSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - RateGyroSensor
 * - max_angular_velocity
   - 計測可能な最大角速度。3次元ベクトルの3要素のリストとして指定する。

.. _body-file-reference-imu-node:

Imuノード
~~~~~~~~~~~~~~~~~~~~

Imuノードは、3軸加速度センサと3軸角速度センサを一体化したIMU（慣性計測ユニット）を定義します。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: Imuノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Imu
 * - max_acceleration
   - 計測可能な最大加速度。3次元ベクトルの3要素のリストとして指定する。
 * - max_angular_velocity
   - 計測可能な最大角速度。3次元ベクトルの3要素のリストとして指定する。

.. _body-file-reference-force-sensor-node:

ForceSensorノード
~~~~~~~~~~~~~~~~~

ForceSensorノードは、力／トルクセンサを定義します。

.. list-table:: ForceSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - ForceSensor
 * - max_force
   - 計測可能な力の最大値。3次元ベクトルの3要素のリストとして指定する。
 * - max_torque
   - 計測可能なトルクの最大値。3次元ベクトルの3要素のリストとして指定する。

.. _body-file-reference-camera-node:

Cameraノード
~~~~~~~~~~~~

Cameraノードは、視覚センサを定義します。

.. list-table:: Cameraノードのフィールド
 :widths: 30,100
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - Camera
 * - format
   - | センサから取得する情報の種類を指定する。
     |   ・"COLOR"  色情報を取得
     |   ・"DEPTH"  深さ情報を取得
     |   ・"COLOR_DEPTH"  色情報と深さ情報を取得
     |   ・"POINT_CLOUD"  3次元点群を取得
     |   ・"COLOR_POINT_CLOUD"  色情報と3次元点群を取得
 * - lens_type
   - | レンズの種類を指定する。
     |   ・"NORMAL"  通常レンズ　(デフォルト値）
     |   ・"FISHEYE"  魚眼レンズ
     |   ・"DUAL_FISHEYE"  全方位カメラ
 * - on
   - true/falseでカメラのON/OFFを指定
 * - width
   - 画像の幅
 * - height
   - 画像の高さ　(lens_type="FISHEYE","DUAL_FISHEYE"の場合はwidthの値から自動で決定 )
 * - field_of_view
   - カメラの視野角度　(lensType="DUAL_FISHEYE"の場合は指定不可)
 * - near_clip_distance
   - 視点から前クリップ面までの距離
 * - far_clip_distance
   - 視点から後クリップ面までの距離
 * - frame_rate
   - カメラが毎秒何枚の画像を出力するか

.. note::
    視点の姿勢は以下のように定義されます。視線前方向 ・・・ ローカル座標系でZ軸の負の向き   視線上方向 ・・・ ローカル座標系でY軸の正の向き。

.. note::
    内部的にはformatが"COLOR"のときCamera、"COLOR"以外のときRangeCameraとして扱われます。レンズのタイプ指定はCameraのときのみ有効です。

.. _body-file-reference-range-sensor-node:

RangeSensorノード
~~~~~~~~~~~~~~~~~

RangeSensorノードは、距離センサを定義します。

.. list-table:: RangeSensorノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - RangeSensor
 * - on
   - 
 * - yaw_range
   - 距離をスキャンする水平面角度。0度を中心として、その両側にyaw_stepの倍数の角度でyaw_rangeの範囲内の角度が計測される。センサに水平方向のスキャン機能がない場合は0とする。0度から360度の範囲でyaw_stepの倍数で指定する。
 * - yaw_step
   - スキャン中に距離が計測される水平面角度の刻み幅
 * - pitch_range
   - 距離をスキャンする垂直面角度。0度を中心として、その両側にpitch_stepの倍数の角度でpitch_rangeの範囲内の角度が計測される。センサに垂直方向のスキャン機能がない場合は0とする。0度から170度の範囲でpitch_stepの倍数で指定する。
     （大きな値を指定すると、処理時間が増え、計測精度が悪くなります。）
 * - pitch_step
   - スキャン中に距離が計測される垂直面角度の刻み幅
 * - scan_rate
   - １秒間あたり行うスキャン回数[Hz]
 * - min_distance
   - 計測可能な最小距離[m]
 * - max_distance
   - 計測可能な最大距離[m]

.. note::
   このセンサが取り付けられているリンクに対するこのセンサの姿勢。センサ座標系において、Z軸マイナス方向が計測正面、スキャンする場合の水平計測面はXZ平面、垂直計測面はYZ平面となります。 これはVisionSensorと同じですので、従来VisionSensorで代用していたモデルを変更する場合は 位置、姿勢はそのまま使えます。
   水平、垂直の両方向にスキャンする場合の回転順は、yaw,pitchの順になります。
   
.. _body-file-reference-spot-light-node:

SpotLightノード
~~~~~~~~~~~~~~~

SpotLightノードは、ライトを定義します。

.. list-table:: SpotLightノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - type
   - SpotLight
 * - on
   - true/falseでライトのON/OFFを指定します。
 * - color
   - ライトの色(R,G,Bそれぞれの値を0.0〜1.0で指定)
 * - intensity
   - 明るさを0.0〜1.0で指定。
 * - direction
   - 光の向き。3次元ベクトルの3要素のリストとして方向を指定。
 * - beam_width
   - 最大輝度で光の広がる角度。デフォルトは90度。
 * - cut_off_angle
   - 完全に光が遮断される角度。デフォルトは45度。
 * - cut_off_exponent
   - 非負の値を指定。デフォルトは1.0。
 * - attenuation
   - 減衰率。非負の3要素のリストを指定。


閉リンク機構を定義するノード
------------------------------

.. _body-file-reference-extra-joint-node:

ExtraJointノード
~~~~~~~~~~~~~~~~

ExtraJointノードはボディに追加の拘束を加えるためのノードです。閉リンク機構を定義します。閉リンクの1つの関節がボールジョイントで接続されていると考え、2つのリンクが離れないように拘束力を発生させます。

.. note:: 本ノードが実現する拘束の種類は現状では非常に限定されたものとなっています。さらに、対応する拘束の種類はシミュレータアイテム（物理エンジン）のタイプにもよります。

.. tabularcolumns:: |p{3.0cm}|p{12.0cm}|

.. list-table:: ExtraJointノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - フィールド
   - 内容
 * - link1_name
   - ボールジョイントを受けているジョイント名
 * - link2_name
   - ボールジョイントが付いているジョイント名
 * - link1_local_pos
   - link1_nameジョイントの拘束位置をそのジョイントのローカル座標で指定
 * - link2_local_pos
   - link2_nameジョイントの拘束位置をそのジョイントのローカル座標で指定
 * - joint_type
   - 拘束の種類  ball：1点で固定  hinge：回転関節 piston：並進（軸回りの回転は拘束されない）
 * - axis
   - joint_typeがhingeまたはpistonのとき、拘束の軸をlink1_nameリンクのローカル座標で指定。

このノードはBodyファイルのトップレベルに "extra_joints" というキーのリストとして記述します。     
閉リンク機構のサンプルとして "share/model/misc/ClosedLinkSample.body" があります。


ノードをグループ化するノード
----------------------------

.. _body-file-reference-group-node:

Groupノード
~~~~~~~~~~~

一部のノードをグループ化するために使用します。

.. list-table:: Groupノードのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - キー
   - 内容
 * - name
   - グループの名前

.. code-block:: yaml

  (使用例)
  elements:
    - &SUBSYSTEM
      type: Group
      name: SUBSYSTEM
      elements:
        -
          (グループの１要素)
        -
          (グループの１要素)
         :

としてグループノードにエイリアスをつけておくと、別の場所にSUBSYSTEMと同じ構成があるとき、

.. code-block:: yaml

  elements: *SUBSYSTEM

で記述できます。
