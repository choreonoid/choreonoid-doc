
URDFモデルファイル
==================

.. contents::
   :local:
   :depth: 3

概要
----

URDF (Unified Robot Description Format)は、XMLを用いたロボットモデルの記法です。ROS (Robot Operating System)をはじめ、近年多くのプラットフォームで利用されています。ただし、現在の公式仕様では、ロボットの運動学的構造のみに対応しており、センサ等は未対応となっています。

.. 英訳指示： 「ただし、」の文は、Note thatではじめてください。

モデルファイルの規約
~~~~~~~~~~~~~~~~~~~~~

本ページは、`URDFの公式ドキュメント <http://wiki.ros.org/urdf/XML/model>`_ に基づき、ロボットシミュレーションに必要となる情報をまとめています。

URDFモデルを作成する際に注意すべき事項は以下の通りです。

* モデルファイルひとつにつき、ロボットや環境のモデル1体を記述するようにします。
* 1体のロボットは、リンクについて1つの木構造を持つ必要があります。すなわち、あるリンクから任意のリンクへは、ジョイントを介して辿り着くことができ、かつ辿り着く経路が一意である必要があります。したがって、パラレルリンク機構等の閉じた機構をURDF上で表現することはできません。
* 数値は全て、接頭辞なしのSI基本単位・組み立て単位になります。例えば、位置の単位にはメートルまたはラジアンを、質量の単位にはキログラムを用いる必要があります。

ノード・属性一覧
-----------------

URDFにおいてロボットモデルは、階層的なノード構造によって記述されます。同じ名前のノードは、場所に依らず同じ記法と役割を持っています。

下記は、Choreonoidでロボットモデルを読み込み、シミュレーションを行うために必要なノードの一覧です。実際のノードの階層構造に基づいています。全てのURDFノードを紹介してはおらず、jointノード以下に記述される、ロボットの実機や動作に関するノードの説明は省略しています。

* :ref:`urdf-file-reference-robot-node` （属性： `name` ）

  * :ref:`urdf-file-reference-link-node` （属性： `name` ）

    * :ref:`urdf-file-reference-inertial-node`

      * originノード（属性： `xyz` , `rpy` ）
      * massノード（属性： `value` ）
      * inertiaノード（属性： `ixx` , `iyy` , `izz` , `ixy` , `iyz` , `ixz` ）

    * :ref:`urdf-file-reference-visual-node`

      * originノード（属性： `xyz` , `rpy` ）
      * geometryノード

        * boxノード（属性： `size` ）
        * cylinderノード（属性： `radius` , `length` ）
        * sphereノード（属性： `radius` ）
        * meshノード（属性： `filename` , `scale` ）

      * materialノード（属性： `name` ）

        * colorノード（属性： `rgba` ）
        * textureノード（属性： `filename` ）

    * :ref:`urdf-file-reference-collision-node`

      * originノード（属性： `xyz` , `rpy` ）
      * geometryノード

        * boxノード（属性： `size` ）
        * cylinderノード（属性： `radius` , `length` ）
        * sphereノード（属性： `radius` ）
        * meshノード（属性： `filename` , `scale` ）

  * :ref:`body-file-reference-joint-node` （属性： `name`, `type` ）

    * originノード（属性： `xyz` , `rpy` ）
    * parentノード（属性： `link` ）
    * childノード（属性： `link` ）
    * axisノード（属性： `xyz` ）
    * limitノード（属性： `lower` , `upper` , `velocity` , `effort` ）

それぞれのノードの役割や記法について、以下にて詳説します。

.. _urdf-file-reference-robot-node:

robotノード
------------

robotノードは、1つのロボットを定義します。ロボットの定義は、全てrobotタグの内部で完結する必要があります。

.. list-table:: robotノードの属性
 :widths: 15,85
 :header-rows: 1

 * - 属性名
   - 内容
 * - name（必須）
   - ロボットの名称。モデル内で重複しない任意の文字列を指定可能です。

なお、Choreonoidで読み込む場合は別途表示名を付けられるため、複数の同じロボットを同時に読み込む場合であっても、モデルファイルは1つあれば十分です。

.. _urdf-file-reference-link-node:

linkノード
~~~~~~~~~~~

linkノードは、ロボットを構成する各リンクを定義します。

.. list-table:: linkノードの属性
 :widths: 15,85
 :header-rows: 1

 * - 属性名
   - 内容
 * - name（必須）
   - リンクの名称。モデル内で重複しない任意の文字列を指定可能です。

最も簡単なlinkノードは、名前だけを持つノードです。すなわち、 ::

     <link name="simple_link"/>

と記述するのみでリンクを構成することができます。このようなリンクは、移動体の回転中心や、マニピュレータの逆運動学に使う参照点など、ロボットに目印を付ける目的でしばしば利用されます。しかし、これだけでは物理シミュレーションや描画などを行うことはできません。そこで、以下の表に示す子ノードを記述することにより、リンクにさらなる情報を与えることができます。

.. list-table:: linkノードが持てる子ノード
 :widths: 15,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - :ref:`urdf-file-reference-inertial-node`
   - 重心位置や質量、慣性モーメントを定めます。
 * - :ref:`urdf-file-reference-visual-node`
   - 外観の形状や色などを定めます。
 * - :ref:`urdf-file-reference-collision-node`
   - 衝突形状。自己干渉や環境との衝突の検出に用いる形状情報を定めます。

.. _urdf-file-reference-inertial-node:

inertialノード
_______________

inertialノードは、物理シミュレーションの計算に用いる、リンクの重心位置や質量、慣性モーメントを定めます。inertialノードの記述は任意ですが、固定接続されたリンクの中で少なくとも1つのリンクにinertialタグが設定されていなければ、物理シミュレーションを行うことはできません。

.. list-table:: inertialノードが持てる子ノード
 :widths: 25,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - originノード（任意）
   - `xyz` 属性で重心位置を、 `rpy` 慣性モーメントの座標系の回転をロール・ピッチ・ヨー表現で、リンクの座標系の上で定めます。各属性は省略可能で、省略された場合はゼロに設定されます。もしノード自体が省略された場合は、 `xyz` ， `rpy` の両者が共にゼロとして扱われます。
 * - massノード（必須）
   - `value` 属性で質量を定めます。質量は正の値である必要があります。
 * - inertiaノード（必須）
   - 慣性モーメントを定めます。慣性モーメントの行列を構成する6成分 (`ixx`, `iyy`, `izz` `ixy`, `ixz`, `ixz`) をそれぞれ属性として指定します。 `ixx`, `iyy`, `izz` には必ず正の値が入ります。また慣性モーメントは正定値対称行列となる必要がありますので、CAD等の情報を用いずに `ixy`, `iyz`, `ixz` を入力する際には注意が必要です。

例::

    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>

.. _urdf-file-reference-visual-node:

visualノード
_____________

visualノードは、ロボットモデルの描画に用いられます。リンクの形状や色等をこのノード内で記述することで、各種GUI上でロボットモデルを表示できるようになります。1つのリンクに対して複数のvisualノードを定義することが可能であり、その場合は全てのvisualノードを重ね合わせたものが、そのリンクの外観となります。

.. list-table:: visualノードが持てる子ノード
 :widths: 25,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - originノード（任意）
   - geometryノードで定義する図形の原点を定めます。 `xyz` 属性で重心位置を、 `rpy` で座標系の回転をロール・ピッチ・ヨー表現で、いずれもリンクの座標系の上で定めます。各属性は省略可能で、省略された場合はゼロに設定されます。もしノード自体が省略された場合は、 `xyz` , `rpy` の両者が共にゼロとして扱われます。
 * - geometryノード（必須）
   - 図形の形状を定めます。詳細は次の表をご参照ください。
 * - materialノード（任意）
   - geometryノードで指定した図形の色とテクスチャを指定します。詳細は下記の表で後述します。

geometryノードは、幾何形状を定義するノードです。次の表に示すノードの中から **一つを選んで** 記述します。

.. list-table:: geometryノードが持てる子ノード一覧
 :widths: 15,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - boxノード
   - 直方体。中心が原点となります。 `size` 属性に、xyzの各軸に平行な辺の長さを与えます。
 * - cylinderノード
   - 円柱。中心が原点、Z軸が円の中心軸となります。 `radius` 属性で半径を、 `length` 属性で長さを、それぞれ与えます。（注：Y軸が円の中心軸となるBodyファイルとは，中心軸が異なります。）
 * - sphereノード
   - 球。中心が原点となります。 `radius` 属性で半径を与えます。
 * - meshノード
   - メッシュファイルを元にしたメッシュ。 `filename` 属性でファイルのパスを与えます。 パスは、ROSパッケージ名とパッケージ内での相対パスを用いて、 `package://<packagename>/<path>` の形で記述します。また、 `scale` 属性に各軸の伸縮倍率を与えることで、メッシュをスケーリングできます。もしメッシュファイルに色情報が含まれていれば、下記materialノードで上書きされない限り、その情報が利用されます。

materialノードは、幾何形状に対して表面の色やテクスチャを指定します。materialノードが指定されていない場合は、原則として白色が適用されます。

.. list-table:: materialノードが持てる子ノード一覧
 :widths: 15,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - colorノード
   - 図形の色。 `rgba` 属性に4つのパラメータを与えることで、RGBと透明度を指定します。それぞれの値は、0.0から1.0の間に正規化されたものを使用します。
 * - textureノード
   - 図形表面のテクスチャ。`filename` 属性でファイルのパスを与えます。 パスは、geometryノード内のmeshノードと同様に、ROSパッケージ名とパッケージ内での相対パスを用いて、 `package://<packagename>/<path>` の形で記述します。

また、materialノードはname属性で名前を与えることができます。一度定義されたmaterialノードは、以降のモデルファイル内では、 `<material name="material1"/>` のように名前を指定するのみで、同じ外観装飾を適用できます。

例1::

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
      <material name="Cyan">
        <color rgba="0 1 1 1"/>
      </material>
    </visual>

例2::

    <visual>
      <geometry>
        <mesh filename="package:://choreonoid/share/model/JACO2/parts/ARM.stl" scale="1 1 1"/>
      </geometry>
    </visual>

.. _urdf-file-reference-collision-node:

collisionノード
________________

collisionノードは、ロボットモデルの衝突判定に用いられます。物理シミュレーションやプランニングにおいて、自己干渉や環境との接触を検出するときに、collisionノードの情報が用いられます。visualノードと同様に、1つのリンクに対して複数のcollisionノードを定義することが可能で、全てのcollisionノードを重ね合わせたものが、そのリンクの衝突モデルとなります。

.. list-table:: collisionノードが持てる子ノード
 :widths: 25,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - originノード（任意）
   - geometryノードで定義する図形の原点を定めます。 `xyz` 属性で重心位置を、 `rpy` で座標系の回転をロール・ピッチ・ヨー表現で、いずれもリンクの座標系の上で定めます。各属性は省略可能で、省略された場合はゼロに設定されます。もしノード自体が省略された場合は、 `xyz` , `rpy` の両者が共にゼロとして扱われます。
 * - geometryノード（必須）
   - 図形の形状を定めます。詳細は :ref:`urdf-file-reference-visual-node` におけるgeometryノードの説明をご参照ください。ただしmeshノードを利用した場合、もしメッシュファイルに色情報が含まれていたとしても、その情報はcollisionノードでは無効になります。

例1::

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>

例2::

    <collision>
      <geometry>
        <mesh filename="package:://choreonoid/share/model/JACO2/parts/ARM.stl" scale="1 1 1"/>
      </geometry>
    </collision>

.. _body-file-reference-joint-node:

jointノード
~~~~~~~~~~~~

jointノードは、2つのリンクの関係を定義します。

.. list-table:: jointノードの属性
 :widths: 15,85
 :header-rows: 1

 * - 属性名
   - 内容
 * - name（必須）
   - ジョイントの名称。モデル内で重複しない任意の文字列を指定可能です。
 * - type（必須）
   - ジョイントの種類。可動域のある回転関節： `revolute`，車輪のように無限回転する関節： `continuous`，直動関節： `prismatic`，固定関係 `fixed`，一切の拘束がない `floating` のいずれかから一つを指定します。（他に平面上を動く `planar` がありますが、Choreonoidでは対応していないため、説明を省略します。）

さらにジョイントを定義するためには、リンクの親子関係や相対位置、可動域などを指定する必要があります。そこで以下の表に示す子ノードに、それらの情報を記述します。

.. list-table:: jointノードが持てる子ノード
 :widths: 30,85
 :header-rows: 1

 * - ノード名
   - 内容
 * - originノード（任意）
   - 2リンク間の相対位置・姿勢を定めます。 `xyz` 属性で親リンクの原点から見た、関節変位がゼロのときの子リンクの原点の相対位置を、 `rpy` で親リンクの座標系から見た子リンクの座標系の相対姿勢をロール・ピッチ・ヨー表現で、それぞれ与えます。各属性は省略可能で、省略された場合はゼロに設定されます。もしノード自体が省略された場合は、 `xyz` ， `rpy` の両者が共にゼロ、すなわち親リンクと子リンクの原点位置・姿勢が一致するようになります。
 * - parentノード（必須）
   - 親リンクを、 `link` 属性にリンク名を指定することで定めます。
 * - childノード（必須）
   - 子リンクを、 `link` 属性にリンク名を指定することで定めます。
 * - axisノード（任意）
   - 関節の軸を指定します。 `xyz` 属性に親リンクの座標系で見た関節の軸の方向を表す3次元ベクトルを指定します。jointノードの `type` 属性が `revolute`, `continuous`, `prismatic` のときのみ有効で、回転関節については回転軸を、直動関節については動作方向を指定します。向きによって関節変位の正負が決まることに留意してください。ノードがない場合のデフォルト値は `xyz="1 0 0"`，すなわち親リンクの座標系におけるx軸方向となります。
 * - limitノード（条件付き必須）
   - 関節の可動域、速度とアクチュエータ出力を定めます。jointノードの `type` 属性が `revolute` および `prismatic` のとき必須です。 `lower` 属性で負方向の可動限界を、 `upper` 属性で正方向の可動限界を指定します。可動域を表すこの両者の属性は、デフォルト値が0であり、両方とも指定されなかった場合関節は不動になります。さらに、 `velocity` 属性で関節速度の上限（単位は[m/s]もしくは[rad/s]）を、 `effort` 属性でアクチュエータの出力上限を（単位は[N]もしくは[Nm]）与えます。速度上限と出力上限は共に正の値である必要があり、これらにデフォルト値は存在しません。

例::

    <joint name="sample_joint" type="revolute">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>
      <axis xyz="1 0 0"/>
      <limit lower="-3.14" upper="3.14" velocity="1.0" effort="30"/>
    </joint>

.. _urdf-file-reference-sample-model:

サンプルモデル
--------------

Choreonoidでは、ヒューマノイドロボットのサンプルモデルである `SR1 <modelfile-sr1.html>`_ のURDF版を用意しています。Bodyファイルものと見比べることで、その対応関係を知ることができます。

* Body: https://github.com/choreonoid/choreonoid/blob/master/share/model/SR1/SR1.body
* URDF: https://github.com/choreonoid/choreonoid/blob/master/share/model/SR1/SR1.urdf
