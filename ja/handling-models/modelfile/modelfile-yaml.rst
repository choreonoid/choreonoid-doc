
.. highlight:: yaml

追加情報の記述
==============

Bodyファイルではトップレベルに任意のYAMLマッピングを記述することが可能で、その内容はプログラム側から取得できるようになっています。
Choreonoid本体においても、この形態で記述される情報を利用する機能がいくつもあります。
ここではそれらのうちの主要なものについて紹介します。

この機能を用いて、 :doc:`modelfile-openhrp` 等の :ref:`modelfile-yaml-add-information-to-another-model-format` を行うことも可能です。

.. note:: 以下で説明する記述のキーについては、 :ref:`body-file-reference-key-style` で説明した形式が当てはまります。以前はキャメルケースとなっていたキーも多いので、古いモデルファイルをみるときはその点にご注意ください。

.. contents::
   :local:
   :depth: 1

SR1サンプルモデルの追加情報
---------------------------

以下に示すのは、SR1サンプルモデル（"share/model/SR1/SR1.body"）において記述されている追加情報です。この例を通して、追加情報の具体的な記述方法を説明したいと思います。 ::

 standard_pose: [ 
    0, -30, 0,  60, -30, 0,
   20, -10, 0, -40,   0, 0, 0,
    0, -30, 0,  60, -30, 0,
   20,  10, 0, -40,   0, 0, 0,
    0,   0, 0 
 ]
 
 link_group:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y, RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, RLEG_KNEE, RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y, LLEG_KNEE, LLEG_ANKLE_P, LLEG_ANKLE_R ]
 
 foot_links:
   - link: RLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
 
 default_ik_setup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]
 
 collision_detection_rules:
   - disabled_link_chain_level: 3

標準姿勢の設定
--------------

:doc:`../pose-editing` の :ref:`model_body_bar` で紹介した「標準姿勢」は、Bodyファイルに追加情報として記述されています。これを行っているのが以下の部分です。 ::

 standard_pose: [ 
     0, -30, 0,  60, -30, 0,
    20, -10, 0, -40,   0, 0, 0,
     0, -30, 0,  60, -30, 0,
    20,  10, 0, -40,   0, 0, 0,
     0,   0, 0 
 ]

このように "standard_pose" というキーに標準姿勢に対応する関節角をリストとして記述します。関節角を並べる順番は関節IDの順で、関節角の単位は [degree] （直動関節の場合は [m]）になります。

リンクのグループ構造の設定
--------------------------

:doc:`../bodymodel` の :ref:`model_structure` で紹介した「リンク／デバイスビュー」では、モデルが有するリンクの一覧が表示され、モデルの構造を確認することができました。また、ここで編集操作の対象となるリンクを選択することもできました。

このリンク／デバイスビューではモデル構造の表示の仕方を上部のコンボボックスで切り替えられるようになっており、その中に「グループ化ツリー」という表示方法があります。これを選択するとSR1モデルの場合は以下のような表示になります。

.. image:: images/linkview_bodyparttree.png

ここでは、リンクが階層的にグループ化された身体部位ごとに分けられて表示されます。これを用いることで、リンクと身体部位の関係が把握しやすくなります。このため、この表示方法はキーポーズによる振り付け機能でも使われています。

このような階層グループ構造を記述しているのが、"link_group" というキーから始まる以下の部分です。 ::

 link_group:
   - name: UPPER-BODY
     links:
       - WAIST_P
       - WAIST_R
       - CHEST
       - name: ARMS
         links:
           - name: R-ARM
             links: [ RARM_SHOULDER_P, RARM_SHOULDER_R, RARM_SHOULDER_Y,
                      RARM_ELBOW, 
                      RARM_WRIST_Y, RARM_WRIST_P, RARM_WRIST_R ]
           - name: L-ARM
             links: [ LARM_SHOULDER_P, LARM_SHOULDER_R, LARM_SHOULDER_Y, 
                      LARM_ELBOW, 
                      LARM_WRIST_Y, LARM_WRIST_P, LARM_WRIST_R ]
   - WAIST
   - name: LEGS
     links:
       - name: R-LEG
         links: [ RLEG_HIP_R, RLEG_HIP_P, RLEG_HIP_Y, 
                  RLEG_KNEE, 
                  RLEG_ANKLE_P, RLEG_ANKLE_R ]
       - name: L-LEG
         links: [ LLEG_HIP_R, LLEG_HIP_P, LLEG_HIP_Y,
                  LLEG_KNEE, LLEG_ANKLE_P,
                  LLEG_ANKLE_R ]


ここでは、マップとリストの組み合わせでグループとそこに分類されるリンクを記述しています。"name" はグループ名を表していて、"links" 以下にそこに所属するリンクや下位のグループを記述しています。

足リンクの設定
--------------

脚型のモデルについては、どのリンクが足のリンクであるかを明示し、さらに足の操作に関する情報を記述しておくことで、Choreonoidが提供する脚型モデルを対象とした機能を活用できるようになります。これを行っているのが以下の部分です。 ::

 foot_links:
   - link: RLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]
   - link: LLEG_ANKLE_R
     sole_center: [ 0.05, 0.0, -0.055 ]

このように、"foot_links" というキーに足に相当する（床と設置可能な足裏を有する）リンクの情報をリストで列挙します。各足リンクの情報は、"link"というキーにリンク名を記述し、"sole_center"というキーに足裏の中心点を足リンクからの相対座標で記述します。これによって、例えば :ref:`model_legged_body_bar` の機能が使えるようになります。

.. note:: "sole_center" に記述する中心点は、重心投影点やZMPがそこにあるときに一番安定となる点を想定したものであり、必ずしも幾何学的な中心である必要はありません。例えば制御上足首付近が安定点である場合は、仮に足首が足裏の中心から外れた位置に接続されている場合でも、sole_centerには足首の位置を設定しておきます。

.. _modelfile_yaml_preset_kinematics:

プリセット運動学の設定
----------------------

:doc:`../pose-editing` - :ref:`model_kinematics_mode` で述べた「プリセット運動学モード」では、ユーザが動かそうとしてるリンクに応じて自動的に順運動学と逆運動学が切り替わるようになっていました。この設定を行っているのが、追加情報ファイルにおける以下の部分です。 ::

 default_ik_setup:
   WAIST: [ RLEG_ANKLE_R, LLEG_ANKLE_R ]
   RLEG_ANKLE_R: [ WAIST ]
   LLEG_ANKLE_R: [ WAIST ]

ここで行っている設定は以下の２つです。

* WAISTリンク（腰）を動かす際には、RLEG_ANKLE_Rリンク（右足）とLLEG_ANKLE_Rリンク（左足）の両方をベースリンクとして固定した逆運動学を行う
* RLEG_ANKLE_Rリンクを動かす際には、WAISTリンクをベースリンクとした逆運動学を行う
* LLEG_ANKLE_Rリンクを動かす際には、WAISTリンクをベースリンクとした逆運動学を行う

このように、プリセット運動学モード時に逆運動学としたいリンクと、その際のベースリンクを指定すればOKです。

.. _modelfile_yaml_collision_detection:

干渉検出の設定
--------------

Choreonoidではリンク間の干渉検出を処理することができます。Choreonoidの設定で干渉検出を有効にすると、基本的には全てのリンクが干渉検出の対象となります。（ただし、ボディアイテムのプロパティで各ボディごとに干渉検出を有効にするか切り替えられるようになっています。自己干渉の検出についても切り替えが可能です。）

他のリンクに埋め込まれた関節や、複数の回転軸を組み合わせた関節において、関節内部での干渉は可動範囲内では本来は起こさないように設計する必要がありますが、モデルファイルの形状をそこまで作りこむのには手間がかかることもあります。逆に、柔軟な表面で覆われたリンクでは設計上干渉が許容されることもあります。そのような場合に、特定のリンクや特定のリンクペアを干渉検出の対象外とすることで、Choreonoid上での干渉検出を適切に処理することが可能となります。

この設定は "collision_detection_rules" に記述することができます。SR1では以下のように記述されています。 ::

 collision_detection_rules:
   - disabled_link_chain_level: 3

collision_detection_rules直下の要素としては、YAMLのリストを記述するようになっており、リストの各要素ごとにルールを記述するようになっています。これにより、複数のルールを組み合わせることができます。

"disabled_link_chain_level" については、関節ツリーにおいて親子関係で隣接しているリンクを自己干渉から外す設定です。このルールが記述されていないか、値が0に設定されている場合は、対象のボディに含まれる全てのリンクのペアについて干渉が無いかをチェックします。一方で、ここに1以上の値を設定すると、リンクツリーにおけるノード間の距離がその値以下のペアは自己干渉の対象から除外するようになります。例えば、1を設定すると直接の親子関係にあるリンク同士は自己干渉チェックから除外されますし、2の場合はあるリンクに対してさらに親の親や、子の子（孫）、兄弟関係にあるリンクも除外されるようになります。

利用可能なルールを以下の表に示します。

.. list-table:: ヘッダのフィールド
 :widths: 15,85
 :header-rows: 1
 :align: left

 * - ルール（キー）
   - 内容
 * - disabled_link_chain_level
   - 対象ボディの自己干渉に関して、リンクツリーにおいて干渉検出を無効化する距離
 * - disabled_links
   - 干渉検出を無効化するリンクをリスト形式で記述
 * - disabled_link_group
   - 干渉検出を無効化するリンクのグループをリスト形式で記述。ここに記述されたリンク同士の干渉検出は行われなくなる。
 * - enabled_links
   - 干渉検出を有効にするリンクのグループをリスト形式で記述。
 * - enabled_link_group
   - 干渉検出を有効にするリンクのグループをリスト形式で記述。ここに記述されたリンク同士の干渉検出は行われる。

ルールの基本として、デフォルトでは全てのリンクに対して干渉検出が有効となっています。
あるリンクの干渉検出が有効である場合、干渉検出が有効となっている他の全てのリンクとの干渉検出が行われます。

これに対して、disabled\_ ではじまるルールを記述することで、干渉検出の対象外となるリンクを設定することができます。
ただし disabled_link_chain_level や disabled_link_group で一括して対象外となったリンクの一部に対して、干渉検出を有効に戻したいことがあります。その場合は enabled\_ ではじまるルールを追加で記述すればOKです。
そのように複数のルールがある場合は、ルールを記述した順番に適用されます。
ですから、まずは無効化するルールを記述して、必要に応じてその中の一部を有効に戻すためのルールを記述するというのが、一般的な記述方法になります。


.. 他に "excludeLinkGroups" というのもある。また、AGXではexcludeSelfCollisionLinksというのも使えるようだ。それらはAGXのマニュアルを参照。ここにそれらのパラメータについても補足を加えておく。

その他の情報の記述について
--------------------------

以上、SR1サンプルで記述されている主な情報について説明しましたが、追加情報はYAMLの文法に従っていて、モデルファイルの既存のキーと競合しなければ、どのような情報を記述してもOKです。その内容はChoreonoid内部で読めるようになっており、各機能はこれによって必要な情報を得ることができます。これによって、新たに導入するプラグインが要求する情報を記述しておけば、そのプラグインの機能を使えるようになりますし、ユーザがプラグインを開発する場合でも、必要な情報をユーザが定義して利用することができます。このように、YAMLによる追加情報は柔軟に扱えるようになっており、Choreonoidの機能拡張においても重要な役割を果たす仕組みとなっています。

.. _modelfile-yaml-add-information-to-another-model-format:

他形式モデルファイルへの情報の追加
----------------------------------

SR1.bodyでは、モデルファイルがChoreonoid標準のBody形式で記述されており、上記の追加情報もそのファイル内にまとめて記述していました。

しかし、他形式で記述された既存のモデルファイルをそのまま使いたい場合など、Body形式以外のモデルファイルに対して、追加情報を設定したいこともあるかと思います。

その場合は、まず追加情報を記述するYAMLのファイルを用意します。拡張子は通常 .yaml としておきます。

そしてそこに追加情報を記述します。

その上で、YAMLのファイル内に以下の記述をします。 ::

 model_file: モデルファイル名

例えば、OpenHRP形式で記述されたモデルファイル "robot.wrl" がある場合、 ::

 model_file: robot.wrl

などとします。

追加情報を記述したYAMLのファイルとモデルファイル本体が同じディレクトリにある場合は、本体のファイル名のみでOKです。異なるディレクトリにある場合は、そのディレクトリへの相対パスまたは絶対パスで記述します。

そして、Choreonoidからの読み込み時には、YAMLのファイルを読み込むようにします。

このようにすると、モデル本体の情報は他形式で読み込みつつ、そのモデルに対する追加情報も読み込まれることになります。
