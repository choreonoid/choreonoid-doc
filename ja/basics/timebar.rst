
時間軸の操作
============

.. contents::
   :local:
   :depth: 1


仮想時間とタイムバー
--------------------

Choreonoid上では仮想的な時間軸が設定されており、その現在時刻をユーザの操作によって変えることができます。これによって、時間的な広がりを持つデータの表示位置（時刻）を変えたり、そのようなデータに従ってアニメーション表示を行ったりすることができます。

そのような時間軸の操作を行うツールバーとして、以下の「タイムバー」が用意されています。

.. image:: images/TimeBar.png

ここではこのタイムバーの操作を中心に、時間軸の操作について解説します。

.. _basics_sequence_data:

シーケンスデータ
----------------

時間軸操作の対象となるのは、時間的な広がりを持つデータで、それらをまとめて「シーケンスデータ」と呼ぶことにします。Choreonoidでは、シーケンスデータに関してもその多くがアイテムとして定義されています。具体的なシーケンスデータアイテムの例としては、以下のようなものがあります。

.. tabularcolumns:: |p{4.0cm}|p{11.0cm}|

.. list-table::
 :widths: 30,70
 :header-rows: 1

 * - アイテム型
   - 概要
 * - MultiValueSeqアイテム
   - 浮動少数スカラ値のシーケンスデータを複数格納するアイテム。ロボットの関節角軌道の格納等に用いられる。
 * - MultiSE3Seqアイテム
   - 3次元位置・姿勢のシーケンスデータを複数格納するアイテム。ロボットの各パーツ（リンク）の位置・姿勢の格納等に用いられる。
 * - Vector3Seqアイテム
   - 3次元ベクトルのシーケンスデータを格納するアイテム。重心位置、ZMP等の軌道の格納に用いられる。
 * - ボディモーションアイテム
   - :doc:`../handling-models/bodymodel` の動作軌道を格納するアイテム。ロボットの動作パターンやシミュレーション結果等が格納される。MultiValueSeqItem、MultiSE3SeqItem、Vector3SeqItem等からなる :ref:`basics_composite_item` として定義されている。
 * - ポーズ列アイテム
   - キーポーズのシーケンスを格納するアイテム。ロボット動作の振り付けで利用する。
 * - 音声アイテム
   - 音声データを格納するアイテム。
 * - メディアアイテム
   - 動画等のメディアデータを格納するアイテム。


以下で説明するタイムバーの操作を試すために、何らかのシーケンスデータアイテムを用意しておきましょう。例えば :ref:`basics_project_sr1walk` で紹介したように、SR1Walkプロジェクトを読み込んでシミュレーション開始ボタンを押すと、シミュレーション結果が "AISTSimulator-SR1" というアイテムとして生成されます。これは上に挙げたボディモーションアイテム型のアイテムですので、時間軸操作の対象とすることができます。


シーケンスデータの関連付け
--------------------------

シーケンスデータの表示に関しては、シーケンスデータそのものを直接表示する場合と、シーケンスデータのある時刻の内容で別のデータを更新し、そのデータを介して間接的にシーケンスデータの内容を確認する場合が考えられます。

例えばロボットの動作軌道データに関しては、動作軌道そのものをグラフ等を用いて表示したり、動作軌道の各時刻の関節角等の数値を表示する場合が前者に該当します。一方、動作軌道の各時刻の関節角等の状態をロボットのモデルに適用し、その際のロボットの姿勢を表示する、というのが後者に該当します。

後者の場合は何らかのかたちでデータ間の関連付けを行っておくことで、シーケンスデータが表示に反映されることになります。

関連付けは通常アイテムのツリー構造を用いて行われます。ロボットモデルとその動作軌道データの場合、ボディアイテムとボディモーションアイテムがそれぞれに対応するアイテムです。これらを関連付けるためには、ボディモーションアイテムがボディアイテムの子アイテムとなっていればOKです。

例えば、SR1Walkのシミュレーションにおいて、アイテムツリーは以下のようになっています。

.. image:: images/ItemTreeView.png

ここでシミュレーション結果の "AISTSimulator-SR1" は "SR1" の子アイテムとして生成されており、これによって両者の関連付けが行われていることが分かります。

ユーザが自分でデータを作成したり読み込んだりする際には、このような関連付けのためのツリー構造となるよう各アイテムの配置を行ってください。

シーケンスデータの選択
----------------------

シーケンスデータの表示に際しては、直接表示、間接表示いずれの場合も、表示したいデータアイテムをアイテムツリービュー上で選択しておく必要がある場合があります。これは、表示の候補となるデータアイテムが複数あり得る場合に、どのアイテムを実際に表示するかを決定するために必要な操作となっています。

ボディモーションアイテムの例でも、アイテムの選択をしておく必要がありますので、その操作を行なってください。

.. note:: シミュレーション結果に関しては、シミュレータアイテム型のアイテム（SR1Walkサンプルでは "AISTSimulator" とあるアイテム）を選択しておけば、シミュレーション結果のボディモーションアイテムが全て選択されているのと同じ意味になります。（シミュレーション直後はこの状態になっています。）この場合、シミュレートした仮想世界に含まれるモデルが複数ある場合でも、シミュレータアイテムをひとつ選択するだけで、全てのモデルの動作結果が表示されることになりますので、シミュレーション結果の表示においては通常この選択操作を行えばOKです。


時刻の表示と変更
----------------

Choreonoid上の仮想時間における現在時刻は、タイムバーの以下の領域に表示されます。単位は通常「秒」となります。

.. image:: images/timebar_time.png

また、この領域は値の入力もできるようになっており、値を入力することにより現在時刻を変更することも可能となっています。また、入力ボックスの矢印ボタンや、キーボードの上下キーを押すことで、一定間隔で数値を変化させることも可能です。

時刻スライダ
------------

以下の時刻スライダの位置により、現在時刻を大まかに把握することができます。

.. image:: images/timeslider.png

また、このスライダをマウスでドラッグすることで、現在時刻を連続的に変化させることも可能です。これに伴ってシーケンスデータの表示も連続的に更新されるので、シーケンスデータの各時刻の内容に変化がある場合、それがアニメーションとなって現れることになります。従って、時刻スライダは手動でアニメーションを行うためのインタフェースにもなっています。


アニメーション再生
------------------

タイムバーの以下のボタンを使うことにより、自動のアニメーション再生を行うこともできます。

.. image:: images/play_buttons.png

これらふたつのボタンはどちらもアニメーションを開始するためのボタンです。左のボタンでは現在時刻がどこにあっても、時刻0から（正確には :ref:`basics_timebar_range` の最小時刻から）再生を開始します。右のボタンの場合、現在時刻からの再生になります。

再生中は現在時刻が一定の速度で更新されていき、実世界と同様の時間経過によるアニメーションを閲覧することができます。

アニメーションの再生中には、右側のボタンの形状が以下のような「再生停止」ボタンに変化します。

.. image:: images/play_stop_buttons.png

このボタンを押すことで再生が停止します。再生が停止すると、ボタンのアイコンと機能は元の再生開始ボタンに戻ります。

なお、再生に関する上記２つのボタンの機能は、ショートカットキーとしてそれぞれ"F5"キーと"F6"キーに割り当てられています。


.. _basics_timebar_range:

時間範囲
--------

タイムバーの扱う時間の範囲は、以下の数値入力ボックスで設定することができます。

.. image:: images/timebar_range.png

左が最小時刻、右が最大時刻を表しており、時刻スライダの位置と時刻の関係もこの範囲によって変わります。扱うシーケンスデータの時間長が長い場合は、それに合わせてこの時間範囲も長めに設定しておきます。ただし必要以上に長くすると、時刻スライダで有効な範囲が狭くなってしまい、スライダを使った頭出しやアニメーションの操作がやりづらくなってしまうので、対象データに合わせて適切な範囲に指定しておくことが推奨されます。

.. _basics_timebar_config:

仮想時間の扱いに関する設定
--------------------------

タイムバーは以下の「設定」ボタンを備えています。

.. image:: images/timebar_config.png

このボタンを押すと、以下の設定ダイアログが表示され、
仮想時間の扱いに関する設定を行うことができます。

.. image:: images/timebar_config_dialog.png

設定項目は以下のようになっています。

.. tabularcolumns:: |p{4.0cm}|p{11.0cm}|

.. list-table::
 :widths: 30,70
 :header-rows: 1

 * - 項目名
   - 設定内容
 * - 内部フレームレート
   - Choreonoidの内部処理で使われる時間分解能を設定します。この値は、例えば動力学シミュレーションにおけるデルタタイムや、キーフレーム補間で生成される動作軌道のフレームレート等に用いられます。
 * - 再生フレームレート
   - アニメーション再生におけるフレームレートを設定します。対象データのフレームレートがこれより細かくても、アニメーションはこの分解能で行われることになります。（ただしこの値は最大のフレームレートで、描画等の処理にかかる時間によってはこの値よりも少ないフレームレートになる場合もあります。）
 * - アイドルループ駆動モード
   - このモードをオンにすると、再生フレームレートの設定によらず、余分なCPUパワーをアニメーション再生時のフレームレート向上に用いるようになります。
 * - 再生スピード倍率
   - アニメーション再生における速度を実時間の何倍にするかを設定します。デフォルトの1.0だと実世界と同じ速度の再生になります。2.0を設定すると２倍速の再生になります。
 * - 進行中の更新に同期
   - 再生スピード倍率によらず、対象シーケンスデータの更新速度に同期した再生にします。例えば、シミュレーションによって更新中の動作軌道を再生している場合、シミュレーションの計算速度に同期した再生となります。
 * - 時間範囲の自動拡張
   - アニメーション再生中に最大時刻に達した場合、最大時刻を更新しながらアニメーションを継続します。
