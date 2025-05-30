============
ビューの作成
============

.. contents:: 目次
   :local:

.. highlight:: cpp      

概要
----

ChoreonoidのGUIの主要な構成要素のひとつとして :ref:`basics_mainwindow_view` があります。
ビューはメインウィンドウ上で任意の位置に配置可能な矩形領域で、その中に各種表示や操作、編集を行うためのインタフェースを備えるものです。本節ではプラグインで独自のビューを作成して使えるようにする方法を解説します。

Viewクラス
----------

Choreonoidの各ビューは全て `Viewクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1View.html>`_ のオブジェクトとして実装されています。独自のビューを作成する場合は、このViewクラスを継承したクラスを定義し、そこにビューの処理内容を実装する必要があります。

Viewクラスは :ref:`plugin-dev-toolbar-class` と同様にQWidgetを継承したウィジェットであり、それをビュー領域の土台として任意のGUIを構築できるようになっています。そこはやはり :ref:`plugin-dev-toolbar-use-qt-classes` によっていかに目的を達成する表示やインタフェースを構築していくかということになります。もちろん目的を達成するにあたっては、プロジェクトアイテムをはじめとするChoreonoidの多様な構成要素を活用することも重要です。

その上で、ViewクラスはViewの状態や機能に関わる関数を備えており、それらも利用しながらビューを実装していくことになります。以下ではViewの実装で必要となる処理をカテゴリごとに紹介し、関連するメンバ関数についても解説します。

.. _plugin-dev-view-initialization:

ビューの初期化
--------------

Viewクラスでは初期化処理としてビューの機能に必要なGUI要素を構築する必要があります。
その際にQtのレイアウトクラスを用いることも多いと思いますが、最上位のレイアウトについては、以下のメンバ関数でビューに設定できます。

* **void setLayout(QLayout* layout, double marginRatio = 0.0)**

  * ビューに最上位のレイアウトを設定します。

  * 引数marginRatioで、ビュー領域の境目からのマージンを、デフォルトのマージン値からの割合で設定します。0.0の場合はマージン無しとなり、1.0の場合はデフォルトのマージンと同じになります。

*  **void setLayout(QLayout* layout, double leftMarginRatio, double topMarginRatio, double rightMarginRatio, double bottomMarginRatio)**

  * setLayout関数の引数が異なるバージョンです。こちらの関数では上下左右の各境界からのマージンを個別に設定できます。

これらの関数はQWidgetのsetLayout関数にマージン調整の機能を追加したものです。
ビューの場合はその領域の大部分がひとつのウィジェットで占められることも多く、その場合は境界からのマージンを無くした方が自然な表示になります。そのような場合も含めてマージンの調整をしやすくするために上記の関数が用意されています。
marginRatioに1.0を指定した場合は、QWidgetクラスのsetLayout関数と同じ挙動になります。

また、ビューをメインウィンドウ上のどの領域に配置するかについて、以下の関数で設定できます。
これはビュークラスのコンストラクタで設定します。

* **void setDefaultLayoutArea(LayoutArea area)**

  * ビューを初めて表示する際にデフォルトで配置されるメインウィンドウ上の領域を指定します。

  * LayoutAreaはViewクラスで定義されている列挙型で以下のいずれかの値を指定します。

    * **TopLeftArea** : 左上
      
    * **MiddleLeftArea** : 左中央
      
    * **BottomLeftArea** : 左下
      
    * **TopCenterArea** : 中央上
      
    * **CenterArea** : 中央
      
    * **BottomCenterArea** : 中央下
      
    * **TopRightArea** : 右上
      
    * **MiddleRightArea** : 右中央
      
    * **ButtomRightArea** : 右下

.. note:: LayoutAreaで指定される領域は、その時々でメインウィンドウ上に実際に存在するビューの領域から当てはまると思われるものが推測されます。ただしビューのレイアウトはユーザが自由に変更可能なので、必ずしもLayoutAreaの各値に対応する領域が存在するとは限りません。また現状では領域を推定する処理が完全ではありません。従って想定とは異なる領域に配置されてしまうことがあります。
	  
またビューはフォントのズーム機能を備えており、以下の関数で有効化できます。    

* **void enableFontSizeZoomKeys(bool on)**

  * この関数でtrueを指定すると、ビューで使用しているフォントのサイズをキーボード操作で拡大縮小できるようになります。

  * 使用するキーは Ctrl + "+" で拡大、 Ctrl + "-" で縮小となります。

.. _plugin-dev-view-state-detection:

ビューの状態の検知
------------------

ビューは「アクティブ」な状態とそうでない状態があります。
アクティブな状態とは、ビューが表示されていて、ユーザから操作できるようになっている状態です。
ビューがメインウィンドウに含まれていても、タブで重ねられている場合もありますが、タブが選択されていないビューについてはユーザからは見えないため、アクティブな状態とはみなされません。
ビューの中にはプロジェクトアイテムの変化やユーザの操作に連動して処理を行うものもありますが、ビューがアクティブでないときは処理を行ってもその結果がユーザから見えるわけではなく、処理が無駄になってしまいます。これを避けるため、ビューの状態がアクティブなときのみビューの処理を行うようにすることが重要です。

このためにViewクラスでは状態の変化を通知する以下のvirtual関数が定義されています。

* **virtual void onActivated()**

  * ビューがアクティブな状態になるときに呼ばれます。
 
* **virtual void onDeactivated()**

  * ビューがアクティブな状態ではなくなるときに呼ばれます。

これらの関数をオーバーライドすることで、ビューがアクティブなときとそうでないときで処理を切り分けることができます。
例えば何らかのシグナルに反応して処理を行うビューの場合、onActivatedの中でシグナルと接続し、onDeactivatedの中でシグナルとの接続を解除するようにすれば、ビューがアクティブなときのみ処理を行うようにできます。

アクティブ状態の変化はViewクラスが備える以下のシグナルで検知することもできます。

* **SignalProxy<void()> sigActivated()**

  * ビューがアクティブ状態になったときに送出されるシグナルです。
 
* **SignalProxy<void()> sigDeactivated()**

  * ビューがアクティブ状態でなくなったときに送出されるシグナルです。

これらのシグナルは主にビューの状態変化を外部から検知したい場合に使用します。

アクティブ状態以外の状態変化として、ビューの領域に対するキーボードフォーカスの変化も検知できます。
これは以下のvirtual関数をオーバーライドすることで実現できます。

* **virtual void onFocusChanged(bool on)**

  * ビューの領域に対するキーボードフォーカスが変化したときに呼ばれます。

ビューのアクティブ状態とフォーカスが実際にどうなっているかについては、以下のメンバ関数で確認することができます。

* **bool isActive() const**

  * アクティブ状態のときにtrueを返します。

* **bool hasFocus() const**

  * キーボードフォーカスが入っているときにtrueを返します。


QWidgetのイベント処理
---------------------

ViewクラスはQWidgetクラスを継承しているので、QWidgetに通知されるQtのイベントをビューの実装に利用できます。
これにより、マウスやキーボードからの入力によってビュー上の操作を行うことも可能となります。
イベントの検知は基本的には対応するvirtual関数をオーバーライドすることで実装します。
実際に利用できるイベントの詳細はQtのマニュアルでQWidgetのページを参照してください。
以下にビューの実装でよく使用されるイベント（に対応するvirtual関数）をいくつか挙げておきます。

* **virtual void keyPressEvent(QKeyEvent* event)**

  * キーボードのキーが押されたときに呼ばれます。

* **virtual void keyReleaseEvent(QKeyEvent* event)**

  * キーボードのキーが離されたときに呼ばれます。

* **virtual void mouseMoveEvent(QMouseEvent* event)**

  * マウスポインタがビュー上で移動する度に呼ばれます。

* **virtual void mousePressEvent(QMouseEvent* event)**

  * マウスのボタンが押されたときに呼ばれます。

* **virtual void mouseReleaseEvent(QMouseEvent* event)**

  * マウスのボタンが離されたときに呼ばれます。

* **virtual void mouseDoubleClickEvent(QMouseEvent* event)**

  * マウスのボタンがダブルクリックされたときに呼ばれます。

* **virtual void wheelEvent(QWheelEvent* event)**

  * マウスのホイールが操作されたときに呼ばれます。

* **virtual void paintEvent(QPaintEvent* event)**

  * 描画要求のイベントです。ウィジェットに直接描画する場合はこの関数に実装します。

各関数に引数として与えられるイベントオブジェクトからイベントに関する情報を取得することができます。
例えばマウス関連のイベントで引数として与えられるQMouseEventのオブジェクトからは、マウスカーソルの座標や押しているボタンの種類などを取得することが可能です。各イベントの詳細についてもQtのマニュアルを参照ください。

.. _plugin-dev-view-attached-menu:

付属メニューの実装
------------------

各ビューにはタブ部分を右クリックすると表示されるメニューが付属しています。
このメニューにはデフォルトで「ビューの分離」という項目が備わっていて、これを実行するとビューをメインウィンドウから分離することができます。
この付属メニューは任意の項目を追加してカスタマイズできるようになっており、ビューの設定や操作を行うひとつの手段として活用することができます。

メニューのカスタマイズはViewクラスの以下のvirtual関数をオーバーライドすることで実現できます。

* **virtual void onAttachedMenuRequest(MenuManager& menuManager)**

  * 付属メニューが表示される際に呼ばれる関数です。

  * 引数のmenuManagerを介してメニュー項目を追加することができます。


ここで引数に使用されている `MenuManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1MenuManager.html>`_ はGUIで使用するメニューを管理するためのクラスで、これを用いて任意のメニューを構築できます。
QtではQMenuクラスやQActionクラスを使用してメニューを構築するようになっており、MenuManagerでも実際には内部でこれらのクラスを使用しています。このMenuManagerを用いることで、Qtのクラスを直接使用するよりも効率的にメニューを構築することができます。

.. メニューの説明の節を別途作成してそこへのリンクをはる

Choreonoidが提供するウィジェットの利用
--------------------------------------

上述したようにビュー上に実装するインタフェースはQtのクラスを用いて自由に構築することができます。
QtはGUIの部品となる「ウィジェット」のクラスを多数備えていて、それらを組み合わせることで様々なインタフェースを構築できます。
そしてウィジェットについてはChoreonoid SDKで定義されているものもあり、それらもビューの構築に利用することができます。
それらの多くは、あるまとまった機能を提供する比較的複雑なウィジェットとなっています。
以下ではChoreonoid SDKで定義されている主要なウィジェットを紹介します。

まずBaseモジュールに含まれるウィジェットとして以下があります。
これらは汎用的または基盤的な機能を有しています。

* `ItemTreeWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemTreeWidget.html>`_

  * プロジェクトアイテムをツリー形式で表示するウィジェットです。

  * アイテムツリービュー（ `ItemTreeView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemTreeView.html>`_ ）の実装に利用されています。

  * 表示対象のアイテム型や各アイテムの外観およびコンテキストメニュー等をカスタマイズすることができます。これにより特定の作業に対象を限定したアイテムツリーを実現できます。

* `ItemPropertyWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemPropertyWidget.html>`_

  * プロジェクトアイテムのプロパティを表示・編集するウィジェットです。

  * アイテムプロパティビュー（ `ItemPropertyView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemPropertyView.html>`_ ）の実装に利用されています。

  * 表示対象のプロパティをカスタマイズすることも可能です。これにより特定の用途に対してプロパティを利用しやすくすることができます。

* `SceneWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneWidget.html>`_

  * シーングラフを3DCGで描画するウィジェットです。マウスやキーボードによる操作も可能です。

  * シーンビュー（ `SceneView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneView.html>`_ ）の実装に利用されています。

* `PositionWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1PositionWidget.html>`_

  * 3次元空間中の物体（剛体）の位置や姿勢を数値形式で表示・編集するためのウィジェットです。

  * 配置ビュー（ `LocationView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LocationView.html>`_ ）や、以下で紹介する `LinkPositionWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LinkPositionWidget.html>`_ の実装に利用されています。

* `GraphWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1GraphWidget.html>`_

  * 軌道データをグラフ形式で表示するためのウィジェットです。

  * `MultiValueSeqGraphView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1MultiValueSeqGraphView.html>`_ 等の各種グラフ表示ビューの実装に利用されています。

またBodyPluginに含まれるウィジェットとして以下も利用できます。
これらはBodyモデルの操作に利用することができます。
独自プラグインをBodyPluginに依存させることこれらのウィジェットも利用できるようになります。

* `LinkDeviceTreeWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LinkDeviceTreeWidget.html>`_

  * Bodyモデルが有するリンクやデバイスをツリーやリストの形式で表示・選択するためのウィジェットです。

  * Bodyプラグインのリンク／デバイスビュー（ `LinkDeviceListView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LinkDeviceListView.html>`_ ）の実装に利用されています。

  * Bodyモデルの構造を確認したり、操作対象のリンクやデバイスを選択するのに利用することができます。

* `LinkPositionWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LinkPositionWidget.html>`_

  * Bodyモデルの構成要素であるリンクの位置姿勢を数値形式で表示・編集するためのウィジェットです。

  * リンクポジションビュー（ `LinkPositionView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1LinkPositionView.html>`_ ）の実装に利用されています。

* `JointDisplacementWidget <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1JointDisplacementWidget.html>`_

  * Bodyモデルの関節変位を数値やスライダーを用いて表示・編集するためのウィジェットです。

  * 関節変位ビュー（ `JointDisplacementView <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1JointDisplacementView.html>`_ ）の実装に利用されています。

各ウィジェットの関数等の詳細についてはリファレンスマニュアルをご参照ください。

.. _plugin-dev-view-project-save:

ビューのプロジェクト保存
------------------------

:doc:`item-project-save` と同様に、ビューの状態もプロジェクトファイルに保存し、プロジェクト読み込み時に復帰することができます。
これはViewクラスの以下のvirtual関数をオーバーライドして実装することで実現できます。

* **virtual bool storeState(Archive& archive)**

  * ビューの状態を保存します。
 
* **virtual bool restoreState(const Archive& archive)**

  * ビューの状態を復帰します。

これらはItemクラスの :ref:`plugin-dev-state-store-restore-functions` であるstore関数とrestore関数に相当するもので、Archive型の引数をとる点も同じです。実装の仕方も基本的に同じですので、 :doc:`item-project-save` と同様に実装を行ってください。

なお、プロジェクトファイル読み込み時に状態復帰関数が呼ばれる順序は以下のようになります。

1. 各ビューのrestoreState関数が呼ばれる
2. アイテムのrestore関数がツリーの深さ優先探索順で呼ばれる

ビューとアイテムの間で状態復帰の依存関係があるときはこの順序を考慮する必要があります。
アイテムの復帰時にビューの状態を参照する場合は、ビューの状態の方が先に復帰していますので、特に問題はありません。
しかしビューの状態復帰においてアイテムの情報が必要な場合は、そのままrestoreState関数に実装しても、その時点ではまだアイテムが読み込まれていないのでアイテムの情報を得ることができません。

これは :ref:`plugin-dev-archive-class` の説明で紹介した :ref:`plugin-dev-archive-post-processing` を用いることで解決できます。ビューのrestoreState関数内で ::

 archive.addPostProcess([this](){ ... });

とすることで、全てのアイテムが読み込まれた後にaddPostProcess関数に与えたラムダ式の処理が実行されるようになります。

この後処理において、 :ref:`plugin-dev-archive-class` の :ref:`plugin-dev-archive-item-reference` で紹介したfindItem関数を用いることで、アイテムのID値からアイテムの実体を取得することができます。このためのID値はプロジェクト保存時にArhiveクラスのgetItemId関数を用いて取得することができます。この値をstoreState関数から適当なキーで出力してプロジェクトファイルに保存するようにしておくと、restoreStateの後処理でID値を得ることが可能となります。

ビューの登録
------------

実装したビューをユーザが利用できるようにするためには、ビューのクラスをシステムに登録する必要があります。
これには `ViewManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ViewManager.html>`_ の以下の関数を使用します。 ::

 template <class ViewType>
 ViewManager& registerClass(
     const std::string& className, const std::string& defaultInstanceName,
     int instantiationFlags = Single);

これはテンプレート関数になっていて、登録するビューのクラスをテンプレート引数に指定します。
各引数の意味を以下に示します。

* **className**

  * クラス名を指定します。

* **defaultInstanceName**

  * デフォルトのインスタンス名を指定します。

  * インスタンス名はビューのインスタンスごとに設定される名前で、この名前がビューのタイトルとしてタブの領域に表示されます。

  * ビューがデフォルトで生成される際にここで指定した名前がインスタンス名として使用されます。ユーザがマニュアル操作でビューを生成する場合は、インスタンス名もユーザが指定します。

* **instantiationFlags**

  * ビューのインスタンス生成に関わるフラグを指定します。ViewManagerクラスで定義されている以下のフラグを組み合わせて指定します。

    * **Single**

      * ビューのインスタンスをひとつだけ生成することができます。引数のデフォルト値はこのフラグとなります。

    * **Multiple**

      * ビューのインスタンスを複数生成することができます。
	
    * **Default**

      * ビューのインスタンスがデフォルトでひとつ生成されます。このフラグを指定しない場合はデフォルトでは生成されず、ビューを利用するにはユーザが生成操作を行う必要があります。ただしプロジェクトの読み込み時にビューの情報が含まれる場合は自動で生成されます。

instantiationFlagsについては通常はデフォルト値の "Single" で問題ないかと思います。
"Multiple" や "Default" は必要に応じて指定します。
なお、Defaultを指定してインスタンスがデフォルトで生成される場合でも、生成されたビューは必ずしもデフォルトでメインウィンドウに表示されるわけではありませんので、ご注意ください。プロジェクトを読み込む場合はそこに記録されているビューのレイアウト情報次第となりますし、プロジェクトを読み込まない場合はChoreonoidの内部に組み込まれているデフォルトのレイアウトが利用されます。

ビューの登録は :ref:`plugin-dev-item-type-registration` と同様に、通常プラグインクラスのinitialize関数から行います。ItemManagerの取得と同様に、 `Pluginクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Plugin.html>`_ の親クラスである `ExtensionManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_ で定義されている以下の関数でViewManagerのインスタンスを取得できます。

* **ViewManager& viewManager()**

例えばFooViewというビュークラスを登録する場合は以下のようにします。 ::

 viewManager().registerClass<FooView>("FooView", "Foo");

登録したビューは、メインメニューの「表示」−「ビューの表示」の該当するビューの項目にチェックを入れることで、メインウィンドウ上に配置され利用できるようになります。また複数生成可能なビューについては、「表示」−「ビューの生成」から該当するビューの項目を選択することで、追加のビューを生成・表示できます。

.. note:: `ViewManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ViewManager.html>`_ には、ビュークラスの登録以外にもビューを管理するための様々な関数が実装されています。これを用いてプログラムからビューの生成や取得、表示を行うこともできます。そのような処理に使用する関数の詳細はAPIリファレンスをご参照ください。
