================================
プロジェクトアイテムのシーン対応
================================

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

Choreonoidは :doc:`scenegraph` で構築される三次元シーンの表示を行う :ref:`basics_sceneview_sceneview` を備えており、それを通してChoreonoid上で扱う仮想世界の確認や編集を行えるようになっています。仮想世界の構成要素は主にプロジェクトアイテムによって提供されていて、シーンビュー上の表示は基本的にそれらのシーン要素を仮想世界全体のシーングラフに導入することによって実現しています。

本節ではプロジェクトアイテムがシーン要素を提供する方法を解説します。
これにより、プラグインで新たに導入する機能やデータをChoreonoidのシーン表示機能に対応させることが可能となります。

.. _plugin-dev-renderable-items:

シーン要素を提供するプロジェクトアイテム
----------------------------------------

Choreonoidの既存プロジェクトアイテムの中では、以下に挙げるようなものがシーン要素を提供しています。

* ボディアイテム

  * ロボットや環境のモデルを提供

* ワールドアイテム

  * ボディモデル同士の干渉データを提供

* ボディ追尾カメラアイテム

  * 特定のボディモデルを追尾するカメラを提供

* センサ可視化アイテム

  * ボディモデルが搭載するセンサの情報を提供

* シーンアイテム

  * 表示用のシーンモデルを提供

* ライティングアイテム

  * シーンを照らす光源を提供

* ポイントセットアイテム

  * 点群データを提供

各アイテムについて、アイテムツリービュー上のチェックボックスをオンにすることで、対応するシーン要素がシーンビューに導入されます。この結果、ボディモデルが表示されたり、ボディ追尾カメラを視点として選択できるようになったりします。

またシーンビュー上でマウス操作を行うことにより、アイテムの編集をできるものもあります。例えばボディアイテムについてはモデルの位置を動かしたり、関節を動かして姿勢を変えるといったことができます。

さらに提供されるシーン要素は他の機能と連携することもあります。例えばシミュレーションにGLビジョンシミュレータアイテムを導入することでカメラ等の視覚センサのシミュレーションが可能となりますが、これはボディアイテムやシーンアイテムが提供するシーン要素を取り込むことで実現しています。

このようにアイテムがシーン要素を提供することで、シーンを介した様々な操作や処理に対応させることができます。
そしてシーン要素の提供は独自プロジェクトアイテム型についても実現することが可能です。

.. _plugin-dev-renderable-item:
	       
RenderableItemによるシーン要素の提供
------------------------------------

シーン要素を提供するアイテム型については、 `RenderableItemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1RenderableItem.html>`_ を継承するようにします。

このクラス自体はアイテム型ではなく、Itemクラスを継承しているわけではありません。実際にはこのクラスは所謂「インタフェースクラス」と呼ばれるもので、クラスにある特定の機能を実装するために利用されるものです。インタフェースクラスは概ね以下のような形態で利用されます。

* 機能と関連するvirtual関数が定義されており、それらをオーバーライドして機能を実装する。
* 利用する側のクラスが他のクラスを継承する場合、それに追加して他重継承して利用する。
* あるクラスが必要に応じて複数のインタフェースクラスを同時に導入することもあり得る。

具体的にみていきましょう。まずRenderableItemの定義における主要部分を抽出すると以下になります。 ::

 class RenderableItem
 {
 public:
     virtual SgNode* getScene() = 0;
 };

これはBaseモジュールで定義されていて、RenderableItemヘッダをインクルードすることで利用できます。

そしてこのRenderableItemインタフェースを実装するアイテムクラスは以下のように定義します。 ::

 class FooItem : public Item, public RenderableItem
 {
     ...
     
     SgNodePtr scene;
 
 public:
     FooItem();

     ...
      
     virtual SgNode* getScene() override
     {
         if(!scene){
             scene = new SgXXX;
             ...
         }
	 return scene;
     }
 }


まず基底クラスはこのように基底となるアイテム型とRenderableItemインタフェースの他重継承とします。
そしてアイテム型に必要な関数を実装しつつ、RenderableItemのgetScene関数も実装するようにします。
getScene関数では、アイテムが提供するシーン要素をSgNode型で返すようにします。
この中身については特に決まりはなく、シーン要素として使用できるものであれば何でも結構です。
通常は生成したシーンをスマートポインタ型のメンバ変数で保持しておき、getSceneでは同じインスタンスを返すようにします。

このようにRenderableItemインタフェースを継承しているアイテム型については、アイテムの生成時にChoreonoidのフレームワークがRenderableItemであることを検知します。そしてシーンビューはRenderableItemのチェックボックスがONになるとそれを検知して、アイテムのgetScene関数によりシーン要素を取得します。取得したシーン要素はシーンビューが管理するシーングラフに追加され、表示対象のシーンに導入されることになります。

既存の :ref:`plugin-dev-renderable-items` は全てこの形態で実装されています。
そして独自のプロジェクトアイテム型についても、同じ形態で実装することで、シーン要素を提供することが可能です。

SceneWidgetEventHandlerによる操作機能の実装
-------------------------------------------

RenderableItemのgetScene関数を介して提供するシーン要素は、シーンビュー上でインタラクティブに操作することも可能です。
これを行うためには、シーン要素の中で操作対象としたいノードについて、 `SceneWidgetEventHandlerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneWidgetEventHandler.html>`_ を継承したノード型として定義しておきます。こちらもインタフェースクラスとなっており、RendrableItemと同様の形態で利用します。ただしSceneWidgetEventHandlerを継承するクラスはシーンのノード型である点が異なります。SceneWidgetEventHandlerではシーンビュー上での操作イベントに対応するvirtual関数が定義されており、対応したい操作のvirtual関数をオーバーライドすることで、シーンビュー上での操作を実装します。

.. note:: SceneWidgetEventHandlerの名前にある "SceneWidget" はSceneViewが内部で利用しているウィジェットで、シーングラフの基本的な描画・操作機能を提供するものです。

こちらも具体的にみていきましょう。まずSceneWidgetEditableインタフェースは以下のように定義されています。 ::

 class SceneWidgetEventHandler
 {
 public:
     // シーンビューのモード（閲覧、編集、etc.）が変更された
     virtual void onSceneModeChanged(SceneWidgetEvent* event);
     // マウスのボタンが押された
     virtual bool onButtonPressEvent(SceneWidgetEvent* event);
     // マウスのボタンが離された
     virtual bool onButtonReleaseEvent(SceneWidgetEvent* event);
     // マウスがダブルクリックされた
     virtual bool onDoubleClickEvent(SceneWidgetEvent* event);
     // マウスポインタが動いた
     virtual bool onPointerMoveEvent(SceneWidgetEvent* event);
     // マウスポインタがビューから離れた
     virtual void onPointerLeaveEvent(SceneWidgetEvent* event);
     // マウスのスクロール操作が行われた
     virtual bool onScrollEvent(SceneWidgetEvent* event);
     // キーボードのキーが押された
     virtual bool onKeyPressEvent(SceneWidgetEvent* event);
     // キーボードのキーが離された
     virtual bool onKeyReleaseEvent(SceneWidgetEvent* event);
     // フォーカスが変わった
     virtual void onFocusChanged(SceneWidgetEvent* event, bool on);
     // コンテキストメニューを要求された
     virtual bool onContextMenuRequest(SceneWidgetEvent* event, MenuManager* menu);
 };

各関数上部のコメントはその関数が呼ばれるイベント（タイミング）を示しています。
実際に関数が呼ばれるのは以下の条件を満たすときです。

* onSceneModeChanged

  * シーンビューのモード変更時に全てのSceneWidgetEventHandlerに対して呼ばれる

* それ以外のイベント

  * シーンビューが編集モード

  * シーンビュー上でマウスポインタが対象ノードもしくはその下位のオブジェクトを指している

戻り値がboolの関数については、実際に処理を行った場合はtrueを、そうでない場合はfalseを返すようにします。

引数eventは `SceneWidgetEventクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneWidgetEvent.html>`_ のオブジェクトで、これを介してイベントが発生した際の状態を得ることができます。具体的には以下の関数で情報を得ることができます。

* **const Vector3& point() const**

  * マウスポインタが指している三次元位置を返します。
    
* **const SgNodePath& nodePath() const**

  * マウスポインタが指しているノードのシーングラフにおける :ref:`パス <plugin-dev-scenegraph-path>` を返します。

* **double x() const**

  * マウスポインタの2次元ピクセル画面上のX座標を返します。

* **double y() const**

  * マウスポインタの2次元ピクセル画面上のY座標を返します。

* **double pixelSizeRatio() const**

  * マウスポインタが指している点の3次元空間サイズとピクセルサイズの比率を返します。

* **int key() const**

  * 押されているキーを返します。Qtの列挙型Qt::Keyで定義されている値になります。

* **int button() const**

  * 押されているマウスのボタンを返します。Qtの列挙型Qt::MouseButtonで定義されている値になります。
  * 左、中央、右ボタンがそれぞれQt::LeftButton、Qt::MidButton、Qt::RightButtonになります。
    
* **int modifiers() const**

  * 押されているキーボードモディファイア（Shift、Ctrl、Alt等のキー）の値を返します。
  * Qtの列挙型Qt::KeyboardModifiersで定義されている値になります。
  * Shift、Ctrl、AltがそれぞれQt::ShiftModifier、Qt::ControlModifier、Qt::AltModifierとなります。
  * 複数のモディファイヤが有効な場合は対応する値が論理和で格納されます。
  
* **double wheelSteps() const**

  * マウスホイールを上下移動量を返します。
  * 通常+1か-1になります。

* **const SgCamera* camera() const**

  * イベントが発生したビューで使用されているカメラを返します。

* **int cameraIndex() const**

  * カメラのインデックスを返します。
    
* **const SgNodePath& cameraPath() const**

  * カメラのノードパスを返します。

* **const Isometry3& cameraPosition() const**
  
  * カメラの三次元位置を返します。

* **bool getRay(Vector3& out_origin, Vector3& out_direction) const**

  * カメラのレイ（視線方向）を始点と向きのベクトルで返します。

* **SceneWidget* sceneWidget() const**

  * イベントが発生したSceneWidgetオブジェクトを返します。
  * ビューのモードはこのオブジェクトから取得できます。
  * SceneWidgetの詳細は `SceneWidgetクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneWidget.html>`_ を参照ください。
  
* **void updateIndicator(const std::string& message) const**
  
  * メインウィンドウ下部のステータスバーに表示されるメッセージを更新します。

シーンビューの操作対象としたいノードは例えば以下のようにして定義します。 ::

 #include <cnoid/SceneGraph>
 #include <cnoid/SceneWidgetEventHandler>

 using namespace cnoid;

 class OperableGroup : public SgGroup, public SceneWidgetEventHandler
 {
 public:
     ...

     virtual bool onButtonPressEvent(SceneWidgetEvent* event) override
     {
         // 子ノードに対してボタンが押された時の処理を記述
         ...
         return true;
     }
  };

このようにすると、OperableGroupはイベントハンドラを備えたグループノードとなります。
グループノード自体はシーン中で実体を持たないのでマウス操作の直接の対象とはなりませんが、これが子ノードとしてSgShapeなどを有している場合、その部分に対してなされた操作がこのクラスの関数を介して伝えられます。実際にマウスが指しているノードや座標などはevent引数の該当する関数で取得できます。

SceneWidgetEventHandlerの実装はこのようにSgGroupやSgPosTransformといったグループノードに実装することが多いですが、シーンノードであればどの型でも実装できるので、例えばSgShapeに実装することも可能です。ただしシーンノードではない（SgNodeを継承していない）シーンオブジェクトに対して実装しても、有効とはなりません。

既存の操作用ノードの利用
------------------------

Choreonoid SDKはシーンビュー上で特定の操作が可能なノード型も提供しています。
具体的には現在以下の2つのノード型を利用できます。

* `PositionDragger <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1PositionDragger.html>`_ 

  * ドラッグ可能な座標軸

* `RectRegionMarker <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1RectRegionMarker.html>`_ 

  * 空間中の領域を切り出すための2次元の矩形マーカ

これらのノード型はSceneWidgetEventHandlerを継承していて、シーンビュー上での操作に対する反応が既に実装されています。
それぞれBaseモジュールで定義されており、同名のヘッダをインクルードすることで使えるようになります。

PositionDragger
~~~~~~~~~~~~~~~

`PositionDragger <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1PositionDragger.html>`_ （ドラッガー）については、SgPosTransformを継承したノード型となっており、位置姿勢を有しています。
そしてX、Y、Zの各軸並進方向に動かしたり、各軸周りの回転を行うためのハンドル形状を備えており、そこをシーンビュー上でドラッグすることで、ユーザが位置姿勢を操作することができます。
ハンドルのサイズや形状などはPositionDraggerの関数で設定できますが、デフォルトの状態では以下のようになります。

.. image:: images/position-dragger.png
    :scale: 70%

ここで矢印の直線は並進移動のためのハンドルで、その間を結ぶ曲線は回転のためのハンドルです。
それぞれマウスポインタで指すとハイライト表示され、マウスの左ボタンを押しながらドラッグすると動かすことができます。

PositionDraggerは様々な設定項目を備えていて、利用形態に合わせたカスタマイズが可能となっています。
以下にの設定のための主要な関数を示します。

* **PositionDragger(int axes = AllAxes, int handleType = StandardHandle)** （コンストラクタ）

  * コンストラクタで表示するハンドル軸とハンドルの形状を指定できます。

  * デフォルトでは標準形状のハンドルの全ての軸が表示されます。
    
  * 軸に指定できる要素はPositionDraggerの列挙型AxisBitの要素となります。
    
  * 形状のタイプはPositionDraggerの列挙型HandleTypeから選択します。

* **void setDisplayMode(DisplayMode mode, SgUpdateRef update = nullptr)**

  * 表示モードを設定します。

  * PositionDraggerで定義されている列挙型DisplayModeから選択します。値はDisplayAlways（常に表示）、DisplayInEditMode（編集モードのときのみ表示）、DisplayInFocus（フォーカスされているときのみ表示）、DisplayNever（表示しない）となります。

  * デフォルトではDisplayInEditModeとなっています。

* **void setHandleSize(double s)**

  * ハンドルのサイズを設定します。
    
  * 設定するサイズは仮想空間内でのサイズであり、シーン表示のズームの変化によって画面上のサイズも変化します。

* **bool adjustSize(const BoundingBox& bb)**

  * ハンドルのサイズをbbで指定するローカルのバウンディングボックスに合うように調整します。
  
* **bool adjustSize()**
 
  * 子ノードのバウンディングボックスを取得してそれをもとに上記のadjustSizeを実行し、子ノードのオブジェクトに合うサイズに調整します。

* **void setPixelSize(int length, int width)**

  * ハンドルのピクセルサイズを設定します。

  * setHandleSizeとは異なり、表示画面の2次元ピクセルでのサイズ指定となります。シーン表示のズームが変化しても画面上のサイズが一定に保たれます。

  * 上記のadjustSizeを実行するとこちらの設定はキャンセルされます。

* **void setTransparency(float t)**

  * ハンドルの透明度を設定します。

  * 値は0〜1で、デフォルトで0.4となっています。

* **void setOverlayMode(bool on)**

  * オーバーレイモードを設定します。

  * このモードがオンの場合、ドラッガーのハンドルは、周囲のオブジェクトとの前後関係に関わらず、常に全体が表示されるようになります。これによって他のオブジェクトに隠されずに常にハンドルを操作できるようなります。

  * デフォルトではオフ（false）となっています。

* **void setDragEnabled(bool on)**

  * マウスによるドラッグ操作が有効となるか設定します。

  * ドラッガーを位置姿勢の確認用などのために表示のみしたい場合はこちらをオフにします。

  * デフォルトでオン（true）となっています。

* **void setContainerMode(bool on)**

  * コンテナモードを設定します。

  * コンテナモードの場合、子ノードを追加して子ノードの位置姿勢を表示・操作する想定となります。この場合ハンドルを操作すると自動でドラッガーの位置姿勢が変化し、それに伴って子ノードの位置姿勢も変化します。

  * コンテナモードでない場合は、ハンドルを操作してもドラッガーの位置姿勢はそのままでは変化しません。後述するシグナルを用いることでハンドルの操作を反映させます。これは既存ノードに小アイテムとしてドラッガーをアタッチする場合や、既存ノードと親子関係を持たずに連携する場合に利用します。またハンドルの操作に対して何らかの内部処理が必要な場合もこちらを使用します。

  * デフォルトではコンテナモードはオフ（false）になっています。

* **void setContentsDragEnabled(bool on)**

  * コンテナモードのときに、子ノードのオブジェクトを直接マウスでドラッグできるか設定します。

  * このモードを有効にするためにはドラッグ操作自体も有効となっている必要があります。

  * デフォルトでオン（true）になっています。

ドラッグ操作の状況については以下のシグナルや関数で検知できます。

* **SignalProxy<void()> sigDragStarted()**

  * ドラッグ開始時に送出されるシグナルです。

* **SignalProxy<void()> sigPositionDragged()**

  * ドラッグで位置姿勢が変化する度に送出されるシグナルです。

* **SignalProxy<void()> sigDragFinished()**

  * ドラッグ終了時に送出されるシグナルです。

* **Isometry3 draggingPosition() const**

  * ドラッグ中の位置姿勢を返します。座標はドラッガーの親ノードからのローカル座標になります。

  * コンテナモードがオフのときはハンドルをドラッグしただけではドラッガーの位置姿勢は変化しませんが、その場合でもこちらの関数でドラッグ先の位置姿勢を取得できます。ドラッグ中にこの値をドラッガーの位置姿勢として設定することで、ドラッグ操作と連動したドラッガーの移動も可能となります。

* **Isometry3 globalDraggingPosition () const**

  * ドラッグ中の位置姿勢をグローバル座標で返します。

    
RectRegionMarker
~~~~~~~~~~~~~~~~

`RectRegionMarker <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1RectRegionMarker.html>`_ は空間中の領域を切り出すための2次元の矩形マーカです。
これを使用するとシーンビュー上でマウスドラッグにより矩形領域を指定できます。
指定した矩形領域はプログラムから取得することが可能で、それをもとに指定した矩形に対応する三次元領域を算出することができます。
取得した領域はオブジェクトの選択や切り出し等に活用できます。

このマーカはシーン表示画面上に2次元のオブジェクトを描画するためのノードである `SgViewportOverlayクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1RectRegionMarker.html>`_ を継承して実装されています。その上でSceneWidgetEventHandlerも実装し、矩形領域を指定する操作を実装しています。

RectRegionMarkerの主な関数を以下に示します。

* **void startEditing(SceneWidget* sceneWidget)**

  * 矩形領域を指定する操作を開始します。操作対象のSceneWidgetを指定します。

* **SignalProxy<void(const PolyhedralRegion& region)> sigRegionFixed()**

  * 矩形領域が指定されるとこのシグナルが送出されます。PolyhedralRegion型の引数regionには対応する三次元領域の情報が含まれています。これを用いることで領域の判定ができます。

* **const PolyhedralRegion& region() const**

  * 指定された領域のPolyhedralRegionを返す関数です。

* **void finishEditing()**

  * 矩形領域の指定操作を終了します。

RectRegionMarkerはChoreonoidの既存プロジェクトアイテムである `PointSetItem <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1PointSetItem.html>`_ で使用されていて、アイテムが有する点群について、RectRegionMarkerで指定した領域内の点群を削除できるようになっています。このアイテムはBaseモジュールで実装されていて、そちらのソースコードがRectRegionMarkerの使用例として参考になるかと思います。
