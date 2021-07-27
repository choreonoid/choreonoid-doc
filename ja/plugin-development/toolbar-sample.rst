==============================
独自ツールバーのサンプル (S04)
==============================

.. contents:: 目次
   :local:

概要
----

前節で紹介したツールバー作成の実践として、ツールバーを利用するプラグインのサンプルを提示します。これはひとつ前のサンプルである :doc:`item-operation-sample` の拡張となっていて、選択したBodyアイテムのモデルを回すという点は同じですが、この機能に関連するツールバーを作成し、ツールバーのボタン等を用いて追加の操作や設定ができるようにしています。

ソースコード
------------

.. highlight:: cpp

今回のサンプルのソースコードです。これまでと同様に、プラグインのソースディレクトリを作成し、DevGuidePlugin.cppというファイル名で以下のソースコードを格納してください。 ::

 #include <cnoid/Plugin>
 #include <cnoid/ItemList>
 #include <cnoid/RootItem>
 #include <cnoid/BodyItem>
 #include <cnoid/ToolBar>
 #include <cnoid/TimeBar>
 #include <cnoid/DoubleSpinBox>
 #include <cnoid/EigenTypes>
 #include <vector>
 #include <cmath>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
     ItemList<BodyItem> bodyItems;
     double initialTime;
     std::vector<Matrix3> initialRotations;
     DoubleSpinBox* speedRatioSpin;
     ToolButton* reverseToggle;
 
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
         require("Body");
     }
 
     virtual bool initialize() override
     {
         RootItem::instance()->sigSelectedItemsChanged().connect(
             [this](const ItemList<>& selectedItems){
                 onSelectedItemsChanged(selectedItems);
             });
                    
         TimeBar::instance()->sigTimeChanged().connect(
             [this](double time){
                 return onTimeChanged(time);
             });
 
         auto toolBar = new ToolBar("DevGuideBar");

         ToolButton* flipButton = toolBar->addButton("Flip");
         flipButton->sigClicked().connect(
             [this](){ flipBodyItems(); });
 
         reverseToggle = toolBar->addToggleButton("Reverse");
         reverseToggle->sigToggled().connect(
             [this](bool on){ updateInitialRotations(); });
 
         toolBar->addSeparator();
 
         toolBar->addLabel("Speed ratio");
         speedRatioSpin = new DoubleSpinBox;
         speedRatioSpin->setValue(1.0);
         speedRatioSpin->sigValueChanged().connect(
             [this](double value){ updateInitialRotations(); });
         toolBar->addWidget(speedRatioSpin);
 
         toolBar->setVisibleByDefault();
         addToolBar(toolBar);
         
         initialTime = 0.0;
 
         return true;
     }
 
     void onSelectedItemsChanged(ItemList<BodyItem> selectedBodyItems)
     {
         if(selectedBodyItems != bodyItems){
             bodyItems = selectedBodyItems;
             updateInitialRotations();
         }
     }
 
     void updateInitialRotations()
     {
         initialTime = TimeBar::instance()->time();
         initialRotations.clear();
         for(auto& bodyItem : bodyItems){
             initialRotations.push_back(bodyItem->body()->rootLink()->rotation());
         }
     }
 
     void flipBodyItems()
     {
         for(auto& bodyItem : bodyItems){
             Link* rootLink = bodyItem->body()->rootLink();
             Matrix3 R = AngleAxis(M_PI, Vector3::UnitZ()) * rootLink->rotation();
             rootLink->setRotation(R);
             bodyItem->notifyKinematicStateChange(true);
         }
         updateInitialRotations();
     }
 
     bool onTimeChanged(double time)
     {
         for(size_t i=0; i < bodyItems.size(); ++i){
             auto bodyItem = bodyItems[i];
             double angle = speedRatioSpin->value() * (time - initialTime);
             if(reverseToggle->isChecked()){
                 angle = -angle;
             }
             Matrix3 R = AngleAxis(angle, Vector3::UnitZ()) * initialRotations[i];
             bodyItem->body()->rootLink()->setRotation(R);
             bodyItem->notifyKinematicStateChange(true);
         }
 
         return !bodyItems.empty();
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)
	       
ビルド用のCMakeLists.txtは :doc:`item-operation-sample` と同じになります。

ツールバーの機能
----------------

本プラグインが読み込まれると、ツールバーの領域に以下のツールバーが表示されます。

.. image:: images/toolbar1.png

もしこれが表示されない場合は、メインメニューの「表示」−「ツールバーの表示」から「DevGuideBar」という項目を探してそこのチェックを入れてください。

プラグインの基本的な機能は :doc:`item-operation-sample` と同じで、アイテムツリービュー上で選択しているBodyアイテムのモデルを、タイムバーの時刻に連動して回転させるというものです。

ツールバーでは、まず "Flip" というボタンがあります。これは通常のプッシュ式のボタンで、マウスでクリックして押すことができます。そしてこのボタンを押すと、選択されているモデルが180度回転します。Flipとあるように、向きが反転する感じになります。

次に "Reverse" というボタンがあります。これは時刻の進行に対する回転方向を逆にするボタンです。こちらはトグルボタンになっていて、マウスでクリックすると押し込まれて、ボタンがONの状態になります。この状態のときに回転方向が逆になります。これはアニメーション中の方が分かりやすいので、タイムバーの再生ボタンでアニメーションを実行して、その最中に押して試してみてください。

ツールバーの最後に "Speed ratio" と表記されていて、そこに数値の入力ボックスがあります。こちらは時間進行に対する回転の比率を設定するボックスです。これもアニメーション中が分かりやすいですが、ここの値を小さくすると回転が遅くなり、大きくすると回転が速くなります。ここが1.0のときは :doc:`item-operation-sample` と同じで、時刻の秒数をそのままラジアンに見立てた回転角度となりますが、Speed ratioの設定で秒数に対する角度の比率を変化させられるというわけです。

例によってこれ自体に特に意味はありませんが、ツールバーの基本的な機能をひととおり使用するサンプルとなっています。2つのボタンは :ref:`plugin-dev-toolbar-functions` で紹介したボタンとトグルボタンの使用例で、さらにセパレータとラベルも使用しており、最後の数値入力ボックスは :ref:`plugin-dev-toolbar-use-qt-classes` の例となっています。

ソースコードの解説
------------------

:doc:`item-operation-sample` に対して追加された箇所を中心に解説します。 ::

 #include <cnoid/ToolBar>

ToolBarクラスのヘッダです。ツールバーを作成する場合はこのヘッダをインクルードしておきます。今回のサンプルでは他にTimeBarも利用していて、そのヘッダによってToolBarのヘッダも取り込まれるのですが、ここではツールバー作成の手順としてあえてToolBarヘッダを明示的にインクルードしています。 ::

 #include <cnoid/DoubleSpinBox>

Speed ratioを入力するための数値ボックスの実装にDoubleSpinBoxを利用しています。これはQtのQDoubleSpinBoxを拡張してChoreonoid形式のシグナルを利用できるようにしたものです。QDoubleSpinBoxは倍精度浮動小数点を扱うことが可能なスピンボックスのウィジェットです。 ::

 #include <cmath>

C言語のmathヘッダです。円周率の値が設定されているM_PIマクロを使用するためにこちらをインクルードしています。

以下はプラグインのメンバ変数の追加分です。 ::

 double initialTime;

時刻に対する回転量を決定するための初期時刻を格納する変数です。
回転の設定を変更した際にもスムーズに連続したアニメーションとするために導入しています。

 DoubleSpinBox* speedRatioSpin;

Speed ratio用スピンボックスへのポインタです。作成したスピンボックスを他のメンバ関数から参照するために定義しています。 ::
 
 ToolButton* reverseToggle;

トグルボタンへのポインタです。こちらも作成後に他のメンバ変数からボタンの状態を取得するために定義しています。

次に初期化関数 initialize の実装に入ります。 ::

 RootItem::instance()->sigSelectedItemsChanged().connect(
     [this](const ItemList<>& selectedItems){
         onSelectedItemsChanged(selectedItems);
     });
 
 TimeBar::instance()->sigTimeChanged().connect(
     [this](double time){
         return onTimeChanged(time);
     });

この部分は基本的に :doc:`item-operation-sample` と同じですが、本サンプルではScopedConnectionSetによるシグナルの接続管理を省略しています。
以前のサンプルでは接続管理の例として含めていましたが、このケースでは接続はアプリケーション終了時まで維持するものなので、接続管理は必ずしも必要ありません。
今後のサンプルコードでも必要ない部分は省くようにします。 ::

 auto toolBar = new ToolBar("DevGuideBar");

ツールバーのオブジェクトを生成しています。ToolBarのコンストラクタには名前を与える必要があり、ここでは "DevGuideBar" としています。このサンプルでは :doc:`toolbar` で紹介した2つの作成方法のうち、「ToolBarクラスのインスタンスを生成し必要なインタフェースを外部から追加する」方法でコーディングしています。 ::

 ToolButton* flipButton = toolBar->addButton("Flip");
 flipButton->sigClicked().connect(
     [this](){ flipBodyItems(); });

ツールバーにFlipボタンを追加し、そのsigClickedシグナルにメンバ関数flipBodyItemsを接続しています。
これにより、Flipボタンが押されるとflipBodyItemsが実行されるようになります。 ::

 reverseToggle = toolBar->addToggleButton("Reverse");
 reverseToggle->sigToggled().connect(
     [this](bool on){ updateInitialRotations(); });

ツールバーにReverseトグルボタンを追加し、そのsigToggledシグナルにメンバ関数updateInitialRotationsを接続しています。
これにより、Reverseトグルのオン／オフ状態が変わると、updateInitialRotationsが実行され、現在の状態を初期状態として設定し直すようになります。 ::
     
 toolBar->addSeparator();
  
ツールバーにセパレータを追加しています。
トグルボタンのラベルと次に追加するラベルがどちらもテキストでつながって見えてしまうので、セパレータを入れて境目を分かりやすくしています。
これはあくまで見た目の好みの問題です。 ::

 toolBar->addLabel("Speed ratio");

ツールバーに "Speed ratio" というラベルを追加しています。次に追加するスピンボックスだけ追加すると何のためのボックス分かりにくいので、ラベルを入れるようにしています。 ::

 speedRatioSpin = new DoubleSpinBox;

数値入力用のスピンボックスとして、DoubleSpinBoxを生成しています。 ::
   
 speedRatioSpin->setValue(1.0);
 
スピンボックスの初期値を1.0に設定しています。こちらはDoubleSpinBoxが継承しているQtのクラスであるQDoubleSpinBoxの関数です。 ::

 speedRatioSpin->sigValueChanged().connect(
      [this](double value){ updateInitialRotations(); });

スピンボックスの値が変わると送出されるsigValueChangedシグナルの接続をしています。
sigValueChangedはQDoubleSpinBoxのvalueChangedシグナルをChoreonoidのシグナルとして利用できるようにしたものです。
こちらも値の変化時ににupdateInitialRotationsが実行されるようにしていて、初期状態を更新するようにしています。 ::

 toolBar->addWidget(speedRatioSpin);

作成したスピンボックスをツールバーに追加しています。 ::

 toolBar->setVisibleByDefault();
  
このツールバーがデフォルトで表示されるようにしています。
こちらは指定しなければデフォルトでは表示されないようになっています。
ツールバーのレイアウトはプロジェクトの設定で決めることが多いので、通常は表示しない設定にしておきます。
こちらはサンプルなので、作成したツールバーが最初から表示されていて動作確認をしやすい方が望ましいので、あえてこの設定にしています。 ::

 addToolBar(toolBar);
 
作成したツールバーを登録しています。この処理を行うことでツールバーを利用できるようになります。 ::

 initialTime = 0.0;

回転量決定のための初期時刻を0に初期化しています。

以下は回転量決定のための初期状態を更新する関数です。 ::

 void updateInitialRotations()
 {
     initialTime = TimeBar::instance()->time();
     initialRotations.clear();
     for(auto& bodyItem : bodyItems){
         initialRotations.push_back(bodyItem->body()->rootLink()->rotation());
     }
 }

まずinitialTimeの値を現在時刻で更新します。回転量はこの時刻を起点とする時刻で計算します。
残りの部分は :doc:`item-operation-sample` の onSelectedItemsChanged で処理していたものです。
こちらの関数にまとめることで、設定変更時にも初期状態を更新できるようにしています。

以下はFlipボタンを押した時に呼ばれる関数です。 ::

 void flipBodyItems()
 {
     for(auto& bodyItem : bodyItems){
         Link* rootLink = bodyItem->body()->rootLink();
         Matrix3 R = AngleAxis(M_PI, Vector3::UnitZ()) * rootLink->rotation();
         rootLink->setRotation(R);
         bodyItem->notifyKinematicStateChange(true);
     }
     updateInitialRotations();
 }

選択されているBodyアイテムについて、ルートリンクの姿勢を180度回転させています。
最後にupdateInitialRotationsも実行して、この処理の後も時刻変化による回転が連続するようにしています。

onTimeChanged関数では時刻の変化に応じて回転量を決定しモデルの状態を更新しています。
この基本的な処理は :doc:`item-operation-sample` と同じですが、回転量を決定する式を修正しています。

まず ::

 double angle = speedRatioSpin->value() * (time - initialTime);

では、時刻をinitialTimeからの相対時刻とした上で、それにSpeed ratioスピンボックスの値をかけて回転角度を決定しています。
これにより設定変更時点からのスムーズな変化を実現しつつ、Speed ratioの反映も行っています。 ::

 if(reverseToggle->isChecked()){
     angle = -angle;
 }

Reverseトグルボタンの状態を確認し、オンのときは回転角度をマイナス方向に反転します。
これによりReverseトグルボタンを機能させています。

