================
ツールバーの作成
================

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

プラグインで追加できるGUI要素のひとつとして「ツールバー」があります。
これは比較的用意に作成でき、実装した機能を使用するためのインタフェースとして活用できます。
ここでは作成方法の概要を解説します。

.. _plugin-dev-toolbar-class:

ToolBarクラス
-------------

ツールバーはChoreonoid SDKの `ToolBarクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ToolBar.html>`_ を用いて作成します。ToolBarはツールバーの土台となるクラスです。これはQtのQWidgetを継承しており、OSのウィンドウシステムにおける最小単位となる領域を、ツールバー用に確保するものです。この上にボタン等の各種GUI部品を並べていくことができます。ただしツールバーに並べるGUI部品の種類はある程度決まっており、また並べ方についてはツールバーのスタイルに従う必要があります。ToolBarクラスはこの点を考慮した上で、ツールバーの構築を効率的に行うための関数を備えています。

実際にツールバーを作成するにあたっては、以下の2つの方法があります。

A. ToolBarクラスのインスタンスを生成し必要なインタフェースを外部から追加する
B. ToolBarクラスを継承した独自のクラスを定義し、必要なインタフェースをクラス内部で管理する

Aの方が簡単ですので、この方法で実装しやすい場合はこちらを採用すればよいかと思います。シンプルなツールバーはこちらの方がマッチしているかと思います。

この場合、まず ::

 ToolBar* toolBar = new ToolBar("MyToolBar");

などとして生成します。このようにツールバーは通常new演算子を用いて動的に生成します。またコンストラクタにはツールバーの名前を与える必要があります。ツールバーの表示を切り替えるメニューや、プロジェクトファイルに保存されるツールバーの情報などにおいて、ここで指定した名前が使用されます。

生成したツールバーオブジェクトに対して、 ::

 toolBar->addButton( ... );
 ...

などとしてツールバーを構築していきます。これはプラグインのinitialize関数などで処理します。作成したツールバーは後ほど登録処理を行います。

Bについては、ツールバーのインタフェースが複雑になる場合や、ツールバー内に各種情報や処理を実装する場合に適しています。この場合は ::

 class MyToolBar : public ToolBar
 {
 public:
     MyToolBar() : ToolBar("MyToolBar")
     {
         addButton( ... );
         ...
     }
 };

などとして構築します。こちらの場合も作成したクラスのオブジェクトを生成して、登録処理を行います。

いずれにしても、ツールバーの作成は以下の作業要素からなるかと思います。

1. ボタン等のインタフェースを構築する
2. 必要に応じて各種状態の管理や機能実現のための処理を実装する
3. インタフェースと各種処理を連携させる

ToolBarクラスのAPIは主に1を支援するために用意されています。また1についてはChoreonoidがGUIの実装に利用しているQtライブラリも活用することになります。

2と3については、特にツールバーに限定される要素ではなく、実現する機能次第です。
ただし3については一般的にシグナルを用いて連携を行うことが多いです。

本節では主に1の概要と3の実装に利用するシグナルについて解説します。1〜3に関する具体的な例は、次の節でサンプルを紹介します。

.. _plugin-dev-toolbar-functions:

インタフェース構築用の関数
--------------------------

`ToolBarクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ToolBar.html>`_ はツールバーのインタフェースを構築するための基本的な関数を備えています。ここではそのような関数の中から主要なものをいくつか紹介します。

まずツールバーに実装することの多いGUI部品としてボタンがあります。ToolBarクラスの以下の関数でボタンを追加できます。

* **ToolButton* addButton(const QString& text, const QString& tooltip = QString())**

  ツールバーにボタンを追加します。textでボタンのテキストを設定します。またtooltipを指定すると、マウスカーソルをボタンの上で静止させたときににこのテキストがツールチップとして表示されます。

* **ToolButton* addButton(const QIcon& icon, const QString& tooltip = QString())**

  上記と同様にボタンを追加しますが、こちらはテキストではなくアイコンによるボタンとなります。iconにアイコン画像を指定します。

これらのボタンはごく普通のボタンで、押したら何か処理をするというためのものです。これをプッシュボタンとも言います。

ボタンは実際にはQtのオブジェクトとして生成されることもあり、上記関数に与える引数は現状ではQtのクラスになっています。文字列はstd::stringではなくQString型になりますし、アイコン画像はQIcon型のオブジェクトで指定します。

Qtの各クラスの詳細はQtのマニュアルを参照ください。QStringの利用方法について簡単に紹介すると、まず文字列リテラルはそのままこの型に渡すことができます。従って ::

 toolBar->addButton("Button");

として設定することが可能です。std::stringの文字列を設定したい場合は、c_str関数でC言語の文字列形式（const char*)にして設定します。 ::

 std::string text("Button");
 toolBar->addButton(text.c_str());

アイコン画像を設定するQIconについては、画像ファイルから生成することができます。例えばアイコン用に"icon.svg"というSVGファイルを用意して ::

 QIcon icon("icon.svg");

とすればその画像のQIconオブジェクトを生成できます。（もちろん実際のファイルパスなどは適切に指定するようにしてください。）

.. note:: Qtではリソースシステムというものがあり、任意のファイルを実行ファイルや共有ライブラリのバイナリに埋め込むことができます。埋め込んだファイルはプログラムから通常のファイルと同様に読み込むことができます。アイコン画像などは実際にはこのリソースシステムを利用してバイナリに埋め込んでおくのがよいでしょう。詳細はQtのマニュアルを参照ください。

ツールバーでは他のタイプのボタンも利用可能です。他によく使われるのはトグルボタンです。これは押し込んだ状態とそうでない状態があるというもので、オン／オフを切り替えるスイッチのように使うものです。これは以下の関数で追加できます。各引数の意味はaddButtonと同じです。

* **ToolButton* addToggleButton(const QString& text, const QString& tooltip = QString())**
* **ToolButton* addToggleButton(const QIcon& icon, const QString& tooltip = QString())**

さらにラジオボタンも利用できます。ラジオボタンはいくつかの選択肢の中から選ぶためのインタフェースで、例えば3個のラジオボタンを用意して、その中のひとつだけオンにできる（押し込める）ようにします。どれかひとつがオンになると、残りのボタンはオフになるので、これによってユーザは選択を伝えることができます。これは以下の関数で追加できます。

* **ToolButton* addRadioButton(const QString& text, const QString& tooltip = QString())**
* **ToolButton* addRadioButton(const QIcon& icon, const QString& tooltip=QString())**
  
なお、ラジオボタンは選択肢のグループごとに作成する必要があります。もしグループが2つ以上ある場合は、新しいグループに属するボタンを追加する前に、以下の関数を実行しておきます。

* **void requestNewRadioGroup()**

他にボタン以外の要素として以下を追加することもできます。

* **QLabel* addLabel(const QString& text)**

  テキストラベルを追加します。指定したテキストがツールバー上に表示されます。

* **QWidget* addSeparator()**

  セパレータを追加します。この前後で追加されたGUI要素を仕切るための縦棒が表示されます。

* **void addSpacing(int spacing = -1)**

  空白を追加します。この前後で追加されたGUI要素の間を設定した量だけ話します。デフォルト引数を用いると、標準幅の空白となります。

まずはこのような関数を用いてツールバーのインタフェースを構築していきます。

ボタンオブジェクト
------------------

ツールバーにボタンを追加してもそれだけでは意味がありません。ボタンが押された際に関連する処理を実行したり、ボタンの状態に従って処理内容を変えるといったことを行う必要があります。そのためにはまず各ボタンのオブジェクトを取得し、そこから状態を取得したり、シグナル接続を行ったりします。

ボタンのオブジェクトは上記のボタン追加関数の戻り値として得ることができます。
これは `ToolButtonクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ToolButton.html>`_ のオブジェクトへのポインタとなります。ToolButtonクラスはQtのQToolButtonクラスを継承したもので、QToolButtonのいくつかのシグナルをChoreonoid形式のシグナルで利用できるようにしたものです。

例えば ::

 ToolButton* button = toolBar->addButton("Button");

とすることで、追加したボタンに対応するToolButtonオブジェクトを取得できます。このToolButtonクラスについて、以下のシグナルが利用できます。

* **SignalProxy<void()> sigClicked()**

  ボタンが押されたときに送出されます。
 
* **SignalProxy<void(bool on)> sigToggled()**

  トグルボタンのオン／オフの状態が変化したときに送出されます。変化後の状態が引数onで与えられます。

.. note:: これらのシグナルには対応するQtのシグナルがあります。対応するシグナルはQToolButtonの親クラスであるQAbstractButtonで定義されているもので、それぞれclickedとtoggledというシグナルになります。それらQtのシグナルをそのまま用いてもOKですが、Choreonoid形式のシグナルを用いることで、コード全体の一貫性を高めることができます。

例えば先程のコードの変数buttoに対して ::

  button->sigClicked().connect([](){ onButtonClicked(); });

とすれば、このボタンを押した時にonButtonClicked関数が呼ばれます。トグルボタンやラジオボタンの場合は ::

  button->sigToggled().connect([](bool on){ onButtonToggled(on); });

とすれば、ボタンの状態が変化したときにonButtonToggled(on)関数が呼ばれます。

このようにして接続した関数でボタンの操作と関連する処理を実行することになります。

また、トグルボタンやラジオボタンの場合は、以下の関数でボタンのオン／オフ状態を取得できます。

* **bool isChecked() const**

取得したボタンオブジェクトへのポインタは、ボタンが存在する間は有効です。ボタンやツールバーを削除するような処理を行わなければ、通常は生成したツールバーとそこに追加したボタンはアプリケーションが終了するまで存在します。従って、取得したポインタを保持しておき、後ほど関連する操作が実行される際に参照することが可能です。

.. note:: ToolButtonが継承しているQtのクラスにはisChedked以外にも多数の関数が定義されており、それらも全て利用することができます。ToolButtonが継承しているQToolButtonはさらにQAbstractButtonを継承していて、その先もQWidget、QObjectとQtのクラス階層が続いており、それらの全てのクラスに含まれるの関数が利用可能です。実際に使用可能な関数はQtのマニュアルでご確認ください。

.. _plugin-dev-toolbar-use-qt-classes:
 
Qtクラスの活用
--------------

ToolBarクラスの以下の関数を用いることで、Qtの任意のウィジェットをツールバーに追加できます。

* **void addWidget(QWidget* widget)**

ウィジェットというのは個々のGUI部品を表現するもので、GUIライブラリで一般的に使用されている用語です。Qtの場合はQWidgetというクラスがウィジェットの基底となるクラスで、これはウィンドウシステムの管理する描画領域の最小単位に対応します。そこを基盤として派生した様々なウィジェットが利用可能となっており、上で紹介したQToolButtonもそのようなウィジェットのひとつです。

そのようにQtで定義されているウィジェットは基本的にどれも上記の関数でツールバーに追加できます。例えばChoreonoid本体のタイムバーは、時刻を数値で表示・入力するスピンボックスや、時刻を視覚的に表示・変更するためのスライダーを備えており、それらはそれぞれQtのQDoubleSpinBoxとQSliderというウィジェットで実現しています。ただしツールバーは横方向に並べるバーとして設計されているので、その形態に適合しないウィジェットについては、無理に追加しない方がよいかと思います。
  
Choreonoid本体のソースコードでTimeBarをはじめとして様々なツールバーの実装をみることができますので、独自のツールバーを作成するにあたっては、それらも参考にしていただければと思います。

.. note:: ChoreonoidはGUIの実装にQtを利用しているので、プラグインでGUIに関わる部分も実装する場合は、Qtの知識が欠かせません。逆に言えばChoreonoidのGUIはほぼQtそのものなので、Qtを理解していれば大抵のGUIは実現できるかと思います。Choreonoidではツールバーの他にビューについてもプラグインで追加することが可能で、それについては本ガイドで後ほど解説しますが、そこでもビューを実装する一定の決まりごとに従えば、あとはいかにQtのクラスを用いて望みのGUIを実現するかという問題になります。

.. 独自ビューの作成のところで、QtをChoreonoidで拡張したクラスについて紹介し、一覧などものせる。主な目的はChoreonoid形式のシグナルをそのまま使えるようにすること。この節ではまだ言わなくてよいと思う。

ツールバーの登録
----------------

ツールバーの生成は通常独自プラグインクラスのinitialize関数で行います。そしてツールバーを生成したら、addToolBar関数でシステムに登録をします。この処理は以下のようになります。 ::

 class ToolBarPlugin : public Plugin
 {
 public:
     ...
      
     virtual bool initialize() override
     {
         auto toolBar = new ToolBar("MyToolBar");
 	 auto button = toolBar->addButton("Button");
 
         ...
	 
         // ツールバーを登録する
         addToolBar(toolBar);
 
         return true;
     }
 }

ここで使用しているaddToolBar関数は、 `ExtensionManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_ で定義されている関数です。 `Pluginクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Plugin.html>`_ はこのExtensionManagerを継承しているので、この関数を使用することができます。

登録したツールバーはツールバー領域に表示されます。もしデフォルトで表示されないようにしたい場合は ::

  toolBar->setVisibleByDefault(false);

としておきます。この場合でも、登録はするようにしてください。登録がされているツールバーについては、メインメニューの「表示」−「ツールバーの表示」から表示のオン／オフを切り替えられます。デフォルトでは表示していなくても、ここから表示するように切り替えることが可能です。
