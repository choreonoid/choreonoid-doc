==============
Pluginクラス
==============

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

PluginクラスはChoreonoidのプラグインを実装する際の基底クラスです。Baseモジュールに含まれており、ヘッダ ``cnoid/Plugin`` で定義されています。Choreonoidのプラグインは、必ずこのクラスを継承した独自クラスとして実装します。

プラグインクラスは、 :doc:`basics` でも述べたように、プラグインの実装の起点となるものです。通常のC言語のプログラムにおけるmain関数のような役割を果たすもので、プラグイン名の登録、他プラグインへの依存関係の指定、初期化・終了処理の記述といった、プラグイン全体に関わる基本的な事柄をここに記述します。

なお、PluginクラスはExtensionManagerクラスを継承しています。ExtensionManagerは、Choreonoid本体に対して機能の登録（拡張）を行うための基本的なインタフェースを提供するクラスで、ItemManager、ViewManager、MenuManagerなど、各種マネージャクラスへのアクセス手段を備えています。プラグインクラスはExtensionManagerの派生クラスでもあるため、これらの機能をそのまま利用することができます。プラグインクラスのinitialize関数の中で、自身（this）を介してExtensionManager由来の関数を呼び出して機能登録を行うのが、典型的な使い方となります。

クラスの定義
------------

プラグインクラスを定義する基本的な形は以下のとおりです。 ::

 #include <cnoid/Plugin>

 class FooPlugin : public cnoid::Plugin
 {
 public:
     FooPlugin();
     virtual bool initialize() override;
     virtual bool finalize() override;
     virtual const char* description() const override;
 };

 CNOID_IMPLEMENT_PLUGIN_ENTRY(FooPlugin)

クラス名は最後が "Plugin" で終わるようにします。コンストラクタでは基底クラスPluginのコンストラクタにプラグイン名を与え、必要に応じて依存関係などの基本情報を設定します。仮想関数のうち、initialize関数は通常必ずオーバーライドし、finalize関数やdescription関数は必要に応じてオーバーライドします。

最後に記述している ``CNOID_IMPLEMENT_PLUGIN_ENTRY`` マクロは、共有ライブラリからプラグインインスタンスを取得するためのエントリ関数を定義するためのもので、プラグインのソースファイルには必ず1つだけ記述する必要があります。

コンストラクタ
--------------

Pluginクラスはデフォルトコンストラクタを持たず、以下のコンストラクタのみが定義されています。 ::

 Plugin(const std::string& name);

引数のnameはプラグインの名前で、独自プラグインのコンストラクタからこのコンストラクタに対して必ず文字列を与える必要があります。例えば ::

 FooPlugin::FooPlugin()
     : Plugin("Foo")
 {

 }

のように、初期化子リストを用いてプラグイン名を渡します。

ここで指定するプラグイン名は、クラス名から末尾の "Plugin" を取り除いた名称とします。FooPluginクラスであれば "Foo" となります。プラグイン名は他のプラグインから依存先として参照される際の識別子にもなりますので、他のプラグインと重複しない名前を選ぶようにしてください。

コンストラクタの中では、プラグイン名の設定に加えて、後述する依存関係の指定や、プラグインの基本属性の設定などを行います。一方、初期化処理そのものはinitialize関数で行うのが原則です。コンストラクタの段階では、Choreonoid本体の初期化が完了しているとは限らないため、機能登録などの本格的な処理はinitialize関数で行うようにしてください。

オーバーライド可能な仮想関数
----------------------------

Pluginクラスには、プラグインの動作をカスタマイズするためにオーバーライド可能な仮想関数がいくつか定義されています。

initialize関数
~~~~~~~~~~~~~~

::

 virtual bool initialize();

プラグインの初期化処理を記述します。プラグインクラスを実装する際にはほぼ必ずオーバーライドすることになります。

この関数は、プラグインが読み込まれてコンストラクタが実行された後、プラグイン間の依存関係を考慮した順番で呼び出されます。プラグインの提供する各種機能の登録は、通常この関数の中で行います。

戻り値はプラグインの初期化に成功したかどうかを表します。初期化処理が成功し、プラグインの機能を利用可能な状態になった場合はtrueを返します。何らかの理由で初期化に失敗した場合はfalseを返します。falseを返した場合、Choreonoid本体はそのプラグインを利用可能な状態とはみなさず、メッセージビューにもその旨が表示されます。

デフォルトの実装は何もせずにtrueを返します。

finalize関数
~~~~~~~~~~~~

::

 virtual bool finalize();

プラグインの終了処理を記述します。Choreonoidの終了時に呼び出されます。

プラグインで使用しているオブジェクトの破棄や、システムリソースの解放処理など、終了時に明示的に行うべき処理がある場合に、この関数をオーバーライドして記述します。戻り値は終了処理に成功したかどうかを表します。

通常、ExtensionManagerのmanage関数で管理させたオブジェクトや、Choreonoid本体に登録した各種要素は、Choreonoid本体側で適切に解放されます。従って多くのプラグインではfinalize関数をオーバーライドする必要はありません。

デフォルトの実装は何もせずにtrueを返します。

description関数
~~~~~~~~~~~~~~~

::

 virtual const char* description() const;

プラグインの説明文を返します。返された文字列は、Choreonoidのメインメニュー「ヘルプ」−「プラグインについて」から表示されるダイアログで参照されます。

開発したプラグインを外部に公開する場合は、この関数をオーバーライドして、プラグインの概要、著作権表示、ライセンス条件などをここに記述しておくとよいでしょう。

なお、Pluginクラスにはオープンソースライセンスの定型文を返す以下の静的関数が用意されており、description関数の実装の中で利用することができます。

* ``static const char* MITLicenseText()`` — MITライセンスの定型文を返す
* ``static const char* LGPLtext()`` — LGPLライセンスの定型文を返す

デフォルトの実装は空文字列を返します。

customizeApplication関数
~~~~~~~~~~~~~~~~~~~~~~~~

::

 virtual bool customizeApplication(AppCustomizationUtil& app);

Choreonoidアプリケーションそのもののカスタマイズを行うための関数です。通常のプラグインではオーバーライドする必要はありません。

この関数は、initialize関数よりもさらに早い、アプリケーションの基本的な初期化段階で呼び出されます。デフォルトの実装はfalseを返します。

依存関係の指定
--------------

プラグインが他のプラグインに依存している場合は、その旨をChoreonoid本体に伝えておく必要があります。これにより、依存先のプラグインが先に初期化されることが保証され、また依存先のプラグインが存在しない場合には適切なエラーメッセージが表示されるようになります。

require関数
~~~~~~~~~~~

::

 void require(const std::string& pluginName);

このプラグインが依存するプラグインの名前を指定します。コンストラクタの中で呼び出します。例えばBodyプラグインに依存する場合は ::

 FooPlugin::FooPlugin()
     : Plugin("Foo")
 {
     require("Body");
 }

のようにします。複数のプラグインに依存する場合は、依存するプラグインごとにrequire関数を呼び出します。

precede関数
~~~~~~~~~~~

::

 void precede(const std::string& pluginName);

このプラグインを、指定したプラグインよりも先に初期化することを指定します。require関数とは異なり、こちらは指定したプラグインの存在を必須とはしません。指定したプラグインが存在する場合に限り、その初期化前に自身が初期化されることを保証します。

addOldName関数
~~~~~~~~~~~~~~

::

 void addOldName(const std::string& name);

プラグイン名を変更した場合に、過去に作成されたプロジェクトファイルとの互換性を維持するために、以前のプラグイン名を登録するための関数です。コンストラクタの中で呼び出します。複数の旧名がある場合は、それぞれについてaddOldName関数を呼び出します。

ExtensionManagerから継承される機能
----------------------------------

前述のとおり、PluginクラスはExtensionManagerクラスを継承しています。ExtensionManagerには、Choreonoid本体に対して各種機能を登録するためのインタフェースが備わっており、プラグインクラスからもそのまま利用することができます。代表的なものを以下に挙げます。

* ``ItemManager& itemManager()`` — :doc:`new-item-type` で解説する、独自アイテム型を登録するためのItemManagerを取得する
* ``ViewManager& viewManager()`` — :doc:`create-view` で解説する、独自ビュー型を登録するためのViewManagerを取得する
* ``MenuManager& menuManager()`` — メインメニューに項目を追加するためのMenuManagerを取得する
* ``void addToolBar(ToolBar* toolBar)`` — :doc:`toolbar` で解説する、ツールバーをChoreonoid本体に追加する
* ``template<class PointerType> PointerType manage(PointerType pointer)`` — オブジェクトの寿命をChoreonoid本体に管理させる

これらの関数は、通常はinitialize関数の中から呼び出します。プラグインクラスはExtensionManagerでもあるため、 ``this`` を介して、あるいは関数を直接呼び出すかたちで利用することができます。

プラグインエントリの定義
------------------------

::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(PluginClassName)

プラグインを実装したソースファイルには、このマクロを必ず1回だけ記述する必要があります。引数にはプラグインクラス名を指定します。

このマクロを記述しておかないと、ビルドされた共有ライブラリがChoreonoidからプラグインとして認識されません。また、ひとつの共有ライブラリには1つのプラグインクラスのみを実装でき、このマクロを2回以上記述することはできません。

参考
----

* `PluginクラスのAPIリファレンス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Plugin.html>`_
* `ExtensionManagerクラスのAPIリファレンス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ExtensionManager.html>`_
