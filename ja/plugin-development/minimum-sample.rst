================================
最小限のプラグインサンプル (S01)
================================

.. contents:: 目次
   :local:

概要
----

本節では実際にプラグインを作成することで、:doc:`basics` で説明した事柄が具体的にどのような手順やコードになるかを示します。
作成するプラグインはこの目的を達成するにあたって必要最小限のものとします。

プラグインの名前は「DevGuideプラグイン」とします。開発ガイドでは今後この名前を維持したままサンプルプラグインを拡張していきます。

ビルド形態の選択
----------------

:ref:`ビルド形態 <plugin-dev-basics-build-forms>` については、まず「Choreonoid本体のビルド環境でビルドする」形態で進めます。従って、まずChoreonoid本体をソースコードからビルドしておいてください。そのビルド環境をプラグインのビルドにも用いることとします。

「Choreonoid本体とは独立してビルドする」形態についても、この節の最後で紹介します。

ソースディレクトリの作成
------------------------

.. highlight:: sh

Choreonoid本体のビルド環境でビルドするので、本体のextディレクトリにプラグイン用のソースディレクトリを作成します。ディレクトリ名は プラグイン名にあわせて"DevGuidePlugin" としましょう。コマンドラインからこれを行う場合は ::

 cd Choreonoid本体のソースディレクトリ/ext
 mkdir DevGuidePlugin

などとします。
もちろんファイルエクスプローラ等のGUIツールから同じことを行ってもOKです。

ソースファイルの作成
--------------------

.. highlight:: cpp

上記のディレクトリに以下のソースファイルを作成します。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
 
     }
 
     virtual bool initialize() override
     {
         MessageView::instance()->putln("Hello World!");
	 return true;
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

C++のプログラムなので、拡張子は通常.cppとします。"DevGuidePlugin.cpp" というファイル名でソースディレクトリに格納するようにしてください。

ソースファイルの解説
--------------------

まずこのソースファイルの内容について解説します。

ヘッダのインクルード
~~~~~~~~~~~~~~~~~~~~

本サンプルでは以下の2つのヘッダをインクルードしています。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>

これらのヘッダは、Choreonoid SDKに含まれるものです。Choreonoid SDKのヘッダは基本的に "cnoid" というサブディレクトリ内に格納されており、このようにcnoidをプレフィックスとして付与して指定します。またC++標準ライブラリと同様に、ヘッダは拡張子を付けずに指定するようになっています。

ここでインクルードしているヘッダの概要を以下に示します。

* **cnoid/Plugin**

  Pluginクラスが定義されたヘッダです。独自プラグインを定義するソースファイルでは必ずこのヘッダをインクルードします。これによってPluginクラスを継承した独自プラグインクラスを定義できるようにします。

* **cnoid/MessageView**

  Choreonoid本体のメッセージビューに対応するヘッダで、MessageViewクラスが定義されています。プラグインでメッセージビューを利用する場合はこのヘッダをインクルードします。


名前空間
~~~~~~~~

Choreonoid SDKに含まれるクラスや関数は全て "cnoid" という名前空間の中で定義されています。以下の宣言でこの名前空間を取り込んでいます。 ::

 using namespace cnoid;

この宣言により、本サンプルで使用している以下のクラスについて、名前空間のcnoidの指定を省略できるようにしています。

* cnoid::Plugin
* cnoid::MessageView

もちろん名前空間は名前の衝突を避けるためのものであり、無闇にusing指令を行うのは良くありません。原則として、ヘッダファイルにおいてはusing指令の利用は避け、名前空間も含めた全ての記述を行うのがよいでしょう。一方で、実装ファイル(.cpp)においては、名前の衝突が問題にならなければ、上のような記述を行うことでコードを簡潔にすることが出来ます。

本ガイドにおいて、サンプルのソースファイルは記述を簡潔にするため基本的にこの宣言をつけて実装するものとします。


プラグインクラスの定義
~~~~~~~~~~~~~~~~~~~~~~

独自プラグインであるDevGuideプラグインは以下のコードで定義しています。 ::

 class DevGuidePlugin : public Plugin
 { 
     ...
 };


Choreonoidのプラグインは、このようにPluginクラスを継承したクラスとして定義します。クラス名は最後が "Plugin" で終わるようにしなければなりません。この原則に従っていればクラス名の残りの部分は自由に決めることが可能です。ただしChoreonoid本体に付属するプラグインや、同じ環境でインストールされる追加プラグインなど、他のプラグインと名前が重ならないようにする必要もあります。

コンストラクタ
~~~~~~~~~~~~~~

プラグインクラスではまずコンストラクタを定義する必要があります。以下の部分でこれを行っています。 ::

 DevGuidePlugin() : Plugin("DevGuide")
 {
 
 }

基底となるPluginクラスはデフォルトコンストラクタを持たず、プラグイン名を引数として受け取るコンストラクタのみが定義されています。従ってこのように初期化子リストを用いてPluginクラスにプラグイン名を与える必要があります。これによりプラグインの名前が必ず設定されることになります。

なおここで設定する名前はプラグインクラスから "Plugin" の部分を省いた名前とします。このためここでは "DevGuidePlugin" ではなく "DevGuide" という文字列を設定しています。

これ以外にコンストラクタ内で記述する処理として、プラグインの依存関係の記述があります。プラグインが他のプラグインに依存している場合は、そのプラグインをChoreonoidに伝えておく必要があります。これはrequireという関数で行うことが可能です。例えばこのプラグインがBodyプラグインに依存する場合は以下のようにします。 ::

 DevGuidePlugin() : Plugin("DevGuide")
 {
     require("Body");
 }

今回はまだ他のプラグインへの依存はありませんので、コンストラクタ内では何も記述していません。他のプラグインに依存するプラグインについては、本ガイドで後ほど紹介します。

initialize関数
~~~~~~~~~~~~~~

プラグインの初期化処理は通常initialize関数に記述します。 ::

 virtual bool initialize() override
 {
     ...
 }

initialize関数は基底となるPluginクラスでvirtual関数として定義されており、これをオーバーライドすることで各プラグインごとの初期化処理を実行するようになっています。

各プラグインのinitialize関数は、プラグインの読み込まれてコンストラクタが実行された後に、プラグイン間の依存関係を考慮した順番で実行されます。初期化処理に成功し、プラグインの機能を利用する準備ができた場合は、この関数がtrueを返すようにします。これによってChoreonoid本体もそのプラグインが使える状態になったことを認識します。もし初期化処理に失敗した場合は、falseを返すようにします。

本プラグインはこの関数に記述した以下のコードでメッセージビューにテキストを出力をしています。 ::

 MessageView::instance()->putln("Hello World!");

ここでは `MessageViewクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1MessageView.html>`_ のinstance関数でメッセージビューのインスタンス（ポインタ）を取得し、それに対してputln関数で改行付きのテキスト出力をしています。これにより、プラグインが初期化されるタイミングでメッセージビューに "Hello World!" と表示されます。この処理自体に特に意味はありませんが、プラグインに記述した処理を実行する第一歩としてこのようにしています。

この例のように、プラグインでは、Choreonoid本体が保有するオブジェクトを取得し、それを利用して様々な処理を行うことができます。Choreonoidはメッセージビュー以外にも様々な機能を有しており、それらを活用することでプラグインで提供したい機能を実現していくことになります。Chorenoidを既にお使いの方であれば、Choreonoidがどのような機能を有しているかは把握されているかと思います。それらは多くの場合Chorenoid SDKにおいて対応するヘッダやライブラリがあり、それらをインクルードしたりリンクすることでプラグインからも利用できるようになります。

なお、Pluginクラスではinitialize関数以外に以下のvirtual関数が定義されています。これらの関数もオーバーライドを前提としたもので、プラグインの実装に利用することができます。

* **virtual bool finalize()**

  プラグインの終了処理を記述します。Choreonoidの終了時に、プラグインで使用しているオブジェクトの破棄や、システムリソースの解放処理等が必要な場合は、この関数内に記述します。

* **virtual const char* description()** 

  プラグインの説明文を返す返す関数です。この関数をオーバーライドすることで、独自プラグインの解説文を設定できます。設定した解説文はChoreonoidメインメニューの「ヘルプ」−「プラグインについて」から確認することができます。開発したプラグインを外部に公開する場合は、、ここにプラグインの概要と、著作権、およびライセンス条件を記しておきます。

プラグインエントリの定義
~~~~~~~~~~~~~~~~~~~~~~~~

最後に以下の記述をしています。 ::

 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

ここではPluginヘッダで定義されている "CNOID_IMPLEMENT_PLUGIN_ENTRY" というマクロを使用しています。このマクロにプラグインのクラス名を記述すると、プラグインのDLLからプラグインインスタンスを取得するための関数が定義されます。この記述をしておかないと、作成したDLLがプラグインとして認識されませんので、忘れないようにしてください。

なお、各プラグインは、ひとつのプラグインを実装したひとつのDLLとして作成する必要があります。ひとつのDLLに複数のプラグインを実装することは出来ません（上記のマクロを２つ以上記述することは出来ません）ので、ご注意ください。

CMakeによるビルド手順の記述
---------------------------

.. highlight:: cmake


次にやるべきことは、CMakeでビルドするためのファイルを作成することです。CMakeでは基本的にCMakeLists.txtというファイルにビルド手順を記述します。以下の内容のファイルをCMakeLists.txtというファイル名でプラグインのソースディレクトリに作成すればOKです。

CMakeLists.txtには以下の一行を記述します。 ::

 choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

choreonoid_add_pluginはChoreonoid本体のCMakeLists.txtで定義されているコマンドで、プラグインのターゲットを設定するものです。このコマンドを用いて ::

 choreonoid_add_plugin(ターゲット名 ソースファイル)

と記述することで、指定したソースファイルからプラグインのバイナリをビルドすることができます。ここでソースファイルは複数記述することが可能です。

プラグイン名を設定しています。プラグインの名前は、このように "Cnoid" で始め、"Plugin" で終わるようにします。ここではこの名前をtargetという変数に設定し、以下に続くコマンドで使用できるようにしています。必ずしも変数に設定する必要はありませんが、このようにすることで、プラグイン名の設定を一元化しています。 ::


このコマンドはCMakeの組み込みコマンドであるadd_libraryをカスタマイズしたものです。プラグインは共有ライブラリ（ダイナミックリンクライブラリ）として作成されるものであり、ライブラリの一種です。従ってライブラリを作成するためのadd_libraryコマンドを用いることでも、プラグインをビルドできることになります。

ただしChoreonoidのプラグインとしてビルドして使用できるようにするためには、いろいろな決まりごとがあります。このためadd_libraryのパラメータもいくつか設定する必要がありますし、それ以外にもいくつかのコマンドを用いて設定を行う必要があります。これを全てCMakeの組み込みコマンドだけで記述すると手間もかかりますし、間違いが生じる可能性も高くなります。そこでChoreonoidではchoreonoid_add_pluginというコマンドを定義して、プラグインのビルドを必要最低限の記述で実現できるようにしています。

choreonoid_add_pluginコマンドは基本的に内部で ::

 add_library(ターゲット名 ソースファイル)

に相当する処理を行っていますので、定義したターゲットに対して追加のビルド設定を行うことも可能です。例えばプラグインが依存しているライブラリがあれば以下のように記述します。 ::

 choreonoid_add_plugin(ターゲット名 ソースファイル)
 target_link_libraries(ターゲット名 ライブラリ名)

これはchoreonoid_add_pluginの部分をadd_libraryに置き換えて考えれば、通常のCMakeの使用方法と同じです。必要に応じて他のCMakeのコマンドもこのターゲットに対して使用することができます。

このように今回のサンプルではたった一行でプラグインのビルドを記述できました。Choreonoid本体のビルド環境でビルドすることで、このような簡潔な記述が可能となります。これはChoreonoid本体のCMakeLists.txtで記述されている各種処理や情報を共有できるからで、ここにこのビルド形態を用いる利点があります。

なお、プラグインのCMakeLists.txtでは、冒頭に以下のような記述をしておくとよいです。 ::

 option(BUILD_DEVGUIDE_PLUGIN "Building a sample plugin of the plugin development guide" OFF)
 if(NOT BUILD_DEVELOPMENT)
   return()
 endif()

この記述により、"BUILD_DEVGUIDE_PLUGIN" というオプションがCMakeの設定に付与されます。ここではデフォルトをOFFとしていて、その場合このプラグインのビルドはスキップされます。プラグインをビルドしたい場合は、CMakeの設定でこのオプションをONにします。このようにプラグインをビルドするかどうかを切り替えられるようにしておくと、プラグインの開発や運用がしやすくなるかと思います。特にプラグインのソースコードを外部に公開する際には、このような記述を入れておくと、利用者にとって使い勝手がよくなるかと思います。

.. note:: CMakeLists.txt の記述方法の詳細は `CMakeのマニュアル <http://www.cmake.org/cmake/help/help.html>`_ を参照してください。また、Choreonoidに含まれるライブラリや他のプラグイン、サンプルのCMakeLists.txtを読むことで、おおよその書き方が分かってくるかと思います。またchoreonoid_add_pluginコマンドはChoreonoidソースのcmake/ChoreonoidBasicBuildFunctions.cmake内に記述されていますので、このコマンドの実装内容を知りたい場合はそちらを参照してください。

ビルド・インストール
--------------------

.. highlight:: text

ここまでの手順により、プラグインに関するディレクトリとファイルは以下のようになっているかと思います。 ::

 + Choreonoid本体のソースディレクトリ
   + ext
     + DevGuidePlugin (プラグインのソースディレクトリ)
       - CMakeLists.txt
       - DevGuidePlugin.cpp

この状態で、Choreonoid本体のビルドを行います。するとext以下のDevGuidePluginに存在するCMaekListst.txtも自動で検出され、Choreonoid本体のCMakeの処理に取り込まれます。その結果Choreonoid本体と共にこのプラグインもビルドされるようになります。もちろんビルド処理は更新が必要な個所のみを対象に行われますので、Choreonoid本体のビルドが既に完了していれば、その部分のビルド処理はスキップされます。従ってこの方法の場合でも特にビルド時間が増えるというわけではありません。

.. note:: DevGuidePluginディレクトリのCMakeLists.txtに対して直接CMakeを処理させることは避けてください。そもそも上記のCMakeLists.txtはそのようにして処理できるようには書かれていませんし、これを行うことでChoreonoid本体のソースディレクトリにビルド用の一時ファイルが余計に作成され、本体のビルドも正常に実行できなくなる可能性があります。

ビルドに成功したらインストールを行います。これも本体のインストール操作を実行すればOKです。Linuxの場合はビルド環境のバイナリを直接実行できるので、ビルドするだけでもOKです。Windowsの場合はインストール操作が必要になりますので、忘れずに実行してください。

プラグインをうまくビルド・インストールできていれば、Choreonoidのプラグインディレクトリにプラグインのバイナリファイルが格納されているはずです。Windowsであればインストール先のlib/choreonoid-x.y以下にCnoidDevGuidePlugin.dllというファイルがあるか確認してください。Linuxであればビルドディレクトリもしくはインストール先のlib/choreonoid-x.y以下にlibCnoidDevGuidePlugin.soというファイルが生成されているはずです。

プラグインの読み込みと実行結果の確認
------------------------------------

ではChoreonoidを起動してみましょう。
Choreonoidのプラグインディレクトリに存在するプラグインファイルはChoreonoid起動時に自動的に読み込まれますので、今回作成したDevGuideプラグインも読み込まれるはずです。

プラグインが実際に読み込まれているかどうかはメッセージビューの表示されるテキストで確認できます。まずプラグインファイルが検出されると、 ::

 プラグインファイル"C:\choreonoid\usr\lib\choreonoid-1.8\CnoidDevGuidePlugin.dll"を検出しました．

といったメッセージが表示されます。さらにこのファイルの読み込みに成功して、プラグインの初期化が完了すると、 ::

 DevGuideプラグインが読み込まれました．

というメッセージが表示されます。

今回作成したプラグインでは初期化関数でメッセージビューにテキストを表示するようにしていましたから、実際には ::

 Hello World!
 DevGuideプラグインが読み込まれました．

と表示されているはずです。これにより、初期化関数の内容が実際に実行されていることを確認できます。

今回はこのようにメッセージを表示するだけで特に意味の無いものではありますが、この部分に機能を追加する処理を実装することで、プラグインによる機能拡張が可能となるわけです。本ガイドではこれを行うための方法を段階を追って説明していきます。


Choreonoid本体とは独立してビルドする方法
----------------------------------------

プラグインはChoreonoid本体とは独立してビルドすることもできます。その場合も通常はCMakeを用いてビルドを行います。以下ではこの方法について紹介します。なお :doc:`basics` :ref:`plugin-dev-basics-build-forms` で述べたように、この方法は現状ではWindows上でVisual Studioを用いる場合は適用出来ませんので、ご注意ください。以下はUbuntu Linuxで使用する場合の手順になります。

.. _plugin-dev-minimum-sample-sdk-setup:

Choreonoid SDKのセットアップ
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

このビルド方法を用いる場合は、Choreonoid SDKが外部から使用可能な状態になっていなければなりません。このため、Choreonoid本体をソースコードからビルドしている場合は、予め以下の作業を行っておく必要があります。（ChoreonoidをSKD込みのバイナリパッケージでインストールしている場合は、それだけで以下の全て満たされている場合があります。）

1. Choreonoid本体のビルド時にCMakeのINSTALL_SDKオプションをONにする
2. ビルド完了後にmake installでインストールする
3. Choroenoid SDKのCMakeファイルへのパスを通す

1についてはLinuxの場合デフォルトでONになっていますので、設定を変えていなければ大丈夫です。ビルド後は2に示すインストール作業も行うようにしてください。詳細は :doc:`../install/build-ubuntu` の :ref:`build-ubuntu_install` を参照ください。

3についてはChoreonoid関連プログラムのビルドに必要なCMakeの情報を外部から取得するために必要です。インストール先がデフォルトの/usr/localである場合は、CMakeファイルへのパスもデフォルトで通っていますので、この作業は必要ありません。しかしそれ以外のディレクトリにインストールシている場合は、設定が必要となります。

.. highlight:: sh

設定は環境変数CMAKE_PREFIX_PATHを用いて行います。例えばChoreonoid本体をホームディレクトリのchoreonoidにインストールしている場合は、 ::

 export CMAKE_PREFIX_PATH=~/chorenoid

などとしておきます。CMAKE_PREFIX_PATHが他のソフトウェア用に既に設定されている場合は ::

 export CMAKE_PREFIX_PATH=~/chorenoid:$CMAKE_PREFIX_PATH

などとしてパスを追加するかたちで設定します。

Choreonoid SDKのCMakeファイルは実際にはインストール先のshare/choreonoid/cmake以下に配置されます。この配置方法はCMake標準の配置方法のひとつであり、上記の設定でこのディレクトリのCMakeファイルが検索対象となります。

.. note:: OSにChoreonoid本体を複数インストールしている場合は注意が必要です。もしそれら複数のChoreonoidインスタンスに関してそれぞれCMakeファイルや実行ファイルへのパスが通っていると、プラグインのビルド時やChoreonoidの実行時に想定外のインスタンスが対象となってしまい、ビルドや実行の際に想定外の挙動となることがあります。これはユーザーが把握していなくても、ソースからの自前ビルド、パッケージシステムによるインストール、ROS等の特殊環境での利用などが混在していて、同じ状況になっている場合も有り得ます。それら複数のインスタンスの間でバージョンやビルドオプション、有効なプラグインなどが異なる場合もあり、そのような差異もまた問題になり得ます。従って、Choreonoidを利用する際には同時にひとつの実態だけが対象となるように気をつけてください。なお、適切な切り分けができていれば、同時に複数のChoreonoidインスタンスを利用すること自体は問題ありません。

ソースディレクトリの作成
~~~~~~~~~~~~~~~~~~~~~~~~

プラグイン用のソースディレクトリを作成します。Choreonoid本体のビルド環境でビルドする場合は、ソースディレクトリをChoreonoid本体のextディレクトリに作成しましたが、今回はそれとは異なる場所に作成する必要があるので注意してください。もしChoreonoid本体のソースディレクトリがある場合でも、そこからは独立した場所に作成します。これが守られていれば、どこにどのような名前で作成してもOKです。

例えばChoreonoid本体のソースディレクトリがホームディレクトリのsrc/choreonoidにあるとして、src/DevGuidePluginというディレクトリを作成します。そこにやはりソースファイルとCMakeLists.txtを作成します。するとディレクトリ構成は以下のようになります。 ::

 + src
   + choreonoid (Choreonoid本体のソースディレクトリ)
   + DevGuidePlugin (プラグインのソースディレクトリ)
     - CMakeLists.txt
     - DevGuidePlugin.cpp

※ これはあくまで一例であり、これと同じ構成にしなければいけないわけではありません。

CMakeLists.txtの記述
~~~~~~~~~~~~~~~~~~~~

.. highlight:: cmake

プラグインをChoreonoid本体とは独立してビルドする場合、プラグインビルド用のCMakeListst.txtに追加の記述が必要となります。今回のサンプルでは以下のように記述すればOKです。 ::

  cmake_minimum_required(VERSION 3.10)
  project(DevGuidePlugin)
  find_package(Choreonoid REQUIRED)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  
  choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

今回はプラグイン用のCMakeLists.txtが単体でCMakeによって処理されることになります。上記の記述はこれを踏まえてビルドに必要な記述を全て含むものとなっています。以下ではこの記述内容について解説します。 ::

 cmake_minimum_required(VERSION 3.10)

最低限必要なCMakeのバージョンを指定しています。CMakeに処理させるCMakeListst.txtでは仕様上まずこの記述をする必要があります。CMakeのバージョンごとに機能の追加や廃止がありますので、お使いのCMakeのバージョンとCMakeLists.txtの記述内容を踏まえて適切なバージョンを指定する必要があります。参考までに、Ubuntu Linuxに標準パッケージでインストールされるCMakeのバージョンは以下のようになっています。

* Ubuntu 16.04: CMakeバージョン3.5.1
* Ubuntu 18.04: CMakeバージョン3.10.2
* Ubuntu 20.04: CMakeバージョン3.16.3

Ubuntu16.04は2021年4月でサポートが終了しています。サポートのあるUbuntu18.04のCMakeバージョン3.10をひとつの目安にするのがよいかもしれません。 

なお実際に使用するCMakeのバージョンはここで指定したものより新しいものであればどれでも利用できます。 ::

 project(DevGuidePlugin)

プロジェクト名を設定します。これも単体で処理するCMakeLists.txtにはかならず含める必要があります。プロジェクト名の決め方に特に決まりはありませんが、プラグインの場合は単純にプラグイン名を設定すれば分かりやすくてよいのではないかと思います。 ::

 find_package(Choreonoid REQUIRED)

find_packageは外部のソフトウェアライブラリ等の情報を得るためのCMake標準のコマンドです。対象のソフトウェアがCMakeの形式で情報を提供していれば（そのためのCMakeファイルがインストールされていれば）、このコマンドで情報を得ることができます。今回これが必要なので、Choreonoid SDKのCMakeファイルへのパスを通しておく必要があったわけです。

このコマンドは ::

 find_package(対象ソフトウェア名)

として使用します。Choreonoidの場合はここにChoreonoidを指定します。検出に成功すると、Choreonoid_FOUNDという変数に真値が設定されます。オプションとしてREQUIREDを指定すると対象のパッケージを必須とします。この場合は検出に失敗するとエラーが出力され、それ以降のCMakeの処理が停止します。今回Choreonoidのプラグインを作成しようとしているので、Choreonoidが必須となるため、REQUIREDを指定しています。

Choreonoid SDKが検出されると、関連する情報として以下のような変数が設定されます。

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - 変数
   - 内容
 * - CHOREONOID_INCLUDE_DIRS
   - SDKのヘッダファイルのディレクトリ
 * - CHOREONOID_LIBRARY_DIRS
   - SDKのライブラリファイルのディレクトリ
 * - CHOREONOID_UTIL_LIBRARIES
   - Utilライブラリ使用時にリンクすべきライブラリ
 * - CHOREONOID_BASE_LIBRARIES
   - Baseモジュール使用時にリンクすべきライブラリ
 * - CHOREONOID_PLUGIN_DIR
   - プラグインファイルをインストールするディレクトリ

他にもいくつかの変数と、Choreonoid関連プログラムをビルドするためのいくつかの関数が定義されます。実際にはそれらの関数でビルドのためのほとんどの記述ができるので、これで定義される変数を使用しなければならないことはそれほど多くありません。 ::

  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})

使用するC++のバージョンを指定しています。ここでCMAKE_CXX_STANDARDはCMakeの組み込み変数で、ここにC++のバージョン番号を入れると、それを使用するためのオプションをコンパイラに付与してくれます。そしてCHOREONOID_CXX_STANDARDは、Choreonoid本体のビルド時に使用されたC++のバージョンです。Choreonoid本体とプラグインとの間でのバイナリ互換性を考慮すると、C++のバージョンはなるべく一致させておくことが望ましいので、この記述を入れています。 ::

 choreonoid_add_plugin(CnoidDevGuidePlugin DevGuidePlugin.cpp)

この部分はChoreonoid本体のビルド環境でビルドする場合と同じです。この記述によってプラグインをビルドするための基本的な処理がなされます。choreonoid_add_pluginコマンドについては、上記のChoreonoid SDKの検出時に合わせて定義されます。要はこのコマンドを使用できるようにするために、SDKのセットアップやCMakeLists.txtへの追加の記述を行ってきたと言えるかもしれません。

ビルド・インストール
~~~~~~~~~~~~~~~~~~~~

.. highlight:: sh

ビルドとインストールについてはChoreonoid本体とは独立に作業します。つまりこのプラグインを対象として通常のCMakeの使い方をすればOKです。例えば上記のディレクトリ構成にあるものとして、コマンドラインで作業を進める場合は以下のようになります。 ::

 cd src/DevGuidePlugin
 mkdir build
 cd build
 cmake ..
 make
 make install

cmakeの実行時に、find_packageに失敗すると以下のようなエラーメッセージが出力されます。

.. code-block:: text

 CMake Error at CMakeLists.txt:3 (find_package):
   By not providing "FindChoreonoid.cmake" in CMAKE_MODULE_PATH this project
   has asked CMake to find a package configuration file provided by
   "Choreonoid", but CMake did not find one.
 
   Could not find a package configuration file provided by "Choreonoid" with
   any of the following names:
 
     ChoreonoidConfig.cmake
     choreonoid-config.cmake
 
   Add the installation prefix of "Choreonoid" to CMAKE_PREFIX_PATH or set
   "Choreonoid_DIR" to a directory containing one of the above files.  If
   "Choreonoid" provides a separate development package or SDK, be sure it has
   been installed.

このように出力される場合は :ref:`plugin-dev-minimum-sample-sdk-setup` に不備がありますので、そちらを確認をするようにしてください。

makeの実行については、 :ref:`install_build-ubuntu_build` で説明しているように、CPUコア数（スレッド数）に応じた並列ビルドを行うとよいです。例えば論理8コアのCPUの場合は ::

 make -j8

とすれば最大8のプロセスを用いて並列にビルドしますので、ビルドが高速になります。

またこの形態でプラグインをビルドする場合は必ずインストールも必要となります。インストール先はfind_packageによる情報取得で認識されていますが、そこがroot権限でないと書き込みできないディレクトリである場合は ::

 sudo make install

としてroot権限でインストールを行います。


その他のビルド方法
------------------

.. highlight:: sh

ここまでCMakeを用いてビルドする方法について紹介しました。このようにChoreonoidのプラグイン開発では通常CMakeを用いますが、これはCMakeが現在広く使われていて一般的なビルドツールとなっており、CMakeによってビルドに関わる様々な設定や処理を自動化できるからです。

ただしCMakeでなければビルドできないということは必ずしもありません。もし特殊な事情で他の方法でビルドしなければならない場合は、それも可能です。その際は以下の点を考慮するようにします。

* Choreonoid SDKやその他の依存ライブラリのヘッダファイル（インクルードディレクトリ）にアクセスできるようになっていること
* Choreonoid SDKやその他の依存ライブラリに関して必要なライブラリへのリンクを行うこと
* Choreonoid本体や依存ライブラリのバイナリと互換性のある設定（コンパイル／リンクオプション）でビルドを行うこと

これはC++のようなコンパイル型言語で記述するソフトウェアモジュールを開発する場合に一般的に考慮すべきことでもあります。CMakeを用いる場合はこのようなことをあまり考慮せずに済みますが、その他の手法を用いる場合はまずこれらの事項を正確に把握している必要があります。

このために必要な情報については、本開発ガイドやChoreonoidのソースコードなどをあたってください。また、Choreonoid本体や既にCMakeでビルドされるようになっているプラグインなどは、ビルド時のmakeコマンドに ::

 make VERBOSE=1

とVERBOSE設定を行うことで、ビルドで実際に使われているコマンドが全て表示されるようになります。この出力にはビルドに必要なほぼ全ての情報が含まれているとも言えますので、こちらを参考にするのもよいかと思います。
