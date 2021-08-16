============================
シグナル利用のサンプル (S02)
============================

.. contents:: 目次
   :local:

概要
----

本節ではシグナルを利用するプラグインのサンプルを提示し、これによってシグナルの実際の利用方法を解説します。ここで紹介するサンプルは、 :doc:`minimum-sample` を少しだけ改修したもので、タイムバーの操作に連動してメッセージを出力するというものです。

ソースコード
------------

.. highlight:: cpp

:doc:`minimum-sample` と同様にプラグインのソースディレクトリを作成し、そこに以下のソースコードをDevGuidePlugin.cppというファイル名で作成します。 ::

 #include <cnoid/Plugin>
 #include <cnoid/MessageView>
 #include <cnoid/TimeBar>
 #include <fmt/format.h>
 
 using namespace cnoid;
 
 class DevGuidePlugin : public Plugin
 {
     ScopedConnection connection;
 
 public:
     DevGuidePlugin() : Plugin("DevGuide")
     {
 
     }
 
     virtual bool initialize() override
     {
         connection =
             TimeBar::instance()->sigTimeChanged().connect(
                 [this](double time){ return onTimeChanged(time); });
         return true;
     }
 
     bool onTimeChanged(double time)
     {
         MessageView::instance()->putln(fmt::format("Current time is {}", time));
         return true;
     }
 };
 
 CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)

ビルド用のCMakeLists.txtは :doc:`minimum-sample` と同じ記述でOKです。

プラグインの実行
----------------

本プラグインがインストールされた状態でChoreonoidを実行すると、タイムバーによる時間操作に連動してこのプラグインのonTimeChanged関数が呼ばれるようになります。その結果、タイムバーのスライドを操作したり、再生ボタンを押したりすると、Choreonodi上の時刻の変化に伴って、メッセージビューに現在時刻がテキストで表示されるようになります。実際にタイムバーを操作して動作を確かめてみてください。

ソースコードの解説
------------------

本サンプルで新たに導入した部分について解説します。（ :doc:`minimum-sample` と同じ個所については解説を省略します。） ::

 #include <cnoid/TimeBar>

本サンプルではタイムバーのシグナルを利用します。このため `TimeBarクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1TimeBar.html>`_ のヘッダを取り込んでいます。 ::

 #include <fmt/format.h>

メッセージビューへのテキスト出力に使用するfmtライブラリのヘッダを取り込みます。 ::

 class DevGuidePlugin : public Plugin
 {
     ScopedConnection connection;

     ...


シグナルとスロットの接続解除用に、ScopedConnectionオブジェクトをメンバ変数として定義しています。このプラグインで行ったシグナル接続をプラグイン解放時に自動的に解除するために使用します。 ::

 virtual bool initialize() override
 {
     connection =
         TimeBar::instance()->sigTimeChanged().connect(
             [this](double time){ return onTimeChanged(time); });
     return true;
 }

プラグインの初期化関数でシグナルとスロットの接続を行っています。まず対象となるシグナルはTimeBarの "sigTimeChanged" というシグナルです。これはTimeBarクラスで以下のように定義されています。 ::

 SignalProxy<bool(double time), LogicalSum> sigTimeChanged();

これは関数の型としてはdouble型の引数をひとつ有していて、戻り値としてbool値を返す関数になります。複数のスロットが接続されている場合のシグナル送出側への戻り値はLogicalSumで決定されます。すなわちどれかひとつでもtrueを返せばtrueになります。そしてSignalProxyを返すメンバ関数として定義されているので、接続のみが可能です。なお ::

 TimeBar::instance()

についてはTimeBarクラスの唯一のインスタンスをTimeBarのポインタで返すstaticメンバ関数です。このようにシングルトンパターンでインスタンスを返すクラスがChoreonoidにはいくつもあり、同じ形式で利用できます。

スロットの本体はDevGuidePluginのメンバ関数onTimeChangedなのですが、これがメンバ関数であるため、シグナルとの接続にあたって以下のラムダ式を用いています。 ::
  
 [this](double time){ return onTimeChanged(time); });

このラムダ式によってプラグインのメンバ関数を呼び出すためのインスタンス変数thisを補っています。これは ::
 
 [this](double time){ return this->onTimeChanged(time); });

と同等のコードで、このようにthisポインタのオブジェクトに対してonTimeChangedが呼ばれています。この書き方の方がより分かりやすいかもしれません。

sigTimeChangedシグナルについては、名前から推測できると思いますが、タイムバーで管理している時刻が変化した際に送出されるシグナルです。これは `TimeBarクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1TimeBar.html>`_ で以下のように定義されています。 ::

 SignalProxy<bool(double time), LogicalSum> sigTimeChanged();

これはタイムバーの時刻が変化する全ての状況において送出されるシグナルでで、時刻表示のスピンボックス上で値を入力する、スライダを操作する、再生ボタンでアニメーションを実施する、といったいずれの状況でも送出されます。アニメーション再生時はタイムバーの再生フレームレートの間隔で周期的に送出されることになります。そして引数timeは現在時刻になります。また、スロットからの戻り値も要求するシグナルとなっていて、その時刻で有効な処理を行った場合はtrueを、そうでなければfalseを返すようにしなければなりません。この戻り値は特にアニメーション中に参照されます。全てのスロットの戻り値のLogicalSumを参照して、どれかひとつでもtrueならアニメーションを継続し、全てがfalseの場合は停止するようになっています。つまりアニメーション中の各時刻において有効な処理があればアニメーションを継続しますが、そうでなくなったタイミングでアニメーションを停止することになります。

なお ::

 connection =
     ...

とすることで、シグナルのconnect関数が返すConnectionオブジェクトをメンバ変数のconnectionに代入しています。上述のようにconnectionはScopedConnection型なので、プラグイン解放（削除）時にこの接続も解除されます。プラグインが削除されるのは通常はアプリケーション終了時なので、実は今回のケースではこの処理がなくても特に問題はありません。そうは言っても、スロットを呼び出せなくなる時にそのスロットへの接続は解除しておくことが望ましいので、メンバ関数をスロットとして接続する際にはなるべくこのような処理を入れたほうがよいでしょう。ここではConnectionオブジェクトの利用例も兼ねて、このように記述しています。 ::

 bool onTimeChanged(double time)
 {
     MessageView::instance()->putln(fmt::format("Current time is {}", time));
     return true;
 }

TimBarのsigTimeChangedシグナルと接続するスロットの本体です。メッセージビューに現在時刻のメッセージを出力します。戻り値としてtrueを返しているので、アニメーションは常に継続します。

ここではメッセージ文字列の作成に `テキスト整形ライブラリfmt <https://github.com/fmtlib/fmt>`_ を利用しています。このライブラリのformat関数はC言語のprintf関数のC++版と考えもらえば結構です。テキスト中の{}は「置換フィールド」と呼ばれるもので、format関数の2番目の引数に指定したtimeの値で置き換えられます。今回のメッセージはシンプルなので、必ずしもこれを使う必要はないかもしれませんが、ライブラリの紹介のために使用しています。fmtライブラリはC++用のテキスト整形ライブラリとして定評のあるものですし、 C++20以降では標準C++ライブラリにも `std::format <https://cpprefjp.github.io/reference/format/format.html>`_ として取り込まれています。Choreonoidのソースコードでも頻繁に使用していますので、これまであまり馴染みのなかった方もこの機会に覚えていただくとよいのではないかと思います。






