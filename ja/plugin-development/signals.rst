==============
シグナルの利用
==============

.. contents:: 目次
   :local:

.. highlight:: cpp

シグナルとは
------------

Choreonoid SDKの中の重要な概念のひとつにシグナルがあります。シグナルとはプログラムを実行する中である特定の処理がなされたりイベントが発生したときに、それをプログラムの他の個所に通知するための仕組みです。通知というのは具体的には何らかの関数を呼び出す処理になります。あるシグナルに対してシグナルの型を満たす任意の関数を紐付けておくことができ、シグナルの通知時にそれらの関数が呼ばれます。あるシグナルに複数の関数を紐付けることもでき、その場合はシグナルの通知時に全ての関数が呼ばれることになります。これはある種の関数コールバック機構と言えますが、関数が型安全で複数設定可能という特徴があります。Choreonoidではシグナルに紐付ける関数を「スロット」と呼び、紐付けの操作を「接続」とも呼びます。シグナルの通知を引き起こす処理をシグナルの「送出」と呼ぶことにします。

シグナルに対するスロットの接続は、任意の個所から自由に行うことができます。その際にプログラムのどこからシグナルが送出されるかを把握している必要はありませんし、他に接続されているスロットを把握する必要もありません。逆にシグナルを送出する側にとっても、実際にどのスロットが呼び出されるかを把握する必要はありません。そのようにシグナルの送出側とそれを受けるスロットは互いに独立しながらも、該当するシグナルを介して処理が連携することになります。このような緩い繋がりによって、多用な処理の連携を柔軟に実装することができ、システム全体の保守性や拡張性を高めることができます。

このようなシグナルの仕組みは、ソフトウェア開発において一般的な技法となっています。この技法は所謂デザインパターンに含まれる `Observerパターン <https://en.wikipedia.org/wiki/Observer_pattern>`_ が該当します。実際の実装における名称はいくつかあり、その中で「シグナル／スロット」という呼び方はC++のフレームワークライブラリ `Qt <https://www.qt.io/>`_ でポピュラーとなったもので、この影響を受けた `libsigc++ <https://libsigcplusplus.github.io/libsigcplusplus/>`_  や `Boost.Signals <https://www.boost.org/doc/libs/1_68_0/doc/html/signals.html>`_ といったライブラリでも採用されています。Choeonoidもこれらのライブラリの影響を受けており、同じ用語を用いるようにしています。

Choreonoidでもこの技法を活用することで、多用な処理の柔軟かつ効率的な連携を実現しています。実際にChoreonoid本体や既存プラグインの実装では多数のシグナル／スロットが実装されており、独自プラグインを開発する際にもこの仕組みを覚えて活用することが欠かせません。

.. note:: 同様の技法で出版-購読パターン（Puslish/Subscribe Pattern)というものもあり、ロボット用ミドルウェアとして有名なROSでもこの技法が取り入れられています。このパターンではシグナルに該当するものが「メッセージ」と呼ばれており、メッセージの送信側と受信側の結合度がより低い構造となっています。

.. note:: Linuxを含むUnix系OSではカーネルによって処理される「シグナル」があります。これは主にOSのプロセスに関わる特定の通知を行う仕組みで、Choreonoid SDKのシグナルとは別物だと考えてください。


Signalクラス
------------

Choreonoidはシグナルの仕組みを実現するために独自のSignalクラスを定義しています。これはUtilライブラリで定義されていて、以下の記述でヘッダを取り込むことができます。 ::

 #include <cnoid/Signal>

Signalクラスを用いることで個々のシグナルに対応するオブジェクトを定義することができます。シグナルオブジェクトというのはある種の関数オブジェクトのようなものです。Signalクラスはテンプレートクラスとして定義されていて、任意の関数型に対応させることができます。

例えばあるシグナルを伝えるのに ::

 void function();

という関数を使用するとしましょう。引数なしで戻り値もなしの関数ですね。この場合シグナルオブジェクトは以下のように定義します。 ::

 cnoid::Signal<void()> signal;

SignalクラスはChoreonoid SDKの名前空間であるcnoid内で定義されていますので、名前空間cnoid外で使用する場合はこのようにcnoid::Signalと記述します。ただし以下の説明では名前空間cnoidの記述を省略することにします。

このsignalは Signal<void()> という型に対応するオブジェクトです。このオブジェクトを用いてスロットの接続やシグナルの送出を行うことができます。

このシグナルの受け手側で ::

 void slot()
 {
     ...
 }

といった関数が定義されているとします。これをシグナルに接続するには、以下のようにします。 ::

 signal.connect(slot);

これでslotがsignalに接続されました。接続したい関数をシグナルのconnect関数に渡せばOKです。

シグナルの送出は以下のようにします。 ::

 signal();

シグナルオブジェクトはある種の関数オブジェクトなので、このように関数として実行するかのような記述ができます。すると実際にはこの個所が実行される際に、シグナルに接続されていた関数が呼び出されます。

複数のスロットを接続することもできます。例えば ::

 void slot2()
 {
     ...
 }

という関数があったとして、単純にまたconnectで接続すればOKです。 ::

 signal.connect(slot2);

この場合、シグナルの送出時に、関数slotとslot2が両方呼び出されます。呼び出す順序はシグナルに接続した順序となります。

このようにしてひとつのシグナルオブジェクトに複数のスロットを接続することができます。実際にはシグナルへのスロットの接続はプログラムの様々な個所で行われる可能性があり、その結果としてあるシグナルにいつの間にか複数のスロットが接続されている、というのが一般的なケースです。この場合、シグナルを定義している部分も、シグナルの送出元も、シグナルの受け手も、それぞれ具体的にどこがシグナルを送出してどの受け手に通知されるかを特定せずに、処理が進められることになります。

「シグナルはある種の関数オブジェクト」という説明をしましたが、より正確には「複数の関数を格納してまとめて呼び出せる関数オブジェクト」と言うことができるかもしれません。実際にSignalクラスもほぼそのような実装になっており、内部で複数の関数オブジェクトをリストの形式で保持しています。

なお、シグナルオブジェクトにスロットが何も接続されていない場合もあり得ます。その状況でシグナルを送出しても、実際には何も起こりません。呼び出すべきスロットがないので、何も呼び出されないまま、シグナル送出処理が終了します。この場合も、シグナルの送出元はシグナルが接続されているかどうか気にせずにシグナルの送出を行えるというわけです。実際にこのようなケースもよくあります。

.. note:: このSignalクラスはChoreonoidで独自に実装しているものですが、その設計はBoost C++ライブラリの `Boost.Signals <https://www.boost.org/doc/libs/1_68_0/doc/html/signals.html>`_ を参考にしています。利用方法については共通する部分も多いので、そちらのドキュメントも参考になるかと思います。なおBoost.SignalsはBoost 1.69以降で廃止されており、後継のBoost.Signals2に置き換えられていますが、そちらは使用方法がやや複雑になっています。実はChoreonoidでもバージョン1.4まではBoost.Signalsを利用しており、さらにBoost.Signals2への置き換えも検討されたのですが、Choreonoidの実装における扱いやすさを考慮した結果、バージョン1.5からは独自のSignalクラスが導入されることになりました。

.. _plugin-dev-signals-parameters:

引数を有するシグナル
--------------------

シグナルには引数を設定することもできます。例えば ::

 Signal<void(bool on)> boolSignal;

とすると、このシグナルはbool型の引数をひとつ有するシグナル型となります。

するとこのシグナルに接続するスロットはこの引数を持つ必要があります。例えば ::

 void boolSlot(bool on)
 {
     ...
 }

という関数であれば、 ::

 boolSignal.connect(boolSlot);

とすることができ、このシグナルに接続するスロットとして使用できます。逆に先程使用した ::

 void slot();

という関数は、引数が異なるので直接このシグナルに接続することはできません。

このように引数をもつシグナルでは、シグナルの送出時に引数を渡すことができます。これは単純に ::

 boolSignal(true);

といったかたちで、シグナルオブジェクトへの関数呼び出しオペレータに引数を与えればOKです。これによって、結果的には ::

 boolSlot(true);

が実行されることになります。

もちろん引数に与える値は何でも（ここではfalseでも）結構です。

このようにしてシグナルは任意の型の引数を任意個持たせることができます。その指定はSignalクラスのテンプレートパラメータに関数のシグネチャ（戻り地と引数を関数定義のように記述したもの）を与えることで行います。

例えばint値とstd::stringオブジェクトへのconst参照とSomethingクラスへのポインタを引数にもつようなより複雑なシグナルも、 ::

 Signal<void(int, const std::string&, Something*)> complexSignal;

と記述することで実現できます。なお関数のシグネチャにおいて引数型に仮引数を記述することもできます。この場合、上記の定義は ::

 Signal<void(int value, const std::string& text, Something* something)> complexSignal;

といったかたちで記述できます。これはシグナルの型自体には影響を与えませんが、定義においてそれぞれの引数が何を意味するか分かりやすくするためには有効かと思います。

このシグナルも、例えば ::

 Something* something = new Something;
 complexSignal(5, "message", something);

といったかたちで送出することができます。

戻り値を要求するシグナル
------------------------

シグナルは戻り値を要求することもできます。この項目はシグナルの比較的高度な使い方になりますので、とりあえず読み飛ばしていただいても結構です。実際に戻り値が必要となるケースはそれほど多くはありません。

戻り値を要求するとシグナルは、例えば以下のように定義できます。 ::

 Siganl<bool()> rvSignal;

Signalクラスに与える関数のシグネチャでbool型の戻り値を持つことが表されています。これがシグナルが要求する戻り値です。これに接続するスロットはこれに合わせてbool値を返す関数である必要があります。例えば ::

 bool rvSlot()
 {
     return true;
 }

といった関数です。

これを ::

 rvSignal.connect(rvSlot);

として接続しておくと、シグナル送出の際に ::

 bool result = rvSignal();

というかたちでboolの戻り値を得ることができます。この例ではtrueが返されて変数resultに設定されます。

これはシグナルが受け手にどのように処理されたかを送出元が知りたい場合に利用できます。戻り値はbool型以外にも任意の型を指定できます。

ただしシグナル送出元に返される戻り値について、値が明確であるとは限りません。上記の例のように、シグナルにひとつだけスロットが接続されている場合は、そのスロットの戻り値がそのまま返されれば問題なさそうです。しかしながら、スロットが何も接続されていなかったり、複数のスロットが接続されている場合は、どのようにして戻り値を決めればよいでしょう？スロットが接続されていない場合は返すべき値がありません。また複数接続されている場合に各スロットが異なる値を返したら、どの値を送出元に返したらよいでしょう？これらは何かルールを与えない限り決めることができません。

そこで戻り値を要求するシグナル用に、これを解決する仕組みも備わっています。これはSignalクラスのテンプレートパラメータの第2引数で設定します。これにはデフォルト値が設定されていて、その場合は「最後に呼び出されたスロットが返す値」がシグナル送出元に返されます。この場合、シグナルが何も接続されていなければ、返される値は不定となります。

この挙動を変えたい場合は、テンプレートパラメータの第2引数を指定します。例えば ::

 Signal<bool(), LogicalSum> rvSumSignal;

とすると、返される値はスロットが返す値の論理和となります。つまり、どれかひとつでもスロットがtrueを返せばtrueになりますし、そうでなければfalseになります。他には ::

 Signal<bool(), LogicalProduct> rvProductSignal;

とすると、返される値はスロットが返す値の論理積となります。この場合全てのスロットがtrueを返せばtrueになりますが、どれかひとつでもfalseを返すとfalseになります。特別な状況として、スロットが存在しない場合はtrueが返ります。

ここで与えているLogicalSumやLogicalProcutは、Combinerと呼ばれるオブジェクトです。これは各スロットの戻り値をイテレータとして受け取って最終的な戻り値を決定する関数オブジェクトです。LogicalSumやLogicalProductはSignalヘッダで予め定義されているCombinerで、例えばLogicalSumは以下のように定義されています。 ::

 class LogicalSum
 {
 public:
     typedef bool result_type;
     template<typename InputIterator>
     bool operator()(InputIterator iter, InputIterator last) const {
         bool result = false;
         while(iter != last){
             if(iter.isReady()){
                 result |= *iter;
             }
             ++iter;
         }
         return result;
     }
 };

関数オブジェクトの引数InputIteratorは、各スロットの戻り値に対応するイテレータで、これを終了点であるlastまで回します。この実装の肝は ::

 result |= *iter;

の部分で、これによって全ての戻り値の論理和が最終的に返されるようになります。

この部分は同じ形式を有する任意の関数オブジェクトを設定できますので、デフォルトの処理やLogicalSum、LogicalProductとは異なる戻り値の決定方法が必要な場合は、それに対応するCombinerを自前で記述して与えるようにしてください。

ラムダ式の利用
--------------

スロットとしてシグナルに接続する関数は、シグナルと同じシグネチャで呼び出せる関数であれば何でも結構です。従って、スロットは必ずしも上記の例のように静的に定義された一般の関数である必要はなく、各種の関数オブジェクトにも対応可能です。そのひとつの例として、C++11で導入されたラムダ式を用いることも可能で、これによりスロット接続の柔軟性が高まります。

ラムダ式を用いる例として、まずクラスのメンバ関数（インスタンス関数）への接続が可能となります。例えば ::

 Signal<void()> signal;

というシグナルと ::

 class A
 {
 public:
     A();
     void functionA();
 };

というクラスが定義されているとします。クラスAのオブジェクトが ::

 A object;

として定義されているとして、シグナルをこのオブジェクトを対象としたメンバ関数functionAの呼び出しに紐付けるには、以下のようにします。 ::

 signal.connect([&object](){ object.functionA(); });

あるいはクラスAの関数、例えばコンストラクタから同様の紐付けを行う場合は、 ::

 A::A()
 {
     signal.connect([this](){ functionA(); });
 }

と記述することもできます。

このようにラムダ式を用いて、オブジェクトのインスタンスをキャプチャし、それに対してラムダ式内で所望のメンバ関数を呼び出すことにより、メンバ関数もスロットとして接続することができます。

このようにメンバ関数の隠れ引数thisを補うだけでなく、関数の通常の引数についてもラムダ式で補うことが可能です。

例えば上記のシグナルに対して ::

 void functionB(const std::string& text);

という関数を接続したいとしましょう。この場合、引数textはシグナルからは得られませんが、他の手段でこの文字列を決定できるのであれば、それをラムダ式に組み込みます。規定の文字列でよいのであれば、 ::

 signal.connect([](){ functionB("Specified Text"); });

などとすることができますし、他の変数で指定したい場合は ::

 string text;

 ...

 signal.connect([text](){ functionB(text); });

などとすることも可能です。

逆にシグナルに含まれる引数を持たない関数と接続することもできます。例えば :ref:`plugin-dev-signals-parameters` で紹介した ::

 Signal<void(int value, const std::string& text, Something* something)> complexSignal;

というシグナルについて、 ::

 void slotWithoutText(int value, Something* something);

という関数を接続したいとしましょう。この関数はシグナルで定義されている引数textを持たないため、このままでは接続できません。そのような場合でも、textも引数として持つラムダ式においてtextの値を無視した呼び出しを行うことで、シグナルへの接続を実現できます。つまり以下のようにします。 ::

 complexSignal.connect(
     [](int value, const std::string&, Something* something){
         slowWithoutText(value, something);
     });

.. _plugin-dev-signal-proxy:
     
SignalProxyクラス
-----------------

Signalヘッダを取り込むとSignalProxyクラスも使えるようになります。これはあるシグナルオブジェクトに対して接続操作だけを可能とするためのプロクシオブジェクトを生成するものです。

例えばあるクラスが自身の状態変更を伝えるためにsigUpdatedというシグナルを定義したとします。そのようなクラスのシグナルの部分だけ抜粋した例を以下に示します。 ::

 class B
 {
 public:
     Signal<void()> sigUpdated;
 };

このクラスのオブジェクトが ::

 B object;

などと定義されているとして、これのシグナルと接続する場合は ::

 object.sigUpdated.connect(slot);

などと記述できます。

ただしこの場合、シグナルオブジェクトの全ての関数が呼び出し可能なので、どこからでも ::

 object.sigUpdate();

としてシグナルの送出ができてしまいます。

しかしこのシグナルは本来オブジェクトの状態変更の際に送出されるべきものであり、どこからでもシグナルを送出してよいものではないとします。一般的にもほとんどのシグナルは送出される状況を限定するものになるかと思います。

この場合、シグナルを接続する側からもシグナルの送出ができてしまうことが問題なわけです。そしてこれを防ぐためにSignalProxyクラスが提供されています。

これを用いて上記のクラスBを書き換えると以下のようになります。 ::

 class B
 {
 public:
     SignalProxy<void()> sigUpdated() { return sigUpdated_; }
 private:
     Signal<void()> sigUpdated_;
 };

SignalProxyもテンプレートクラスとして定義されていて、Signalクラスと同様に関数のシグネチャをテンプレートパラメータにとります。このシグネチャは対象とするシグナルと同一にしておく必要があります。そしてSignalProxyのコンストラクタにより、対象とするSignalオブジェクトに対応するプロクシオブジェクトを生成できます。

この場合、接続側は ::

 object.sigUpdated().connect(slot);

として関数を接続できます。SignalProxyを得るためにメンバ関数の呼び出しとなっている点が先程と異なりますが、connect関数については先程と同様に使用できます。

この場合、SignalProxy経由で接続はできるのですが、シグナルの送出はできません。つまり、 ::

 objedt.sigUpdate()();

などとすることはできないわけです。

シグナルの本体であるsigUpdated_はクラスBのプライベートメンバとして定義されているので、クラスBの実装によってこのシグナルの送出を管理できます。このように、SignalProxyを導入することで、シグナルの定義元と接続側でできることを分けられるようになります。つまり接続側では接続のみを可能とし、送出の仕方はシグナルの定義元で管理できるようにするということです。

実際にChoreonoid SDKのクラスで定義されているシグナルは、ほどんどのケースでこのSignalProxyが返されるようになっていて、シグナルの送出については別の手段で行われるようになっています。

.. _plugin-dev-signals-connection-class:

Connectionクラス
----------------

シグナルに一旦スロットを接続すると、シグナルオブジェクトが存在している間はずっとその接続が保たれることになります。しかしシグナルとスロットの接続を解除したいこともあります。これを行うためのクラスがConnectionクラスです。このクラスはSignalヘッダを取り込むとSignalクラスと同様に使えるようになっています。

実はこれまで示してきたconnect関数によるスロットの接続では、関数の戻り値がありました。それがConnectionクラスのオブジェクトです。

このオブジェクトをconnect関数の呼び出し元で受け取ることができます。これは以下のように記述できます。 ::

 Connection connection = signal.connect(slot);

このconnectionオブジェクトによって接続を管理できます。接続の解除をするには、以下のようにdisconnect関数を実行すればOKです。 ::

 connection.disconnect();

これによって接続が解除され、それ以降はこのシグナルが送出されてもslot関数が呼ばれなくなります。シグナルの受け手のオブジェクトが破棄され、スロット関数が呼べなくなるような状況では、必ず接続の解除をしておく必要があります。

なお、接続解除の前にシグナルが破棄される場合は、そのシグナルへの接続は全て自動で解除され、対応するConnectionオブジェクトも無効化されます。従って、シグナルが先に破棄される場合は、その後スロット側で接続解除をする必要はありませんし、仮にしたとしても無視されるだけで問題はありません。

接続の解除以外には、 ::

 bool connected = connection.connected();

によって、現在接続されているかどうかを判定できます。

またConnectionクラスのもうひとつ重要な機能が接続ブロック機能です。これは ::

 connection.block();

とすると、それ以降シグナルが送出されてもスロットが呼ばれなくなります。ただしこの場合は接続自体は解除されていません。そして ::

 connection.unblock();

とすると、再びスロットが呼ばれるようになります。現在ブロックされているかどうかは ::

 bool blocked = connection.isBlocked();

で判定できます。

このブロック機能は一時的にスロットの呼び出しを回避したい状況で使用します。

ちなみにこのブロック機能では比較的短い実行範囲でblockとunblockを一対一に対応させる必要がありますが、 ::

 auto block = connection.scopedBlock();

とすると、このblock変数の生存期間の最初と最後で自動的にblockとunblockが実行されるようになります。これはscopedBlock関数が返すConnection::ScopedBlock型のオブジェクトのコンストラクタとデストラクタによって処理されます。

ConnectionSetクラス
-------------------

Connectionは単一の接続を管理するクラスでしたが、複数の接続を一度に管理するためのクラスとしてConnectionSetも利用可能です。使い方はConnectionとほぼ同じですが、複数のConnectionを保持できる点が異なります。

例えば ::

 Signal<void()> signalA;
 Signal<void(bool)> signalB;
 Signal<void(int)> signalC;

という3つのシグナルがあり、それぞれ対応するスロット関数を接続して利用したいとします。ただしその3つの接続について、必要なタイミングで一度に解除したいものとします。

これは3つのConnectionオブジェクトを ::

 Connection connectionA;
 Connection connectionB;
 Connection connectionC;

と定義しておいて ::

 connectionA = signalA.connect(slotA);
 connectionB = signalB.connect(slotB);
 connectionC = signalC.connect(slotC);

として接続し、解除が必要になったタイミングで ::

 connectionA.disconnect();
 connectionB.disconnect();
 connectionC.disconnect();

としてもよいのですが、Connectionオブジェクトを3つ管理する必要が出てくるので、少々面倒になります。これより多くのシグナル接続を一度に扱うことも珍しくありません。

これについて、以下の記述でConnectionSetクラスを使えるようにしておき ::

 #include <cnoid/ConnectionSet>

ConnectionSet型のオブジェクトを ::

 ConnectionSet connections;

としてひとつ定義しておきます。そして接続時に ::

 connections.add(signalA.connect(slotA));
 connections.add(signalB.connect(slotB));
 connections.add(signalC.connect(slotC));

としておけば、 ::

 connections.disconnect();

とするだけで全ての接続を解除できます。この際各接続はconnectionsの管理から解放されます。

接続のブロックについても、Connectionと同様に ::

 connections.block();

 ...


 connections.unblock();

として利用できますし、scopedBlockも同様に ::

 auto block = connections.scopedBlock();

として使えます。

現在管理している接続があるかどうかは ::

 bool empty = connections.empty();

で判定できますし、管理している接続の数も ::

 int n = connections.numConnections();

で取得できます。

.. _plugin-dev-signals-scoped-connection:

ScopedConnection、ScopedConnectionSetクラス
-------------------------------------------

ConnectionやConnectionSetを用いてシグナル接続を解除する場合は、明示的にdisconnect関数を呼ぶ必要があります。
この場合は接続解除するタイミングについてプログラマが気をつける必要があります。

その一方で、シグナル接続はある処理スコープやオブジェクト生存期間と一致させたい場合が多々有ります。
その場合は処理スコープやオブジェクト生存期間にあわせて自動で接続を解除するのが合理的です。
そしてこれを行うためのクラスとして、ScopedConnectionとScopedConnecitonSetがあります。
それぞれConnectionとConnectionSetに接続自動解除の機能を加えたものです。

例えばある処理スコープの間のみでシグナルからの通知を受けたい場合は ::

 {
     ScopedConnection connection = signal.connect(slot);

     ...

 }

とします。するとこのスコープを抜けるタイミング、すなわちconnectionが破棄されるタイミングで、signalとslotの接続が自動で解除されます。

あるいはオブジェクトの生存期間と一致させて接続解除を行いたい場合は、ScopedConnectionをクラスのメンバ変数として定義しておきます。 ::

 class C
 {
 public:
     C();
     void slot();
 private:
     ScopedConnection connection;
 };


例えばこのクラスのコンストラクタでシグナルとの接続を行うとして、 ::

 C::C()
 {
     connection = signal.connect([this](){ slot(); });
 }

としておけば、 ::

 C* c = new C;

 ...


 delete c;

といった処理の流れがある場合に、オブジェクトcの生成時にシグナル接続がされて、cが破棄される際に自動でシグナル接続が解除されます。

これは通常のConnectionクラスを使用する場合でも、Cのデストラクタを定義してそこで ::

 C::~C()
 {
     conneciton.disconnect();
 }

と明示的に接続を解除すればよいのですが、ScopedConnectionではこの記述を省けるので、より確実かつ効率的に接続解除ができることになります。

なお、 :ref:`plugin-dev-signals-connection-class` でも述べたように、シグナルの方が先に破棄される可能性もありますが、その場合でも特に問題はありません。その場合接続は解除済みとなり、ScopedConnectionが破棄されるタイミングでは何も処理されないことになります。

ScopedConnectionSetについてはConnectionSetと同様に複数のConnectionを管理できることに加えて、ScopedConnectionと同様にデストラクタで接続が解除されます。もちろんこの場合は管理している全ての接続が解除されます。


Choreonoid SDKで利用可能なシグナル
----------------------------------

以上シグナルの利用方法についてひととおり解説しました。

Choreonoid SDKでは多くのシグナルが定義されています。実際にどのようなクラスがあるかは、 `APIリファレンスマニュアル <https://choreonoid.org/ja/documents/reference/latest/index.html>`_ で確認することができます。例えばChoreonoid SDKの主要なクラスのひとつである `Itemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ では、10個のシグナルが定義されています。その中から一部のシグナルの定義を抜粋したものを以下に示します。 ::

 SignalProxy<void(const std::string &oldName)> sigNameChanged();
 SignalProxy<void()> sigPositionChanged();
 SignalProxy<void()> sigDisconnectedFromRoot();
 SignalProxy<void(bool on)> sigSelectionChanged();
 SignalProxy<void(bool on)> sigCheckToggled(int checkId=PrimaryCheck);
 SignalProxy<void()> sigUpdated();

ここでは各シグナルの詳細は省きますが、いずれも以下の特徴を備えていることが分かります。

* SignalProxyを返すメンバ関数として定義されている

 これらはシグナル接続側で利用するために定義されています。シグナルの送出は別途オブジェクトの動作に連動して行われるか、シグナル送出のための専用の関数で行われます。

* メンバ関数の名前はsigXXXXXという形式になっていて、いずれもsigというプレフィックスが付与されている

 Choreonoid SDKで定義されているシグナルは全てこの命名規則になっています。sigというプレフィックスが付くことで、それがシグナルであることが分かります。

以上のルールを理解すれば、 `APIリファレンスマニュアル <https://choreonoid.org/ja/documents/reference/latest/index.html>`_ 等でクラスの定義を確認することで、どのようなシグナルが利用可能か分かります。本ガイドでもこれ以降シグナル利用の実例を紹介していきます。実際のプラグイン開発でもシグナルをうまく利用することで目的とする機能を実現しやすくなりますので、ぜひ活用するようにしてください。
