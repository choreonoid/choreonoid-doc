
コントローラの実装
==================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

.. contents:: 目次
   :local:

.. highlight:: cpp

コントローラの実装
------------------

ここでは、コントローラの実装方法について、その基本を解説します。

コントローラが行うことは、基本的には以下の３つで、これらを「制御ループ」として繰り返し実行します。

1. ロボットの状態を入力
2. 制御計算
3. ロボットへ指令を出力

これらの処理を単体のコントローラで行うこともあれば、複数のソフトウェアコンポーネントを組み合わせて行うこともあります。また、「制御計算」とひとくくりにした処理には、実際には各種認識・動作計画等の多様な処理がからむものですし、ロボット以外を対象とした入出力も含まれる可能性があります。しかし、ロボットを中心としてみると、最終的にコントローラがやっていることは上記の3つの処理に整理して考えることができます。

このように考えると、コントローラというものは、上記３つを行うためのインタフェースを備えたソフトウェアモジュールであると言えます。そのための実際のAPIはコントローラの形式によって異なってくるわけですが、本質的な部分は同じです。

以下では :doc:`howto-use-controller` でも用いた "SR1MinimumController"サンプルを通して解説を行います。コントローラの形式はChoreonoidのサンプル用に設計された「シンプルコントローラ」形式であり、制御の内容は関節に対するPD制御でロボットの姿勢を維持するというだけのものです。記述言語はC++です。

実際にコントローラ開発する際には、このサンプルを通して述べた基本的な事柄を希望のコントローラ形式や制御内容に置き換えて、取り組んでいただければよいかと思います。一般的に、ロボットのコントローラ開発では制御やプログラミング、ハードウェア等に関する様々な知識とスキルが必要となります。それらの多くについては本マニュアルの対象外となりますので、やりたいことに応じて、別途取り組むようにしてください。


サンプルコントローラのソースコード
----------------------------------

まず、SR1MinimumControllerのソースコードを以下に示します。本ソースコードはChoreonoidソースの"sample/SimpleController"ディレクトリ内にある"SR1MinimumController.cpp"というファイルです。 ::

 #include <cnoid/SimpleController>
 #include <vector>
 
 using namespace cnoid;
 
 const double pgain[] = {
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
     3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
     8000.0, 8000.0, 8000.0 };
     
 const double dgain[] = {
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
     100.0, 100.0, 100.0 };

 class SR1MinimumController : public SimpleController
 {
     BodyPtr ioBody;
     double dt;
     std::vector<double> qref;
     std::vector<double> qold;

 public:

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 ioBody = io->body();
	 dt = io->timeStep();

         for(int i=0; i < ioBody->numJoints(); ++i){
             Link* joint = ioBody->joint(i);
             joint->setActuationMode(Link::JointTorque);
	     io->enableIO(joint);
	     qref.push_back(joint->q());
	 }
	 qold = qref;

	 return true;
     }

     virtual bool control() override
     {
	 for(int i=0; i < ioBody->numJoints(); ++i){
	     Link* joint = ioBody->joint(i);
	     double q = joint->q();
	     double dq = (q - qold[i]) / dt;
	     double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
	     qold[i] = q;
	     joint->u() = u;
	 }
	 return true;
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

このコントローラはChoreonoidに付属のサンプルであり、デフォルトでChoreonoid本体と一緒にビルドされるようになっています。（CMakeの設定で **BUILD_SIMPLE_CONTROLLER_SAMPLES** がONになっていればOKです。）

サンプルとは別に新たにシンプルコントローラを実装してビルドする方法については、:doc:`howto-build-controller` をご参照ください。

.. _simulation-implement-controller-simple-controller-class:

SimpleControllerクラス
----------------------

シンプルコントローラ形式のコントローラは、SimpleControllerクラスを継承することで実装します。このクラスは ::

 #include <cnoid/SimpleController>

により、cnoid/SimpleControllerヘッダをインクルードすることで使えるようになります。

このクラスは基本的には以下のような定義になっています。 ::

 class SimpleController
 {
 public:
     virtual bool initialize(SimpleControllerIO* io);
     virtual bool control();
};


このクラスのvirtual関数を継承先のクラスでオーバーライドすることにより、コントローラの処理内容を記述します。各関数の内容は以下のようになっています。

* **virtual bool initialize(SimpleControllerIO\* io)**

 コントローラの初期化処理を行います。引数 io を通して制御に関わるオブジェクトや情報を取得できます。

* **virtual bool control()**

 コントローラの入力・制御・出力処理を行います。制御中この関数は制御ループとして繰り返し実行されることになります。

SimpleControllerを継承したクラスを定義したら、そのファクトリ関数を定義しておく必要があります。これは以下のようにマクロを用いて記述すればOKです。 ::

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SR1MinimumController)

これにより、このソースからコンパイルされた共有（ダイナミックリンク）ライブラリのファイルが、実際のコントローラとしてシンプルコントローラアイテムから利用可能となります。

.. note:: SimpleControllerクラスの詳細については、ソースアーカイブにおいてこのクラスを定義している "src/SimpleControllerPlugin/library/SimpleController.h" をご参照ください。また本節最後の :ref:`simulation-implement-controller-simple-controller-class-supplement` も参考にしてください。

.. _simulator-simple-controller-io:
.. _simulation-implement-controller-simple-controller-io:

IOオブジェクト
--------------

上記のinitialize関数の引数 io として渡されるSimpleControllerIO型のオブジェクトは、コントローラとロボットの間の入出力に必要な情報を扱うオブジェクトです。以下ではこのオブジェクトを「IOオブジェクト」と呼ぶことにします。

このクラスはControllerIOを継承したものになっています。ControllerIOクラスで定義されている関数としては以下のようなものがあり、コントローラの実装に用いることができます。

* **std::string controllerName() const**

 コントローラの名前を返します。

* **Body\* body()**

 入出力に使うためのBodyオブジェクトを返します。

* **std::string optionString() const**

 コントローラに与えられたオプション文字列を返します。

* **std::vector<std::string> options() const**

 オプション文字列についてスペースで区切って分解したものを返します。

* **std::ostream& os() const**

 コントローラからのメッセージを出力する出力ストリームを返します。

* **double timeStep() const**

 タイムステップを返します。単位は秒です。

* **double currentTime() const**

 現在時刻を返します。単位は秒で、シミュレーション開始時が時刻 0 となります。

.. _simulator-io-by-body-object:
.. _simulation-implement-controller-io-by-body-object:

Bodyオブジェクトを介した入出力
------------------------------

シンプルコントローラでは、「Bodyオブジェクト」を介して入出力を行います。Bodyオブジェクトは、 :doc:`../handling-models/bodymodel` のChoreonoid内部での表現で、C++で定義された「Bodyクラス」のインスタンスです。Bodyクラスはロボットのモデルとその状態を格納するためのデータ構造なので、入出力対象となる関節角度やトルク、センサの状態に関する値も格納できます。そこで、シンプルコントローラではこのBodyオブジェクトを介して入出力を行うこととしています。このためのBodyオブジェクトはIOオブジェクトのbody関数で取得することが可能です。

.. Bodyクラスはモデルに関する様々な情報と機能を有するので、入出力だけを行うためには実はオーバースペックなデータ構造です。シンプルコントローラでは実装の簡便さを優先してこれを用いていますが、入出力のインタフェースとしては通常このようなデータ構造は用いずに、特定の入出力要素のやりとりに最適化されたデータ構造を用いるのが一般的です。

Linkオブジェクト
~~~~~~~~~~~~~~~~

Bodyオブジェクトでは、モデルを構成する個々のパーツ（剛体）が「Linkクラス」のオブジェクトとして表現されており、関節に関する情報もこれに含まれるようになっています（ :ref:`model_structure` 参照）。LinkオブジェクトはBodyクラスの以下のような関数を用いて取得することができます。

* **int numJoints() const**

 モデルが有する関節の数を返します。

* **Link\* joint(int id)**

 関節番号(id)に対応するLinkオブジェクトを返します。
  
* **Link\* link(const std::string& name)**

 nameで指定した名前を有するLinkオブジェクトを返します。
 
取得したLinkオブジェクトに関して、以下のメンバ関数（状態変数）を用いて関節状態値へのアクセスが可能です。

* **double& q()**

 関節変位値への参照を返します。単位は[rad]または[m]です。

* **double& q_target()**

 関節変位の指令値を参照・設定するためのメンバです。単位は[rad]または[m]です。

* **double& dq()**

 関節速度値への参照を返します。単位は[rad/s]または[m/s]です。

* **double& dq_target()**

 関節速度の指令値を参照・設定するためのメンバです。単位は[rad/s]または[m/s]です。

* **double& ddq()**

 関節加速度値への参照を返します。単位は[rad/s^2]または[m/s^2]です。

* **double& u()**

 関節トルク（並進力）値を参照・設定するためのメンバです。単位は[N・m]または[N]です。

シンプルコントローラでは、各関節への入出力を基本的には上記の状態変数を用いて行います。すなわち、入力するときは対応する変数の値を読み込み、出力するときには対応する変数に値を書き込むことになります。（上記メンバ関数は対応する変数への参照を返すので、代入演算子を用いて値を代入することも可能です。）ただし、そのような入出力を行うにあたっては、後述する :ref:`simulation-implement-controller-enable-io` を予め行っておく必要があります。

実際にどの状態変数をアクチュエータへの指令値とするか、またどの状態変数を入力として読み込むかについては、アクチュエータのタイプや制御方式によって変わってきます。また、入出力に利用可能な状態変数は使用する :ref:`simulation_simulator_item` （物理エンジン）によっても変わってきます。

状態変数シンボル
~~~~~~~~~~~~~~~~

Choreonoidでは入出力の対象となる状態変数を識別するためのシンボルが定義されており、これを用いて指令値や入力値にどの状態変数を使用するかを指定します。シンボルはLinkクラスにてStateFlagという列挙型の要素として以下のように定義されています。（外部からはLinkクラスのスコープ解決演算子 Link:: を付けてアクセスできます。）

.. list-table:: **Link::StateFlag列挙型のシンボル**
 :widths: 20,50,30
 :header-rows: 1

 * - シンボル
   - 内容
   - 対応する状態変数
 * - **StateNone**
   - 該当する状態なし。
   - 
 * - **JointEffort**
   - 関節に加わるトルク（回転関節の場合）または力（直動関節の場合）
   - Link::u()
 * - **JointTorque**
   - JointEffortと同じ。回転関節向けに記述を分かりやすくするために定義されている。
   - 同上
 * - **JointForce**
   - JointEffortと同じ。直動関節向けに記述を分かりやすくするために定義されている。
   - 同上
 * - **JointDisplacement**
   - 関節変位（関節角度や関節並進位置）
   - Link::q() (現在値）またはLink::q_target() (指令値）
 * - **JointAngle**
   - JointDisplacementと同じ。対応する変位が関節角度である場合に記述をより分かりやすくするために定義されている。
   - 同上
 * - **JointVelocity**
   - 関節の速度成分。回転関節の角速度または直動関節の変位速度に該当。
   - Link::dq() (現在地）またはLink::dq_target() (指令値）
 * - **JointAcceleration**
   - 関節の加速度成分。回転関節の角加速度または直動関節の変位加速度に該当。
   - Link::ddq()
 * - **LinkPosition**
   - リンク位置。デカルト空間におけるリンク座標フレームの6自由度の位置・姿勢に該当。
   - Link::T()
 * - **LinkTwist**
   - リンク速度。リンク座標フレームの並進速度・角速度。
   - Link::v() (並進速度）、Link::w() (角速度）
 * - **LinkAcceleration**
   - リンク加速度。リンク座標フレームの並進加速度・角加速度。
   - Link::dv() (並進加速度）、Link::dw() (角加速度）


複数の要素を組み合わせることも可能です。その場合は複数シンボルをビット演算子の '|' で列挙します。例えば、 ::

 JointDisplacement | JointVelocity

と指定することで、関節変位と関節速度の両方を指定することができます。

.. note:: Choreonoid 1.7までのシンボルは大文字とアンダースコアを組み合わせた "JOINT_EFFORT" といった形式でしたが、Choreonoid 1.8からは上記の形式のシンボルとなりました。古いシンボルもしばらくは使用できますが、今後は新しいシンボルを使用するようにしてください。

.. _simulation-implement-controller-actuation-mode:

アクチュエーションモード
~~~~~~~~~~~~~~~~~~~~~~

コントローラから各リンク／関節への出力に関わる概念として、「アクチュエーションモード」があります。これはどの状態変数を制御指令値として使用するかを決めるものです。モードの指定には上記のStateFlag列挙型のシンボルを用います。

基本的な関節指令値に対応するモードは以下のようになります。


.. list-table:: **基本的なアクチュエーションモード**
 :widths: 20,60,20
 :header-rows: 1

 * - モード
   - 内容
   - 状態変数
 * - **StateNone**
   - 駆動なし。関節はフリーの状態となる。
   - 
 * - **JointEffort**
   - 関節を駆動する力／トルクを指令値とする。
   - Link::u()
 * - **JointDisplacement**
   - 関節変位を指令値とする。
   - Link::q_target()
 * - **JointVelocity**
   - 関節の角速度や変位速度を指令値とする。
   - Link::dq_target()

アクチュエーションモードは、Linkクラスの以下の関数を用いて参照・設定します。

* **void setActuationMode(int mode)**

 アクチュエーションモードを設定します。モード値modeはLink::StateFlagのシンボルで指定します。複数のシンボルをビット和として組み合わせて指定することも可能です。

* **int actuationMode() const**

 現在設定されているアクチュエーションモードを返します。値は通常Link::StateFlagの要素ひとつとなりますが、複数要素の組み合わせ（ビット集合）の場合もあります。


.. _simulation-implement-controller-enable-io:

入出力の有効化
~~~~~~~~~~~~~~

コントローラからどの状態変数の入出力を行うかについては、IOオブジェクトを用いて設定します。SimpleControllerIOクラスにはこれを行うための以下の関数が定義されています。

* **void enableInput(Link\* link)**

 指定したリンク（関節）が有する状態量のコントローラへの入力を有効にします。リンクに対して設定されているアクチュエーションモードに対して適切な状態量が入力対象となります。例えばアクチュエーションモードとしてJointEffortが設定されている場合は、関節変位の現在値であるLink::q()が入力対象となります。これはPD制御を行うために必要となるからです。

* **void enableInput(Link\* link, int stateFlags)**

 指定したリンク（関節）が有する状態量のうちstateFlagsで指定したものをコントローラに入力するようにします。stateFlagsはLink::StateFlagシンボルの論理和で指定します。入力したい値が明確な場合はこの関数を使用するのがよいです。

* **void enableOutput(Link\* link)**

 指定したリンク(関節）に対する出力を有効にします。リンクに対して設定されているアクチュエーションモードに対応する状態変数が出力対象となります。例えばアクチュエーションモードとしてJointEffortが設定されている場合は、関節トルク／力に対応するLink::u()が出力対象となります。

* **void enableOutput(Link\* link, int stateFlags)**

 指定したリンク(関節）に対する出力を有効にします。出力する状態変数はstateFlagsにLink::StateFlagのシンボルを指定することで行います。

* **void enableIO(Link\* link)**

 指定したリンクの入出力を有効にします。リンクに対して設定されているアクチュエーションモードに対して適切な状態量が入出力対象となります。

.. note:: SimpleControllerIO には setLinkInput、setJointInput、setLinkOutput、setJointOutput といった関数も定義されています。これらはChoroenoid 1.5以前のバージョンで使われていた関数であり、バージョン1.6以降ではこれらの関数に代わるものとして上記の enableIO、enableInput、enableOutput 関数を導入されており、今後はそちらの関数を使うようにしてください。

   
実際に利用可能なアクチュエーションモードは、シミュレータアイテム（≒物理エンジン）のタイプや設定によって変わってきます。ほとんどのシミュレータアイテムではJOINT_EFFORTに対応しており、これとJOINT_DISPLACEMENTの入力を組み合わせることで、PD制御等を行うことが可能です。

Linkオブジェクトに設定されているアクチュエーションモードに対して、入出力対象は通常以下のようになります。

.. list-table::
 :widths: 50,25,25
 :header-rows: 1

 * - アクチュエーションモード
   - 入力
   - 出力
 * - JointEffort
   - Link::q()
   - Link::u()
 * - JointDisplacement_DISPLACEMENT
   - なし
   - Link::q_target()
 * - JointVelocity
   - Link::q()
   - Link::dq_target()

.. note:: **LinkPosition** を指定することで、３次元空間中のリンクの位置と姿勢を直接入出力の対象とすることも可能です。これについては後ほど :ref:`simulation-implement-controller-link-position` にて解説します。


初期化処理
----------

SimpleController継承クラスのinitialize関数では、コントローラの初期化を行います。

サンプルでは、まず ::

 ioBody = io->body();

によって、入出力用のBodyオブジェクトを取得し、メンバ変数ioBodyに格納しています。これにより、このオブジェクトをコントローラの他の関数内でも使えるようにしています。

同様に、制御計算で必要となるタイムステップ（デルタタイム）値について、 ::

 dt = io->timeStep();

によって値をdtというメンバ変数に格納しています。

次に、以下のfor文でロボットの全関節に対してループを回して初期化の処理を行っています。 ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     ...
 }

まずこのループの中の ::

 Link* joint = ioBody->joint(i);

によってi番目の関節に対応するリンクオブジェクトを取得し、変数jointに設定しています。

そして ::

 joint->setActuationMode(Link::JointTorque);

によって、この関節に対してアクチュエーションモードの設定を行っています。ここでは Link::JointTorque を指定することで、関節トルクを指令値としています。また、 ::

 io->enableIO(joint);

とすることで、この関節に対する入出力を有効化しています。アクチュエーションモードに JointTorque が設定されているため、出力は関節トルク、入力は関節角度となります。これによってPD制御を行います。

次に ::

 qref.push_back(joint->q());

によってロボットの初期状態における関節角度をベクタ変数qrefに格納しています。こちらもPD制御で用います。ここで各関節に対するforループを終了します。

次に ::

 qold = qref;

によってベクタ変数qoldをqrefと同じ値で初期化しています。これはPD制御において1ステップ前の関節角度を参照するための変数となります。

最後に、initialize関数の戻り値としてtrueを返すことで、初期化に成功したことをシミュレータに伝えます。

制御ループ
----------

SimpleController継承クラスでは、そのcontrol関数に制御ループを記述します。

初期化の時と同様に、以下のfor文 ::

 for(int i=0; i < ioBody->numJoints(); ++i){
     Link* joint = ioBody->joint(i);
     ...
 }

により、全ての関節に対して制御計算を行っています。この中身が各関節に対する処理コードです。

まず、 現在の関節角度の入力を行います。 ::

 double q = joint->q();

PD制御によって関節トルクの指令値を計算します。まず、制御ループの前回の関節角度との差分から、関節角速度を算出します。 ::

 double dq = (q - qold[i]) / dt;

制御の目標は初期姿勢の維持ですので、関節角度は初期関節角度、角速度は0（静止状態）を目標として、トルク指令値を計算します。 ::

 double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];

ソースの冒頭で設定したpgain, dgainの配列から、各関節に関するゲイン値を取り出しています。ゲイン値についてはモデルごとに調整が必要ですが、その方法についてはここでは割愛します。

次回の計算用に、関節角度をqold変数に保存しておきます。 ::

 qold[i] = q;

計算したトルク指令値を出力します。これにより、関節が初期関節角度を維持するように制御されます。 ::

 joint->u() = u;

以上が全ての関節に対して適用されることにより、ロボット全体の姿勢も維持されることになります。

最後にこのcontrol関数がtrueを返すことで、制御が継続している旨をシミュレータに伝えています。これにより、control関数が繰り返し呼ばれることになります。

.. _simulation-device:

デバイスに対する入出力
----------------------

デバイスとは
~~~~~~~~~~~~

これまでは入出力の対象として、関節角度や関節トルクといった関節に関わる状態量への入出力を扱いました。一方で、関節とは独立した入出力要素もあります。Choreonoidではそれらを「デバイス」として定義しており、Bodyモデルの構成要素となります。

.. 以上の例では関節角度を入力し、関節トルクを出力しました。これは関節に備え付けられたエンコーダ、アクチュエータといったデバイスを対象に入出力を行っていると考えることができます。

.. そのように入出力の対象となるデバイスは他にも様々なものが存在し得ます。例えば、エンコーダと同様に、センサとして主に入力の対象となるものとして、

.. 一般的にロボットは関節エンコーダ、アクチュエータ以外にも多様なデバイスを備えています。

デバイスの例としては、まず

* 力センサ、加速度センサ、角速度センサ（レートジャイロ）
* カメラ、レーザーレンジセンサ

といったデバイスが挙げられます。これらはセンサとして主に入力の対象となるものです。

.. が、カメラのズーム変更等、操作指令を出力したい場合もあります。
.. 主に出力の対象となるものとして、

また、主に出力の対象として外界に働きかけるものとして、

* ライト
* スピーカ
* ディスプレイ

といったデバイスもあり得ます。(スピーカ、ディスプレイは例として挙げただけでまだ実装されていません。）

実際のコントローラ開発においては、これらの多様なデバイスに対しても入出力を行う必要が出てきます。これを行うためには、

* モデルにおいてデバイスがどのように定義されているか
* 使用するコントローラ形式において所定のデバイスにどのようにアクセスするか

を把握している必要があります。

.. _simulation-device-object:

デバイスオブジェクト
~~~~~~~~~~~~~~~~~~~~

Choreonoidのボディモデルにおいて、デバイスの情報は「Deviceオブジェクト」として表現されます。これは「Deviceクラス」を継承した型のインスタンスで、デバイスの種類ごとにそれぞれ対応する型が定義されています。標準で定義されている主なデバイス型は以下のようになっています。

.. code-block:: text

 + Device
   + ForceSensor (力センサ)
   + RateGyroSensor (角速度センサ)
   + AccelerationSensor (加速度センサ)
   + Imu（慣性計測ユニット：加速度センサと角速度センサの機能を併せ持つ）
   + Camera (カメラ）
     + RangeCamera (カメラ＋距離画像センサ）
   + RangeSensor (レンジセンサ）
   + Light
     + PointLight (点光源ライト）
     + SpotLight (スポットライト）

※ IMUはC++ソースコードにおけるクラス名としては "Imu" となります。

ロボットに搭載されているデバイスの情報は、通常はモデルファイルにおいて記述します。標準形式のモデルファイルでは、 :doc:`../handling-models/modelfile/yaml-reference` の :ref:`body-file-reference-devices` を記述します。

シンプルコントローラでは、Body、Linkオブジェクトと同様に、デバイスに対してもChoreonoidの内部表現であるDeviceオブジェクトをそのまま用いて入出力を行います。

本節で使用しているSR1モデルが有するデバイスオブジェクトは以下のようになっています。

.. tabularcolumns:: |p{3.5cm}|p{3.5cm}|p{6.0}|

.. list-table::
 :widths: 30,30,40
 :header-rows: 1

 * - 名前
   - デバイスの型
   - 内容
 * - WaistAccelSensor
   - AccelerationSensor
   - 腰リンクに搭載された加速度センサ
 * - WaistGyro
   - RateGyroSensor
   - 腰リンクに搭載されたジャイロ
 * - WaistIMU
   - Imu
   - 腰リンクに搭載された慣性計測ユニット
 * - LeftCamera
   - RangeCamera
   - 左目に対応する距離画像センサ
 * - RightCamera
   - RangeCamera
   - 右目に対応する距離画像センサ
 * - LeftAnkleForceSensor
   - ForceSensor
   - 左足首に搭載された力センサ
 * - RightAnkleForceSensor
   - ForceSensor
   - 右足首に搭載された力センサ

.. _simulation-obtain-device-object:

デバイスオブジェクトの取得
~~~~~~~~~~~~~~~~~~~~~~~~~~

DeviceオブジェクトはBodyオブジェクトから以下のメンバ関数を用いて取得できます。

* **int numDevices() const**

 デバイスの数を返します。

* **Device\* device(int i) const**

 i番目のデバイスを返します。デバイスの順番はモデルファイル中の記述順になります。

* **const DeviceList<>& devices() const**

 全デバイスのリストを返します。

* **template<class DeviceType> DeviceList<DeviceType> devices() const**

 指定した型のデバイスのリストを返します。

* **template<class DeviceType> DeviceType\* findDevice(const std::string& name) const**

 指定した型と名前を有するデバイスがあればそれを返します。

特定の型のデバイスを取得するには、テンプレートクラスDeviceListを使用します。DeviceListは指定した型のデバイスオブジェクトを格納する配列であり、そのコンストラクタや抽出オペレータ(<<)等を用いて、他の型も含むDeviceListから対応する型のみを抽出できます。例えばBodyオブジェクト"ioBody"の保有する力センサを取得したい場合は、 ::

 DeviceList<ForceSensor> forceSensors(ioBody->devices());

としてもよいですし、既存のリストに対して ::

 forceSensors << ioBody->devices();

として追加することもできます。

DeviceListはstd::vectorと同様の関数や演算子を備えており、例えば ::

 for(size_t i=0; i < forceSensors.size(); ++i){
     ForceSensor* forceSensor = forceSensor[i];
     ...
 }

といったかたちで各オブジェクトにアクセスできます。

findDevice関数を用いることで、型と名前でデバイスを特定して取得することもできます。例えばSR1モデルは腰リンクに搭載された "WaistAccelSensor" という名前の加速度センサを有しています。これを取得するには、Bodyオブジェクトに対して ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");

などとすればOKです。

.. _simulation-implement-controller-device-io:

デバイスの入出力方法
~~~~~~~~~~~~~~~~~~~~

Deviceオブジェクトを介した入出力は、以下のようにして行います。

* **入力**

 シンプルコントローラのIOオブジェクトに対して関数

 * **void enableInput(Device\* device)**

 を実行し、デバイスへの入力を有効にしておく。その上で、対応するDeviceオブジェクトのメンバ関数を用いて値を取得する。

* **出力**

 対応するDeviceオブジェクトのメンバ関数を用いて値を設定した後、Deviceオブジェクトの関数

 * **void notifyStateChange()**

 を実行し、デバイスの状態の更新をシミュレータに伝える。

これらを行うためには、使用するデバイスのクラス定義を知っている必要があります。例えば加速度センサのクラスである"AccelerationSensor"に関しては、その状態にアクセスするための"dv()"というメンバ関数があります。これは加速度をVector3型の3次元ベクトルで返します。

SR1モデルの加速度センサの入力は以下のような流れになります。まずコントローラの initialize 関数で ::

 AccelerationSensor* accelSensor =
     ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
 io->enableInput(accelSensor);

などとして、accelSensorへの入力を有効化しておきます。そして、control関数内で加速度センサの値を参照した箇所で ::

 Vector3 dv = waistAccelSensor->dv();

といったかたちで取得することができます。

同様に、ForceSensorやRateGyroSensor、Imuに関しても、該当するメンバ関数を用いて状態の入力を行うことが可能です。

カメラやレンジセンサ等の視覚センサを使用する際には、そのための準備が必要になります。これについては :doc:`vision-simulation` で解説します。

デバイスへの出力については、ライトのオン・オフを行う "TankJoystickLight.cnoid" というサンプルを参考にしてください。

.. 立たせるのもやめて、倒れるシミュレーションにして、加速度が一定値以上のときだけ表示するようなサンプルを作る？


.. * **void enableInput(Device\* device)**
..
.. deviceで指定したデバイスの状態やデータのコントローラへの入力を有効にします。

.. _simulation-implement-controller-link-position:

リンク位置姿勢の入出力
----------------------

コントローラの入出力の対象としては、他にリンクの位置姿勢があります。ここで言う位置姿勢というのは関節角度のことではなく、リンクという剛体そのもののグローバル座標における位置と姿勢を意味します。この値は通常ロボット実機に対して入出力を行うことはできません。空間中に固定されていないロボットに対して、あるリンクの正確な位置と姿勢を知ることは（かなり性能のよいモーションキャプチャでも無ければ）困難ですし、あるリンクの位置姿勢をコントローラからの出力で直接変えることは物理的に不可能です。しかしながら、シミュレーションにおいてはそのようなことも可能となるため、シミュレーション限定での利用を想定してこの値の入出力機能も備えています。

これを行うためには、状態量のシンボルとして **LinkPosition** を指定します。出力を行う場合はLinkオブジェクトのsetActuationMode関数に **Link::LinkPosition** を指定し、IOオブジェクトのenableIO関数やenableOutput関数を用いて出力を有効化します。入力についても同様にIOオブジェクトのenableInput関数で **Link::LinkPosition** を指定すればOKです。

Linkオブジェクトにおいて、その位置姿勢はIsometry3型の値として格納されています。これはChoreonoidの実装に用いているEigenという行列・ベクトルライブラリの"Transform"型をカスタマイズしたもので、基本的には３次元の同次座標変換行列を格納したものとなっています。この値にはLinkクラスの以下のような関数を用いてアクセスできます。

* **Isometry3& T(), Isometry3& position()**

 位置姿勢に対応するIsometry3値への参照を返します。

* **Isometry3::TranslationPart translation()**

 位置成分に対応する３次元ベクトルを返します。

* **void setTranslation(const Eigen::MatrixBase<Derived>& p)**
   
 位置成分を設定します。引数はEigenの3次元ベクトル相当の型が使えます。

* **Isometry3::LinearPart rotation()**

 姿勢（回転）成分に対応する3x3行列を返します。

* **setRotation(const Eigen::MatrixBase<Derived>& R)**

 姿勢（回転）成分を設定します。引数はEigenの3x3行列相当の型が使えます。

* **setRotation(const Eigen::AngleAxis<T>& a)**

 姿勢（回転）成分を設定します。引数は回転軸と回転角度で回転を表現するEigenのAngleAxis型になります。

例として、ルートリンクの位置を入力する場合は、まずコントローラのinitialize関数にて ::

 io->enableInput(io->body()->rootLink(), LinkPosition);

などとします。そして control 関数にて ::

 Position T = io->body()->rootLink()->position();
 Vector3 p = T.translation();
 Matrix3 R = T.rotation();

などとすることにより、ルートリンクの位置姿勢を取得できます。

リンク位置姿勢の出力については、これをサポートしたシミュレータが必要で、特殊な利用形態となります。例えばAISTシミュレータアイテムでは、「動力学モード」を「運動学」にすると、シミュレーションにおいて動力学計算を行わず、与えた位置姿勢を再現するだけのモードとなります。この場合、ロボットのルートリンクの位置姿勢を出力することで、ルートリンクがその位置姿勢へ移動します。また、関節角も出力しておけば、ルートリンクからの順運動学の結果となる姿勢が再現されます。

.. note:: Choreonoid 1.7以前のバージョンでは位置姿勢を格納する型名に "Position" を使用していました。この型の内容は上記のIsometry3とほぼ同様で、相互に変換もできますが、今後はIsometry3を使うようにしてください。

.. _simulation-implement-controller-link-velocity-and-acceleration:

リンク速度・加速度の入力
------------------------

剛体としてのリンクの状態量として、その速度や加速度も取得することができます。その場合は状態量のシンボルとして **LinkTwist** と **LinkAcceleration** をそれぞれ使用します。これらの状態量の入力はLinkPositionと同様です。例えばIOオブジェクトから取得されるあるリンクオブジェクトが変数linkに格納されているとして、速度を取得する場合は、まずコントローラのinitialize関数にて::

 io->enableInput(link, LinkTwist);

としておき、control関数にて ::

 Vector3 dv = link->v(); // 並進速度
 Vector3 dw = link->w(); // 角速度

というかたちで取得できます。
それぞれグローバル座標における速度と角速度になります。

加速度の場合も、状態量シンボルLinkAccelerationで設定し、リンクの状態量関数dv()、dw()を用いることで、加速度と角加速度を取得できます。

もちろんこれらの状態量を同時に入力することも可能です。例えば ::

 io->enableInput(link, LinkPosition | LinkTwist | LinkAcceleration);

としておけば、リンクオブジェクトlinkに対して、位置、速度、加速度の全ての状態量を取得できます。

.. note:: リンクの角速度と並進加速度については、それぞれレートジャイロ、加速度センサで取得することも可能です。実機のロボットではむしろそれらのセンサを介してしか状態量を得ることができないのが普通です。本節で示した手法はシミュレータ特有の状態量取得方法となります。

  それらのセンサを利用するには、まずモデルにセンサを定義し、その上で :ref:`simulation-implement-controller-device-io` で示した手順でセンサからの入力を行います。レートジャイロのクラス名は "RateGyroSensor" で、加速度センサのクラス名は "AccelerationSensor" となります。注意点として、センサから得られる状態量の座標系はセンサのローカル座標となります。また加速度センサについては重力加速度の成分も含む値となります。

.. _simulation-implement-controller-simple-controller-class-supplement:

補足: SimpleControllerクラスの定義について
------------------------------------------

:ref:`simulation-implement-controller-simple-controller-class` ではこのクラスのvirtual関数として initialize と control の2つを紹介しました。SimpleControllerはこれ以外にも以下に示すvirtual関数を備えていて、それぞれオーバーライドして処理を記述できるようになっています。

* **virtual bool configure(SimpleControllerConfig\* config)**

 こちらもコントローラの初期化を行うための関数ですが、initialize関数とは実行のタイミングが異なります。initialize関数はシミュレーションを開始するタイミングで実行されますが、こちらの関数はそれ以前に、コントローラがプロジェクトに導入されて（アイテムツリーに組み込まれて）特定のモデルと関連付けられた時点で実行されます。シミュレーション開始前に実行しておきたい処理がある場合は、こちらに記述するようにします。引数configを通して初期化に関わる情報を取得できます。

* **virtual bool start()**

 こちらもコントローラの初期化を行うための関数ですが、タイミング的にはinitialize関数よりも後に実行される関数です。initialize関数も含めてシミュレーション全体の初期化がひととおり完了し、コントローラが稼働を開始するタイミングで実行されます。

* **virtual void stop()**

 こちらはシミュレーション停止時に実行される関数です。

* **virtual void unconfigure()**

 こちらはconfigureと対をなすもので、コントローラが削除されたり対象モデルから切り離されるなどして、コントローラが無効となるタイミングで実行されます。

configure関数の引数として与えられるconfigオブジェクトは :ref:`simulation-implement-controller-simple-controller-io` と同様のもので、対象モデルの情報を得るための関数

* **Body\* body()**

をはじめとして、IOオブジェクトと同様のメンバを有しています。

ただしconfigから得られるbodyオブジェクトはIOオブジェクトから得られるものとは異なることにご注意ください。IOオブジェクトから得られるものはシミュレーション中のBodyオブジェクトとの入出力を行うもので、シミュレーション実行時に生成されるものです。一方、configから得られるものは、Bodyアイテムが元々有するBodyオブジェクトで、シミュレーションを開始する前から存在するものです。


その他のサンプル
----------------
 
Choreonoidでは、SR1MinimumController以外にも様々なコントローラのサンプルを用意しています。それらを用いたプロジェクトが :ref:`basics_sample_project` に挙げてありますので、参考にしてください。
