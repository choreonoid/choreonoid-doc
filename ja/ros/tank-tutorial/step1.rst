ステップ1: JoyトピックのSubscribeによるTankモデルの制御
=======================================================

ステップ1では、ロボットへの指令をROSを用いた通信によってロボットのコントローラに伝え、それに基づいてロボットの制御を行う方法について説明します。具体的には、ジョイスティックの状態をJoyトピックとして送信し、これをコントローラ側で受信することで、ジョイスティックによるTankモデルの操作を行えるようにします。

.. contents::
   :local:

.. _ros_tank_tutorial_invoke_choreonoid_node:

Choreonoidノードの起動
----------------------

前節で説明した準備が完了しましたら、Choreonoidを起動してください。

この際にChoreonoidをROSのノードとして起動する必要があることにご注意ください。 :doc:`../run-choreonoid` で説明しているように、以下のコマンドでROSのChoreonoidノードを起動できます。 ::

 rosrun choreonoid_ros choreonoid

起動に成功すると、Choreonoidのメインウィンドウが表示されます。


シミュレーション用プロジェクトの構築
------------------------------------

Choreonoid上にシミュレーション対象となるモデルを読み込んで、プロジェクトを構築しましょう。このチュートリアルではロボットに該当するモデルとして戦車風の :ref:`tank_model` を使用します。これを用いたプロジェクトの作成については、 :doc:`../../simulation/tank-tutorial/index` の :doc:`../../simulation/tank-tutorial/step1` でも行っていますので、そちらを参照の上まずは同じプロジェクトを構築してください。

そこでは環境モデルとしてシンプルな床のモデルを使用していますが、同じチュートリアルの :ref:`tank_tutorial_use_labo_model` で使用しているプラントモデルを使用してもかまいません。そちらの方が、本チュートリアルの後半でカメラ画像の通信を行う際に、よりそれらしい画像を得ることができます。（ただしモデルは重くなりますので、PC環境によってはシンプルな床のモデルの方が扱いやすいかもしれません。）Githubに公開している本チュートリアルのリポジトリでは、プラントモデルを使用しています。

プロジェクトが構築できたら、メインメニューの「ファイル」-「名前を付けてプロジェクトを保存」を使用して、プロジェクトファイルに保存してください。保存先は本チュートリアル用に作成したディレクトリ内にさらに "project" というサブディレクトリを作成し、そこに "step1.cnoid" という名前で格納するようにしましょう。

これにより、本チュートリアル用パッケージにおいて以下のようにディレクトリとファイルが追加されることになります。 ::

 + choreonoid_ros_tank_tutorial
   + project
     + step1.cnoid

Choreonoid終了後に再度プロジェクトを読み込む場合は、:ref:`ros_tank_tutorial_invoke_choreonoid_node` で用いたコマンドにプロジェクトファイル名をオプションとして付与します。例えば、 ::

 roscd choreonoid_ros_tank_tutorial

としてチュートリアル用ディレクトリに移動し、そこで ::

 rosrun choreonoid_ros choreonoid project/step1.cnoid

などとすることにより、step1のプロジェクト込みでChoreonoidノードを起動することができます。

以下で :ref:`ros_tank_tutorial_introduce_launch_file` を行うまでは、この方法でchoreonoidの起動とプロジェクトの読み込みを行うとよいかと思います。

プロジェクトを構築できたら、シミュレーションを開始してみてください。すると、:doc:`../../simulation/tank-tutorial/index` の :ref:`tank-tutorial-step1-start-simulation` でも説明しているように、砲身部分は重力で落下してしまいますし、車体も特に動くことはありません。これはTankがただ存在するだけで、それを制御するためのコントローラが導入されていないからで、当然と言えば当然の結果です。

Step1では、このTankモデルを自由に操作できるようにすることを目標とします。


ゲームパッドの準備
------------------

Tankを自由に操作する手段として、本チュートリアルではジョイスティックと呼ばれる入力デバイスを使用することにします。ジョイスティックにはいろいろなタイプのものがありますが、この手のロボットの操作には、ゲームパッドと呼ばれるものがよいでしょう。チュートリアルを進めるために、適当なゲームパッドを用意してください。USBで接続するタイプのものであれば、大抵のものは使えるかと思います。ただし後ほど説明する :ref:`ros_tank_tutorial_use_choreonoid_joy` と共に、これに対応したゲームパッドを使用することで、ロボットの操作をスムーズに行うことができます。対応しているゲームパッドについては、 :doc:`../../simulation/tank-tutorial/index` の :ref:`simulation-tank-tutorial-gamepad` を参照してください。

ゲームパッドを用意できたら、予めPCに接続しておきます。


Joyノードによるゲームパッドの状態のPublish
------------------------------------------

本チュートリアルのテーマはROSの活用にありますので、ゲームパッドの状態もROSの機能を用いてやりとりすることにします。そのようにすることで、ROSに対応している様々なデバイスが使用できたり、リモートホスト間の通信によって遠隔操作を行ったりすることが可能となります。ここではまずゲームパッドの状態を送信するための準備を行います。

JoyトピックのPublish
~~~~~~~~~~~~~~~~~~~~

ROSでは様々なデータを「メッセージ」として定義して、それを「トピック」として送信することが可能です。トピックの送信はROSでは「Publish（出版）」と呼ばれ、Publishされたトピックの受信は「Subscribe（購読）」と呼ばれます。これは「Publish-Subscribeモデル」というソフトウェア設計モデルに基づくもので、データは受け手を特定せずにPublishされ、それをどこからでもSubscribeすることができるというものです。この仕組みはROSユーザはご存知かと思いますが、そうでない場合はROSの解説を参照するようにしてください。

本チュートリアルでは、ゲームパッドの状態をROSトピックとしてPublishし、それをロボットのコントローラからSubscribeします。これを実現するために、まずゲームパッドの状態をPublishするプログラムが必要となります。そのようなプログラムは「ROSノード」と呼ばれます。実はゲームパッド（ジョイスティック）の状態をPublishするROSノードとして、「Joyノード」というものがROSの標準パッケージとして用意されていますので、まずはそれを試してみることにします。

以下ではJoyノードの起動方法とともに、ROSのトピックやメッセージが具体的にどのようなものかについて理解していただけるよう説明します。ROSの基本的な事柄を既に習得されている方は、以下は読み飛ばして次の :ref:`ros_tank_tutorial_use_choreonoid_joy` まで進んでいただいて結構です。

Joyノードのインストールと起動
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

まずJoyノードを利用できるようにするため、対応する「Joyパッケージ」をインストールします。 ::

 sudo apt install ros-melodic-joy

これはUbuntu 18.04 (Melodic) の場合のパッケージ名です。Ubuntu 16.04 (Kinetic) の場合は以下のコマンドでインストールできます。 ::

 sudo apt install ros-kinetic-joy

Joyパッケージのインストールに成功していれば、以下のコマンドでJoyノードを起動できます。 ::

 rosrun joy joy_node

ただしこれを実行する前にジョイスティックをPCに接続しておいてください。ジョイスティックは一般的なUSB接続のものでしたら使用できるかと思います。

Joyトピックの確認
~~~~~~~~~~~~~~~~~

Joyノードが起動しジョイスティックの検出に成功すると、ジョイスティックの軸やボタンの状態をPublishするトピックが生成されます。これは以下のコマンドで確認できます。 ::

 rostopic list

このコマンドにより、現在システムで利用可能なトピックの一覧が表示されます。ここに ::

 /joy

という表示があるか確認してください。これがJoyノードがPublishしているトピックで、/joyという名前がつけられています。トピックの名前はファイルシステムと同様に階層的に管理できるようになっており、最初のスラッシュは最上位階層に定義された名前であることを示しています。

このjoyトピックがどのようなものかについて確認してみましょう。以下のコマンドを実行してみてください。 ::

 rostopic info /joy

これにより、/joyに対応するトピックの情報が表示されます。これは以下のようになるかと思います。 ::

 Type: sensor_msgs/Joy
 
 Publishers: 
  * /joy_node (http://hostname:34541/)
 
 Subscribers: None

ここで Type の項目に表示されている "sensor_msgs/Joy" というのがこのトピックのメッセージ型です。これはこのトピックがどのようなデータになるかを表しています。他に、このトピックをPublishしているのが /joy_node というノードであることや、現時点ではこれをSubscribeしているノードが存在しないことが表示されています。

ではメッセージ型の内容を確認してみましょう。以下のコマンドを実行してください。 ::

 rosmsg show sensor_msgs/Joy

すると以下のように表示されるかと思います。 ::

 std_msgs/Header header
   uint32 seq
   time stamp
   string frame_id
 float32[] axes
 int32[] buttons

これはメッセージ型 "sensor_msgs/Joy" のデータ構造を表しています。具体的には、"axes" は32ビット浮動小数点型の配列としてジョイスティックの各軸の倒し具合が格納されますし、"buttons" には32ビット整数型で各ボタンの状態（押しているかどうか）が格納されます。他には "header" 以下のこのメッセージのタイムスタンプやID値などが格納されます。これらはROSのコーディングを行う各言語において対応する型（C++の std::vector<float> など） にマッピングされ、アクセスすることが可能となります。

実際にPublishされているメッセージの内容を確認してみましょう。まず以下のコマンドを実行してください。 ::

 rostopic echo /joy

これは指定したトピックの内容をテキストにしてコンソールに表示してくれるコマンドです。これを実行後に、ゲームパッドの軸を操作したり、ボタンを押したりしてみてください。するとコンソールに以下のような出力がされるはずです。 ::

 header: 
   seq: 1
   stamp: 
     secs: 1585302374
     nsecs: 941266549
   frame_id: ''
 axes: [0.0, 0.03420161083340645, 0.0, 0.0, 0.0, 0.0]
 buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

先程のメッセージ型に対応するかたちで、各メンバの現在の値が表示されています。ここでは例えば "buttons" の2番目の要素が "1" となっているので、2番目のボタンが押されていることが分かります。

このコマンドを終了させるのは、Ctrl + C を押してください。もし上記のような表示が出ない場合は、ゲームパッドが正しく接続されていない可能性があります。本チュートリアルを進めるためには、まずこれが正常に動作するようにしてください。

.. _ros_tank_tutorial_use_choreonoid_joy:

Choreonoid版Joyノードの利用
---------------------------

前節で紹介したJoyノードによってゲームパッドの状態をPublishできますが、本チュートリアルではこれに代わって「Choreonoid版Joyノード」を使用したいと思います。これは :ref:`ros_tank_tutorial_package_setup` で導入した "choreonoid_joy" パッケージが対応しており、以下のコマンドで起動できます。 ::

 rosrun choreonoid_joy node

機能的には標準のJoyノードとほぼ同じなのですが、こちらはゲームパッドの軸やボタンのマッピングを標準化するという点が異なります。

これについて説明します。上記のようにゲームパッドの状態はJoyメッセージに格納されるのですが、そこの "axes" や "buttons" の配列にどのような順番で実際の軸やボタンが並んでいるかは、ゲームパッドの機種によって異なります。これはそもそも各ハードウェアデバイスがドライバを通して返す順番が異なっているからですが、ROS標準のJoyノードでは、その順番をそのまま axes や buttons に格納するようになっています。しかしそうすると、ゲームパッドの様々な機種を同じように使用することが困難となります。ゲームパッド自体は最近のものはどれも同じような軸やボタンを持っているのですが、それにもかかわらず、実際には同じような軸やボタンを操作しても、ロボットの動きが変わってしまうことになります。

そこでChoreonoidのJoyノードでは、軸やボタンに関して標準の並び（マッピング）というものを定義し、実際のゲームパッドの機種ごとにそのマッピングに変換してJoyメッセージに格納します。するとJoyメッセージの購読側では、その標準のマッピングを前提として読み込むだけで、ゲームパッドの様々な機種を同様に扱えるようになるというわけです。

チュートリアルにおいては、サンプルのプログラムはなるべくシンプルなことが望ましく、その上で同じように操作できることも必要です。そこで本チュートリアルではJoyトピックのPublishにChoreonoid版のJoyノードを使用することにしました。チュートリアルを進めるにあたっては、上記のコマンドでchoreonoid_joyノードを起動しておくようにしてください。動作確認は標準のJoyノードと同様に行っていただければOKです。

なお、様々な機種に対応できるように書いていますが、実際に対応しているのは :ref:`simulation-tank-tutorial-gamepad` に記載されている機種のみとなりますので、ご了承ください。それ以外の機種に対しても、JoyトピックはPublishされますが、マッピングの標準化はされないので、ROS標準のJoyノードと同じ出力となります。


コントローラのビルド
--------------------

ゲームパッドの状態がPublishされるようになったので、これを用いて、ゲームパッドによるTankモデルの操作を可能とするためのコントローラを導入したいと思います。以下で行うことは、本質的には :doc:`../../simulation/tank-tutorial/index` の :doc:`../../simulation/tank-tutorial/step2` と同様です。ただし、本チュートリアルではROSのcatkin環境においてコントローラをビルドし、使用できるようにしなければなりません。まずはそのビルド方法について説明します。

コントローラのソースコード
~~~~~~~~~~~~~~~~~~~~~~~~~~

まずはコントローラのソースコードを掲載します。このコントローラは :doc:`../../simulation/tank-tutorial/index` の :doc:`../../simulation/tank-tutorial/step2` と同様に、SimpleControllerを継承したものとなっています。SimpleController自体はROSとは独立したものですが、そこに単純にROSのコードを加えることで、ROSの機能を活用できるようになります。 ::

 #include <cnoid/SimpleController>
 #include <cnoid/Joystick>
 #include <ros/node_handle.h>
 #include <sensor_msgs/Joy.h>
 #include <mutex>
 
 using namespace std;
 using namespace cnoid;
 
 namespace {
 const int trackAxisID[]  = { Joystick::L_STICK_H_AXIS, Joystick::L_STICK_V_AXIS };
 const int turretAxisID[] = { Joystick::R_STICK_H_AXIS, Joystick::R_STICK_V_AXIS };
 }
 
 class JoyInputController : public SimpleController
 {
     unique_ptr<ros::NodeHandle> node;
     ros::Subscriber joystickSubscriber;
     sensor_msgs::Joy latestJoystickState;
     std::mutex joystickMutex;
     
     Link* trackL;
     Link* trackR;
     Link* turretJoint[2];
     double qref[2];
     double qprev[2];
     double dt;
 
 public:
     virtual bool configure(SimpleControllerConfig* config)
     {
         if(!ros::isInitialized()){
             config->os() << config->controllerName()
                          << " cannot be configured because ROS is not initialized." << endl;
             return false;
         }
         node.reset(new ros::NodeHandle);
         return true;
     }
 
     virtual bool initialize(SimpleControllerIO* io) override
     {
         ostream& os = io->os();
         Body* body = io->body();
         dt = io->timeStep();
 
         trackL = body->link("TRACK_L");
         trackR = body->link("TRACK_R");
         trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
         trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
         io->enableOutput(trackL);
         io->enableOutput(trackR);
 
         turretJoint[0] = body->link("TURRET_Y");
         turretJoint[1] = body->link("TURRET_P");
         for(int i=0; i < 2; ++i){
             Link* joint = turretJoint[i];
             qref[i] = qprev[i] = joint->q();
             joint->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
             io->enableIO(joint);
         }
 
         joystickSubscriber = node->subscribe("joy", 1, &JoyInputController::joystickCallback, this);
 
         return true;
     }
 
     void joystickCallback(const sensor_msgs::Joy& msg)
     {
         std::lock_guard<std::mutex> lock(joystickMutex);
         latestJoystickState = msg;
     }
 
     virtual bool control() override
     {
         sensor_msgs::Joy joystick;
         {
             std::lock_guard<std::mutex> lock(joystickMutex);
             joystick = latestJoystickState;
             joystick.axes.resize(10, 0.0f);
             joystick.buttons.resize(10, 0);
         }
             
         double pos[2];
         for(int i=0; i < 2; ++i){
             pos[i] = joystick.axes[trackAxisID[i]];
             if(fabs(pos[i]) < 0.2){
                 pos[i] = 0.0;
             }
         }
         // set the velocity of each tracks
         trackL->dq_target() = -2.0 * pos[1] + pos[0];
         trackR->dq_target() = -2.0 * pos[1] - pos[0];
 
         static const double P = 200.0;
         static const double D = 50.0;
 
         for(int i=0; i < 2; ++i){
             Link* joint = turretJoint[i];
             double pos = joystick.axes[turretAxisID[i]];
             if(fabs(pos) < 0.15){
                 pos = 0.0;
             }
             double q = joint->q();
             double dq = (q - qprev[i]) / dt;
             double dqref = 0.0;
             double deltaq = 0.002 * pos;
             qref[i] += deltaq;
             dqref = deltaq / dt;
             joint->u() = P * (qref[i] - q) + D * (dqref - dq);
             qprev[i] = q;
         }
 
         return true;
     }
 
     virtual void stop() override
     {
         joystickSubscriber.shutdown();
     }
 };
 
 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JoyInputController)


このソースコードは、パッケージディレクトリに "src" というサブディレクトリを作成し、そこに "JoyInputController.cpp" というファイル名で保存してください。すると、これまで追加したファイルの構成は以下のようになるかと思います。 ::

 + choreonoid_ros_tank_tutorial
   + project
     + step1.cnoid
   + src
     + JoyInputController.cpp


以下ではまずこのソースコードをビルドしてシミュレーションで動かす方法について解説し、その後ソースコードの内容について解説します。


CMakeLists.txtの編集
~~~~~~~~~~~~~~~~~~~~

:ref:`ros_tank_tutorial_edit_package_xml` では、Catkinのパッケージを構築するためにこのXMLファイルが必要なことを説明しました。実はパッケージの構築に必要なファイルとして、他に "CMakeLists.txt" というファイルもあります。これはビルドシステムのひとつであるCMakeのファイルで、パッケージにC++のソースコードが含まれる場合など、何らかのビルド処理が必要な場合に使用されます。

CMakeやCMakeLists.txtの詳細についてはCMakeのマニュアルなどを参照してください。CMakeは非常にポピュラーなツールであり、ROSでもChoreonoidでも元々使用されているものなので、その基本的な事柄は理解されているという前提で説明します。

CMakeLists.txtの雛形となるものは、 :ref:`ros_tank_tutorial_make_package` において自動で生成されており、プロジェクトディレクトリ直下に保存されています。そのファイルを編集して、以下と同じ内容になるようにします。 ::

 cmake_minimum_required(VERSION 3.5.0)
 project(choreonoid_ros_tank_tutorial)
 
 set(CHOREONOID_SKIP_QT_CONFIG true)

 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   image_transport
   choreonoid
   )
 
 catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)
 
 set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
 set(CMAKE_CXX_EXTENSIONS OFF)

 set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS ${CHOREONOID_COMPILE_DEFINITIONS})
 include_directories(
   ${catkin_INCLUDE_DIRS} 
   ${CHOREONOID_INCLUDE_DIRS}
   )
 link_directories(
   ${CHOREONOID_LIBRARY_DIRS}
   )
 
 add_subdirectory(src)

この内容について解説します。まず ::

 cmake_minimum_required(VERSION 3.5.0)

で、CMakeのバージョンが3.5.0以上であることを条件としています。現在最新のChoreonoid開発版では、内部で使用しているCMakeのコマンドの都合などで、最低限このバージョンが必要です。自動生成されたCMakeLists.txtではこれよりも低いバージョンが記述されている場合がありますが、その場合そのままではChoreonoid関連パッケージのビルドができないので、ここの記述が3.5.0以上になるようにしてください。なお、Ubuntuの16.04以降であれば標準でインストールされるCMakeはこの条件を満たしています。

次に ::

 project(choreonoid_ros_tank_tutorial)

で、このパッケージのプロジェクト名を設定しています。これは通常パッケージ名と同じにします。 ::

 set(CHOREONOID_SKIP_QT_CONFIG true)

については、必ずしも必要ではないのですが、本チュートリアルでは付与しています。この記述を入れると、次に記述するfind_packageにおいてChoreonoidパッケージが検出・初期化される際に、Qtライブラリの検出を行わなくなります。QtライブラリはChoreonoidのGUIの構築に使用しているライブラリで、Choreonoidのプラグインのビルドには必要となるのですが、今回はコントローラのみのビルドとなるので、この記述を入れることで無駄な処理を省くことができます。なお、この記述を入れなくてもビルドは問題なく実行できます。 ::

 find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   image_transport
   choreonoid
   )

依存パッケージの検出を行います。ここでは以下のパッケージを依存対象としています。

* roscpp: ROSのC++ライブラリ
* std_msgs: ROSの標準的なメッセージ
* sensor_msgs: センサ関連のメッセージ
* image_transport: 画像転送のためのライブラリ
* choreonoid: Choreonoid本体

ここに記述する内容は、概ね :ref:`ros_tank_tutorial_edit_package_xml` で記述している依存パッケージと重なります。ただしこちらに書くのはあくまでC++のプログラムをビルドする際に必要なライブラリが対象なので、完全に同じになるとは限りません。 ::

 catkin_package(SKIP_CMAKE_CONFIG_GENERATION SKIP_PKG_CONFIG_GENERATION)

については、CatkinによるCMakeのConfigファイルやpkg-configファイルの生成を行わないようにするためのものです。それらのファイルは、ここで作成したパッケージをさらに他のパッケージから利用する際に必要となるもので、主にライブラリが対象となるものです。今回作成するのはそのようなものではないので、この処理は必要ありません。また、 :ref:`ros_tank_tutorial_edit_package_xml` においてパッケージのビルドタイプを "cmake" にする旨述べましたが、このビルドタイプの場合にはConfigファイル等の生成処理がうまく機能しないようです。以上の理由により、本パッケージではこの記述を入れています。 ::

 set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
 set(CMAKE_CXX_EXTENSIONS OFF)

ここではコンパイルで使用するC++のバージョンを設定しています。Choreonoidはライブラリの公開APIも含めてC++11以上を前提にコーディングがされており、それを利用する側も同等以上のC++バージョンでビルドしなくてはなりません。しかしコンパイラによっては、それよりも古いC++のバージョンがデフォルトになる場合があります。これについてCatkinでは特に何も設定しないようなので、C++バージョンの設定が必要となります。

find_packageでchoreonoidを指定すると、CHOREONOID_CXX_STANDARDという変数にChoreonoid本体で使用しているC++のバージョンが設定されるので、基本的にはこれと一致するように設定します。CMakeではCMAKE_CXX_STANDARDという変数でC++のバージョンを設定できます。CMAKE_CXX_EXTENSIONS については、OFFにするとコンパイラ独自の拡張を使用しなくなります。GCCの場合この記述を入れないと独自の拡張が有効になるのですが、保守性を高めるために、あえてこの記述を入れています。この記述が無くてもビルドすることは可能です。

なお、GCCバージョン6以上ではC++14がデフォルトで使用されるようです。Ubuntu 18.04のGCCはバージョン7なので、Ubuntu 18.04であれば特にこの記述を行わなくてもビルドを行うことができます。一方でUbuntu 16.04でインストールされるGCCはそれよりも古いバージョンのものであり、デフォルトではC++11以上のバージョンにならないようですので、この記述がないとコンパイルエラーになります。 ::

 set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS ${CHOREONOID_COMPILE_DEFINITIONS})

この記述により、Choreonoidの関連モジュールをコンパイルする際に必要となるプリプロセッサ定義を取り込みます。変数 CHOREONOID_COMPILE_DEFINITION は、find_package で choreonoid を指定すると設定されます。 ::

 include_directories(
   ${catkin_INCLUDE_DIRS} 
   ${CHOREONOID_INCLUDE_DIRS}
   )

追加のインクルードディレクトリを指定しています。変数 catkin_INCLUDE_DIRS には、find_packageで指定した依存パッケージを使用する際に必要なインクルードディレクトリが設定されています。また、Choreoonidのライブラリについては別途 CHOREONOID_INCLUDE_DIRS 変数で対応するインクルードディレクトリを取り込む必要があります。この変数も find_package で choreonoid を指定すると設定されます。 ::

 link_directories(
   ${CHOREONOID_LIBRARY_DIRS}
   )

依存ライブラリのリンクディレクトリを追加します。CHOREONOID_LIBRARY_DIRS についてもfind_package で choreonoid を指定すると設定されるので、これを利用してChoreonoidのライブラリのディレクトリを取り込みます。 ::

 add_subdirectory(src)

本チュートリアルでは、C++で記述されるコントローラのソースファイルを別途 "src" ディレクトリに格納するようにしています。この構造にあわせて、各ソースファイルに直接対応する記述はsrcディレクトリのCMakeLists.txtにて行うものとし、ここではそのファイルを取り込むようにしています。

CMakeLists.txtの追加
~~~~~~~~~~~~~~~~~~~~

上記の「srcディレクトリのCMakeLists.txt」については、以下の内容で作成して追加します。 ::

 add_cnoid_simple_controller(JoyInputController JoyInputController.cpp)
 target_link_libraries(JoyInputController ${roscpp_LIBRARIES})

add_cnoid_simple_controller は、find_packageでchoreonoidを検出すると利用可能になる関数です。これはChoreonoidのシンプルコントローラのバイナリをビルドするための関数で、CMake組み込みのadd_executableやadd_libraryといった関数と同様の記述で利用できます。ここではJoyInputControllerというターゲット名を設定し、ソースコードとしてJoyInputController.cppを指定しています。

また、target_link_librariesで依存ライブラリへのリンクを指定しています。ここで指定しているのは、C++でrosを使用するためのroscppライブラリのリンクです。find_packageでroscppを指定すると、変数roscpp_LIBRARIESにroscppのライブラリが設定されるので、それを使用しています。なお、シンプルコントローラにリンクすべきChoreonoidのライブラリは、add_cnoid_simple_controllerを実行することで自動的に設定されるので、target_link_librariesに指定する必要はありません。

この記述によって、JoyInputController.cppからシンプルコントローラのバイナリが生成され、Choreonoidのシンプルコントローラ用のバイナリ格納ディレクトリに出力されることになります。


コントローラのビルド
~~~~~~~~~~~~~~~~~~~~

コントローラのソースコードとCMakeLists.txtの記述ができたら、ビルドの準備は整ったことになります。ビルドはCatkinの以下のコマンドで行います。 ::

 catkin build

このコマンドは、Catkinのワークスペース内であればどこのディレクトリで実行してもOKです。ビルドの方法については :doc:`../build-choreonoid` における :ref:`ros_catkin_build_command` の節も参考にしてください。

ビルドの際には、 :ref:`ros_catkin_cmake_build_type` も行っておくとよいです。通常はビルドタイプを "Release" にしておきます。そうすることで、コンパイルの最適化が有効となり、より効率的なバイナリを生成することができます。特に設定しなければ最適化は有効になりませんので、注意が必要です。

なお、CMakeLists.txtに記述を追加することで、パッケージ側でデフォルトのビルドタイプを指定することもできます。その場合は以下のような記述をメインのCMakeLists.txtに追加します。 ::


 if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release CACHE STRING
     "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
     FORCE)
 endif()

追加する場所は、project関数によるプロジェクト名の設定の直後が適切です。この記述をしておけば、CatkinでCMakeのビルドタイプを設定しておかなくても、最適化の効いたReleaseビルドが適用されます。


コントローラの導入
------------------


ノード接続状況の可視化
----------------------


.. _ros_tank_tutorial_introduce_launch_file:

Launchファイルの導入
--------------------


ソースコードの解説
------------------


補足：ChoreonoidのROS連携機能の現状
-----------------------------------

.. 自前のコーディングで大抵のことは対応可能




