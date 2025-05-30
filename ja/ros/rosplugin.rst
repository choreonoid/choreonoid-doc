ROSプラグイン
=============

.. contents::
   :local:

.. highlight:: sh

ROSプラグインとは
-----------------

ROSプラグインは、choreonoid_rosパッケージに含まれるChoreonoid用のプラグインです。

このプラグインの基本的な役割は、C++からROSの機能を使用するためのライブラリである "roscpp" をChoreonoid上でも利用できるようにすることにあります。そのためのビルド環境がchoreonoid_rosパッケージによって整備され、ROSプラグインの中ではroscppの初期化がまず行われるようになっています。

この仕組みによって、Choreonoidフレームワークの構成要素であるビューやアイテム、あるいはシミュレーション機能の構成要素であるコントローラやサブシミュレータ等において、ROSで通信を行うコントローラによってロボットを制御したり、ROS上でのデータを可視化・編集したりといったことが、Choreonoid上で可能となるわけです。

これを実現する第一の手段は、ユーザが必要な機能をC++プログラムとして自前でコーディングすることです。roscppはよく設計されたライブラリであり、ドキュメントも充実しているため、rosccppを用いてROSの機能を利用するソフトウェアを実装することはそれほど難しいことではありません。ROSとC++に慣れている場合は、この手段によって目的を達成することを考えてください。

これに加えて、ROSプラグインはシミュレーション中のロボットのセンサ情報をROSトピックとしてPublishする機能を有しています。
また、ros_controlを用いてロボットの制御を行うことも可能です。
これらの機能を用いることで、必要最小限のコーディングでロボットの制御を行うことも可能となります。

ROSプラグインの読み込み
-----------------------

ROSプラグインは chorenoid_ros パッケージに含まれています。ROSプラグインを利用するためには、choreonoid_ros が提供するROSノードとしてChoreonoidを起動する必要があります。これをChoreonoidノードと呼ぶことにします。

:ref:`choreonoid_ros_run_choreonoid_node` でも述べたように、Choreonoidノードは他のROSノードと同様にrosrunコマンドを用いて ::

 rosrun choreonoid_ros choreonoid

とすることで起動できます。もちろん、ROSノードを起動する他の方法として、roslaunch等も利用可能です。

このようにROSノードとして起動されたChoreonoidでは、ROSプラグインが読み込まれており、利用できるようになっています。その場合、起動されたChoreonoidのメッセージビュー内に、 ::

 ROSプラグインが読み込まれました．

.. 英訳指示： 「ROSプラグインが読み込まれました．」は "ROS-plugin has been activated." としてください。

というメッセージが出力されます。

このメッセージが出力されていない場合、ROSプラグインは読み込まれておらず、Choreonoid上でROSの機能を利用することはできませんので、ご注意ください。（ROSとは独立してインストールしたChoreonoidを通常の方法で起動した場合は、そのようになります。）

なお、Choreonoidノード起動時に、メッセージビューに

.. code-block:: none

 Warning: The ROS master is not found.

と表示されることがあります。この場合、ROSのマスターが起動されておらず、やはりROSの機能を利用することはできません。

この場合はまず :ref:`choreonoid_ros_run_ros_master` を行うようにしてください。

ROSプラグインによるroscppの初期化
---------------------------------

ROSプラグインが読み込まれると、プラグインの初期化関数にて、以下のコードに相当する処理が実行されます。

.. code-block:: c++

 ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
 auto spinner = new ros::AsyncSpinner(0);
 spinner->start();

まず ros::init 関数でroscppの初期化を行います。ノード名は標準で "choreonoid" となるようにしています。また、Choreonoidノードの起動時に与えたROS関連のコマンドラインオプションが、argc、argvに格納されており、この初期化関数に渡されます。これによって、ノード名やトピック名などのリマップも処理されます。

初期化完了後、ros::AsyncSpinner が作成され、ROSのコールバックキューのバックグラウンド処理が開始します。これにより、Choreonoidノード内で生成されたSubscriber等の処理が、バックグラウンドスレッドで行われるようになります。Choreonoidのメインスレッド上では通常通りGUI等を処理するためのメインループが動作しますが、それと並行してROSの処理が行われるようになっています。

以上の初期化処理により、Choreonoid上でros::NodeHandle等を用いてPublisherやSubscriber等を自由に生成して使用することが可能となります。逆にroscppの初期化はROSプラグインが担当するので、Choreonoid上で動作する他のモジュールで初期化関数などを実行してはいけません。

また、上記のようにコールバックキューの処理はメインスレッドとは別のスレッドで行われるため、各コールバック関数が実行されるスレッドもメインスレッドとは異なります。コールバック関数の実装においてはこの点に注意し、必要に応じて排他制御を入れるようにしてください。

ros::NodeHandle等の具体的な使用方法については、 :doc:`tank-tutorial/index` にて解説します。
