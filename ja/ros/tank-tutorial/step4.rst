ステップ4: 加速度センサ、レートジャイロの状態出力
=================================================

ステップ4ではTankロボットが搭載する加速度センサ、レートジャイロの状態をROSトピックとして出力するプログラムを作成します。

.. contents::
   :local:

.. highlight:: c++

概要
----

ロボットの関節に関る関節角度等はロボットが有する基本的な状態値のひとつですが、ロボットは一般的にこれ以外にも各種センサ等で取得できる様々な状態値を有しています。ステップ4ではその例として、Tankロボットが備える加速度センサ、レートジャイロの状態値を出力する方法を学びます。

これらのセンサはChoreonoid内部では通常「デバイス」と呼ばれるタイプのオブジェクトとしてモデル化されており、対応するデバイスオブジェクトの状態をPublishすることで、センシングされた状態値を出力することが可能となります。
これらのセンサ値を出力するのに適切なメッセージ型として、ROSでは標準でsensor_msgs/Imuという型が定義されているので、今回はそれを用いて状態をPublishすることにします。

この方法を習得すれば、同様の方法で他のセンサやデバイスの状態を出力することも可能となります。

センシング値出力用コントローラ
------------------------------

本ステップの処理についても、そのためのシンプルコントローラを作成して実現することにします。
まずそのソースコードを以下に示します。

.. code-block:: c++
 :linenos:

 #include <cnoid/SimpleController>
 #include <cnoid/AccelerationSensor>
 #include <cnoid/RateGyroSensor>
 #include <ros/node_handle.h>
 #include <sensor_msgs/Imu.h>

 using namespace std;
 using namespace cnoid;

 class RttImuStatePublisher : public SimpleController
 {
     std::unique_ptr<ros::NodeHandle> node;
     ros::Publisher imuPublisher;
     sensor_msgs::Imu imu;
     AccelerationSensorPtr accelSensor;
     RateGyroSensorPtr gyro;
     Vector3 dv_sum;
     Vector3 w_sum;
     int sensingSteps;
     double time;
     double timeStep;
     double cycleTime;
     double timeCounter;

 public:
     virtual bool configure(SimpleControllerConfig* config) override
     {
	 node.reset(new ros::NodeHandle(config->body()->name()));
	 imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);
	 return true;
     }

     virtual bool initialize(SimpleControllerIO* io) override
     {
	 accelSensor = io->body()->findDevice<AccelerationSensor>();
	 gyro = io->body()->findDevice<RateGyroSensor>();

	 io->enableInput(accelSensor);
	 io->enableInput(gyro);

	 dv_sum.setZero();
	 w_sum.setZero();
	 sensingSteps = 0;

	 for(int i=0; i < 9; ++i){
	     imu.orientation_covariance[i] = 0.0;
	     imu.angular_velocity_covariance[i] = 0.0;
	     imu.linear_acceleration_covariance[i] = 0.0;
	 }
	 imu.orientation_covariance[0] = -1.0;
	 imu.orientation.x = 0.0;
	 imu.orientation.y = 0.0;
	 imu.orientation.z = 0.0;
	 imu.orientation.w = 0.0;

	 time = 0.0;
	 timeStep = io->timeStep();
	 const double frequency = 20.0;
	 cycleTime = 1.0 / frequency;
	 timeCounter = 0.0;

	 return true;
     }

     virtual bool control() override
     {
	 dv_sum += accelSensor->dv();
	 w_sum += gyro->w();
	 ++sensingSteps;

	 time += timeStep;
	 timeCounter += timeStep;

	 if(timeCounter >= cycleTime){
	     imu.header.stamp.fromSec(time);

	     auto dv = dv_sum / sensingSteps;
	     imu.linear_acceleration.x = dv.x();
	     imu.linear_acceleration.y = dv.y();
	     imu.linear_acceleration.z = dv.z();
	     dv_sum.setZero();

	     auto w = w_sum / sensingSteps;
	     imu.angular_velocity.x = w.x();
	     imu.angular_velocity.y = w.y();
	     imu.angular_velocity.z = w.z();
	     w_sum.setZero();

	     sensingSteps = 0;

	     imuPublisher.publish(imu);

	     timeCounter -= cycleTime;
	 }

	 return true;
     }
 };

 CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttImuStatePublisher)


以下の節ではこのコントローラの実装のポイントを解説し、その中でコードの内容についても適宜解説していきます。


sensor_msgs/Imu型のメッセージ
-----------------------------

本ステップではROS標準のメッセージ型のひとつである `sensor_msgs/Imu <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html>`_  型のメッセージを使用します。
これは慣性計測装置（Inertial Measurement Unit, IMU）の状態値を格納するためのメッセージ型です。
IMUは基本的には3軸のジャイロと加速度計によって3次元の角速度と加速度を求めるものであり、そこからさらに姿勢や位置などを推定する用途にも使用されます。

このImu型の内容は以下のコマンドでも確認できます。

.. code-block:: sh

 rosmsg show sensor_msgs/Imu

このコマンドで表示されるように、この型は以下のように定義されています。

.. code-block:: none

 std_msgs/Header header
   uint32 seq
   time stamp
   string frame_id
 geometry_msgs/Quaternion orientation
   float64 x
   float64 y
   float64 z
   float64 w
 float64[9] orientation_covariance
 geometry_msgs/Vector3 angular_velocity
   float64 x
   float64 y
   float64 z
 float64[9] angular_velocity_covariance
 geometry_msgs/Vector3 linear_acceleration
   float64 x
   float64 y
   float64 z
 float64[9] linear_acceleration_covariance

今回使用するのはジャイロに対応する "angular_velocity" の部分と、加速度センサに対応する "linear_acceleration" の部分です。
それらのx、y、z成分に、対応するセンサの状態値を格納することにします。
angular_velocity_covarianceとlinear_acceleration_covarianceの部分には、それぞれの要素に対応する共分散行列を格納しますが、今回これらの値は未知であるものとして、要素を0で埋めておきます。

"orientation" の部分には姿勢の推定値を格納することになりますが、今回想定するIMUではこちらの要素はカバーしないものとします。
リファレンスマニュアルによると、"orientation_covariance" の第一要素を-1にすることでこれを表現できます。

上記コントローラのソース中では、まず ::

 #include <sensor_msgs/Imu.h>

によってこの型の定義を取り込んでコードで使えるようにしています。

そしてこの型のメッセージをPublishするために、コントローラのメンバ変数として ::

 sensor_msgs::Imu imu;

を定義しています。

そしてconfigure関数で ::

 imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);

とすることにより、このメッセージ型のトピック "imu" を出力するためのパブリッシャーを生成しています。

またinitialize関数の以下のコードで、Imu型の要素で今回使用しない部分を無効化する値をセットしています。 ::

 for(int i=0; i < 9; ++i){
     imu.orientation_covariance[i] = 0.0;
     imu.angular_velocity_covariance[i] = 0.0;
     imu.linear_acceleration_covariance[i] = 0.0;
 }
 imu.orientation_covariance[0] = -1.0;
 imu.orientation.x = 0.0;
 imu.orientation.y = 0.0;
 imu.orientation.z = 0.0;
 imu.orientation.w = 0.0;

各covarianceの要素を0で埋めて共分散行列を未知とし、orientationについてもcovarianceの第一要素に-1をセットすることでorientationの値全体を未知としています。

センサデバイスの取得
--------------------

Imu型のメッセージに格納する角速度と加速度の値は、Tankロボットが備えるレートジャイロと加速度センサのデバイスから取得します。

まずこれらのデバイスは、Tankロボットのモデルファイルの一部である "TankBody.body" において、CHASSISリンクの子要素として以下のように定義されています。

.. code-block:: yaml


 links:
   -
     name: CHASSIS
     ...
 
     elements:
       ...
 
       -
         type: AccelerationSensor
         name: ACCEL_SENSOR
         id: 0
       -
         type: RateGyroSensor
         name: GYRO
         id: 0
    ...


これにより、Tankロボットの車体の原点（中央）に、加速度センサとジャイロがひとつずつデバイスとして搭載されています。
これらのデバイスについて、 :ref:`simulation-device` で解説した方法で、シンプルコントローラからの状態取得を行います。

まず :ref:`simulation-obtain-device-object` を行います。

このためにまず使用するデバイス型のヘッダをインクルードしておきます。 ::

 #include <cnoid/AccelerationSensor>
 #include <cnoid/RateGyroSensor>

により、加速度センサに対応するAccelerationSensor型とレートジャイロに対応するRateGyroSensor型の定義を取り込んでいます。

そしてこれらのセンサ型のオブジェクトを保持するためのスマートポインタをメンバ変数を定義しています。 ::

 AccelerationSensorPtr accelSensor;
 RateGyroSensorPtr gyro;

そしてコントローラのinitialize関数で、これらのデバイスの入出力用オブジェクトを取得しています。 ::

 accelSensor = io->body()->findDevice<AccelerationSensor>();
 gyro = io->body()->findDevice<RateGyroSensor>();

これまでのコントローラと同様に :ref:`simulation-implement-controller-simple-controller-io` から入出力用のBodyオブジェクトを取得し,ています。それに対してfindDevice関数で取得するデバイスの型を指定することで、対応するセンサデバイスを取得しています。

この部分は同じ型のデバイスが複数存在する場合は、 ::

 accelSensor = io->body()->findDevice<AccelerationSensor>("ACCEL_SENSOR");

などとして、デバイスの名前などでオブジェクトを特定する必要がありますが、Tankロボットではその必要はありません。

これらのデバイスに対して ::

 io->enableInput(accelSensor);
 io->enableInput(gyro);

とすることで入力を有効にしています。これにより、対応するセンサの状態がシミュレーション中に更新されると、それがこれらの入出力用オブジェクトにも反映されるようになります。

センサ状態値の取得
------------------

センサの状態値は、コントローラのcontrol関数内で取得しています。
まず加速度センサの値について以下のコードで取得しています。 ::

 dv_sum += accelSensor->dv();

ここではメンバ変数dv_sumの値を更新しています。
dv_sumは加速度の積算を格納するVector3型の変数で、毎制御ループごとの加速度の値を次回のPublishまでここに積算していきます。

加速度の値はシミュレーションのタイムステップ毎に大きく変化する可能性があり、それをそのまま出力してしまうと現実のセンサよりもノイジーな変化になってしまうことがあります。またトピックをPublishする頻度は一般的にシミュレーションのタイムステップよりも長い周期になります。
このことを考慮すると、出力する加速度の値を安定させるひとつの手法として、ある時点のPublishから次のPublishまでの値を平均化することが考えられます。変数dv_sumはこれを実現するために用いられています。

レートジャイロについても、同様の方法で角速度の値を取得し積算します。これは以下のコードになります。 ::

 w_sum += gyro->w();

そして、積算の回数を整数型のメンバ変数sensingStepsに記録します。 ::

 ++sensingSteps;

実際に出力する平均化された値は、積算値をsensingStepsで割ることで算出できます。
この計算は以下のコードで表されます。 ::

 auto dv = dv_sum / sensingSteps;
 auto w = w_sum / sensingSteps;

この値のPublishについては後ほど解説します。

なお、これらの変数の値はinitialize関数の以下のコードで全て0に初期化しています。 ::

 dv_sum.setZero();
 w_sum.setZero();
 sensingSteps = 0;

.. note:: ここで適用している平均化の手法は必要最低限の実装で出力値を安定化させるためのもので、必ずしも最適な手法とは限りません。現実のIMUセンサでも精度向上のための各種補正処理が入っている場合があり、必要に応じてそのような現実のセンサと同様の補正処理を導入した方がよい場合もあるかもしれません。そのような処理をセンサデバイスのシミュレーションの一環としてChoreonoid内部のシミュレーション処理に導入することも考えられますが、現状ではそのような処理は入っていないため、今回の例のように値を利用する側で適切に処理する必要があります。

センシング値のPublish
---------------------

センシング値のPublishにあたっては、ステップ2の :ref:`ros_tank_tutorial_publish_joint_state` と同様に、メッセージに付与するタイムスタンプや、Publishを実行するタイミング（周期）等、時間に関る部分についても適切に処理する必要があります。
この部分はステップ2と同様に処理していますが、本ステップでもあらためてこの部分のコードを解説します。

まずPublishのための時間関連の情報を格納する以下の4つのメンバ変数を定義しています。 ::

 double time;
 double timeStep;
 double cycleTime;
 double timeCounter;

timeはコントローラ稼働開始後の経過時間、timeStepは制御ループのタイムステップ、cycleTimeはPublishを実施する周期、timeCounterは次のPublishの周期に達したかどうかを判定する時間カウンタです。

これらの変数はまずinitialize関数で以下のように初期化しています。 ::

  time = 0.0;
  timeStep = io->timeStep();
  const double frequency = 20.0;
  cycleTime = 1.0 / frequency;
  timeCounter = 0.0;

ここでは1秒間に20回PublishするようにcycleTimeを設定しています。
この値は通信環境やトピックの用途を考慮して適切に調整するようにします。

Publishはcontrol関数内で行います。
ただしcontrol関数実行の度にPublishするのではなく、上記のように設定したcycleTimeの周期でPublishするように、タイミングを調整します。
この調整を行うコードが以下になります。 ::

  time += timeStep;
  timeCounter += timeStep;

  if(timeCounter >= cycleTime){

      // ここでPublishを実行

      timeCounter -= cycleTime;
  }

このようにすることで、Publishの周期がcycleTimeと一致するように調整されます。

このif文の中でImu型の変数の内容を更新して、Publishを行います。
まず ::

  imu.header.stamp.fromSec(time);

でヘッダに含まれる時刻の情報を更新しています。

そして加速度については上述の手法で平均化された値を算出します。 ::

  auto dv = dv_sum / sensingSteps;

このdvの値をImu型のlinear_accelerationの各要素に代入します。 ::

  imu.linear_acceleration.x = dv.x();
  imu.linear_acceleration.y = dv.y();
  imu.linear_acceleration.z = dv.z();

次のPublishのためにdv_sumの値をクリアしておきます。 ::

  dv_sum.setZero();

加速度に関する処理と同様の処理を角速度に対しても実行します。 ::

  auto w = w_sum / sensingSteps;
  imu.angular_velocity.x = w.x();
  imu.angular_velocity.y = w.y();
  imu.angular_velocity.z = w.z();
  w_sum.setZero();

次のPublishのため、sensingStepsの値もクリアしておきます。 ::

  sensingSteps = 0;

これでImu型メッセージの変数の内容を最新の状態に更新できました。
最後に以下のコードで実際にPublishを行います。 ::

  imuPublisher.publish(imu);


コントローラの導入
------------------

これまでのステップと同様に、上記のソースコードに対応するコントローラをビルドしてシミュレーションプロジェクトに導入します。

まず上記のソースコードをsrcディレクトリ内に "RttImuStatePublisher.cpp" というファイル名で作成します。そして同じディレクトリのCMakeLists.txtに以下の記述を追加します。

.. code-block:: cmake

 choreonoid_add_simple_controller(RttImuStatePublisher RttImuStatePublisher.cpp)
 target_link_libraries(RttImuStatePublisher ${roscpp_LIBRARIES})

この状態で catkin build を行うと、RttImuStatePublisherがビルドされます。ビルドに成功したら、これまでのステップと同様にRttImuStatePublisherをプロジェクトに追加します。具体的にはステップ2で作成したプロジェクトにこのコントローラを追加して、アイテムツリーを以下のように構成します。

.. code-block:: none

 + World
   + Tank
     - RttTankController
     - RttJointStatePublisher
     - RttImuStatePublisher <- これを追加
   - Labo1
   - AISTSimulator

ここで追加するRttImuStatePubisherはSimpleController型のアイテムで、その「コントローラモジュール」に "RttJointStatePublisher.so" を指定します。このプロジェクトを "step4.cnoid" というファイル名で保存してください。

あわせてこのプロジェクトを実行するためのLaunchファイルも作成します。
以下の内容のLaunchファイルを "step4.launch" として作成してください。

.. code-block:: xml

 <launch>
   <node pkg="choreonoid_joy" name="choreonoid_joy" type="node" />
   <node pkg="choreonoid_ros" name="choreonoid" type="choreonoid"
	 args="$(find choreonoid_ros_tank_tutorial)/project/step4.cnoid --start-simulation" />
   <node pkg="rqt_graph" name="rqt_graph" type="rqt_graph" />
 </launch>

ここまで作業を進めると、本チュートリアル用パッケージは以下のファイル構成になります。

.. code-block:: none

 + choreonoid_ros_tank_tutorial
   - CMakeLists.txt
   - package.xml
   + launch
     - step1.launch
     - step2.launch
     - step3.launch
     - step4.launch
   + project
     - step1.cnoid
     - step2.cnoid
     - step3.cnoid
     - step4.cnoid
   + src
     - CMakeLists.txt
     - RttTankController.cpp
     - RttJointStatePublisher.cpp
     - RttJointStateSubscriber.cpp
     - RttImuStatePublisher.cpp

.. _ros_tank_tutorial_step3_check_topic_values:

出力トピックの確認
------------------

.. highlight:: sh

step4.launchを実行すると、シミュレーションが開始され、RttImuStatePublisherによってIMUのトピックが追加されます。
この確認のため ::

 rostopic list 

を実行すると、

.. code-block:: none

 /Tank/imu

という項目も表示されるかと思います。これがIMUのトピックです。 ::

 rostopic info /Tank/imu

を実行すると、

.. code-block:: none

 Type: sensor_msgs/Imu
 
 Publishers: 
  * /choreonoid (http://rynoid:44641/)
 
 Subscribers: None

と表示されます。これによってメッセージの型が実際に "sensor_msgs/Imu" となっていることを確認できます。 ::

 rostopic echo /Tank/imu

を実行すると、センサの状態値が以下のように表示され続けます。

.. code-block:: none

 header: 
   seq: 3399
   stamp: 
     secs: 170
     nsecs:         0
   frame_id: ''
 orientation: 
   x: 0.0
   y: 0.0
   z: 0.0
   w: 0.0
 orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 angular_velocity: 
   x: -1.3141583564318781e-09
   y: -6.139951539231158e-12
   z: -1.0749827270382294e-13
 angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 linear_acceleration: 
   x: -1.220294155439848e-08
   y: 4.219067333397275e-09
   z: 9.806650065226014
 linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 ---

この状態でTankロボットの車体を動かしてみてください。angular_velocityやlinear_accelerationの値が変化するかと思います。それぞれ単位は [rad/sec]、[m/s^2] となります。

車体の旋回速度がangular_velocityのz成分に対応し、車体の前後方向の加速度がlinear_accelerationのx成分に対応しますので、車体を旋回させたり前後に動かすことで生じるこれらの値の変化が比較的分かりやすいのではないかと思います。

加速度、角速度のグラフ表示
--------------------------

ステップ2の関節角度と同様に、加速度と角速度をグラフ表示してみましょう。
本ステップのシミュレーションを実行している状態で、端末から以下のコマンドを入力します。 ::

 rosrun rqt_plot rqt_plot /Tank/imu/linear_acceleration

これにより加速度のX、Y、Z軸成分がグラフにプロットされます。
これについては車体を前後に動かした時のX軸成分の変化が分かりやすいかと思います。
また、車体を環境に衝突させることでも、グラフ上で加速度の大きな変化を確認できるかと思います。

同様にして、角速度についても以下のコマンドでプロットできます。 ::

 rosrun rqt_plot rqt_plot /Tank/imu/angular_velocity

これにより角速度のX、Y、Z軸成分がグラフにプロットされます。
これについてはTankロボットの車体を旋回させることで生じるZ軸成分の変化が分かりやすいかと思います。
