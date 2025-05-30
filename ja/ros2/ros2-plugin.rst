ROS2プラグイン
==============

.. contents::
   :local:

.. highlight:: sh

.. 英訳指示： 「ROS2プラグイン」はプラグイン名なので、ROSと2の間にスペースを入れないようにしてください。

ROS2プラグインとは
-----------------

ROS2プラグインは、choreonoid_rosパッケージに含まれるChoreonoid用のプラグインです。

このプラグインは、Choreonoid上でのシミュレーションに関する情報を、ROS 2のトピックを介して提供します。

現在、シミュレーション時刻を `/clock` トピックで提供する WorldROS2 アイテムと、ロボットのセンサデータを提供する BodyROS2 アイテムが利用可能です。


ROS2プラグインの読み込み
-----------------------

ROS2プラグインは chorenoid_ros パッケージに含まれています。ROS2プラグインを利用するためには、choreonoid_ros を介してChoreonoidを起動する必要があります。これは、:ref:`choreonoid_ros2_run_choreonoid` でも述べたように、ros2 run コマンドを用いて ::

   ros2 run choreonoid_ros choreonoid

とすることで起動できます。あるいは ros2 launch コマンドを用いて、 ::

   ros2 launch choreonoid_ros choreonoid.launch.xml

としても起動が可能です．


このように、ros2 run/launch コマンドによって、 choreonoid_ros パッケージの一部として起動されたChoreonoidでは、ROS2プラグインが自動で読み込まれます。その際、起動されたChoreonoidのメッセージビュー内に、 ::

   ROS2プラグインが読み込まれました．

というメッセージが出力されます。

.. 英訳指示：上のメッセージは "ROS2-plugin has been activated" としてください。

もしこのメッセージが出力されていない場合、ROS2プラグインは読み込まれていません。このとき、Choreonoid上でのROS 2との連携機能は利用できなくなりますので、ご注意ください。（上記の方法以外でChoreonodiを起動する場合や、ROSとは独立してインストールしたChoreonoidを通常の方法で起動する場合は、そのようになります。）
