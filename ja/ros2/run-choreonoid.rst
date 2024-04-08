ROS環境におけるChoreonoidの実行
===============================

.. contents::
   :local:

.. highlight:: sh

.. _choreonoid_ros2_run_choreonoid_node:

Choreonoidの起動
----------------

ROS 1 環境においては、Choreonoidは通常ROSノードとして扱われました。一方 ROS 2 環境においては、Choreonoid本体はノードとして扱われません。

例えば、シミュレーション時刻を提供する WorldROS2Item やロボットのセンサ情報を提供する BodyROS2Item 、そしてロボットを動かすための個々のコントローラなどが、単一の ROS 2 ノードとなります。

Choreonoidは ROS 2ノードではありませんが、 choreonoid_ros パッケージを利用することによって、 ROS 2 のツールを用いて起動できます。
例えば、 ros2 run コマンドを使用して、ターミナル上で ::

   ros2 run choreonoid_ros choreonoid

とすることで、Choreonoidを起動できます。

起動に成功すると、Choreonoidのメインウィンドウが表示されます。これはChoreonoidを通常の方法で起動した場合と基本的には同じもので、操作方法も同じです。

このコマンドの後ろに、Choreonoidへの各種オプションを付与することも可能です。
