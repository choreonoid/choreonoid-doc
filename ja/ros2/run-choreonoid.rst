ROS 2環境におけるChoreonoidの実行
================================

.. contents::
   :local:

.. highlight:: sh

.. _choreonoid_ros2_run_choreonoid:

Choreonoidの起動
----------------

ROS 2環境では以下のコマンドでChoreonoidを起動できます。 ::

   ros2 run choreonoid_ros choreonoid

ここでは、"ros2 run" コマンドを使用して、"choreonoid_ros" パッケージの実行ファイルである "choreonoid" を起動しています。

起動に成功すると、Choreonoidのメインウィンドウが表示されます。これはChoreonoidを通常の方法で起動した場合と基本的には同じもので、操作方法も同じです。

このコマンドの最後に、Choreonoidへの各種オプションを付与することも可能です。

.. note:: Choreonoid本体の通常の起動コマンドである "choreonoid" を実行しても、ROS 2連携機能は使用できません。ROS 2連携機能を使用する場合は、上記の方法でChoreonoidを起動するようにしてください。
