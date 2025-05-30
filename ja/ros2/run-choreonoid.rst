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

このコマンドの後ろに、Choreonoidへの各種オプションを付与することも可能です。

.. note:: Choreonoid本体に対応する "choreonoid" パッケージにも実行ファイル "choreonoid" が含まれます。そちらは単に "choreonoid" と入力して実行することもできます。しかしその場合は :doc:`rosplugin` が読み込まれませんので、ROS 2連携機能は使用できません。ROS 2連携機能を使用する場合は、上記の方法でChoreonoidを起動するようにしてください。混乱を避けるため、Choreonoid本体の "choreonoid" コマンドは、 :ref:`ros2_build_choreonoid_cmake_options` で紹介している "--cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF" オプションによって、ビルドしないようにしておくとよいでしょう。
