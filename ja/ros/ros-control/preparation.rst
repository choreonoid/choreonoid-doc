準備
====

.. contents::
   :local:

.. _ros_cnoid_tank_setup:

ROSとChoreonoid関連パッケージのセットアップ
-------------------------------------------
本チュートリアルの実施にあたっては，まず :doc:`../install-ros` と :doc:`../build-choreonoid` を行っておく必要があります．これらのページの手順に従ってインストールやビルドを行い， :doc:`../run-choreonoid` が実行できることを確認しておいてください．

なお， :ref:`ros_choreonoid_add_package_sources` については実行する必要はありません．（もちろん，導入されていても特に問題はありません．）

.. _ros_cnoid_pull_and_build:

cnoid_tank_pkgs の ダウンロードとビルド
------------------------------------------

.. highlight:: sh

本チュートリアルでは，catkinワークスペースまでのパスは "<catkin_ws>" として表記しますので，適宜読み替えてください．なお，ワークスペース自体の名前は"catkin_ws"で無くても問題ありません（例えば"choreonoid_ws"等）．プロジェクトごとにワークスペースを分けてお使いになることをお勧めします．
               
まずはcatkinワークスペースの下に"cnoid_tank_pkgs"リポジトリをダウンロード（git clone）します． ::

 mkdir -p <catkin_ws>/src
 cd <catkin_ws>/src
 git clone https://github.com/choreonoid/cnoid_tank_pkgs.git

完了したら，依存パッケージのダウンロードを行います．
ここではまず `wstool <http://wiki.ros.org/wstool>`_ を利用して依存パッケージをダウンロードします． ::

  cd <catkin_ws>
  wstool init src
  wstool merge -t src src/cnoid_tank_pkgs/melodic.rosinstall
  wstool up -t src

ここまでで "<catkin_ws>/src" ディレクトリの下に自動で依存パッケージがダウンロードされます．
次に，"apt"のパッケージとして提供されている依存パッケージをダウンロードします．これには `rosdep <http://wiki.ros.org/rosdep>`_ を利用します．::

  cd <catkin_ws>
  rosdep update
  rosdep install -i -y -r --from-paths src

ここまでで，依存パッケージのダウンロードが完了しました．
最後に， `catkin_make <http://wiki.ros.org/catkin/commands/catkin_make>`_ もしくは， `catkin tools <https://catkin-tools.readthedocs.io/en/latest/>`_ を使ってパッケージのビルドを行いましょう．ここでは， `catkin tools <https://catkin-tools.readthedocs.io/en/latest/>`_ を利用します．::

  cd <catkin_ws>
  catkin build

以上で，パッケージのダウンロードとビルドまで完了です．
お疲れ様でした！
