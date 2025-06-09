ROS 2のインストール
=================

.. contents::
   :local:

.. highlight:: sh

ROS 2ディストリビューションのインストール
-----------------------------------------

まずROS 2のディストリビューションをインストールしておく必要があります。
最新かつ詳細なインストール方法については、以下の公式マニュアルを参照するようにしてください。

* `ROS 2 Documentation: Jazzy - Installation <https://docs.ros.org/en/jazzy/Installation.html>`_
* `ROS 2 Documentation: Humble - Installation <https://docs.ros.org/en/humble/Installation.html>`_

.. note:: 2025年6月1日より、ROS2の公式のインストール方法が変わりました。それに伴い、このページの記述も更新しています。インストール方法の変更に関する詳細は `ROS signing key migration guide <https://discourse.ros.org/t/ros-signing-key-migration-guide/43937>`_ を参照ください。

2025年6月時点では、以下のコマンド操作でROS 2ディストリビューションをインストールできます。

**各バージョンに共通の準備**  ::

  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
  sudo apt install /tmp/ros2-apt-source.deb
  sudo apt update

使用するバージョンに応じて、以下のいずれかを実行してください。

**Ubuntu 24.04 LTSにROS 2 Jazzyをインストール** ::

  sudo apt install ros-jazzy-desktop
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  source ~/.bashrc

**Ubuntu 22.04 LTSにROS 2 Humbleをインストール** ::

  sudo apt install ros-humble-desktop
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc

.. note:: ROS 2はUTF-8をサポートしたロケール環境で動作します。Ubuntuを日本語環境となるようにインストールすると、通常は "ja_JP.UTF-8" のロケールとなって、UTF-8をサポートした環境になっているはずです。Dockerコンテナなどで必要最低限の環境にある場合は "POSIX" 等の最低限のロケールとなっている場合もあり得ますが、その場合はUTF-8をサポートしたロケールとなるように追加設定してください。詳細はROS 2公式ドキュメントのインストールのページを参照してください。

.. note:: 最後のsourceコマンドは setup.bash の内容を現在のシェルに反映させるためのものです。一連のコマンドによるインストールの直後に、続けて同じシェルで作業する場合に必要となります。インストール後にあらためてシェルを起動する場合は、bashrcにより setup.bash の内容が反映されますので、このコマンドを実行する必要はありません。

.. _ros2_install_ros2_install_dev_tools:

開発用ツールのインストール
-------------------------

ROS 2でChoreonoidを利用する際には、現状ではROS 2環境上で :doc:`build-choreonoid` を行う必要があります。
また、ロボットを動かすための制御プログラムなどをROSパッケージとして作成しなければならないこともあります。
このため、ROS 2用開発ツールのパッケージ "ros-dev-tools" を以下のコマンドでインストールしておきます。 ::

  sudo apt install ros-dev-tools

このパッケージをインストールすると、ROS 2用のビルドシステムである "colcon" もインストールされます。

rosdepの初期化
--------------

ROS 2で使用するパッケージを依存関係に基づきインストールするツールとして、rosdepがあります。
Choreonoidを使用する場合はrosdepを使うこともあるので、rosdepの初期化もしておきましょう。
rosdep自体は上記のros-dev-toolsパッケージでインストールされますが、インストール後に以下のコマンドで初期化をしておく必要があります。 ::

  sudo rosdep init
  rosdep update

※ 2行目のコマンドについてはsudoをつけずに実行します。

rosdepの詳細については以下をご参照ください。

* `ROS 2 Documentation: Jazzy - Managing Dependencies with rosdep <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html>`_
* `ROS 2 Documentation: Humble - Managing Dependencies with rosdep <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>`_
