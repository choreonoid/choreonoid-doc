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

2024年10月時点では、以下のコマンド操作でROS 2ディストリビューションをインストールできます。

**各バージョンに共通の準備**  ::

  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt upgrade

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

.. note:: 上記の設定でROS 2のリポジトリを登録し、その後しばらくの期間が経過すると、ROS 2リポジトリにアクセスできなくなってしまうことがあるようです。その場合は関連するエラーによってOSのパッケージ更新もできなくなってしまうことがあります。これを解決するためには、上記の公式ドキュメントに書かれた最新の情報に従って、リポジトリ用の鍵の更新などをお試しください。それによって問題が解決することがあります。


Colconのインストール
--------------------------

ChoreonoidをROS 2で使う場合、ROS 2向けに開発されたビルドツールであるcolconを使用します。
これは以下のコマンドで必要なパッケージがインストールされて、使用できるようになります。 ::

   sudo apt install python3-colcon-common-extensions
