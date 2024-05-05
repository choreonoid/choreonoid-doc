ROS 2のインストール
=================

.. contents::
   :local:

.. highlight:: sh

ROS 2ディストリビューションのインストール
---------------------------------------

ROS 2のディストリビューションがまだインストールされていない場合は、 `ROS 2 Documentation: Humble - Installation <https://docs.ros.org/en/humble/Installation.html>`_ に従ってインストールを行ってください。

2024年4月時点では、以下のコマンド操作でROS環境をインストールできます。最新のインストール方法については、上記ドキュメントを参照してください。

.. http://wiki.ros.org/noetic/Installation/Ubuntu

**Ubuntu 22.04 LTS (Jammy Jellyfish) に ROS 2 Humbleをインストールする場合** ::

  # Add the ROS 2 apt repository
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  # Install ROS 2 packages
  sudo apt update
  sudo apt upgrade
  sudo apt install ros-humble-desktop

  # Sourcing the setup script (for bash)
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc

.. note:: ROS 2はUTF-8をサポートしたロケール環境で動作します。Ubuntuを日本語環境となるようにインストールすると、通常は "ja_JP.UTF-8" のロケールとなって、UTF-8をサポートした環境になっているはずです。Dockerコンテナなどで必要最低限の環境にある場合は "POSIX" 等の最低限のロケールとなっている場合もあり得ますが、その場合はUTF-8をサポートしたロケールとなるように追加設定してください。詳細はROS 2公式ドキュメントのインストールのページを参照してください。

.. note:: 最後のsourceコマンドは setup.bash の内容を現在のシェルに反映させるためのものです。一連のコマンドによるインストールの直後に、続けて同じシェルで作業する場合に必要となります。インストール後にあらためてシェルを起動する場合は、bashrcにより setup.bash の内容が反映されますので、このコマンドを実行する必要はありません。

.. note:: 上記の設定でROS 2のリポジトリを登録し、その後しばらくの期間が経過すると、ROS 2リポジトリにアクセスできなくなってしまうことがあるようです。その場合は関連するエラーによってOSのパッケージ更新もできなくなってしまうことがあります。これを解決するためには、上記の公式ドキュメントに書かれた最新の情報に従って、リポジトリ用の鍵を更新などをお試しください。それによって問題が解決することがあります。


Colconのインストール
--------------------------

ChoreonoidをROS 2で使う場合、ROS 2向けに開発されたビルドツールであるcolconを使用します。

Ubuntu 22.04では、以下のコマンドで必要なパッケージをインストールすることで、colconを使えるようになります。 ::

   sudo apt install python3-colcon-common-extensions

colconのより詳細な説明は、公式ドキュメント `ROS 2 Documentation: Humble - Using colcon to build packages <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_ をご覧ください。
