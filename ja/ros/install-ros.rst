ROSのインストール
=================

.. contents::
   :local:

.. highlight:: sh

ROSディストリビューションのインストール
---------------------------------------

ROSのディストリビューションがまだインストールされていない場合は、 `ROS.org <http://wiki.ros.org>`_ - `ROS/Installation <http://wiki.ros.org/ROS/Installation>`_ の記述に従ってインストールを行ってください。

なお、ROSのバージョンについては、Noetic Ninjemys (Ubuntu 20.04)、Melodic Morenia (Ubuntu 18.04)での動作を確認をしています。

それぞれ以下のコマンド操作でROS環境をインストールできます。（最新のインストール方法については、上記のRSO.orgの情報を参照してください。）

.. http://wiki.ros.org/noetic/Installation/Ubuntu

**Ubuntu 20.04 (ROS Noetic Ninjemys) の場合** ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt install curl # if you haven't already installed curl
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo apt update
 sudo apt install ros-noetic-desktop-full
 echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
 sudo rosdep init
 rosdep update

**Ubuntu 18.04 (ROS Melodic Morenia) の場合** ::

 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 sudo apt install curl # if you haven't already installed curl
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo apt update
 sudo apt install ros-melodic-desktop-full
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
 source ~/.bashrc
 sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
 sudo rosdep init
 rosdep update

.. note:: 上記のsourceコマンドはsetup.bash の内容を現在のシェルに反映させるためのもので、インストール（上記設定）直後に続けて同じシェルで作業する場合に必要となるものです。インストール後にあらためてシェルを起動する場合は、上記の設定によりsetup.bashの内容が反映されますので、このコマンドは必要ありません。

.. note:: 上記の設定でROSのリポジトリを登録し、その後しばらくの期間が経過すると、ROSリポジトリにアクセスできなくなってしまうことがあるようです。その場合は関連するエラーによってOSのパッケージ更新もできなくなってしまうことがあります。これを解決するためには、上記のROS.orgの該当ページで最新の情報に従ってリポジトリ用の鍵を更新するなどしてみてください。それによって問題が解決することがあります。


Catkin Toolsのインストール
--------------------------

ChoreonoidをROSで使う場合、ビルドツールCatkinの新しいバージョン ( `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ 、通称Catkin Tools）を使用します。

Ubuntu 20.04の場合、以下のようにして必要なパッケージをインストールすることでCatkin Toolsを使えるようになります。 ::

 sudo apt install python3-osrf-pycommon python3-catkin-tools

Ubuntu 18.04の場合はPythonの環境が異なるため、インストールは以下のコマンドで行うことになります ::

 sudo apt install python-catkin-tools
