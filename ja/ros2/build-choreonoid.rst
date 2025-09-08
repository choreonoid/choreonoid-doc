Choreonoid関連パッケージのビルド
===============================

ここではChoreonoidとROS 2を連携させるのに必要なROS 2パッケージのビルド方法について解説します。
なお、ChoreonoidとROS 2を用いて新たに構築するロボットシステムのROS 2パッケージも、この方法でビルドできます。

.. contents::
   :local:

.. highlight:: sh


ROS 2ワークスペースの作成
------------------------

ROS 2では新たなプログラムやデータの導入に、「ワークスペース」という作業ディレクトリを用います。
ワークスペース上でプログラムやデータを「パッケージ」と呼ばれる形態で構築していくことになります。

ここではChoreonoidとの連携用に新たにワークスペースを作成することにします。
ワークスペースは通常、ホームディレクトリ上に作成します。ここでは、ワークスペースの名前を "ros2_ws" とします。この名前は自由に設定可能です。異なるワークスペース名を使う場合は、以下の説明で登場する "ros2_ws" を、その名前に置き換えるようにしてください。

まず空のワークスペースを作成します。 ::

   mkdir -p ~/ros2_ws/src
   cd ros2_ws

既存のワークスペースがある場合は、そちらを用いてもかまいません。

.. note:: ROS 2ワークスペース上でのビルド作業には通常 "colcon" というビルドシステムを用います。colconのより詳細な説明は、ROS 2公式ドキュメントの以下のページをご参照ください。

 * `ROS 2 Documentation: Jazzy - Using colcon to build packages <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_
 * `ROS 2 Documentation: Humble - Using colcon to build packages <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_

.. _add_choreonoid_package_sources_for_ros2:

パッケージソースの追加
---------------------

ChoreonoidとROS 2を連携させるためには、choreonoid_rosというROS 2パッケージが必要になります。

このソースリポジトリをワークスペースの "src" ディレクトリにクローンして配置します。 ::

   cd src
   git clone https://github.com/choreonoid/choreonoid_ros.git

:doc:`install-choreonoid` で、Choreonoid本体は :ref:`ROS 2のパッケージとしてインストールする <ros2_install_choreonoid_install_as_ros2_package>` ことも可能であると述べました。その場合は、Choreonoid本体のソースリポジトリをやはりこのワークスペースのsrcディレクトリにクローンしておきます。 ::

  git clone https://github.com/choreonoid/choreonoid.git

※ Choreonoid本体を :ref:`ros2_install_choreonoid_standard_method` でインストール済みの場合は、これは行わないようにしてください。

他にもこの環境で使用したい追加のROS 2パッケージがあれば、そのソースリポジトリをここにクローンしておきます。

例えば、 :doc:`ros2-mobile-robot-tutorial` のソースリポジトリをクローンしておけば、このチュートリアルを試せるようになります。その場合は同じsrcディレクトリで以下のようにします。 ::

  git clone https://github.com/choreonoid/choreonoid_ros2_mobile_robot_tutorial.git

.. _install-choreonoid-ros2-dependencies:

choreonoid_rosの依存パッケージのインストール
-------------------------------------------

choreonoid_rosパッケージについては、ROS 2のいくつかのパッケージに依存するようになっており、それらのパッケージもインストールしておく必要があります。こちらはROS 2の方法で依存パッケージをインストールします。具体的には、rosdepコマンドを以下のように実行します。 ::

   rosdep install -y --from-paths ~/ros2_ws/src --ignore-src

このコマンドにより、choreonoid_rosの "package.xml" に記述されている依存パッケージが追加でインストールされることになります。
他にもパッケージがあれば、その依存パッケージもあわせてインストールされます。

.. note:: Choreonoid本体への依存を解決するため、 :ref:`ros2_install_choreonoid_register_to_rosdep` 必要があります。それを行っていない場合は、上記のrosdepコマンドに "--skip-keys choreonoid" オプションを追加し、Choreonoid本体への依存を一時的に無視することで、このコマンドを実行することが可能となります。

.. _ros2_colcon_build_command:

ビルド
------

.. 設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

以下のコマンドを用いてビルドを行いましょう。コマンドを実行するときのディレクトリは、ワークスペースのトップである必要があります。 ::

   cd ~/ros2_ws
   colcon build --symlink-install

ビルドオプションとして付けている `--symlink-install` は、インストール時に各種ファイルをシンボリックリンクを用いてインストールというものです。ファイルのコピーが生じない分、PCの記録容量の消費が少なく、またコンパイルが不要なファイルについては、編集した内容が直ちに反映されるという利点があります。例えば、Choreonoidでは .body ファイルや .project ファイル、ROS 2では .urdf ファイル や .yaml ファイルなどが、編集内容の即時反映の対象になります。

このコマンドのオプションの詳細は `colconの公式ドキュメント <https://colcon.readthedocs.io/en/released/index.html>`_ の `build - Build Packages <https://colcon.readthedocs.io/en/released/reference/verb/build.html>`_ を参照ください。

ビルドに成功すると、

.. code-block:: none

   Starting >>> choreonoid_ros
   Finished <<< choreonoid_ros [10.0s]

   Summary: 2 packages finished [10.0s]

といったメッセージが出力されます。

.. _loading_ros2_workspace_setup_script:

ワークスペースセットアップスクリプトの取り込み
--------------------------------------------

ビルドをすると、 ワークスペースのinstallディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

   source $HOME/ros2_ws/install/setup.bash

という記述を追加しておきます。そうすると、端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

上記コマンドの追加は、以下のコマンドで追加可能です。 ::

   echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のsourceコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。

.. note:: このスクリプトは :doc:`install-ros2` で導入したROS 2本体のsetup.bashとは **異なります** ので注意してください。ワークスペース上のパッケージを正常に動作させるためには、どちらのスクリプトも読み込んでおく必要があります。

参考：リポジトリ管理ツールの使用
-------------------------------

リポジトリ管理ツールである `vcstool <https://github.com/dirk-thomas/vcstool>`_  を使用することで、複数リポジトリのクローンや更新などを一括して行えます。

vcstoolのインストールは以下のコマンドで行えます（ :ref:`ros2_install_ros2_install_dev_tools` でインストールされます。） ::

   sudo apt install python3-vcstool

使い方は ::

   vcs help

で確認してください。

各リポジトリよりも上位にあるディレクトリで ::

 vcs pull

を実行すると、全てのリポジトリに対して git pull が実行され、全てのリポジトリを最新のものに更新することができます。

例えば、以下のコマンドで、 :ref:`add_choreonoid_package_sources_for_ros2` で導入した choreonoid および choreonoid_ros を含む、 "src" ディレクトリ内の全てのクローンを最新版に更新できます。 ::

   cd ~/ros2_ws
   vcs pull src

.. _ros2_catkin_config_cmake_build_type:

参考：ビルドタイプの設定
-----------------------

一般的に、C/C++のプログラムをビルドする際には、"Release" や "Debug" といったビルドのタイプを指定することができます。Release（リリースモード）の場合は最適化が適用されて実行速度が速くなりますし、Debug（デバッグモード）の場合はデバッグ情報が付与されてデバッガによるデバッグがしやすくなります。

colconコマンドでビルドする際にこれらのビルドタイプを指定したい場合は、CMakeのオプションを設定するための--cmake-args オプションを使用します。

例えば ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

とすればリリースモードでビルドすることができますし、 ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

とすればデバッグモードになります。

Choreonoid関連のパッケージはデフォルトでReleaseが設定されるようにしてあります。しかし一般的には、パッケージによってはデフォルトでビルドタイプをReleaseに設定しないものもありますし、自前のパッケージでそこまで設定していないこともあるかもしれません。その場合最適化が適用されず、ビルドされたプログラムの実行速度が大幅に落ちることになってしまいます。そのようなパッケージをビルドする可能性がある場合は、上記の方法でReleaseビルドを指定しておくとよいでしょう。

.. _ros2_build_choreonoid_as_ros2_package_note:

参考：Choreonoid本体をワークスペース上でビルドする際の注意
---------------------------------------------------------

Choreonoid本体を :ref:`ros2_install_choreonoid_standard_method` でインストールした上で、更に :ref:`ROS 2のパッケージとしてインストールする <ros2_install_choreonoid_install_as_ros2_package>` 場合は、両者の干渉に注意が必要です。

ROS 2環境のセットアップスクリプトがシステムに読み込まれると、共有ライブラリのパスにROS 2ワークスペースの該当するディレクトリが加わります。（環境変数LD_LIBRARY_PATHに追加されます。）この状態では、システムに同じ名前の共有ライブラリが複数あった場合、通常ROS 2環境のものが優先して読み込まれることになります。元々ROS 2とは独立にインストールされているソフトウェアについて、これが適用されると、バージョンやビルド設定が異なるライブラリが読み込まれてしまい、ソフトウェアが正常に動作しなくなることがあります。複数の環境を混ぜて使うのは危険ということです。

これを避けるためには、上記の :ref:`loading_ros2_workspace_setup_script` や :doc:`install-ros2` で述べた "setup.bash" スクリプトの取り込みについて、ROS 2とは独立したソフトウェアを使用する際には無効にしておくのが無難です。設定ファイル ".bashrc" の該当部分をコメントアウトするなどしてから、OSや端末を起動し直すことで、無効にすることができます。

なお、Choreonoidに関しては、実行ファイルや共有ライブリファイルの中に埋め込まれたRPATHという情報により、他の環境でビルドされたライブラリと混ざらないように実行することが可能となっています。この機能はビルドディレクトリ内に生成される実行ファイルやライブラリに関してはデフォルトで有効になります。また、CMakeのENABLE_INSTALL_RPATHをONにすることで、"make install" によってインストールされるファイルに関してもこれが有効になります。

そのような仕組みによって、Choreonoidの共有ライブラリは他の環境のものとなるべく混ざらないようにはなっています。ただし環境設定によってはやはり混ざってしまうこともあり得ますし、Choreonoidと連携させて使用する他のソフトウェアにおいてライブラリが混ざってしまう可能性もあります。したがって、Choreonoidに限らない話として、同じソフトウェアが同一OS上で複数の環境にインストールされている場合、それらが混ざらないように使用するということが、不具合を避けるにあたって大変重要です。

.. note:: Choreonoidビルド時のCMakeのオプションで "ENABLE_NEW_DTAGS" をONにすると、RPATHよりもLD_LIBRARY_PATHの情報が優先されるようになり、混ざってしまう危険性が高くなります。このオプションは特に必要が無い場合はデフォルトのOFFのままとしてください。
