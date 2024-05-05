Choreonoid関連パッケージのビルド
================================

ここではROS 2のパッケージの一つとして、Choreonoidをビルド（インストール）します。あわせていくつかのChoreonoidに関連するパッケージもビルドします。

本ドキュメントでは :doc:`../install/build-ubuntu` とは異なる手順でChoreonoidをインストールします。既にそちらの手順でインストール済みのChoreonoidがあったとしても、それとは独立してROS 2用のChoreonoidを別途インストールすることになりますので、ご注意ください。

ご興味のある方は、ROS 2が提供するcolconの公式ドキュメント `ROS 2 Documentation: Humble - Using colcon to build packages <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_ もあわせてご覧ください。

.. contents::
   :local:

.. highlight:: sh


ROS 2ワークスペースの作成
--------------------------

Choreonoid用のROS 2ワークスペースを作成します。

ワークスペースは通常、ホームディレクトリ上に作成します。ここでは、ワークスペースの名前を "ros2_ws" とします。この名前は自由に設定可能です。異なるワークスペース名を使う場合は、以下の説明で登場する "ros2_ws" を、その名前に置き換えるようにしてください。

まず空のワークスペースを作成します。 ::

   mkdir -p ~/ros2_ws/src
   cd ros2_ws

.. _add_choreonoid_package_sources_for_ros2:

パッケージソースの追加
----------------------

作成したワークスペースの "src" ディレクトリ内に、Choreonoid本体とROS 2プラグインのソースコードリポジトリをクローンします。 ::

   cd src
   git clone https://github.com/choreonoid/choreonoid.git
   git clone https://github.com/choreonoid/choreonoid_ros.git

これらはそれぞれ、以下のGithubリポジトリに対応します。

* `choreonoid <https://github.com/choreonoid/choreonoid>`_ : Choreonoid本体
* `choreonoid_ros <https://github.com/choreonoid/choreonoid_ros>`_ : ChoreonoidでROS 1/2の機能を使用するためのROSパッケージ

.. note:: Choreonoid の ROS 2 連携機能は、バージョン2.1.1以上の Choreonoid を対象としています。上記リポジトリのクローンでは、最新の開発版を使用します。

各リポジトリの内容はなるべく最新に保つようにしてください。


依存パッケージのインストール
----------------------------

Choreonoidのビルドや実行に必要となる依存パッケージをインストールします。

Choreonoidのソースディレクトリに移動して、対応するスクリプトを実行します。Ubuntu 22.04であれば、 ::

   misc/script/install-requisites-ubuntu-22.04.sh

を実行します。

なお、OS上でROS 2とは独立して既に最新のChoreonoidをインストールしている場合、この作業を改めて実行する必要はありません。


.. _ros2_colcon_build_command:

ビルド
------

.. 設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

以下のコマンドを用いてビルドを行いましょう。コマンドを実行するときのディレクトリは、ワークスペースのトップである必要があります。 ::

   cd ~/ros2_ws
   colcon build --symlink-install

ビルドオプションとして付けている `--symlink-install` は、インストール時に各種ファイルをシンボリックリンクを用いてインストールします。ファイルのコピーが生じない分、PCの記録容量の消費が少なく、またコンパイルが不要なファイルについては、編集した内容が直ちに反映されるという利点があります。例えば、Choreonoidでは .body ファイルや .project ファイル、ROS 2では .urdf ファイル や .yaml ファイルなどが、編集内容の即時反映の対象になります。

colconコマンドのオプションの詳細については `colconのドキュメント <https://colcon.readthedocs.io/en/released/reference/verb/build.html>`_ を参照してください。

ビルドに成功すると、

.. code-block:: none

   Starting >>> choreonoid
   Finished <<< choreonoid
   Starting >>> choreonoid_ros
   Finished <<< choreonoid_ros

   Summary: 2 packages finished

と表示されます。

なお、colconコマンドではCMakeオプションの設定が可能です。詳しくは :ref:`ros2_build_choreonoid_cmake_options` をご覧ください。

.. _loading_ros2_workspace_setup_script:

ワークスペースセットアップスクリプトの取り込み
----------------------------------------------

ビルドをすると、 ワークスペースのinstallディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

   source $HOME/ros2_ws/install/setup.bash

という記述を追加しておきます。そうすると、端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

上記コマンドの追加は、以下のコマンドで追加可能です。 ::

   echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のsourceコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。

.. note:: このスクリプトは :doc:`install-ros2` で導入したROS 2本体のsetup.bashとは **異なります** ので注意してください。ワークスペース上のパッケージを正常に動作させるためには、どちらのスクリプトも読み込んでおく必要があります。

.. 補足: 複数のChoreonoid環境の併用について
.. ----------------------------------------

.. ここではROS環境（Catkinワークスペース）上で動作するChoreonoidのインストール方法を紹介しました。冒頭でも述べたように、ChoreonoidはROSとは独立してインストールすることも可能です。ただしそれらを同じOS上で併用する場合は、注意が必要です。

.. ROS環境のセットアップスクリプトがシステムに読み込まれると、共有ライブラリのパスにROS（Catkin）の該当するディレクトリが加わります。（環境変数LD_LIBRARY_PATHに追加されます。）この状態では、システムに同じ名前の共有ライブラリが複数あった場合、通常ROS環境のものが優先して読み込まれることになります。元々ROSとは独立にインストールされているソフトウェアについて、これが適用されると、バージョンやビルド設定が異なるライブラリが読み込まれてしまい、ソフトウェアが正常に動作しなくなることがあります。複数の環境を混ぜて使うのは大変危険ということです。

.. これを避けるためには、上記の :ref:`loading_catkin_workspace_setup_script` や :doc:`install-ros` で述べたsetup.bashスクリプトの取り込みについて、ROSとは独立したソフトウェアを使用する際には無効にしておくのが無難です。.bashrc の該当部分をコメントアウトするなどしてから、OSや端末を起動し直すことで、無効にすることができます。

.. なお、Choreonoidに関しては、実行ファイルや共有ライブリファイルの中に埋め込まれたRPATHという情報により、他の環境でビルドされたライブラリと混ざらないように実行することが可能となっています。この機能はビルドディレクトリ内に生成される実行ファイルやライブラリに関してはデフォルトで有効になります。（ただし比較的新しいUbuntuのバージョンに関しては `この更新 <https://github.com/choreonoid/choreonoid/commit/7f7900c3ec945f9da97b0e2ee484c1ddfe63d978>`_  以降であることが必要。）また、CMakeのENABLE_INSTALL_RPATHをONにすることで、"make install" によってインストールされるファイルに関してもこれが有効になります。

.. 上記の更新以降では、CMakeのオプションで ENABLE_NEW_DTAGS というオプションが追加されています。これはデフォルトではOFFですが、ONにするとRPATHよりもLD_LIBRARY_PATHの情報が優先されるようになり、混ざってしまう危険性が高くなります。このオプションは特に必要が無い場合はOFFのままとしてください。

.. そのようにChoreonoidではなるべく共有ライブラリが混ざらないようにするための仕組みがありますが、環境設定によってはやはり混ざってしまうこともあり得ますし、Choreonoidと連携させて使用する他のソフトウェアにおいてライブラリが混ざってしまう可能性もあります。したがって、Choreonoidに限らない話として、同じソフトウェアが同一OS上で複数の環境にインストールされている場合、それらが混ざらないように使用するということが、不具合を避けるにあたって大変重要です。

参考：パッケージ管理ツールの使用
--------------------------------

ROS 2では、複数のパッケージをまとめて管理する標準ツールとして、 `vcstool <https://github.com/dirk-thomas/vcstool>`_  があります。これを使用することで、複数リポジトリのクローンや更新などを一括して行えます。

vcstoolのインストールは以下のコマンドで行えます。 ::

   sudo apt install python3-vcstool


使い方は ::

   vcs help

で確認してください。

各リポジトリよりも上位にあるディレクトリで ::

 vcs pull

を実行すると、全てのリポジトリに対して git pull が実行され、全てのリポジトリを最新のものに更新することができます。

例えば、以下のコマンドで、 :ref:`add_choreonoid_package_sources_for_ros2` で導入した choreonoid および choreonoid_ros を含む、 "src" ディレクトリ内の全てのクローンを最新版に更新できます。 ::

   cd ~/ros2_ws
   vsc pull src


.. _ros2_build_choreonoid_cmake_options:

参考：CMakeオプションの設定
---------------------------

ChoreonoidのビルドにおいてCMakeのオプションを設定したい場合は、colconコマンドの "--cmake-args" オプションを使用します。

例えば、Choreonoidの通常の実行ファイルの生成を禁止するオプションを設定できます。ROS 2連携時は、 choreonoid_ros パッケージがChoreonoidの実行ファイルを生成します。そのため、Choreonoidの通常の実行ファイルと、ROS 2用の実行ファイルの両方があることになります。 :ref:`ros2_colcon_build_command` で紹介したビルドコマンドの代わりに、以下のように "BUILD_CHOREONOID_EXECUTABLE" オプションをOFFにしてビルドを行うことで、前者の、通常の実行ファイルは生成されなくなります。 ::

   colcon build --symlink-install --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF


CMakeオプションを利用して、Choreonoidのオプションのプラグインを有効にすることも可能です。例えばChoreonoid上で動画や音声のファイルを再生するための「メディアプラグイン」を利用したい場合は、以下のようにします。 ::

   colcon build --symlink-install --cmake-args -DBUILD_MEDIA_PLUGIN=ON

複数のオプションを設定したい場合、オプションを列挙すればOKです。例えば以下のコマンドで通常の実行ファイルの生成禁止とメディアプラグインのビルドを両方設定できます。 ::

   colcon build --symlink-install --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

.. note:: この設定方法では、ワークスペースの全てのパッケージに対してこれらのオプションが有効になってしまい、他のパッケージで意図しないオプションが有効になってしまうことに注意が必要です。

.. note:: 上記のBUILD_MEDIA_PLUGINオプションはあくまで説明のための例として挙げたもので、ChoreonoidとROSを使用する際に必ずしも必要なものではありません。動画などのメディアファイルをChoreonoid上で再生する必要がなければ、このオプションはONにしなくて結構です。


このように、ROS 2環境でもCMakeのオプションを設定できます。ROS 2環境で使いたいオプションがあれば、適宜そちらを有効にするようにしてください。

.. _ros2_catkin_config_cmake_build_type:

参考：ビルドタイプの設定
------------------------

一般的に、C/C++のプログラムをビルドする際には、"Release" や "Debug" といったビルドのタイプを指定することができます。Release（リリースモード）の場合は最適化が適用されて実行速度が速くなりますし、Debug（デバッグモード）の場合はデバッグ情報が付与されてデバッガによるデバッグがしやすくなります。

colconコマンドでビルドする際にこれらのビルドタイプを指定したい場合は、やはり --cmake-args オプションを使用します。

例えば ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

とすればリリースモードでビルドすることができますし、 ::

   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

とすればデバッグモードになります。

これらは :ref:`ros2_build_choreonoid_cmake_options` で指定するオプションに追加して指定するようにしてください。

Choreonoid関連のパッケージはデフォルトでReleaseが設定されるようにしてあります。しかし一般的には、パッケージによってはデフォルトでビルドタイプをReleaseに設定しないものもありますし、自前のパッケージでそこまで設定していないこともあるかもしれません。その場合最適化が適用されず、ビルドされたプログラムの実行速度が大幅に落ちることになってしまいます。そのようなパッケージをビルドする可能性がある場合は、上記の方法でReleaseビルドを指定しておくとよいでしょう。

参考：複数のChoreonoid環境の併用について
----------------------------------------

ここではROS 2環境（ROS 2ワークスペース）上で動作するChoreonoidのインストール方法を紹介しました。冒頭でも述べたように、ChoreonoidはROS 2とは独立してインストールすることも可能です。ただしそれらを同じOS上で併用する場合は、若干注意が必要です。

ROS 2環境のセットアップスクリプトがシステムに読み込まれると、共有ライブラリのパスにROS 2ワークスペースの該当するディレクトリが加わります。（環境変数LD_LIBRARY_PATHに追加されます。）この状態では、システムに同じ名前の共有ライブラリが複数あった場合、通常ROS 2環境のものが優先して読み込まれることになります。元々ROS 2とは独立にインストールされているソフトウェアについて、これが適用されると、バージョンやビルド設定が異なるライブラリが読み込まれてしまい、ソフトウェアが正常に動作しなくなることがあります。複数の環境を混ぜて使うのは危険ということです。

これを避けるためには、上記の :ref:`loading_ros2_workspace_setup_script` や :doc:`install-ros2` で述べた "setup.bash" スクリプトの取り込みについて、ROS 2とは独立したソフトウェアを使用する際には無効にしておくのが無難です。設定ファイル ".bashrc" の該当部分をコメントアウトするなどしてから、OSや端末を起動し直すことで、無効にすることができます。

なお、Choreonoidに関しては、実行ファイルや共有ライブリファイルの中に埋め込まれたRPATHという情報により、他の環境でビルドされたライブラリと混ざらないように実行することが可能となっています。この機能はビルドディレクトリ内に生成される実行ファイルやライブラリに関してはデフォルトで有効になります。また、CMakeのENABLE_INSTALL_RPATHをONにすることで、"make install" によってインストールされるファイルに関してもこれが有効になります。

そのような仕組みによって、Choreonoidの共有ライブラリは他の環境のものとなるべく混ざらないようにはなっています。ただし環境設定によってはやはり混ざってしまうこともあり得ますし、Choreonoidと連携させて使用する他のソフトウェアにおいてライブラリが混ざってしまう可能性もあります。したがって、Choreonoidに限らない話として、同じソフトウェアが同一OS上で複数の環境にインストールされている場合、それらが混ざらないように使用するということが、不具合を避けるにあたって大変重要です。

.. note:: Choreonoidビルド時のCMakeのオプションで "ENABLE_NEW_DTAGS" をONにすると、RPATHよりもLD_LIBRARY_PATHの情報が優先されるようになり、混ざってしまう危険性が高くなります。このオプションは特に必要が無い場合はデフォルトのOFFのままとしてください。

