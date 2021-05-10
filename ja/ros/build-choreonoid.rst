Choreonoid関連パッケージのビルド
================================

ここではROS環境におけるパッケージとしてChoreonoidをビルド（インストール）します。あわせていくつかのChoreonoid関連パッケージもビルドします。

本ドキュメントでは :doc:`../install/build-ubuntu` とは異なる手順でChoreonoidをインストールします。既にそちらの手順でインストール済みのChoreonoidがあったとしても、それとは独立してROS用のChoreonoidを別途インストールすることになりますので、ご注意ください。通常の手順でインストールされたChoreonoidをROS環境で使用することも可能なのですが、そちらについては必要なROSパッケージやドキュメントを現在整備中ですので、当面は本ドキュメントの解説に従って、ROS用のChoreonoidをインストールするようにしてください。ここでインストールするChoreonoidは、通常の手順でインストールされたものとは区別し、それぞれ独立に管理するようにしてください。

.. contents::
   :local:

.. highlight:: sh

.. _ros_make_catkin_workspace:

Catkinワークスペースの作成
--------------------------

Choreonoid用のCatkinワークスペースを作成します。

ワークスペースは通常これはホームディレクトリ上に作成します。ワークスペースの名前は通常 "catkin_ws" とします。この名前は変更しても結構ですが、その場合は以下の説明の "catkin_ws" をその名前に置き換えるようにしてください。

まず空のワークスペースを作成します。 ::

 mkdir catkin_ws
 cd catkin_ws
 mkdir src
 catkin init

.. note:: ここではワークスペースの初期化に "catkin init" というコマンドを使っていますが、同様の操作を行うコマンドとして "catkin_init_workspace" というコマンドもあります。前者はCatkinの新しいコマンド体系である `Catkin Command Line Tools <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ に含まれるコマンドで、後者はCatkinの古い形式のコマンドです。両者はパッケージのビルドに使用するコマンドも異なっており、それぞれ "catkin build" と "catkin_make" になります。ChoreonoidのROS連携は Catkin Command Line Tools の使用を前提としていますので、古い形式のCatkinコマンド（catkin_make等）は使用しないでください。

.. _ros_choreonoid_add_package_sources:

パッケージソースの追加
----------------------

作成したワークスペースの "src" ディレクトリ内に、Choreonoid本体とROSプラグインのソースコードリポジトリをクローンします。 ::

 cd src
 git clone https://github.com/choreonoid/choreonoid.git
 git clone https://github.com/choreonoid/choreonoid_ros.git

それぞれ以下のGithubリポジトリに対応しています。

* `choreonoid <https://github.com/choreonoid/choreonoid>`_ : Choreonoid本体
* `choreonoid_ros <https://github.com/choreonoid/choreonoid_ros>`_ : ChoreonoidでROSの機能を使用するためのROSパッケージ

.. note:: ChoreonoidのROS連携機能は現状ではChoreonoid開発版を対象としています。上記リポジトリのクローンにより最新の開発版を使用することができます。

また、次節以降の解説を参照する場合は、そこで使用するサンプルもクローンしておきましょう。 ::

 git clone https://github.com/choreonoid/choreonoid_ros_samples.git
 git clone https://github.com/choreonoid/choreonoid_joy.git

それぞれ以下のGithubリポジトリに対応しています。

* `choreonoid_ros_samples <https://github.com/choreonoid/choreonoid_ros_samples>`_ : ChoreonoidでROSを使用するサンプル
* `choreonoid_joy <https://github.com/choreonoid/choreonoid_joy>`_ : ジョイスティック（ゲームパッド）をChoreonoidのマッピングで使うためのROSノード

各リポジトリの内容はなるべく最新に保つようにしてください。

.. note:: ChoreonoidのROS関連パッケージとしては、上に挙げたもの以外にも、 `choreonoid_ros_pkg <https://github.com/fkanehiro/choreonoid_ros_pkg>`_ や `choreonoid_rosplugin <https://github.com/s-nakaoka/choreonoid_rosplugin>`_ があります。これらは現在公式にサポートされているものではないので、本マニュアルの方法でChoreonoidを使用する際には導入しないようにしてください。これらがバイナリパッケージとしてインストールされていたり、catkinのワークスペースに含まれていたりすると、本ドキュメントで解説する機能がうまく動作しない可能性があります。本マニュアルの記述に従う場合は、ワークスペースを新規に作成して、まずは指定されたパッケージのみを導入し、動作を確認するようにしてください。


リポジトリ管理ツールの使用
--------------------------

複数のリポジトリをまとめて管理するためののツールとして、 `wstool <http://wiki.ros.org/wstool>`_ や `vcstool <https://github.com/dirk-thomas/vcstool>`_  があります。これらを使用することでの複数リポジトリの更新なども一括して行うことができるので、活用されるとよいかと思います。

ここではvcstoolについて簡単に紹介します。vcstoolを使う場合は、 ::

 sudo apt install python3-vcstool

でインストールできます。

使い方は ::

 vcs help

で確認してください。

各リポジトリよりも上位にあるディレクトリで ::

 vcs pull

を実行すると、全てのリポジトリに対して git pull が実行され、全てのリポジトリを最新のものに更新することができます。

依存パッケージのインストール
----------------------------

Choreonoidのビルド・実行に必要となる依存パッケージをインストールしておきます。

Choreonoidのソースディレクトリに移動し対応するスクリプトを実行します。Ubuntu 20.04であれば、 ::

 misc/script/install-requisites-ubuntu-20.04.sh

を実行します。

Ubuntu 18.04、16.04の場合はそれぞれ

* misc/script/install-requisites-ubuntu-18.04.sh
* misc/script/install-requisites-ubuntu-16.04.sh

を実行してください。

この処理は本来Catkin用の依存パッケージ情報で解決すべきなのですが、Choreonoidについてはそこがまだ完全でない部分があり、インストールを確実にするため、この作業を行っておく必要があります。

なお、OS上でROSとは独立して既に最新のChoreonoidをインストールしている場合この作業は適用済みのはずですので、あらためて実行する必要はありません。

.. _ros_build_choreonoid_cmake_options:

CMakeオプションの設定
---------------------

ChoreonoidのビルドにおいてCMakeのオプションを設定したい場合は、catkin の config コマンドの "--cmake-args" オプションを使用します。

まず、 ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF

によって、Choreonoidの通常の実行ファイルの生成を禁止しておくとよいです。ROSでは通常の実行ファイルの代わりに「ノード」と呼ばれる形態でプログラムを起動するようになっており、Choreonoid用のノード実行ファイルはchoreonoid_rosパッケージに含まれています。Choreonoidの通常の実行ファイルとノード版の実行ファイルの両方があると混乱してしまう可能性がありますが、上記のオプションによりこれを回避することができます。

Choreonoidのオプションのプラグインを有効にすることも可能です。例えばChoreonoid上で動画や音声のファイルを再生するための「メディアプラグイン」を利用したい場合は、以下のようにします。 ::

 catkin config --cmake-args -DBUILD_MEDIA_PLUGIN=ON

複数のオプションを設定したい場合、オプションを列挙すればOKです。例えば以下のコマンドで通常の実行ファイルの生成禁止とメディアプラグインのビルドを両方設定できます。 ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

設定後 ::

 catkin config

を実行すると、ワークスペースの設定が表示されます。そこに

.. code-block:: none

 Additional CMake Args: -DBUILD_CHOREONOID_EXECUTABLE=OFF -DBUILD_MEDIA_PLUGIN=ON

といった表示があればOKです。

.. note:: このように設定すると、ワークスペースの全てのパッケージに対してこれらのオプションが有効になってしまい、他のパッケージで意図しないオプションが有効になってしまうこともあり得ます。しかしCatkinではパッケージごとに個別にCMakeのオプションを設定する機能が無い（ `要望はあるものの見送られている <https://github.com/catkin/catkin_tools/issues/205>`_ ）ようですので、やむを得ずこのようにしています。

.. note:: 上記のBUILD_MEDIA_PLUGINオプションはあくまで説明のための例として挙げたもので、ChoreonoidとROSを使用する際に必ずしも必要なものではありません。動画などのメディアファイルをChoreonoid上で再生する必要がなければ、このオプションはONにしなくて結構です。

設定したオプションを解除したい場合は ::

 catkin config --no-cmake-args

を実行します。

以上の方法でCMakeのオプションを設定できますので、ROS環境で使いたいオプションがあればそちらを有効にするようにしてください。

.. _note_on_ros_python_version:

Pythonバージョンの設定
^^^^^^^^^^^^^^^^^^^^^^

ChoreonoidではデフォルトでPythonプラグインとPython用ラッパライブラリがビルドされますが、そこで使用するPythonのバージョンには注意が必要です。ChoreonoidではデフォルトでPython3を使用するようになっていますが、ROSの従来のバージョン、具体的にはUbuntu 18.04用のMelodic Morenicaまでは、Python2（バージョン2.7）が使用されています。そのようなROSのバージョンを使用する場合、そのままではChoreonoidのPython3とROSのPython2が競合してしまい、不具合が生じることになります。

.. note:: Ubuntu 20.04に対応するROSのNoetic NinjemysからはPython3が使用されるようになった模様で、デフォルトの設定で不具合は生じないものと思われます。Ubuntu 20.04の場合は以下の説明は読み飛ばしてください。

Python2を使用する従来のROSバージョンにおいては、ChoreonoidでもPython2を使用するように設定しておきます。これはChoreonoidビルド時のCMakeでUSE_PYTHON3 というオプションをOFFにすればOKです。そのようにするとChoreonoidでもPythonバージョン2が使用されるようになります。

catkin においては ::

 catkin config --cmake-args -DUSE_PYTHON3=OFF

とすることでこれを実現できます。

あるいは、ChoreonoidのPython機能が必要ない場合は、以下のようにしてPython機能自体をオフにしてしまってもよいかと思います。 ::

 catkin config --cmake-args -DENABLE_PYTHON=OFF

.. _ros_catkin_build_type:

ビルドタイプの設定
------------------

一般的に、C/C++のプログラムをビルドする際には、"Release" や "Debug" といったビルドのタイプを指定することができます。Release（リリースモード）の場合は最適化が適用されて実行速度が速くなりますし、Debug（デバッグモード）の場合はデバッグ情報が付与されてデバッガによるデバッグがしやすくなります。

Catkin上でビルドする際にこれらのビルドタイプを指定したい場合は、やはり --cmake-args オプションを使用します。

例えば ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

とすればリリースモードでビルドすることができますし、 ::

 catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug

とすればデバッグモードになります。

これらは :ref:`ros_build_choreonoid_cmake_options` で指定するオプションに追加して指定するようにしてください。

Choreonoid関連のROSパッケージはデフォルトでReleaseが設定されるようにしてありますが、パッケージによってはデフォルトでビルドタイプをReleaseに設定しないものもありますし、自前のパッケージでそこまで設定していないこともあるかもしれません。その場合最適化が適用されず、ビルドされたプログラムの実行速度が大幅に落ちることになってしまいますので、そのようなパッケージをビルドする可能性がある場合は、上記の方法でReleaseビルドを指定しておくとよいです。

.. _ros_catkin_build_command:

ビルド
------

ここまで :ref:`ros_build_choreonoid_cmake_options` や :ref:`ros_catkin_build_type` について説明しましたが、細かいオプションについてよく分からない場合はとりあえず以下の設定としておきましょう。

**Ubuntu 20.04 (ROS Noetic Ninjemys) の場合** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DCMAKE_BUILD_TYPE=Release

**Ubuntu 18.04 (ROS Melodic Morenia) 以前の環境の場合** ::

 catkin config --cmake-args -DBUILD_CHOREONOID_EXECUTABLE=OFF -DUSE_PYTHON3=OFF -DCMAKE_BUILD_TYPE=Release

設定が完了したら、ビルドを行いましょう。ワークスペース内のディレクトリであれば、以下のコマンドでビルドできます。 ::

 catkin build

ビルド方法の詳細については `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ を参照してください。

ビルドに成功すると、

.. code-block:: none

 [build] Summary: All 4 packages succeeded!

といった表示がされます。

.. note:: Emacsでは "M-x compile" コマンドでビルドを行うことが可能ですが、Catkin環境でもこの機能を利用することができます。ただしCatkinの出力は通常色付けされるのですが、Emacs上ではその制御コードが表示されてしまい、そのままでは表示が見にくくなってしまいます。これを回避するため、 "M-x compile" 実行時にビルド用のコマンドとして "catkin build --no-color" を入力するとよいです。"--no-color" を入れることで、Cakin出力の色付け用の制御コードが無効化され、表示の乱れがなくなります。また、"-v" オプションを追加して "catkin build -v --no-color" とすることで、ビルド時に実際のコマンド（コンパイルオプションなど）を確認することもできます。

.. _ros_catkin_cmake_build_type:


なお、 :ref:`ros_catkin_build_command` は、catkin build に付与する --cmake-argsオプションによって設定することもできます。例えば ::

 catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

とすることで、このビルドについてはReleaseモードでのビルドとなります。このようにすることで、ビルドごとにビルドタイプのみを切り替えることが可能です。

さらに、Catkin Command Line Tools の Profile機能を使えば、設定ごとに予めプロファイルとして登録しておき、ビルドの際にプロファイルを指定することで、オプションの組み合わせを丸ごと切り替えることもできます。この方法については、:doc:`catkin-profile` で説明しています。

.. _loading_catkin_workspace_setup_script:

ワークスペースセットアップスクリプトの取り込み
----------------------------------------------

ビルドをすると、 ワークスペースのdevelディレクトリに "setup.bash" というファイルが生成されます。このスクリプトに記述されている設定は、ワークスペース内のパッケージを実行したりする際に必要となりますので、デフォルトで実行されるようにしておきます。通常はホームディレクトリの .bashrc ファイルに ::

 source $HOME/catkin_ws/devel/setup.bash

という記述を追加しておきます。

すると端末起動時に自動でこのファイルが実行され、設定が読み込まれるようになります。

初回ビルド時はまだこの設定が取り込まれていませんので、端末を起動し直すか、上記のコマンドをコマンドラインから直接入力して、設定を反映させるようにしてください。

.. note:: このスクリプトは :doc:`install-ros` で導入したROS本体のsetup.bashとは **異なります** ので注意するようにしてください。ワークスペース上のパッケージを正常に動作させるためには、どちらのスクリプトも読み込んでおく必要があります。

補足: 複数のChoreonoid環境の併用について
----------------------------------------

ここではROS環境（Catkinワークスペース）上で動作するChoreonoidのインストール方法を紹介しました。冒頭でも述べたように、ChoreonoidはROSとは独立してインストールすることも可能です。ただしそれらを同じOS上で併用する場合は、注意が必要です。

ROS環境のセットアップスクリプトがシステムに読み込まれると、共有ライブラリのパスにROS（Catkin）の該当するディレクトリが加わります。（環境変数LD_LIBRARY_PATHに追加されます。）この状態では、システムに同じ名前の共有ライブラリが複数あった場合、通常ROS環境のものが優先して読み込まれることになります。元々ROSとは独立にインストールされているソフトウェアについて、これが適用されると、バージョンやビルド設定が異なるライブラリが読み込まれてしまい、ソフトウェアが正常に動作しなくなることがあります。複数の環境を混ぜて使うのは大変危険ということです。

これを避けるためには、上記の :ref:`loading_catkin_workspace_setup_script` や :doc:`install-ros` で述べたsetup.bashスクリプトの取り込みについて、ROSとは独立したソフトウェアを使用する際には無効にしておくのが無難です。.bashrc の該当部分をコメントアウトするなどしてから、OSや端末を起動し直すことで、無効にすることができます。

なお、Choreonoidに関しては、実行ファイルや共有ライブリファイルの中に埋め込まれたRPATHという情報により、他の環境でビルドされたライブラリと混ざらないように実行することが可能となっています。この機能はビルドディレクトリ内に生成される実行ファイルやライブラリに関してはデフォルトで有効になります。（ただし比較的新しいUbuntuのバージョンに関しては `この更新 <https://github.com/choreonoid/choreonoid/commit/7f7900c3ec945f9da97b0e2ee484c1ddfe63d978>`_  以降であることが必要。）また、CMakeのENABLE_INSTALL_RPATHをONにすることで、"make install" によってインストールされるファイルに関してもこれが有効になります。

上記の更新以降では、CMakeのオプションで ENABLE_NEW_DTAGS というオプションが追加されています。これはデフォルトではOFFですが、ONにするとRPATHよりもLD_LIBRARY_PATHの情報が優先されるようになり、混ざってしまう危険性が高くなります。このオプションは特に必要が無い場合はOFFのままとしてください。

そのようにChoreonoidではなるべく共有ライブラリが混ざらないようにするための仕組みがありますが、環境設定によってはやはり混ざってしまうこともあり得ますし、Choreonoidと連携させて使用する他のソフトウェアにおいてライブラリが混ざってしまう可能性もあります。したがって、Choreonoidに限らない話として、同じソフトウェアが同一OS上で複数の環境にインストールされている場合、それらが混ざらないように使用するということが、不具合を避けるにあたって大変重要です。
