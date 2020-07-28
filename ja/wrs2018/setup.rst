シミュレーション環境の構築
==========================

.. contents::
   :local:

.. highlight:: sh

	       

シミュレーション用PCの用意
--------------------------

まずはシミュレーション用のPCを用意して、Choreonoidをインストールします。

実際の競技会ではOSとしてUbuntu 16.04を使用しましたが、サンプルを動かすのであれば最新の環境でも動作するかと思います。以下ではUbuntu 18.04の使用を想定して説明します。

なお、Ubuntuはネイティブインストールされたものを使用してください。仮想マシンでも動かないことはありませんが、シミュレーションが遅くなったり、一部不具合が生じる可能性があります。

Gitのインストール
-----------------

以下の作業を進めるにあたって、バージョン管理システムのGitが必要となります。まだインストールしていない場合は、以下のコマンドでインストールしておきます。 ::

 sudo apt install git

.. _wrs2018_install_agx:

AGX Dynamicsのインストール
--------------------------

AGX Dynamicsのライセンスをお持ちの場合は、あらかじめ AGX Dynamics をインストールしておきます。販売元より提示されたAGX Dynamicsのダウンロードサイトから、対応するUbuntuバージョン用のパッケージをダウンロードします。また、USBドングルの提供を受けている場合は、それをPCに挿しておくようにしてください。

パッケージがダウンロードできたら、:doc:`../agxdynamics/install/install-agx-ubuntu` の説明に従ってインストールを行います。

AGX Dynamicsのラインセンスをお持ちでない場合、この作業はスキップしてください。

.. _wrs2018_install_openrtm:

Choreonoidのインストール
------------------------

`Choreonoid最新版（開発版）マニュアル <../index.html>`_ の `ソースコードからのビルドとインストール (Ubuntu Linux編) <../install/build-ubuntu.html>`_　に従って、Choreonoidの最新の `開発版 <../install/build-ubuntu.html#id4>`_ をインストールします。

インストールの詳細は上記ドキュメントを参照いただくとして、Ubuntu 18.04においては、以下のコマンドを実行していきます。

まずGitリポジトリからChoreonoidのソースコードを取得します。 ::

 git clone https://github.com/choreonoid/choreonoid.git

取得したソースコードのディレクトリに移動します。 ::

 cd choreonoid

依存パッケージのインストールを行います。 ::

 misc/script/install-requisites-ubuntu-18.04.sh

CMakeによるビルドの設定を行います。Choreonoidのデフォルトの機能だけ利用するのであれば、 ::

 cmake .

を実行します。

ただしWRS2018のサンプルを実行するためには、以下のオプションも有効（ON）にする必要があります。

* WRS2018サンプル

 * BUILD_WRS2018

* AGX Dynamics を利用する場合

 * BUILD_AGX_DYNAMICS_PLUGIN
 * BUILD_AGX_BODYEXTENSION_PLUGIN

* 煙や炎を再現する場合

 * BUILD_SCENE_EFFECTS_PLUGIN

* マルチコプタを使用する場合

 * BUILD_MULTICOPTER_PLUGIN
 * BUILD_MULTICOPTER_SAMPLES

これらのオプションの設定はccmakeコマンドを使ってインタラクティブに行うこともできますが、cmakeコマンドに-Dオプションを与えることも可能です。例えば、BUILD_SCENE_EFFECTS_PLUGINをONにするには、以下のように入力します。 ::

 cmake -DBUILD_SCENE_EFFECTS_PLUGIN=ON

このオプションは複数つけることができます。上記のオプション全てを有効にする場合は、以下のように入力してください。 ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DBUILD_SCENE_EFFECTS_PLUGIN=ON -DBUILD_MULTICOPTER_PLUGIN=ON -DBUILD_MULTICOPTER_SAMPLES=ON

AGX Dynamicsをインストールしていない場合は、対応するオプションを上記のコマンドライン引数から除去して実行してください。

次に、makeコマンドでビルドを行います。 ::

 make

なお、マルチコアCPUをお使いの場合は、makeコマンドに -j オプションをつけてビルドを並列化するとよいです。例えば次のようにします。 ::

 make -j 8

この場合、最大で8つのプロセスを同時に実行してビルドを行います。4コア8スレッドのCPUの場合はこのように入力するとよいでしょう。通常、CPUの論理コア数を指定します。

一度インストールを行った後も、上記の作業を行ったソースディレクトリ上で以下のように実行することで、常に最新版のChoreonoidを利用することができます。 ::

 git pull
 make -j 8

グラフィックス環境のセットアップ
--------------------------------

WRS2018のシミュレーションでは高度な描画能力が要求されるため、 :doc:`../install/setup-gpu` を参照の上、なるべくよいグラフィックス環境を構築するようにしてください。できればNVIDIA製のGeForceやQuadroといったGPUのハイエンドモデルを使用するようにし、 :ref:`setup_gpu_ubuntu_gpu_driver` も実行するようにしてください。また、 :ref:`setup_gpu_3d_rendering_engine` については、デフォルトの新描画エンジン（GLSL描画エンジン）を使用するようにします。（特に理由がなければ、旧描画エンジンには切り替えないようにしてください。）これらの条件が満たされないと、描画速度が出なかったり、ライトや影、煙、炎等の表現がされなかったりしてしまいます。

また、 :ref:`build_ubuntu_qt_style` についても適切に設定しておくのが望ましいです。


ゲームパッドの準備
------------------

今回のサンプルでは、ゲームパッドでロボットを操作することができます。これを行うために、ゲームパッドを用意して、PCに接続しておいてください。

使用可能なゲームパッドについては、:doc:`../simulation/tank-tutorial/index` の :ref:`simulation-tank-tutorial-gamepad` を参照してください。おすすめはプレイステーション4用の `DUALSHOCK4 <http://www.jp.playstation.com/ps4/peripheral/cuhzct1j.html>`_ コントローラです。DUALSHOCK4は `USBワイヤレスアダプター <http://www.jp.playstation.com/ps4/peripheral/cuhzwa1j.html>`_ によるワイヤレス接続も可能です。
