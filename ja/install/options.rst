
オプション機能
==============

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

ここでは、Choreonoidのビルド設定において選択可能なオプション機能を紹介します。

オプション機能はCMakeの設定で有効化／無効化します。この操作については、 :doc:`build-ubuntu` の :ref:`build-ubuntu-cmake` もしくは :doc:`build-windows` の :ref:`build-windows-cmake` を参照してください。

.. highlight:: sh

主要なオプション
----------------

* **ENABLE_GUI**

 ChoreonoidのGUI上で実現される機能を有効にします。デフォルトでONになっていて、Choreonoidをアプリケーションソフトとして使用する場合は必須のオプションです。GUIには依存しないライブラリの部分だけを利用したい場合は、これをOFFにすることでビルド時間を短縮することができます。

* **BUILD_ASSIMP_PLUGIN**

 Assimpプラグインをビルドします。Assimpは各種3Dモデルファイルを読み込むためのライブラリで、このプラグインによりCOLLADA、Blender、X、DXF形式のファイルを読み込むことが可能となります。Windowsでこれをビルドする場合は :ref:`build-windows-options` をご参照ください。UbuntuではデフォルトでONになっています。

* **ENABLE_FREE_TYPE**

  FreeTypeライブラリを利用して、シーンビュー上へのテキストの描画をできるようにします。現状のChoreonoidではこのオプションがONになっていると、距離計測機能を利用する際にシーンビュー上に距離が数値表示されるようになります。UbuntuではデフォルトでONになっています。WindowsではデフォルトでOFFになっており、利用にあたっては別途FreeTypeライブラリをインストールしておく必要があります。

* **BUILD_POSE_SEQ_PLUGIN**

 PoseSeqプラグインをビルドします。このプラグインはキーポーズによる振り付け機能を提供します。

* **BUILD_BALANCER_PLUGIN**

 バランサープラグインをビルドします。このプラグインにより、振り付け機能において自動バランス補正を行うことが可能となります。二足歩行ロボットの振り付けを行う際にこの機能を使うことで（理論上）転倒しない動作を作成できます。

* **BUILD_MOCAP_PLUGIN**

 モーションキャプチャの動作データを読み込んだり表示したりするためのプラグインです。現在BVH形式のモーションキャプチャデータに対応しています。

* **ENABLE_PYTHON**

 ChoreonoidのPythonバインディグを有効にします。PythonバインディングによりC++で実装されているコレオノイドのライブラリをPythonで利用することが可能となります。例えばBodyライブラリのPythonバインディングである "cnoid.Body" モジュールをインポートすることで、Python上でBodyクラスを中心とした各種ロボティクス関連処理を記述することができます。

 また、ENABLE_GUIがONの場合、ChoreonoidのGUI上でPythonを利用するためのPythonプラグインとPythonSimScriptプラグインもあわせてビルドされます。

 UbuntuではデフォルトでONになっています。 Windowsでこれを有効にする場合は :ref:`build-windows-options` をご参照ください。

* **BUILD_ODE_PLUGIN**

 ODEプラグインをビルドします。このプラグインにより、オープンソースの動力学計算ライブラリである "Open Dynamics Engine (ODE)" を、シミュレーションのための物理エンジンとして利用できるようになります。利用の際には、 `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ をインストールしておく必要があります。UbuntuのパッケージインストールスクリプトではODEもインストールされますので、このオプションをONにするだけでビルドできます。Windowsでのビルドについては :ref:`build-windows-options` をご参照ください。なお、ODEプラグインがなくても、組み込みのAIST物理エンジンを用いたシミュレーションが可能です。

* **BUILD_AGX_DYAMICS_PLUGIN**

 AGX Dynamicsプラグインをビルドします。AGX DynamicsはスウェーデンのAlgoryx社が開発した商用の物理エンジンで、これを用いたシミュレーションを利用できるようにします。詳細は :doc:`../agxdynamics/index` を参照ください。

* **BUILD_MULTICOPTER_PLUGIN**

 マルチコプタプラグインをビルドします。このプラグインによりマルチコプタのシミュレーションが可能となります。詳細は :doc:`../multicopter/index` を参照ください。

* **BUILD_SCENE_EFFECTS_PLUGIN**

 シーンエフェクトプラグインをビルドします。このプラグインにより、シーン上に炎や煙等のエフェクトを描画できるようになります。

* **BUILD_MEDIA_PLUGIN**

 メディアプラグインをビルドします。このプラグインによりChoreonoid上で動画や音声の各種メディアファイルを再生することが可能となります。

* **BUILD_TRAFFIC_CONTROL_PLUGIN**

 TrafficControlプラグインをビルドします。このプラグインにより、通信遅延、帯域制限、パケットロス等の各種通信障害を模擬することが可能となります。詳細は :doc:`../trafficcontrol/index` を参照ください。

* **BUILD_FCL_PLUGIN**

 FCLプラグインをビルドします。このプラグインにより、オープンソースの干渉検出ライブラリである　 `Flexible Collision Library (FCL) <https://github.com/flexible-collision-library/fcl>`_ を干渉検出に利用することが可能となります。UbuntuのパッケージインストールスクリプトではFCLもインストールされますので、このオプションをONにするだけでビルドできます。Windowsでは自前でFCLをインストールしておく必要があります。なお、FCLプラグインがなくても、組み込みの干渉検出機能が利用できますので、特に問題はありません。


サンプル関連オプション
----------------------

以下はサンプルのデータやプログラムをビルドするためのオプションです。

* **ENABLE_SAMPLES**

 サンプルを有効化します。これにより、基本的なサンプルのビルドに加えて、以下のオプションで追加のサンプルもビルドできるようになります。デフォルトでONになっています。

* **BUILD_SIMPLE_CONTROLLER_SAMPLES**

 シンプルコントローラで実現される各種シミュレーションサンプルをビルドします。デフォルトでONになっています。

* **BUILD_SUBMERSIBLE_SAMPLE**

 水中型ロボットの簡易シミュレーションを行うサンプルです。

* **BUILD_WRS2018**

 2018年に開催された国際ロボット競技会 World Robot Summit 2018における「トンネル事故災害対応・復旧チャレンジ」の競技用モデルをサンプル化したものです。詳細は :doc:`../wrs2018/index` を参照ください。

拡張機能基盤オプション
----------------------

以下は拡張機能の基盤となるオプションです。特に必要がない限り有効にする必要はありません。

* **BUILD_MANIPULATOR_PLUGIN**

 Choreonoid上でマニピュレータの教示・シミュレーション機能を実現するための基盤となるプラグインです。

* **ENABLE_CORBA**

 CORBA関連機能を有効にします。OpenRTMやOpenHRP関連の機能を利用するにあたって必要となります。 `omniORB <http://omniorb.sourceforge.net/>`_ を用いて実装されています。Ubuntuであれば以下のコマンドで必要なomniORB関連パッケージをインストールできます。 ::

  sudo apt install libomniorb4-dev libcos4-dev omniidl omniorb-nameserver python-omniorb omniidl-python

 なおこのオプションはメニュー形式のCMake設定ツールでは通常表示されません。Advanced Modeに切り替えることで表示されるようになります。

実験／開発段階機能のオプション
----------------------------

以下に紹介するのはまだ実験／開発段階にある機能のオプションで、必ずしも正常に動作するものではありません。これらは主に該当機能の開発のために利用するオプションとなっています。メニュー形式のCMake設定ツールでは通常これらのオプションは表示されず、Advanced Modeへの切替時のみ表示されるようになっています。（以下で紹介する各プラグインのビルド方法に関するページは内容が古くなっている部分もありますが、ご了承ください。）

* **BUILD_BULLET_PLUGIN**

 Bulletプラグインをビルドします。このプラグインにより、オープンソースの物理計算ライブラリである `Bullet Physics <https://github.com/bulletphysics/bullet3>`_ をコレオノイドのシミュレーション機能で利用できるようになります。ビルド方法については、 :doc:`build-bullet-plugin` を参照ください。
 
* **BUILD_PHYSX_PLUGIN**

 PhysXプラグインをビルドします。このプラグインにより、物理計算ライブラリ `PhysX <https://developer.nvidia.com/gameworks-physx-overview>`_ をコレオノイドのシミュレーション機能で利用できるようになります。ビルド方法については、 :doc:`build-physx-plugin` を参照ください。

* **BUILD_ROKI_PLUGIN**

 ROKIプラグインをビルドします。このプラグインにより、ロボット運動学ライブラリ `RoKi <https://github.com/zhidao/roki>`_ をコレオノイドのシミュレーション機能で利用できるようになります。ビルド方法については、 :doc:`build-roki-plugin` をご覧ください。

* **BUILD_SPRINGHEAD_PLUGIN**

 Springheadプラグインをビルドします。このプラグインにより、動力学計算ライブラリ"Springhead"をコレオノイドのシミュレーション機能の計算エンジンとして利用できます。Springheadの詳細については `Springheadホームページ <http://springhead.info/wiki/>`_ を、ビルド方法については、 :doc:`build-springhead-plugin` をご覧ください。

* **BUILD_SDF_PLUGIN**

 SDFプラグインをビルドします。このプラグインにより、Simulation Description Format (SDFormat) で記述されたモデルを読み込むことが可能となります。実装には `SDFormat library <https://github.com/osrf/sdformat>`_ を用いています。Ubuntuでビルドする場合は、以下のコマンドで必要なライブラリをインストールできます。 ::

  sudo apt install libsdformat6-dev libogre-1.9-dev

* **ENABLE_LUA**

 `プログラミング言語Lua <http://www.lua.org/>`_ によるバインディングやスクリプト実行機能を有効にします。Ubuntuでビルドする場合は、以下のコマンドでLua関連のパッケージをインストールしておきます。 ::

  sudo apt install lua5.3 iblua5.3-dev lua-posix
 
その他のオプション
------------------

Choreonoidでは上に挙げた以外にもオプションがありますが、それらについて何であるかが分からない場合は基本的にONにしないようにしてください。
