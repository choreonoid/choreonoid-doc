===========================================================
AGXDynamicsプラグインのビルドとインストール(Ubuntu Linux編)
===========================================================

.. highlight:: sh

CMakeオプションによるAGXDynamicsプラグインの有効化
--------------------------------------------------

AGXDynamicsプラグインはChoreonoid本体のソースコードに含まれています。
これを有効にするには、Choreonoid本体のビルド時の :ref:`build-ubuntu-cmake` で以下のオプションを **ON** にしておきます。

* **BUILD_AGX_DYNAMICS_PLUGIN**      : AGXDynamicsプラグイン - AGX Dynamicsのシミュレーションプラグイン
* **BUILD_AGX_BODYEXTENSION_PLUGIN** : AGXBodyExtensionプラグイン - 専用モデルプラグイン(ワイヤーなど)

これはcmakeコマンドのオプションとして ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON

などとしてもよいですし、ccmakeコマンドで表示されるメニュー上でこれらのオプションの値をONに切り替える操作でもOKです。

このCMake設定をした上でChoreonoid本体のビルドを行うと、AGXDynamicsプラグインも同時にビルドされ、使えるようになります。

.. note:: AGXBodyExtensionプラグインはAGXDynamicsプラグインに依存しているため、BUILD_AGX_DYNAMICS_PLUGINがONにならないとccmakeで表示されません。一度BUILD_AGX_DYNAMICS_PLUGINをONにしてconfigureを実行してみてください。

.. note:: ccmakeでconfigureの操作を行うとAGX DynamicsのパスAGX_DIRが自動的に設定されますが、設定されない場合には手動で設定をしてください。デフォルトのパスは /opt/Algoryx/AGX-<version>です。

.. _agxdynamics-plugin-build-ubuntu-option-for-library-reference-resolution:

AGX Dynamics共有ライブラリへの参照解決用オプション
--------------------------------------------------

:ref:`agxdynamics-plugin-install-ubuntu-library-reference-resolution-problem` は、Choreonoidビルド時のCMakeオプションによっても解決することができます。

これを行うには、CMakeのオプションとして **ENABLE_INSTALL_RPATH_USE_LINK_PATH** に **ON** を設定します。

この場合、cmakeコマンドによるオプション設定は以下のようにします。 ::

 cmake -DBUILD_AGX_DYNAMICS_PLUGIN=ON -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON -DENABLE_INSTALL_RPATH_USE_LINK_PATH=ON

ccmakeコマンドの場合は、まず "T" キーを押してアドバンスドモードに切り替えます。
そしてメニューを操作してENABLE_INSTALL_RPATH_USE_LINK_PATHの項目をONにしておきます。

この設定をした状態でビルドを行うと、AGX Dynamicsプラグインの共有ライブラリファイルに対して、動的リンクするAGX Dynamicsの共有ライブラリへのパスが埋め込まれます。これは共有ライブラリの"RPATH"という機能を使って実現されます。このようにして生成したプラグインファイルは、自身に埋め込まれた依存ライブラリへのパスを用いてライブラリの参照解決をします。この場合、該当するライブラリがOSの共有ライブラリパスに存在しない場合でも、実行時にリンクして実行することができます。

AGX Dynamicsプラグインを使用する場合は、通常このオプションをONにしてビルドしておくのがおすすめです。

