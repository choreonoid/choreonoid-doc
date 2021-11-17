
AGXDynamicsプラグインのビルドとインストール（Windows編）
-------------------------------------------------------

AGXDynamicsプラグインはChoreonoidのソースコードに同梱されております。
Choreonoidのビルド前に行うCMakeの設定で以下のオプションを **ON** にすることでビルドすることができます。

* **BUILD_AGX_DYNAMICS_PLUGIN**

  * AGXDynamicsプラグイン - AGX Dynamicsのシミュレーションプラグイン

* **BUILD_AGX_BODYEXTENSION_PLUGIN**

  * AGXBodyExtensionプラグイン - 専用モデルプラグイン(ワイヤーなど)

CMakeのGUIで設定する場合は、 **BUILD_AGX_DYNAMICS_PLUGIN** にチェックを入れ、Configureを押します。
その際にAGXのライブラリが自動で検出されます。検出されない場合は、 **AGX_DIR** にインストール先のディレクトリを設定してください。
次に **BUILD_AGX_BODYEXTENSION_PLUGIN** にチェックを入れ、もう一度configureを押します。

.. note:: AGXBodyExtensionプラグインはAGXDynamicsプラグインに依存しているため、BUILD_AGX_DYNAMICS_PLUGINがONにならないとccmakeで表示されません。一度BUILD_AGX_DYNAMICS_PLUGINをONにしてconfigureを実行してみてください。

Generateを押して、ソリューションファイルを生成します。後は、 :ref:`build-windows-visualstudio` と同様にビルド、インストールを行ってください。するとAGXDynamicsプラグイン、AGXBodyExtensionプラグインがビルド・インストールされ、Choreonoid上で使用できるようになります。

.. note:: インストールを行なうと、AGX Dynamicsのランタイムライブラリ（DLLファイル）もあわせてChoreonoidのbinディレクトリにインストールされます。もしこれを行いたくない場合は、CMakeの設定で **INSTALL_AGX_RUNTIME** のオプションをオフにしておきます。ただしその場合はAGX Dynamicsのbinディレクトリにパスを通して、AGX DynamicsのDLLが読み込まれるように予め設定しておく必要があります。

