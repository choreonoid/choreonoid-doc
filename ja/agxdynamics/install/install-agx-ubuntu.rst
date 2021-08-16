==========================================
AGX Dynamicsのインストール(Ubuntu Linux編)
==========================================

.. contents::
   :local:
   :depth: 1

AGX Dynamics Ubuntu版のインストール方法について説明します。

パッケージのインストール
------------------------

AGX Dynamicsダウンロードサイトからdebパッケージをダウンロードしてください。
ここではAGX Dynamicsバージョン2.30.4.0をUbuntu20.04にインストールする例を紹介します。

まずダウンロードしたdebファイルをdpkgコマンドでインストールします。

.. code-block:: sh

 dpkg -i agx-2.30.4.0-amd64-ubuntu_20.04.deb

ただしdpkgコマンドでは他のパッケージへの依存が解決されません。依存を解決するには、以下の方法があります。

1. ファイラからdebファイルをダブルクリックして表示されるGUIツールを用いてインストールする
2. gdebiコマンドでインストールする

1の方法が手軽ですが、debファイルによってはこの方法でインストールできないこともあるようです。

2については以下のようにします。

.. code-block:: sh

 sudo apt install gdebi-core
 sudo gdebi agx-2.30.4.0-amd64-ubuntu_20.04.deb

インストールに成功すると、/opt/Algoryx/AGX-<version>ディレクトリにインストールされます。

ライセンスのインストール
------------------------

次にAGX実行ライセンスファイル(agx.lic)をインストールディレクトリに配置し、AGX Dynamicsを実行できるようにします。

環境設定の取り込み
------------------

最後にAGXの環境変数設定を取り込んでおきます。
これはAGXディレクトリの"setup_evn.bash"を実行することで実現できます。

通常はホームディレクトリの.bashrcファイルに以下の記述をしておきます。

.. code-block:: sh

 source /opt/Algoryx/AGX-2.30.4.0/setup_env.bash

ディレクトリ "AGX-2.30.4.0" の部分は実際に使用するAGXのバージョンに合うものとしてください。

この設定をしておけば、それ以降起動される端末上ではAGXの環境設定がなされた状態となります。

.. _agxdynamics-plugin-install-ubuntu-library-reference-resolution-problem:

AGX共有ライブラリの参照解決の問題
---------------------------------

最近のAGXのバージョン（今回2.30.4.0で確認）では、以前のバージョンとは異なり、AGXの共有ライブラリへのパスが上記のスクリプトでは設定されなくなったようです。そのままでは、AGXの共有ライブラリへの参照を解決できず、Choreonoid実行時にAGX Dynamicsプラグインが読み込めないことがあります。

共有ライブラリパスについては環境変数LD_LIBRARY_PATHで設定することができます。
AGXの共有ライブラリをパスに追加する場合は、この環境変数を以下のように設定します。

.. code-block:: sh

 export LD_LIBRARY_PATH=/opt/Algoryx/AGX-2.30.4.0/lib:$LD_LIBRARY_PATH

ただし、Choreonoid開発版のコミットb71e314d098eb24e0beaf571f6ae0fe9fdb618a2以降では、この環境変数を設定しなくても、Choreonoidビルド時の設定でも解決することができます。そちらの方が他に影響を与えずに済むので、より望ましいかもしれません。その方法については次節で説明します。
