パッケージによるインストール（Ubuntu Linux編)
=============================================

Choreonoidはバージョン2.3.0からUbuntu Linux用のdebパッケージが用意されており、APTを用いてインストールすることが可能です。
Ubuntu Linuxにインストールする場合はこの方法が最も簡単です。ここではその方法について解説します。

.. contents::
   :local:

.. highlight:: sh

利用可能なAPTリポジトリ
-----------------------

公式で用意しているChoreonoidのdebパッケージは、 `Launchpad <https://launchpad.net/>`_ `PPA <https://launchpad.net/~launchpad/+archive/ubuntu/ppa>`_ のリポジトリで公開されています。
開発版とリリース版の2種類があり、それぞれ以下のリポジトリとなります。

* 開発版

  * ppa:choreonoid.org/testing

* リリース版

  * ppa:choreonoid.org/stable

開発版は、 `Choreonoidの公式Githubリポジトリ <https://github.com/choreonoid/choreonoid>`_ のmasterブランチをパッケージ化したもので、開発中の機能を含むChoreonoidの最新版となります。
Choreonoidは現在も活発に開発が行われているので、特に理由がなければこの開発版を利用することをおすすめします。
なお、現状ではmasterブランチの更新の度に常にパッケージを更新しているわけではなく、masterブランチの更新がある程度まとまった適当なタイミングで更新しています。

リリース版は開発の節目でバージョン番号をつけてリリースされているものです。
こちらは特定のバージョンを使用する必要がある場合や、開発版で不具合が発生しているときなどに利用するとよいかと思います。

APTリポジトリの登録
-------------------

上記のリポジトリからChoreonoidのパッケージをインストールするためには、予めリポジトリを登録しておく必要があります。
開発版を利用する場合は以下のコマンドでリポジトリを登録します。 ::

 sudo add-apt-repository ppa:choreonoid.org/testing

リリース版を利用する場合は以下のコマンドとなります。 ::

 sudo add-apt-repository ppa:choreonoid.org/stable

Choreonoidのインストール
------------------------

上記のリポジトリの登録ができていれば、aptコマンドで "choreonoid" パッケージをインストールすればOKです。
具体的には以下のコマンドを実行します。 ::

 sudo apt update
 sudo apt install choreonoid

このコマンドで登録しているリポジトリから最新のバージョンがインストールされます。
既にインストールされている場合、上位のバージョンがあればそちらにアップグレードします。

Choreonoidの起動
----------------

以下のコマンドでChoreonoidを起動できます。 ::

  choreonoid

インストールされる内容
----------------------

現状ではパッケージは "choreonoid" のみとなっており、これでChoreonoid本体付属のほとんどのプラグインとサンプルが有効になったものがインストールされます。また、開発用の各種ファイル（CMakeファイル、ヘッダファイル、ライブラリファイル等）もインストールされるので、Choreonoidのプラグインやコントローラを開発することも可能です。

実際に有効となるプラグインについては、Choreonoid起動時のメッセージビューでご確認ください。

.. note:: debパッケージではソフトウェア本体とプラグイン、開発用ファイルをそれぞれ別のパッケージに分けて用意することも一般的ですが、現状のChoreonoidではそのようにパッケージは分けられておらず、ひとつの "choreonoid" パッケージに全て含まれる状態となっています。

アンインストール
----------------

以下のコマンドでアンインストールできます。 ::

 sudo apt remove choreonoid

リポジトリの登録を解除したい場合は以下を実行します。 ::

 sudo add-apt-repository --remove リポジトリ名

リポジトリ名は、開発版の場合は ppa:choreonoid.org/testing 、リリース版の場合は ppa:choreonoid.org/stable となります。

:doc:`ソースコードからビルドする <build-ubuntu>` などして、パッケージでインストールしたのとは別のChoreonoidのバイナリが同一PC上に存在する場合、干渉を起こしてしまう可能性があります。正常に動作させるためには、干渉を起こさないよう設定しておく必要がありますので、ご注意ください。これがよく分からない場合は、ソースコードからビルド・インストールする際は、パッケージでインストールしたChoreonoidはアンインストールしておくのが安全です。

補足：インストールするバージョンの指定方法
------------------------------------------

aptでインストールする場合は基本的にはリポジトリに登録されている最新版がインストールされますが、それよりも古いバージョンをインストールしたい場合は以下のようにします。

まず、利用可能なバージョンを以下のコマンドで確認します。 ::

 apt-cache policy choreonoid

このコマンドで例えば以下のような出力がされるとします。

.. code-block:: text

   choreonoid:
     インストールされているバージョン: (なし)
     候補:               2.4.0~git20251020.1758.4e4a671b7-1~noble
     バージョンテーブル:
        2.4.0~git20251020.1758.4e4a671b7-1~noble 500
           500 https://ppa.launchpadcontent.net/choreonoid.org/testing/ubuntu noble/main amd64 Packages
        2.3.0-1~noble 500
           500 https://ppa.launchpadcontent.net/choreonoid.org/stable/ubuntu noble/main amd64 Packages

.. 英訳指示： 上記の出力結果は英語環境では "Installed: (none)", "Candidate", "Version table:" といった表記になります。

ここで「バージョンテーブル」にある "2.4.0~git20251020.1758.4e4a671b7-1~noble" というのが、詳細なバージョン名です。
このバージョン名を ::

  sudo apt install choreonoid=2.4.0~git20251020.1758.4e4a671b7-1~noble

と指定することにより、特定のバージョンのインストールができます。
