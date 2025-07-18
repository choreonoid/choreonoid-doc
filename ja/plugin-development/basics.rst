====================
プラグイン作成の基本
====================

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

本節ではプラグイン作成の基本事項について解説します。

まずプラグインを作成する際の手順は形式的に以下のように表されます。

1. ビルドの形態を選択する
2. プラグインのソースディレクトリを作成する
3. ソースファイルを作成し、その中で :doc:`plugin-class` を継承した独自のプラグインクラスをC++言語で実装する
4. CMakeによるビルド手順の記述を行う
5. プラグインのビルドを行う
6. 必要に応じてインストールを行う
7. Choreonoidを起動すると作成したプラグインが読み込まれるので、機能を利用する

以下ではこの手順に沿ってプラグイン作成の基本的な事柄を解説し、Choreonoidのプラグイン開発の概要を把握できるようにします。

.. _plugin-dev-basics-build-forms:

ビルド形態
----------

ChoreonoidはC++言語で記述されており、プラグインについても基本的にC++のプログラムとして記述する必要があります。このためプラグインの作成においてはまずC++のソースファイルを作成しこれをコンパイルして実行バイナリを生成する必要があります。このビルド処理の進め方については基本的に以下の2つの形態があります。

1. Choreonoid本体のビルド環境でビルドする
2. Choreonoid本体とは独立してビルドする

1の場合、プラグインはChoreonoid本体のビルド環境に含まれるコンポーネントのひとつとしてビルドすることになります。Choreonoid本体をソースコードから自前でビルドしている場合は、この形態で追加のプラグインをビルドすることが可能です。 :doc:`introduction` でChoreonoid本体の機能もその多くはプラグインとして実装されている旨を説明しました。形態1はそれら本体付属プラグインと同様の方法で自前のプラグインもビルドするというものです。既存プラグインの多くは１の形態でビルドされるようになっており、多くの場合はこちらを選択しておけば問題ありません。

一方で2の形態ではプラグインをChoreonoid本体とは独立してビルドします。これはソフトウェアモジュールをビルドする形態として一般的ではあるのですが、Choreonoidではこの形態がまだ十分に整備できていないところがあります。特にWindowsでVisual Studioを使用して開発する場合はまだこの形態を利用できるようになっていません。従って現状ではこの形態はLinuxのみが対象となります。バイナリパッケージとしてインストールしたChoreonoid本体と組み合わせる場合や、特定のビルド手順を要求するフレームワークと組み合わせる場合などは、この形態でプラグインを作成する必要があります。実際にこの形態を採用しているプラグインの例として :doc:`../ros/rosplugin` があります。

現状では1の形態の方がより適用しやすいところがあるため、本開発ガイドでは主に1の形態で説明を進めます。今後2の形態も選択しやすくなるよう、Choreonoid本体のバイナリパッケージの整備やビルド用スクリプトの整備などを進めていきたいと思います。

.. _plugin-dev-basics-source-directory:

ソースディレクトリの作成
------------------------

プラグインのビルド形態を決定したら、次にプラグインのソースファイルを格納するディレクトリを作成します。これは通常プラグインひとつに付きひとつずつ作成することになります。

ソースディレクトリを作成する場所は以下のようにします。

1. Choreonoid本体のビルド環境でビルドする場合

  Choreonoid本体のソースディレクトリに含まれる"ext"というサブディレクトリの中にプラグイン用のディレクトリを作成する。

2. Choreonoid本体とは独立してビルドする場合

  アクセス権のある任意の場所にプラグイン用のディレクトリを作成する。

このように、1の場合は作成する場所が決まっているので注意が必要です。複数のプラグインを開発する場合は、ext以下に複数のソースディレクトリを作成すればOKです。なお、本体ビルド時のCMakeの設定で "ADDITIONAL_EXT_DIRECTRIES" を指定することで、プラグインのソースディレクトリを配置するディレクトリを追加することができます。

2の場合はソースディレクトリを任意の場所に作成することが可能です。ただしChoreonoid本体のビルド環境が存在する場合は、その外部に作成するようにしてください。


ソースファイルの作成
--------------------

前述のようにプラグインのソースファイルはC++言語で記述します。Choreonoid本体が提供するC++ライブラリのクラスや関数を用いることで、プラグインとして機能するC++プログラムを記述することができます。

Choreonoid本体が提供するC++ライブラリからなる開発環境をChoreonoid SDKと呼びます。Choreonoid SDKは複数のライブラリ（モジュール）で構成されており、その中でも基盤となるものとして以下があります。

* Utilライブラリ： ChoreonoidのGUIには依存しない様々なユーテリティクラス／関数を提供する
* Baseモジュール： ChoreonoidのGUIの基盤となるクラスや関数を提供する

ここでUtilは他のプログラムからも使用できる汎用的なものなので「ライブラリ」という表記を使用しています。一方BaseについてはChoreonoidのコアに相当するものであり、Choreonoid内でのみ使用するものなので「モジュール」という表記としています。どちらも形態としては共有ライブラリ（ダイナミックリンクライブラリ）となります。

プラグインもこれらのライブラリ／モジュールを基盤として実装します。その中でBaseモジュールに含まれる :doc:`plugin-class` がChoreonoidのプラグインを実装する起点となるものです。通常のC言語のプログラムで言えば、main関数のようなものです。

新たにプラグインを作成する場合は、まずPluginクラスを継承したクラスを定義します。そのコンストラクタではプラグインの基本情報を記述し、initialize関数で初期化処理を記述します。このための必要最低限のクラス定義は以下のように表されます。 ::

 class FooPlugin : public Plugin
 {
 public:
    FooPlugin();
    virtual bool initialize() override;
 };


ここではFooPluginという名前でプラグインクラスを定義しています。デフォルトコンストラクタに加えて、初期化を行うinitialize関数も定義します。initialize関数の中では、新たに追加する機能をChoreonoid本体に登録する処理を行います。

登録する機能はC++を用いて自由に実装することができます。その際にChoreonoid SDKのライブラリを用いることで、Choreonoidのフレームワークや既存機能との連携を行うことができます。例えばロボット関連の処理であれば、以下のライブラリを用いることができます。

* Bodyライブラリ： ロボット関連の機能のうち、GUIに依存しない部分を提供する
* BodyPluginライブラリ： ロボット関連の機能のうち、GUIに関わる部分を提供する

もちろん標準ライブラリや外部のライブラリを用いることも可能です。

また、ChoreonoidはC++11以上のC++バージョンを必須としており、プラグインの開発ではC++11以降の機能を利用することができます。OSや開発環境が対応していれば、C++14やC++17等のより新しいC++バージョンも利用することができます。

そのようにして作成したソースファイルはプラグインのソースディレクトリに格納しておきます。

CMakeによるビルド手順の記述
---------------------------

ChoreonoidではビルドシステムとしてCMakeを採用しており、プラグインの開発においても通常はCMakeを使用してビルド手順を記述します。CMakeは近年多くのソフトウェアの開発で採用されており、ビルドシステムとして一般的なものとなっているので、使用にあたって特に問題になるようなことはないのではないかと思います。

CMakeではビルド手順をCMakeLists.txtというファイルに記述するようになっており、これはChoreonoidのプラグイン開発においても同じです。基本的にプラグインひとつに付きひとつのCMakeLists.txtを作成し、プラグインのソースディレクトリに格納しておきます。

プラグイン用のCMakeLists.txtにおいては、通常のCMakeの変数やコマンドに加えて、Choreonoidが提供する変数やコマンドも用いることができます。それらを適宜組み合わせて記述することで、プラグインのビルド手順を記述します。

CMakeLists.txtの書き方は、上述したビルドの形態によって若干異なります。Choreonoid本体のビルド環境でビルドする場合は、Choreonoid本体のCMakeプロジェクトの一部として記述し、本体のCMakeLists.txtで定義れている関数や変数をそのまま利用します。一方Choreonoid本体とは独立してビルドする場合は、Choreonoid本体が提供するCMakeのパッケージファイルを取り込んだ上で、独立したCMakeプロジェクトとして記述します。

CMakeLists.txtの記述方法については、別途 :doc:`sdk-cmake` で解説します。

ビルド
------

必要なソースファイルとCMakeLists.txtを作成できたらビルドを行います。

Choreonoid本体のビルド環境でビルドする場合は、Choreonoid本体を通常の方法でビルドすれば同時にプラグインもビルドされます。Choreonoid本体のビルド方法については以下のページをご参照ください。

* :doc:`../install/build-ubuntu`
* :doc:`../install/build-windows`

Choreonoid本体とは独立してビルドする場合は、プラグインのソースディレクトリに対してCMakeのビルドディレクトリを作成し、CMakeを実行してビルド用ファイルを生成します。これはCMakeでビルドを行う一般的なプログラムと同様の手順になります。

ビルドに成功すると、プラグインのバイナリファイルが生成されます。これはLinux上では共有ライブラリ、Windows上ではダイナミックリンクライブラリと呼ばれるもので、それぞれ .so 、.dll という拡張子をもつファイルになります。通常はファイル名にChoreonoidのプラグインであることを判別するためのプレフィックスが付与されます。プラグイン名がFooPluginの場合は

* libCnoidFooPlugin.so (Linux)
* CnoidFooPlugin.dll (Windows)

といったファイル名になります。

インストール
------------

ビルドしたプラグインはChoreonodi本体のプラグインディレクトリに格納します。プラグインディレクトリは以下の場所にあります。

* [Chorenoid本体のビルド／インストール先]/lib/choreonoid-x.y

ここでx.yのところにはChoreonoid本体のバージョン番号が入ります。

Choreonoid本体のビルド環境でビルドする場合、プラグインのバイナリファイルは本体のビルド／インストール時に同時にプラグインディレクトリに格納されます。

Choreonoid本体とは独立してビルドする場合は、プラグインのインストールもプラグイン側で行う必要があります。通常はCMakeListst.txtにそのための記述をしておき、ビルドシステムのインストール機能でインストールするようにします。

読み込みと利用
--------------

.. highlight:: text

プラグインファイルがプラグインディレクトリに格納されていると、Choreonoid起動時にプラグインが読み込まれます。その場合は以下のようねメッセージがメッセージビューに出力されます。 ::

 プラグインファイル"C:\choreonoid\choreonoid-1.8\CnoidFooPlugin.dll"を検出しました．

 ...

 
 Fooプラグインが読み込まれました．

このメッセージを確認することで、実際にプラグインが読み込まれていることが分かります。

プラグインの読み込みに問題がある場合は通常エラーメッセージが表示されますので、作成したプラグインが機能していない場合は、読み込みに失敗していないかまずメッセージビュー上で確認するようにしてください。

プラグインの読み込みに成功すれば、プラグインに実装した機能が利用できるようになります。
