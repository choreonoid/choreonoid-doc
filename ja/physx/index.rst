
PhysXプラグイン
===============

.. contents::
   :local:
   :depth: 1

概要
----

PhysXについて
~~~~~~~~~~~~~

PhysXはNVIDIA社が開発している物理エンジンです。もともとは同社のGPU上で動作する高速な物理計算エンジンとして登場しましたが、現在はCPU上で動作する部分もオープンソース化されており、 `GitHub上 <https://github.com/NVIDIA-Omniverse/PhysX>`_ でBSD 3-Clauseライセンスのもと公開されています。剛体、関節機構、接触計算に加え、布や軟体、粒子などのシミュレーションにも対応する汎用物理エンジンであり、ゲームやロボティクス、CGなど幅広い分野で利用されています。

PhysXの特徴として、まず計算の高速性が挙げられます。PhysXはリアルタイム性を重視する用途において長年磨かれてきた実装を持ち、SIMD命令の活用や並列化、接触計算の効率化などにより高速に動作します。またシーン内の物体数が増えた際のスケーラビリティにも優れており、多数の剛体が接触しあう場面においても実用的な速度で計算を進められる設計となっています。

関節をもつ多体系ロボットのシミュレーションについては、 **Articulation Reduced Coordinate** と呼ばれる縮約座標系に基づく手法が用いられています。これはChoreonoid標準の物理エンジンであるAISTシミュレータと同じく、ロボットの関節空間上で運動方程式を解く方式であり、多関節ロボットをAISTシミュレータと同様に安定に処理することができます。

PhysXプラグインについて
~~~~~~~~~~~~~~~~~~~~~~~

ChoreonoidのPhysXプラグインは、PhysXをChoreonoidの物理エンジンとして利用するためのプラグインです。シミュレータアイテムとして「PhysXシミュレータ」を提供し、他の物理エンジンのシミュレータアイテムと同様の手順で利用することができます。

本プラグインでは、剛体・関節をもつロボットモデルのシミュレーションを主な対象としています。Choreonoidで扱う標準的なモデルをそのまま利用でき、位置制御・速度制御・トルク制御いずれの方式にも対応しています。

また本プラグインでは、履帯(クローラ)機構を実現するためのデバイスとして **PxContinuousTrack** を提供しています。これは履帯を構成する多数のシューを個別の剛体として扱うのではなく、駆動輪や従動輪の周囲に巻き回された連続的な帯として扱い、車輪の回転に応じて履帯のシューが実際に周回して地面を捉える挙動を再現するものです。シューを1つ1つ剛体として扱う方式と比較して計算コストを大幅に抑えつつ、履帯特有の接地挙動を安定にシミュレーションすることができます。これはPhysXプラグイン独自の機能であり、クローラ型ロボットのシミュレーションにおいて特に有用です。

PhysXはChoreonoidにおける準標準の物理エンジンとして位置付けられており、ソースコードがChoreonoid本体のリポジトリに同梱されています。これによりPhysXを別途ダウンロード・ビルドする必要がなく、Choreonoidをビルドするだけで自動的に利用できる状態になります。

セットアップ
------------

基本的なビルド方法
~~~~~~~~~~~~~~~~~~

PhysXプラグインはChoreonoid本体にソースコードが同梱されており、デフォルトで有効になっています。そのため通常のChoreonoidのビルド手順に従うだけで、PhysXプラグインも一緒にビルドされ、すぐに利用できる状態となります。

CMakeのオプションとしては次の2つが用意されていますが、通常は特に変更する必要はありません。

* ``ENABLE_PHYSX`` : 同梱のPhysXライブラリをビルドします。デフォルトは ``ON`` です。

* ``BUILD_PHYSX_PLUGIN`` : PhysXプラグイン本体をビルドします。デフォルトは ``ENABLE_PHYSX`` の値(つまり通常は ``ON``)です。

PhysXを使いたくない場合は、CMakeの設定で ``ENABLE_PHYSX`` と ``BUILD_PHYSX_PLUGIN`` の両方を ``OFF`` にしてください。

.. note::

  ``BUILD_PHYSX_PLUGIN`` のデフォルト値は ``ENABLE_PHYSX`` の値に連動しますが、これはCMakeの仕様により「キャッシュに値が保存されていない場合のみ」有効です。そのため、一度 ``ENABLE_PHYSX=ON`` でCMake設定を行った後に ``ENABLE_PHYSX`` のみを ``OFF`` に変更すると、 ``BUILD_PHYSX_PLUGIN`` はキャッシュ値の ``ON`` のままとなり、外部PhysXを使う経路としてエラーになります。両方を明示的に ``OFF`` に設定するか、ビルドディレクトリを作り直してから設定してください。

補足：外部ビルドのPhysXを使用する場合
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

同梱のPhysXではなく、ユーザが独自にビルドしたPhysXライブラリを利用したい場合は、以下の手順でまず外部のPhysX SDKをビルドし、そのうえでChoreonoid側のCMake設定を行います。同梱のPhysXに不具合があった場合や、新しいバージョンのPhysXを試したい場合などに利用できます。

PhysX SDKのソースコードは `NVIDIA-Omniverse/PhysX (GitHub) <https://github.com/NVIDIA-Omniverse/PhysX>`_ で公開されています。

なお、ここではLinux環境(Ubuntu 24.04にて動作確認)での手順を説明します。

**依存パッケージのインストール**

PhysX SDKのビルドには以下のパッケージが必要です。

.. code-block:: sh

  sudo apt install cmake build-essential python3 curl

**PhysX SDKのソース取得**

GitHubからPhysX SDKのソースコードを取得します。

.. code-block:: sh

  git clone https://github.com/NVIDIA-Omniverse/PhysX.git

特定のバージョンを使用したい場合は、そのリリースタグに切り替えてください。例えば同梱のPhysXと同じバージョンを使用したい場合は、以下のようにします。

.. code-block:: sh

  cd PhysX
  git checkout 107.3-physx-5.6.1

**Snippetsビルドの無効化**

デフォルトの設定ではPhysXのサンプル(Snippets)もビルドされますが、ビルドエラーになる場合があります。Choreonoidで使用するライブラリのビルドには不要なので、無効化しておきます。

以下のファイルをテキストエディタで開いてください。

.. code-block:: text

  physx/buildtools/presets/public/linux-gcc-cpu-only.xml

``PX_BUILDSNIPPETS`` の value を ``"True"`` から ``"False"`` に変更します。

.. code-block:: text

  変更前: <cmakeSwitch name="PX_BUILDSNIPPETS" value="True" .../>
  変更後: <cmakeSwitch name="PX_BUILDSNIPPETS" value="False" .../>

**PhysX SDKのビルド**

physxディレクトリに移動し、ビルドプロジェクトを生成します。

.. code-block:: sh

  cd physx

以下のいずれかの方法でプリセットを指定します。

方法A: コマンドライン引数で直接指定する場合:

.. code-block:: sh

  ./generate_projects.sh linux-gcc-cpu-only

方法B: 対話メニューで選択する場合:

.. code-block:: sh

  ./generate_projects.sh

この場合はプリセットの一覧が表示されるので、 ``linux-gcc-cpu-only`` を選択してください。

続いて、生成されたreleaseビルドディレクトリに移動してビルドします。

.. code-block:: sh

  cd compiler/linux-gcc-cpu-only-release
  make -j$(nproc)

ビルドが完了すると、 ``PhysX/physx/bin/linux.x86_64/release/`` 以下に静的ライブラリファイルが生成されます。

**ChoreonoidのCMake設定**

Choreonoid側では、外部PhysXを使用するようCMake設定を行います。ビルドディレクトリでCMakeを呼び出す際に、以下のいずれかの方法で設定します。

方法A: コマンドラインで直接指定する場合:

.. code-block:: sh

  cmake <choreonoidのソースディレクトリ> \
        -DENABLE_PHYSX=OFF \
        -DBUILD_PHYSX_PLUGIN=ON \
        -DPHYSX_DIR=<PhysXのphysxディレクトリへのパス> \
        -DPHYSX_BINARY_TYPE=release

``PHYSX_DIR`` には上記手順でクローンしたPhysXリポジトリ内の ``physx`` ディレクトリの絶対パスを指定してください(例: ``-DPHYSX_DIR=$HOME/PhysX/physx`` )。

方法B: ``ccmake`` で対話的に設定する場合:

.. code-block:: sh

  ccmake <choreonoidのソースディレクトリ>

ccmakeの画面で以下の項目を設定してください。

* ``ENABLE_PHYSX`` を ``OFF`` に変更
* ``BUILD_PHYSX_PLUGIN`` を ``ON`` に変更
* ``PHYSX_DIR`` にPhysXの ``physx`` ディレクトリの絶対パスを入力
* ``PHYSX_BINARY_TYPE`` に ``release`` を入力

設定後、 ``[c]`` キーで設定を適用し、 ``[g]`` キーでMakefileを生成します。

CMakeの設定が完了したら、通常どおりChoreonoidをビルドしてください。

``PHYSX_BINARY_TYPE`` はPhysX SDKのビルド種別を指定する変数で、以下の4種類があります。

* ``release`` : 通常利用向けの最適化ビルド。最も高速
* ``profile`` : 最適化 + プロファイリング機能 + PVD対応
* ``checked`` : 最適化 + API呼び出しの引数チェック
* ``debug`` : 最適化なし + PhysX内部のデバッグ情報あり

特別な理由がなければ ``release`` を指定してください。指定する種別は事前にPhysX SDK側でビルドしておく必要があります。

サンプル
--------

PhysXプラグインを利用したサンプルプロジェクトが ``choreonoid/sample/PhysX/`` ディレクトリに用意されています。

PxTank
~~~~~~

:file:`sample/PhysX/PxTank.cnoid` は、戦車型のロボットモデルを用いた履帯シミュレーションのサンプルです。PhysXプラグイン特有の履帯(クローラ)シミュレーション機能を利用しており、複数の従動輪と駆動輪の間に履帯が巻かれた構造のロボットが、平坦地および起伏地形の上を走行する様子を確認できます。

履帯については、1つ1つのシューを剛体として扱う一般的な手法とは異なり、PhysXプラグインでは履帯を単一の仮想的な帯として扱う専用の機構が用意されています。この機構は履帯の接地状態を接触力として表現しつつ、計算コストを抑えて安定なシミュレーションを実現するものです。

このプロジェクトを開いてシミュレーション実行ボタンを押すと、タンクが自動的に動作を始めます。

PxCrawlerJoystick
~~~~~~~~~~~~~~~~~

:file:`sample/PhysX/PxCrawlerJoystick.cnoid` は、クローラ型の走行ロボットをジョイスティックで操縦するサンプルです。PxTankと同じく履帯シミュレーション機能を利用していますが、ユーザがゲームパッド等のジョイスティックを用いてリアルタイムに操作できる点が異なります。

シミュレーション実行中に、接続したジョイスティックのスティックを操作することで、ロボットを前進、後退、旋回させることができます。
