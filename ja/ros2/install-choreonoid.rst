Choreonoid本体のインストール
===========================

ここではROS 2と連携させるChoreonoidのインストールについて説明します。

.. contents::
   :local:

.. highlight:: sh

Choreonoid本体のインストール方法
-------------------------------

ROS 2と連携させるChoreonoid本体のインストールについて、以下の選択肢があります。

1. ROS 2とは独立して、通常のインストールを行う
  A. パッケージをインストールする
  B. ソースコードからビルドしてインストールする
2. ROS 2のパッケージとしてインストールする

.. _ros2_install_choreonoid_standard_method:

通常のインストール方法
---------------------

既にインストールしてあるChoreonoid本体があるのであれば、それを使うのが簡単かつ確実でおすすめです。
ただし、この方法が使用できるようになったのは、Choreonoid本体のバージョン2.3.0以降になりますので、その点にはご注意ください。

Choreonoid本体のインストール方法として一番簡単なのは、上記1-A の :doc:`パッケージによるインストール <../install/install-package-ubuntu>` になります。

必要であれば 1-B の :doc:`ソースコードからのビルドとインストール <../install/build-ubuntu>` を行ってもかまいません。
その場合は、 :ref:`ビルド後のインストール作業 <build-ubuntu_install>` も行っておくようにします。
インストール先にパスが通っている必要がありますので、setup.bashスクリプトも確実に実行されるようにしておいてください。

.. _ros2_install_choreonoid_register_to_rosdep:

通常のインストール方法でインストールしたChoreonoidをrosdepに登録する
------------------------------------------------------------------

通常のインストール方法でChoreonoid本体をインストールした場合、rosdepがChoreonoidを認識できるように登録しておくことで、他のROS 2パッケージからの依存関係を適切に解決できるようになります。

rosdepへの登録は以下の手順で行います。

まず、rosdepのソースリストファイルを作成します。 ::

 echo "yaml file:///etc/ros/rosdep/choreonoid.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/50-choreonoid.list

次に、Choreonoid用のルール定義ファイルを作成します。

パッケージインストールの場合： ::

 sudo tee /etc/ros/rosdep/choreonoid.yaml << EOF
 choreonoid:
   ubuntu: choreonoid
 EOF

ソースコードからビルドしてインストールした場合： ::

 sudo tee /etc/ros/rosdep/choreonoid.yaml << EOF
 choreonoid:
   ubuntu:
     source:
       uri: 'file:///usr/local'
 EOF

上記の ``/usr/local`` の部分は、実際のChoreonoidのインストール先に合わせて変更してください。

設定ファイルを作成したら、rosdepのデータベースを更新します。 ::

 rosdep update

更新が完了したら、以下のコマンドで確認できます。 ::

 rosdep resolve choreonoid

これで、他のROS 2パッケージが ``package.xml`` でChoreonoidを依存パッケージとして指定している場合でも、rosdepが正しく認識できるようになります。

.. note:: この設定を行うことで、 ``rosdep install`` コマンドを実行した際に、Choreonoidが既にインストールされていることが認識され、不要な再インストールやエラーを避けることができます。

.. _ros2_install_choreonoid_install_as_ros2_package:

ROS 2のパッケージとしてインストールする方法
------------------------------------------

以前はこの方法が用いられていました。
次の :doc:`build-choreonoid` で説明するROS 2のワークスペース上でChoreonoid本体もソースからビルドします。

この方法を用いる必要が特にない場合は、「通常のインストール方法」を行ってください。

なお、この方法でインストールした場合は、ChoreonoidがROS 2のワークスペース内にパッケージとして存在するため、rosdepへの登録は必要ありません。

また、この方法を用いる場合は、既にインストールされているChoreonoidがあると、そちらと干渉する可能性があります。
実行ファイルや共有ライブラリ、CMake、インクルードファイル等のパスについて干渉しないよう設定に注意するようにしてください。
詳しくは :ref:`ros2_build_choreonoid_as_ros2_package_note` を参照ください。

