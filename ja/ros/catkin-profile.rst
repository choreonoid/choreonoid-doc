Catkinプロファイル機能の活用
============================

ROSのノードやシステムの開発をしていると、様々なビルドオプションを切り替えて使用したいことがあります。これを行うにあたっては、Catkinが備えているプロファイル機能が便利です。ここではその使い方を紹介します。

.. contents::
   :local:

.. highlight:: sh

.. _ros_catkin_profile_overview:

プロファイル機能
----------------

Catkinでは "catkin config" コマンドでビルドや実行に関わる様々な設定を行うことができます。これについて、いくつかの設定の組を切り替えて使用したい場合があります。この例として以下のようなケースが考えられます。

* 通常はリリース用のビルドオプションを適用して高速に動作する設定を使用するが、デバッグの際にはデバッグ用のオプションを適用したい
* システムにおける特定の機能の開発に集中するため、ビルド・実行対象を一時的に絞りたいが、その作業が終わったら全体のビルド・実行に戻したい
* 実験的な機能（ノード）を一時的に含めたいが、その作業が終わったら元に戻したい

特にシステムにC++で記述されたノードが含まれる場合、そのバイナリの挙動はビルド設定による影響が大きいため、状況によってビルドの設定を変える必要が出てくることが多いです。

このような状況で、切り替えの度に何度も catkin config で関連する設定をひとつずつ指定していくことには手間がかかりますし、また切り替えの際に設定をミスしてしまう可能性も高くなります。設定ごとにcatkinのワークスペースを用意することも考えられますが、その場合はワークスペースを追加で用意する手間がかかりますし、必要なストレージ容量やビルド時間も無駄に増えてしまいます。

この問題はCatkinのプロファイル機能で解決できます。プロファイル機能では、ワークスペースの設定を複数の「プロファイル」で管理します。切り替えたい設定をそれぞれプロファイルとして用意しておけば、それらを簡単に切り替えて使用することができます。上記のデバッグの例で言えば、「リリース版のプロファイル」と「デバッグ版のプロファイル」を用意することで、それらを切り替えることが可能となります。

この機能の詳細は `Catkin Command Line Tools のマニュアル <https://catkin-tools.readthedocs.io/en/latest/index.html>`_ の `profile コマンドのページ <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_profile.html>`_ を参照してください。使用例についてはマニュアルの `Profile Cookbook <https://catkin-tools.readthedocs.io/en/latest/cheat_sheet.html#profile-cookbook>`_ が参考になります。

.. _ros_catkin_profile_operations:

プロファイルの基本操作
----------------------

プロファイルは基本的には catkin config で行う設定に対応します。通常は default というプロファイル名で設定が行われますが、設定の対象となるプロファイル名を指定することが可能です。 例えば、デバッグ用のプロファイルを作成する場合は以下のようにします。 ::

 catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug ...

ここではまず "--profile" オプションで "debug" というプロファイル名を与えています。また "-x" オプションで "_debug" というサフィックスを指定しています。このサフィックスはビルドに関連するディレクトリに付与されます。例えば通常は "build" ディレクトリ上でビルドが行われ、実行対象となるものは "devel" ディレクトリで管理されますが、上記の例ではそれぞれ "build_debug" 、 "devel_debug" というディレクトリが使われるようになります。これを指定しておけば、プロファイルを切り替える度にビルドしなおさなくてもよくなります。

そして通常の catkin config コマンドで行うように、 "--cmake-args" などを用いたビルド設定を行います。ここではCMakeのビルドタイプを "Debug" とすることで、デバッグ用のオプションをつけてビルドするようにしています。他にも任意の設定を行うことが可能です。

プロファイルの作成は他に ::

 catkin profile add プロファイル名

として行うことも可能です。

.. note:: addコマンドのヘルプによると、"--copy" もしくは "--copy-active" のオプションによって、既存のプロファイルの設定をコピーすることができるように書いてあります。ただし catkin tools バージョン 0.5.0 で試したところ、これらのオプションを指定するとプロファイルが作成されませんでしたし、既存のプロファイルに対して "--force" オプションと共に指定しても結局設定内容はコピーされなかったりして、うまく動作しませんでした。このバージョンのcatkinのバグかもしれません。

ワークスペース上で管理されているプロファイルは以下のコマンドで確認できます。 ::

 catkin profile list

このコマンドを実行すると、以下のような出力がされます。 ::

 [profile] Available profiles:
 - default (active)
 - debug

ここではデフォルトのプロファイル（default）の他に、debugというプロファイルが存在することを確認できます。またデフォルトのプロファイルがアクティブであることも分かります。

catkin のコマンドにおいて特にプロファイルの指定を行わなければ、現在アクティブなプロファイルが使用されます。アクティブなプロファイルは "catkin profile set" コマンドで切り替えられます。debugプロファイルをアクティブにする場合は、以下のようにします。 ::

 catkin profile set debug

アクティブなプロファイルを変えずに、catkin のコマンドごとに対象とするプロファイルを指定することも可能です。その場合は "--profile" オプションを用います。先程紹介した catkin config の例でもこれを使っていました。このオプションはcatkinの他のコマンドでも有効なので、例えばcatkin buildに対しても以下のように適用できます。 ::

 catkin build --profile debug

このようにすると、debugプロファイルの設定に基づいたビルドが行われます。もちろん、アクティブプロファイルをあらかじめdebugに切り替えておけば、このオプションを付けなくてもdebugプロファイルを対象としたビルドになります。


特定プロファイルの生成物の実行
------------------------------

設定やビルドにおけるプロファイルの切り替えは上記のようにして実現できますが、切り替えたプロファイルの生成物を実行する場合は、注意が必要です。例えばこれまで想定してきたように、デフォルトのプロファイルがリリース版で、それとは別にデバッグ用のdebugというプロファイルがあるとします。この状況で以下のコマンドでChoreonoidを実行する場合 ::

 rosrun choreonoid_ros choreonoid

リリース版とデバッグ版のバイナリをどのようにして切り替えたらよいでしょうか？

プロファイルに対してサフィックスの設定を行っていなければ、ワークスペース上でビルドした生成物が格納されるディレクトリは同じになります。この場合ワークスペース上で利用できる生成物は最後にビルドを行った時のプロファイルのものしかありません。従って、実行するバイナリを切り替える際にはそれを生成するために再度ビルドを行う必要が生じます。ビルドオプションが異なる場合は基本的にビルドを全てやり直す必要が出てくるので、頻繁にプロファイルを切り替えて実行したい場合、この方法は適していません。（プロファイル切り替え後は、多くの場合、catkin buildの前にcatkin cleanも実行する必要があるようです。）

これとは異なり、プロファイルに対して異なるサフィックスを指定していた場合は、各プロファイルに対応する生成物がワークスペース上に同時に存在することが可能となります。例えばdebugプロファイルについてサフィックスを "_debug" としていた場合は、各プロファイルの生成物はそれぞれ以下のディレクトリに格納されます。

* デフォルトプロファイル（リリース版）
 * ワークスペースディレクトリ/devel
* debugプロファイル（デバッグ版）
 * ワークスペースディレクトリ/devel_debug

.. note:: サフィックスは :ref:`ros_catkin_profile_operations` で示したように、 catkin config コマンドの "-x" オプションで指定できます。サフィックスをdebugとしたい場合は "catkin config -x debug" とします。

この場合、プロファイルの切換ごとにビルドをやり直す必要はなくなります。その一方で、実行時にどちらのディレクトリのものが実行されるかが問題となります。

どうもCatkin自体には実行対象のプロファイルを切り替える機能は無いようです。これを行うためには、 :ref:`loading_catkin_workspace_setup_script` の対象となるスクリプトを変える必要があるようです。これについて、デフォルトでは "ワークスペースディレクトリ/devel/setup.bash" というスクリプトを取り込むようにしていました。この場合、devel以下の生成物が実行されるよう、環境変数PATHの設定などが行われます。サフィックスを "_debug" としている生成物を実行する場合は、取り込むスクリプトを "ワークスペースディレクトリ/devel_debug/setup.bash" に切り替えればOKです。

具体的には、 .bashrc の記述内容を ::

 source $HOME/catkin_ws/devel_debug/setup.bash

に置き換えて、端末を起動しなおすか、 端末上でこのコマンドを実行します。（ただし端末上でこのコマンドを実行する場合、PATHの設定内容がその度に追加されていくことになるので、あまりよくないかもしれません。）

プロファイルにサフィックスをつけている場合は、この方法で実行対象のプロファイルを切り替えることができます。


デバッガとの連携
----------------

上記の方法でデバッグ版のバイナリを用意するることで、Choreonoid本体やプラグイン、その他C++で記述されたROSノードをデバッガを用いてデバッグすることが可能となります。

ROSでは基本的にプログラムをrosrunやroslaunchといったコマンドで実行することになるので、デバッガ上で直接実行するのは若干やりづらいかもしれません。これについては、デバッガには通常「既存のプロセスにアタッチする」機能がありますので、それを用いて、まずプログラムを起動してから、デバッガをそれに接続するようにします。この方法でROSのシステムをデバッグできます。
