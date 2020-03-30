準備
====

.. contents::
   :local:

.. _ros_tank_tutorial_package_setup:

ROSとChoreonoid関連パッケージのセットアップ
-------------------------------------------

本チュートリアルの実施にあたっては、まず :doc:`../install-ros` と :doc:`../build-choreonoid` を行っておく必要があります。これらのページの手順に従ってインストールやビルドを行い、 :doc:`../run-choreonoid` ができるようにしておいてください。

なお、:ref:`ros_choreonoid_add_package_sources` においては、Choreonoid関連パッケージとして以下の3つが導入されていればOKです。

* choreonoid
* choreonoid_ros
* choreonoid_joy

上記ページではchoreonoid_ros_samplesも導入していますが、そちらは本チュートリアルの実施には必要ありません。（もちろん、導入されていても特に問題はありません。）

.. _ros_tank_tutorial_make_package:

チュートリアル用パッケージの作成
--------------------------------

.. highlight:: sh

ROS上で新たに作成／開発して使用するデータやプログラムは基本的にCatkinの「パッケージ」という単位で構築します。そこで本チュートリアルにおいても、チュートリアルで作成するプログラムやデータを格納するためのパッケージをまず作成しておきます。

パッケージはcatkinワークスペース上で以下のコマンドを実行することで作成します。 ::

 catkin create pkg choreonoid_ros_tank_tutorial

このようにすると、"choreonoid_ros_tank_tutorial" という名前のパッケージが作成されます。これに対応するディレクトリがワークスペース内の "src" ディレクトリに生成され、その中にパッケージに関わるいくつかのファイルが追加されます。

本チュートリアルではこの "choreonoid_ros_tank_tutorial" というパッケージ名を使用するものとします。少し長いですが、これは他のChoreonoid関連パッケージと統一感を持たせるためで、ご了承ください。他の名前を使用しても結構ですが、その場合はチュートリアルにおいて対応する箇所を全て差し替えるようにしてください。

なお、本チュートリアルのパッケージは、Githubにてソースを公開しています。そのリポジトリをクローンすることで、ソースコードの入力を行わずに本チュートリアルを試すことも可能です。この場合、上記のコマンドの代わりに、以下を実行します。 ::

 cd ~/catkin_ws/src
 git clone https://github.com/s-nakaoka/choreonoid_ros_tank_tutorial.git

これで、パッケージを作成した上で、本チュートリアルで必要な全てのファイルを作成したのと同じ状況になります。

.. _ros_tank_tutorial_edit_package_xml:

package.xmlの編集
-----------------

.. highlight:: xml

上記の操作により、パッケージディレクトリには "package.xml" というXMLファイルが追加されています。これはパッケージの情報を記述するためのファイルとなっており、全てのパッケージに必要です。

package.xmlの内容は、 パッケージ生成コマンドを実行する際に、そのコマンドラインオプションによって、いろいろと設定することが可能です。その詳細については `catkin create のマニュアル <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html>`_ を参照してください。

一方でpackage.xmlの内容はテキストエディタを用いて直接編集することも可能です。そのため、まずはオプション無しでパッケージを生成して、その後package.xmlの内容が適切になるようにマニュアルで修正することも可能です。コマンドラインオプションによる指定も複雑になりますので、マニュアルで編集した方が分かりやすいかもしれません。ここではパッケージ生成コマンドのオプションには触れずに、本チュートリアル用に作成すべきpackage.xmlの内容について解説したいと思います。実際のpackage.xmlの内容は以下のようにする必要があります。 ::

 <?xml version="1.0"?>
 <package format="2">
   <name>choreonoid_ros_tank_tutorial</name>
   <version>1.0.0</version>
   <description>
     Tutorial on how to implement a robot controller with I/O using ROS
   </description>
   <maintainer email="nakaoka@choreonoid.co.jp">Shin'ichiro Nakaoka</maintainer>
   <license>MIT</license>
   <author email="nakaoka@choreonoid.co.jp">Shin'ichiro Nakaoka</author>
   <url type="website">http://choreonoid.org</url>
   <url type="repository">https://github.com/s-nakaoka/choreonoid_ros_tank_tutorial.git</url>
   <buildtool_depend>catkin</buildtool_depend>
   <depend>choreonoid</depend>
   <depend>choreonoid_ros</depend>
   <depend>choreonoid_joy</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>image_transport</depend>
   <export>
     <build_type>cmake</build_type>
   </export>
 </package>

記述の詳細は `package.xmlのマニュアル <http://wiki.ros.org/catkin/package.xml>`_ をご参照ください。ここではいくつか重要な部分について解説します。

まず、 ::

 <package format="2">

のタグで、パッケージ記述を開始するとともに、記述のフォーマットがバージョン2であることを明示しています。

Catkinには古い実装とと新しい実装があり、それぞれ使用方法などが多少異なっています（ :ref:`ros_make_catkin_workspace` 参照）。本チュートリアルでは新しい実装を使用するようにしていて、それに対応するためにこの記述を行っています。

次に ::

   <name>choreonoid_ros_tank_tutorial</name>

の記述で、パッケージ名を指定しています。これは他のパッケージと重複しないようにする必要があります。

他に重要なのが、 ::

   <buildtool_depend>catkin</buildtool_depend>
   <depend>choreonoid</depend>
   <depend>choreonoid_ros</depend>
   <depend>choreonoid_joy</depend>
   <depend>std_msgs</depend>
   <depend>sensor_msgs</depend>
   <depend>image_transport</depend>

の部分で、このパッケージが依存している他のパッケージを明示しています。ここでは

* choreonoid: Choreonoid本体
* choreonoid_ros: ChoreonoidのROS連携機能
* choreonoid_joy: Choreonoid用のジョイスティックノード 
* std_msgs: ROSの標準的なメッセージ型
* sensor_msgs: 標準的なセンサに対応するメッセージ型
* image_transport: 画像データ通信のための機能

の各パッケージへの依存を明示しています。Choreonoid本体とROS連携機能のパッケージは当然必要となるわけですが、その他のパッケージについては本チュートリアルで随時解説します。

最後に ::

   <export>
     <build_type>cmake</build_type>
   </export>

という記述をしています。実はこれはROSにおいてはあまり標準的ではない記述です。この "build_type" は、パッケージのビルドをCMakeで行う際の記述方法に関わるオプションです。これは2つの選択肢があり、それぞれ `catkin tools の Supported Build Types <https://catkin-tools.readthedocs.io/en/latest/build_types.html>`_ において以下のように説明されています。

* **catkin**: CMake packages that use the Catkin CMake macros
* **cmake**: "Plain" CMake packages

デフォルトでは "catkin" となります。そちらはCatkinでカスタマイズされたCMakeのマクロを用いてビルドを行います。一方で、これを "cmake　とすると、そのようなマクロは使用せず、通常のCMakeの記述方法が使えるようになるようです。

通常はデフォルトでよいのですが、それはビルドしたファイルをCatkinで決められた場所に配置するのが前提となっているようです。例えば、ノードの実行ファイルや、ライブラリのファイルなどに対して、それぞれ配置される場所が決まっています。

一方で、ビルドしたファイルをそれ以外の場所に配置したい場合もあるかと思います。例えば、ChoreonoidではC++で記述されたプラグインやコントローラのバイナリについて、それ専用のディレクトリが用意されており、通常はそこに格納するようにします。しかし、筆者が試した限りでは、デフォルトの"catkin"のビルドオプションでは、それをどのように行ったらよいか分かりませんでした。そこで "cmake" のビルドオプションを試したところ、そのようなこともできることが分かりました。本チュートリアルでもコントローラを開発を行いますので、"cmake"のビルドオプションを使用するようにします。

このオプションの変更によって、パッケージをビルドするためのCMakeファイルの記述が多少異なってくる場合もありますが、その部分は基本的にChoreonoid本体やchoreonoid_rosパッケージのCMakeマクロに記述されているので、それらの利用側はそれほど気にしなくても大丈夫かと思います。

.. note:: 上記の説明はこれまでの筆者独自の試行錯誤に基づくものであり、このやり方でよいのか確証があるわけではありません。ROSは定められた方法で使用する分には楽な部分がある一方で、そこから少し外れたことをしようとするとあまり情報がなかったり、実現に苦労することもあるように思います。この件に関しても、マニュアルでの説明や掲示板などでの議論があまりないようでした。この件に関して何かご存知の方がいらっしゃいましたら、教えていただけるとうれしいです。

ROSマスターの起動
-----------------

.. highlight:: sh

端末を開いて以下を入力し、ROSマスターを起動しておきます。 ::

  roscore

roslaunchコマンドを用いる場合は、ROSマスターがなければ自動で起動されるようです。チュートリアルではroslaunchも使用しますので、ROSマスターを明示的に起動しておかなくても大丈夫な場合もありますが、一般的にはこの作業は予め行っておきます。

作業用端末の起動
----------------

ROSマスターの起動用とは別に、チュートリアルの作業用に端末を開いて、上記のチュートリアル用ディレクトリに移動しておくようにしてください。なお、端末はチュートリアルを進める上で複数必要となる場合もあります。
