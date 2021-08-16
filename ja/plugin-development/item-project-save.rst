==============================
プロジェクトアイテムの状態保存
==============================

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

:doc:`../basics/item` で解説しているように、Choreonoidではプロジェクトアイテムの構成や状態、各種データと、関連するツールバーやビューなどの各種インタフェースの状態について、一括してプロジェクトファイルに保存できます。保存したプロジェクトファイルを読み込むことでプロジェクト全体の状態を復帰し、作業を継続することができます。

本節では、プロジェクトアイテムの状態をプロジェクトファイルに保存できるようにする方法について解説します。

.. _plugin-dev-state-store-restore-functions:

状態保存・復帰用関数
--------------------

アイテムの状態をプロジェクトファイルに保存したりそこから復帰するための関数として、 `Itemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ には以下のvirtual関数が定義されています。

* **virtual bool store(Archive& archive)**

  * アイテムの状態を保存します。

* **virtual bool restore(const Archive& archive)**

  * アイテムの状態を復帰します。

各アイテムクラスにおいてこれらの関数をオーバーライドして実装することで、アイテムの状態保存・復帰に対応させることができます。
それぞれ処理に成功した場合はtrueを、失敗した場合はfalseを返すようにします。
falseを返した場合は、プロジェクトファイルへの保存やプロジェクトファイルからの復帰はスキップされます。

これらの関数では `Archive型 <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Archive.html>`_ の引数archiveを介してアイテムの状態に関する情報をやりとりします。この詳細については後述します。

:ref:`plugin-dev-put-property-function` と同様に、この関数を実装するアイテム型が他の（Itemクラスでは無い）アイテム型を継承している場合、通常親クラスの同じ関数を呼び出す必要があります。例えばFooItemを継承しているBarItemについて、store、restore関数はそれぞれ以下のような形態で実装します。 ::

 bool BarItem::store(Archive& archive)
 {
     if(FooItem::store(archive)){
         // BarItemのstore処理
         ...
         return true;
     }
     return false;
 }

 bool BarItem::restore(const Archive& archive)
 {
     if(FooItem::restore(archive)){
         // BarItemのrestore処理
         ...
         return true;
     }
     return false;
 }

.. _plugin-dev-yaml-structured-data-classes:

YAML型構造化データクラス群
--------------------------

状態保存・復帰用関数の引数である `Archiveクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Archive.html>`_ は、 `Mappingクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Mapping.html>`_ を継承してプロジェクトの保存・復帰に関連する機能を追加したものです。Mappingクラスは「構造化データクラス群」と呼んでいるクラス群に含まれるもので、このクラスを介して各種データを構造化して格納できます。

「YAML型構造化データクラス群」に冠されている `YAML <http://yaml.org/>`_ は、構造化されたデータやオブジェクトをテキストで記述するための汎用的なデータ記述言語です。データやオブジェクトのシリアライズを主な目的として定義されていて、シンプルで可読性が高く、様々なプログラミング言語が対応していて、現在広く使われています。またYAMLで記述されたテキストは多くの場合ファイルに保存されて使用されます。このファイルをYAMLファイルといいます。

Archiveを含む「YAML型構造化データクラス群」の使用にあたっては、まずYAMLについて基本的な知識を有していることが望ましいです。それが無い場合は、以下に挙げるような資料を参考にして基本的な仕様や使い方を把握するようにしてください。

* `プログラマーのためのYAML入門（初級編） <https://magazine.rubyist.net/articles/0009/0009-YAML.html>`_ 

* `WikipediaのYAMLのページ <https://en.wikipedia.org/wiki/YAML>`_

「YAML型構造化データクラス群」はこのYAMLの構造をC++のクラスで表現したもので、Choreonoid SDKのUtilライブラリで定義・実装されています。YAMLのテキストを読み込んでこのクラス群のオブジェクトを生成するパーザや、このクラス群のオブジェクトをYAMLのテキストとして出力するライタも用意されており、YAMLファイルの読み込みや書き込みも容易に行なえます。

このクラス群は以下のクラスで構成されます。

* `Mapping <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Mapping.html>`_ 

  * YAMLのMaaping(連想配列、ハッシュ）に対応
  * キーと値のペアを複数格納可能
  * 各値はMapping、Listing、ScalarNodeのいずれかのオブジェクト
  * 対応するスマートポインタ型としてMappingPtrが定義済み

* `Listing <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Listing.html>`_ 

  * YAMLのSequence（配列）に対応
  * 配列要素として複数の値を格納可能
  * 各値はMapping、Listing、ScalarNodeのいずれかのオブジェクト
  * 対応するスマートポインタ型としてListingPtrが定義済み

* `ScalarNode <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ScalarNode.html>`_ 

  * YAMLのスカラ値に対応
  * スカラ値としてbool、int、double、stringのいずれかの値を格納
  * 対応するスマートポインタ型としてScalarNodePtrが定義済み

いずれも基底クラスとして `ValueNodeクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ValueNode.html>`_ を継承しています。ValueNodeはReferencedを継承しているので、上記クラスは全て :doc:`referenced` となります。このクラス階層を図示すると以下になります。

.. code-block:: text

 + Referenced
   + ValueNode
     + Mapping
     + Listing
     + ScalarNode

これらのクラスはUtilライブラリのValueTree.h、ValueTree.cppで定義・実装されていて、Choreonoid SDKにおいてはValueTreeヘッダをインクルードすることで使用できるようになります。
    
これらのクラスによるデータの構築例について紹介します。例えばYAMLで以下のように記述されるデータがあるとします。

.. code-block:: yaml

 color: red
 height: 1.8
 translation: [ 0.0, 1.0, 2.0 ]

これに対応するデータは構造化データクラスのオブジェクトを用いて以下のように構築できます。

* **Mapping**

  * キー: color

    * 値: **ScalarNode("red")**

  * キー: height

    * 値: **ScalarNode(1.8)**

  * キー: translation

    * 値： **Listing**

      * 値： **ScalarNode(0.0)**

      * 値： **ScalarNode(1.0)**

      * 値： **ScalarNode(2.0)**

ここで太字になっているところが上記クラスのオブジェクトです。
（キーの部分はそれ単体ではオブジェクトではなく、Mappingオブジェクトの一部です。）
これらのオブジェクトを、データ構造における「ノード」と呼びます。
複数のノードが階層的な親子関係を構築するツリー構造になっています。
正確にはあるノードを複数の親ノードが共有することが可能なので、グラフ構造になります。

この例では、データ全体に対応するのが最上位のMappingノードです。
そこから階層的に、各値がノードとして保有されています。

このデータを生成するC++コードは以下のように記述できます。 ::

 #include <cnoid/ValueTree>
 ...
 
 // 最上位ノードのMappingオブジェクトを生成
 MappingPtr data = new Mapping;
 // ノードにキーと値（ScalarNode）のペアを追加
 data->write("color", "red");
 data->write("height", 1.8);
 // 値としてListingノードを追加
 auto translation = data->createListing("translation");
 // Listingノードの要素（ScalarNode）を追加
 translation->append(0.0);
 translation->append(1.0);
 translation->append(2.0);

translationノードの構築については、値が三次元ベクトル型Vector3に格納されている場合、EigenArchiveヘッダの関数を用いて以下のように記述できます。 ::

 #include <cnoid/EigenArchive>
 ...

 Vector3 translation;
 ...

 write(data, translation);

このデータは `YAMLWriterクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1YAMLWriter.html>`_ を用いることでYAMLファイルとして出力できます。これは以下のようにします。 ::

  #include <cnoid/YAMLWriter>
  ...

  YAMLWriter writer("data.yaml")
  writer.putNode(data);

逆にYAMLファイルを読み込んで構造化データを構築することもできます。これは `YAMLReaderクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1YAMLReader.html>`_ を用いて以下のようにします。 ::

  #include <cnoid/YAMLReader>
  ...

  YAMLReader reader;
  MappingPtr data;
  try {
      data = reader.loadDocument("data.yaml")->toMapping()
  }
  catch(const ValueNode::Exception& ex){
      ...
  }

この場合、読み込みに成功するとMapping型のオブジェクトが変数nodeに代入されます。
YAMLファイルに問題がある場合は、ValueNode::Exception型の例外がスローされます。

またMappingオブジェクトに上記の構造でデータが格納されていることを期待して読み込むコードは以下のように記述できます。 ::

  std::string color;
  double height;
  Vector3 translation;

  data->read("color", color);
  data->read("height", height);

  // translationの3要素の読み込み
  auto translationNode = data->findListing("translation");
  if(translationNode->isValid()){
      if(translationNode->size() == 3){
          for(int i=0; i < 3; ++i){
              translation[i] = translationNode->at(i)->toDouble();
          }
      }
  }

この例では、データが想定した構造であれば変数color、height、translationに読み込んだ値が代入されます。

read関数の代わりにget関数を用いることで、デフォルト値を指定した読み込みができます。例えば ::

  std::string color = data->get("color", "red");
  double height = data->get("height", 1.8);

とすると、最上位ノードにcolorやheightのキーが含まれない場合は、それぞれ"red"と1.8がデフォルト値として返されます。
  
またtranslationの読み込みについては、EigenArchiveヘッダの関数を用いて、以下のように一行で書くこともできます。 ::

 #include <cnoid/EigenArchive>
 ...
 
 read(data, "translation", translation);

このように、 YAML型構造化データクラス群やその関連クラスを用いることで、YAMLと同じ構造で構造化データを読み書きすることが可能となります。構造化データクラス群の各クラスは読み書きのための様々なメンバ関数を備えていて、それらを用いて読み書きのコードを柔軟に記述できます。またEigenArchiveヘッダの関数のように、特定の型の読み書きを簡潔に記述するための関数も用意されています。それらの詳細については、APIリファレンスマニュアルなどを参照してください。またChoreonoidのソースコードでstoreやrestore関数を実装している部分についても、使用方法の参考になるかと思います。

.. _plugin-dev-archive-class:

Archiveクラス
-------------

これまでの例にもみられるように、YAML型構造化データはMappingを最上位ノードとして使用することが多く、Mappingがデータの読み書きにおいて中心的な役割を果たします。そこでアイテムの状態保存・復帰用関数でもMappingクラスのオブジェクトを介して状態の読み書きを行うことが考えられます。

ただしMappingクラスは構造化データ格納のための汎用的なクラスなので、プロジェクトの保存・復帰を行うために必要な機能が必ずしも全て備わっていない部分があります。その部分についてもひとつの引数にまとめられた方がAPIを簡潔にすることができるので、そのためのクラスとして `Archiveクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Archive.html>`_ が定義されています。これはMappingクラスに対してプロジェクトの保存・復帰に関連する機能（関数）を追加したものとなっています。

追加された関数は `Archiveクラスのリファレンス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Archive.html>`_ を参照ください。以下ではプロジェクトアイテムの状態保存・復帰の実装で利用できる主要な関数についてカテゴリごとに紹介します。

.. _plugin-dev-archive-post-processing:

後処理に関わる関数
~~~~~~~~~~~~~~~~~~

プロジェクトファイルは :ref:`plugin-dev-project-file-structure` で述べるように、アイテムをはじめとしてプロジェクトに関わる様々なオブジェクトの状態が記録されていて、それを順次読み込んでいくようになっています。その中で、あるオブジェクトの状態が他のオブジェクトに依存している場合もあります。しかしそのようなオブジェクトの読み込み時に、依存先のオブジェクトがまだ読み込まれていないこともあり得ます。その場合は依存先のオブジェクトが読み込まれるのを待って、その後に関連する状態の読み込みを行う必要があります。

そのような処理を状態復帰の「後処理」と呼びます。Archiveクラスの以下の関数でこの後処理を行うことが可能です。

* **void addProcessOnSubTreeRestored(const std::function<void()>& func) const**

  * 現在読み込んでいるアイテムのサブツリーが全て読み込まれたタイミングで指定した関数を実行します。

  * アイテムの状態が自身のサブツリー内の他のアイテムに依存している場合に使用します。

* **void addPostProcess(const std::function<void()>& func, int priority = 0) const**

  * プロジェクト内のオブジェクトが全て読み込まれた後に指定した関数が実行されます。

  * 複数の後処理関数がある場合は、引数priorityで実行順序が決まります。省略した場合のデフォルト値は0で、priorityが小さいほうが先に実行されます。

* **void addFinalProcess(const std::function< void()>& func) const**

  * addPostProcessによって実行される全ての後処理が完了した時点で、指定した関数が呼ばれます。

これらは基本的にアイテムのrestore関数内で使用します。
addPostProcessとaddFinalProcessについては、後処理関数の中で再帰的に使用して、更なる後処理を行うことも可能です。

.. _plugin-dev-archive-item-reference:

アイテムの参照に関わる関数
~~~~~~~~~~~~~~~~~~~~~~~~~~

プロジェクトに含まれる他のアイテムの参照に関わる以下の関数を利用できます。

* **Item* currentParentItem() const**

  * プロジェクト読み込み時に、現在読み込み中のアイテムの親アイテムを返します。状態の復帰において親の情報が必要となる場合に使用します。

  * アイテムは読み込みが完了（成功）してから親アイテムに追加されますので、読み込み中は自身の親を参照することができません。この関数により、親になる予定のアイテムを参照することができます。

* **ValueNodePtr getItemId(Item *item) const**

  * 同じプロジェクトに含まれるアイテムのIDを取得します。プロジェクト保存時に他のアイテムへの参照を記録するために使用します。

  * IDは通常は整数値を格納したスカラノード（ScalarNode）となりますが、 :ref:`basics_composite_item` を構成するサブアイテムの場合は、「本体アイテムの整数ID値」 + 「サブアイテムに至るアイテム名（複数可）」を格納したListingになります。

* **Item* findItem(ValueNodePtr id)const**

  * 同じプロジェクトに含まれるアイテムをIDで指定して取得します。プロジェクト読み込み時にアイテムへの参照を解決するために使用します。IDに対応するアイテムがみつからない場合はnullptrを返します。

  * IDはgetItemIdの戻り値と同じ形式になります。

  * 取得するアイテムの型を指定可能なテンプレート版も利用できます。

getItemIdとfindItemの使用例を以下に示します。FooItemがBarItemへのポインタを保持していて、これをプロジェクト読み込み時に復帰させたいとします。 ::

 class FooItem : public Item
 {
     BarItem* barItem;

 public:
     ...

     virtual bool store(Archive& archive) override;
     virtual bool restore(const Archive& archive) override;
 };

この場合、store関数で以下のようにします。 ::

 bool FooItem::store(Archive& archive)
 {
     auto id = archive.getItemId(barItem);
     archive.write("bar_item_id", id);
     ...
     return true;
 }

するとFooItemの状態データとして、"bar_item_id"というキーにBarItemのIDが書き込まれます。

restore関数では以下のように実装します。 ::

 bool FooItem::restore(const Archive& archive)
 {
     archive.addPostProcess(
         [this, &archive](){ barItem = archive.findItem<BarItem>(archive.find("bar_item_id"); });
     ...
     return true;
 }

ここではaddPostProcessを使用して、後処理の中で参照の解決をするようにしています。BarItemがどこに存在するか分からない場合や、FooItemよりも後に読み込まれる場合は、このようにしておく必要があります。何故ならFooItemが読み込まれる時点でBarItemは必ずしも存在していないからです。

あるいは、BarItemがFootItemのサブツリー内に存在することが分かっている場合は、addPostProcessの代わりにaddProcessOnSubTreeRestoredを使用することも可能です。BarItemが確実にFooItemより上位に存在するという場合は、restore関数内で直接参照解決をしてもかまいません。

IDに対応するアイテムの取得にはfindItem関数のテンプレート版を使用しています。
これにより直接BarItem型のオブジェクトを得ています。

.. note:: この方法によって他のアイテムへの参照を直接的に解決することは通常は行わない方がよいでしょう。その代わりに、アイテム間の親子関係などを頼りに参照を解決するのがより望ましいですし、Choreonoidで標準的な方法になります。アイテムのIDによる参照の解決は、どちらかと言うとビューなどの他の種類のオブジェクトが、関連するアイテムへの参照を解決するために使用するものとなります。

.. _plugin-dev-relocatable-filepath-functions:

再配置可能ファイルパスに関する関数
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

プロジェクトアイテムの中にはファイルからデータを読み込むものもあります。
その場合アイテムはファイルパスやファイル形式といったファイル関連の情報も保持する必要がり、それらの情報はプロジェクト保存においても記録することになります。
そこで注意すべき点として、ファイルはファイルシステムの様々なディレクトリに配置されている可能性があり、それらは環境やユーザが変わると別の場所になり得るということがあります。
そのような場合でも元のプロジェクトを復帰できるように、各ファイルのパスについてはなるべく可搬性のある形式で記録しておくことが望ましいです。そのような形式のファイルパスを「再配置可能ファイルパス」と言うことにします。
Archiveクラスはこの形式を利用するための関数も備えています。

まず再配置可能ファイルパスを取得するための関数として以下を備えています。これらは主にstore関数から利用します。

* **std::string getRelocatablePath(const std::string& path) const**

  * 任意のファイルパス文字列を再配置可能ファイルパスに変換します。
 
* **bool writeRelocatablePath(const std::string& key, const std::string& path)**

  * 任意のファイルパスを再配置可能ファイルパスに変換し、指定したキーで書き込みます。

再配置可能ファイルパスは、実際には以下の要素で構成されます。

1. プロジェクトディレクトリからの相対パス
2. パス変数
3. ユーザ変数

1については、ファイルがプロジェクトファイルの保存先ディレクトリ（これをプロジェクトディレクトリとします）の中か、その下位ディレクトリにある場合は、プロジェクトディレクトリからの相対パスで記述するというものです。例えばプロジェクトファイル "simulation.cnoid" とプロジェクトで使用しているその他のファイルが以下のディレクトリ構成で配置されているとします。

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       - robot.body
       + data
         - command.dat

ここで、

* プロジェクトディレクトリ

  * **/home/choreonoid/project**

* 各ファイルのプロジェクトファイルからの相対パス

  * **robot.body**

  * **data/command.dat**

となります。

この場合、プロジェクトディレクトリを別の場所に移したとしても、その中身が変わっていなければ、プロジェクト "simulation.cnoid" を読み込む際に、 "robot.body" と "command.dat" の場所も確定することができます。

これとは少し異なる状況として、Choreonoid本体に付属のモデルファイルを使用しているとしましょう。そちらはChoreonoidインストール先のshareディレクトリ以下に入っていますので、例えば以下のような構成になります。

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       - robot.body
       + data
         - command.dat
 + usr
   + local
     + share
       + choreonoid-1.8
         + model
           + misc
             - floor.body

ここではChoreonoidのインストール先を "/usr/local" と想定しており、その中に含まれる "floor.body" というモデルを使用するものとしています。

この状況で上記2の「パス変数」を使用できます。これを使用すると、floor.body へのパス

* **/usr/local/share/choreonoid-1.8/model/misc/floor.body**

を、以下のように記述できます。

* **${SHARE}/model/misc/floor.body**

${SHARE}の部分が「パス変数」で、これはshareディレクトリに対応しています。これは環境が変わってChoreonoidの実際のインストール先やバージョンが変わっても、常にその環境でのshareディレクトリを指すことになります。従ってファイルパスがこの形式で記録されていれば、プロジェクトをどの環境に移しても、読み込むことが可能となります。

このようなパス変数として以下が用意されています。

* **PROGRAM_TOP**

  * Choreonoidインストール先のトップディレクトリ

* **SHARE**

  * Choreonoidインストール先のshareディレクトリ

* **HOME**

  * 利用中のユーザのホームディレクトリ

これらのパス変数は、再配置可能ファイルパスに変換する際に、可能であれば適用されます。その場合パスが最短となる（対象ファイルからみて直近の）パス変数が自動的に割り当てられます。ただしプロジェクトディレクトリが直近となる場合は「プロジェクトディレクトリからの相対パス」が優先して適用されます。

さらに上記3の「ユーザ変数」として、パス変数に相当するものをユーザが独自に定義しておくことが可能です。例えば以下のようなファイル構成を想定します。

.. code-block:: text

 + home
   + choreonoid
     + project
       - simulation.cnoid
       + data
         - command.dat
     + model
       + robot
         - robot.body
 + usr
   + local
     + share
       + choreonoid-1.8
         + model
           + misc
             - floor.body

先程までは "robot.body" がプロジェクトディレクトリに格納されていましたが、これがロボットのモデルだとすると、他のプロジェクトからも使用したいことがあります。その場合は、各プロジェクトのディレクトリにコピーを作成するよりも、プロジェクトとは独立したディレクトリに配置して複数のプロジェクトで共有する方が効率的に運用できます。そこで上記の構成では "robot.body" をモデル格納用の独立したディレクトリに格納して利用しています。

この場合パス変数HOMEにより

* **${HOME}/model/robot/robot.body**

と記述することも考えられますが、モデルファイルの配置は環境やユーザーごとに自由に決めたいものとします。その場合は、ユーザ変数として例えば

* 変数：**MODEL**

* パス：**/home/choreonoid/model**

を定義しておけば、パスは

* **${MODEL}/robot/robot.body**

と記述できます。そしてユーザごとにパス変数MODELを設定しておけば、その場所が変わったとしても、プロジェクトを読み込むことができます。

なおユーザ変数の設定はChoreonoidのGUI上から行うことが可能です。これについては :doc:`../basics/config` - :ref:`basics_project_pathset` を参照してください。

再配置可能ファイルパスから実際のファイルパスを得るには以下の関数を使用します。これらは主にrestore関数から利用します。

* **std::string resolveRelocatablePath(const std::string& relocatable, bool doAbsolutize = true) const**

  * 引数に与えた再配置可能ファイルパスを実際のファイルパスに変換します。

  * 変換に失敗した場合は空文字列が返ります。

  * doAbsolutizeがtrueの場合、必ず絶対パス（フルパス）になるようにします。falseの場合、必ずしも絶対パスとはなりません。（パス変数が使用されていない相対パスを与えた場合は結果も相対パスとなります。）

* **bool readRelocatablePath(const std::string& key, std::string& out_value) const**

  * keyで指定したキーの値を再配置可能ファイルパスとして取り出して、それをresolveRelocatablePathで実際のパスに変換し、結果をout_valueにセットします。セットされるパスは絶対パスとなります。

.. _plugin-dev-file-io-functions:
    
ファイル入出力に関わる関数
~~~~~~~~~~~~~~~~~~~~~~~~~~

Archiveクラスはファイルの読み込みを支援する以下の関数も備えています。

* **bool writeFileInformation(Item* item)**

  * アイテムに記録されているファイルパスとファイル形式の情報を所定の形式で書き込みます。

* **bool loadFileTo(Item* item) const**

  * 所定の形式で書き込まれたファイルパスとファイル形式の情報を読み込んで、対応するファイルを実際にアイテムに読み込みます。

これらの関数の使用方法については「プロジェクトアイテムのファイル保存」の節で解説します。

.. _plugin-dev-project-file-structure:

プロジェクトファイルの構造
--------------------------

アイテムクラスのstore関数でarchiveに出力されたデータは、最終的にYAMLのプロジェクトファイルとして保存されます。
これは通常拡張子cnoidで保存されます。

このプロジェクトファイルには、以下の情報が記録されます。

* アイテムの状態

* ビューの状態

* ツールバーの状態

* その他オブジェクトの状態

* ビューのレイアウト

* ツールバーのレイアウト

※ ビューとツールバーのレイアウトについては、 :ref:`basics_layout_save` で紹介したように、メインメニューの「ファイル」-「プロジェクトファイルオプション」-「レイアウト」のチェックが入っている場合のみ有効となります。

アイテムクラスのstore、restoreの対象となるのは、上記のうちの「アイテムの状態」になります。それ以外の情報については、それぞれ該当するオブジェクトの同様の関数で保存・復帰の処理がされます。それらについては別途解説します。

プロジェクトファイルの記述に使用されているYAMLは可読性の高いフォーマットなので、プロジェクトファイルをテキストエディタ等で開くと、概ねその内容を把握できるかと思います。プロジェクトファイルはあくまでChoreonoidのプロジェクト保存によって生成されるものであり、直接生成・編集することを前提としたものではありませんが、必要に応じてYAMLの利点を活かした柔軟な運用をしていただければと思います。

