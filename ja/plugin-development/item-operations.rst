==========================
プロジェクトアイテムの操作
==========================

.. contents:: 目次
   :local:

概要
----

Choreonoidのフレームワークの主要な構成要素のひとつが「プロジェクトアイテム」です。これについてはChoreonoidをユーザーとして利用した経験があれば概ねどのようなものか把握されているかと思いますが、 :doc:`../basics/item` でもひととおり説明していますので、そちらも確認いただければと思います。

Choreonoid上で何か意味のある機能を実現しようとすると、多くの場合このプロジェクトアイテムと関わることになります。独自プラグインの開発においても、このプロジェクトアイテムを使いこなすことが、目標とする機能を実現するにあたって重要となります。

そこで本節ではプロジェクトアイテムに対応するChoreonoid SDKのクラスである「Itemクラス」について紹介し、プラグインからこのクラスを介してプロジェクトアイテムを操作する方法の概要を解説します。

なおプロジェクトアイテムについて以下では略称の「アイテム」という用語を用いることにします。

Itemクラス
----------

Choreonoid SDKでは各アイテムの基底となるクラスとしてItemクラスが定義されています。このクラスにはアイテムを参照したり操作するにあたって基本となる各種情報や処理が定義・実装されています。

このクラスはChoreonoid SDKのBaseモジュールで定義されています。クラス定義の詳細はAPIリファレンスの `Itemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ を参照ください。Choreonoid本体のソースリポジトリにおいては src/Base 以下の `Item.h <https://choreonoid.org/ja/documents/reference/latest/Item_8h_source.html>`_ と Item.cpp というソースファイルが対応しています。

このクラスに実装されている主要な機能を以下に示します。

* アイテム名の設定・取得・変更通知
* 親子／兄弟関係の設定・取得・変更通知
* アイテムの探索
* アイテムの選択／チェック状態の設定・取得・変更通知

また個別のアイテム型で実装される以下の機能を統一的に扱うためのAPIを提供します。

* アイテムのファイル保存・読み込み
* アイテムのプロジェクト保存・読み込み
* アイテムのプロパティ表示・編集

本節ではこの中から「アイテム名」「親子／兄弟関係」「探索」「選択／チェック状態」に関する操作を紹介します。
これによりプログラムからアイテムを操作する方法の概要を把握していただければと思います。
他の項目については、本開発ガイドの中で追って解説していきます。

Item継承クラス
--------------

Itemクラスはアイテムの基底となるクラスですが、それ自体はアプリケーションの目的を果たすための特定のデータや機能を有するものではありません。特定のデータや機能は、このItemクラスを継承した個別のアイテム型で定義・実装されます。

.. highlight:: text

そのような個別のアイテム型として、Choreonoid本体では様々なアイテム型が定義されています。このイメージをつかんでいただくため、いくつかの既存アイテム型を以下に示します。 ::

 + Item
   + WorldItem: 仮想世界
   + BodyItem: ロボットや環境の物理／描画モデル
   + ControllerItem: ロボット制御アイテムの基底
     + SimpleControllerItem: シンプルコントローラの実装
   + SimulatorItem: シミュレータアイテムの基底
     + AISTSimulatorItem: AISTシミュレータの実装
     + KinematicSimulatorItem: 運動学シミュレータの実装
   + SceneItem: 3D描画用モデル
   + ScriptItem: スクリプトアイテムの基底
     + PythonScriptItem: Pythonスクリプト機能を実装

ここでは継承関係をツリーの形式で表しています。これを見て分かるように、Itemクラスを直接継承したものもあれば、2段階の継承を行っているアイテム型もあります。2段階の場合は、1段階目で特定の機能の共通APIを定義し、そのAPIに基づく個別の実装を2段階目で行うということが多いです。もちろん継承の深さに制限は無く、3段階以上の例もあります。

現実の機能実装では、これらの個別アイテムの機能を活用することも欠かせません。また、独自のアイテム型を定義して、それを利用することもできます。これを踏まえると、アイテムの操作・活用については、大きく以下の3段階があると言えます。

1. Itemクラスの機能によりアイテムの基本的な操作を行う
2. 個別アイテム型の機能により、特定のデータや処理に関する操作を行う
3. 独自のアイテム型を定義・実装し、独自のデータや処理を使用可能にする

本節で紹介する内容は主に1に該当するものです。次節のサンプルでは個別アイテム型であるBodyアイテムも利用し、2の例として紹介します。そして今後の節で3についても解説します。

.. note:: 本節以降の説明やサンプルではロボットや環境のモデルに対応するBodyアイテムを使用します。また関連するBodyモデルやその他のオブジェクトについてもあわせて使用します。本ガイドの読者はChoreonoidを用いてロボットのシミュレーションを行った経験があることを想定しており、その場合BodyアイテムやBodyモデルについては概ね把握されているかと思います。もしそうでない場合は必要に応じて :doc:`../handling-models/index` を参照ください。また本ガイドを読み進める中でより理解が深まるかと思います。

アイテムの親子／兄弟関係
------------------------

Choreonoidでは一般的に複数のアイテムを組み合わせて作業を行います。その際複数のアイテムはツリー構造にまとめられるようになっており、これを :ref:`basics_item_tree` と呼んでいます。

ツリー構造になるということは、アイテム間で親子関係や兄弟関係が構築されるということでもあります。この情報はItemクラスが有しており、ItemクラスのAPIで関係の設定や参照を行えるようになっています。

.. highlight:: cpp

例えばitemAとitemBという2つのアイテムのインスタンスがあるときに、 ::

 itemA->addChildItem(itemB);

とすると、itemBはitemAの子アイテムとして設定されます。この関係は以下のように表されます。

.. code-block:: text

 + itemA
   + itemB

.. note:: アイテムは全て :doc:`Referenced型 <referenced>` のオブジェクトであり、ヒープメモリ上に動的に生成され、スマートポインタref_ptrで保持されます。従ってアイテムに関するコーディングは上記のようにポインタの形態となります。なおアイテムは基本的に親アイテムからref_ptrによる参照で保持されており、ref_ptrは生ポインタと相互に変換が可能なため、アイテムの一時的な参照においては生ポインタを使用しても特に問題はありません。

この場合以下の条件が成り立ちます。 ::

 itemA->childItem() == itemB
 itemB->parentItem() == itemA

このように各アイテムの子アイテムや親アイテムを取得することができます。

この時点で、itemAの親は無く、itemBの子は無い状態なので、 ::

 itemA->parentItem() == nullptr
 itemB->childItem() == nullptr

となります。

次にitemCというインスタンスもあるとして、これもitemAの子アイテムとして追加するとします。 ::

 itemA->addChildItem(itemC);

するとアイテムツリーは以下のようになります。

.. code-block:: text

 + itemA
   + itemB
   + itemC

ここでitemAの子アイテムが2つになり、2つの子アイテム間で兄弟関係ができました。すると以下の条件が成立します。 ::

 itemB->nextItem() == itemC
 itemC->prevItem() == itemB

このように兄弟関係はnextItem、prevItemを参照することで分かります。またこの場合 ::

 itemB->prevItem() == nullptr
 itemC->nextItem() == nullptr

となります。

親アイテムであるitemAからは以下の情報も取得できます。 ::

 itemA->lastChildItem() == itemC
 itemA->numChildren() == 2

子アイテムを追加する位置を指定することもできます。その場合はinsertChild関数を使います。例えば ::

 itemA->insertChild(itemC, itemD);

とするとアイテムツリーは以下になります。

.. code-block:: text

 + itemA
   + itemB
   + itemD
   + itemC

このようにitemDはitemCの手間の位置に追加されました。このようにinsertChildは ::

 Item::insertChild(挿入位置となるアイテム、挿入するアイテム）

という引数になります。

.. note:: addChildItemと同様の命名であればinsertChildItemとなるべきですが、ここではinsertChildという関数を使用しています。実はinsertChildItemという関数もあるのですが、引数の順序が逆で、標準ライブラリなどでみられる順序とも逆になっています。一般的には挿入位置を最初の引数にとることが多いので、そのための修正版としてinsertChildが定義され、insertChildItemは現在deprecatedとなっています。

あるアイテムの全ての子アイテムに対して所定の処理を適用したいことがよくあります。これはchildItem関数とnextItem関数を使用して以下のようにコーディングします。 ::

 for(auto child = itemA->childItem(); child; child = child->nextItem()){
     doSomething(child);
 }

親子関係を解消したいときは子アイテムに対してremoveFromParentItem関数を使用します。例えば ::

 itemB->removeFromParentItem();

を実行すると、アイテムツリーは以下になります。

.. code-block:: text

 + itemA
   + itemD
   + itemC

この場合itemBに関わる兄弟関係も解消されます。また親アイテムitemAに対して ::

 itemA->clearChildren();

とすると、itemAの子アイテムが全て解消されます。

ルートアイテム
--------------

Choreonoid上では常にただひとつの「ルートアイテム」が存在します。各アイテムは、ルートアイテムとの繋がりを持つことによって、GUI上で表示され、操作できるようになります。逆に言えばルートアイテムとの繋がりの無いアイテムは基本的に操作対象とはなりません。

ルートアイテムは専用のアイテム型である `RootItemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1RootItem.html>`_ のシングルトンインスタンスとなります。これは ::

 #include<cnoid/RootItem>

 ...

 auto rootItem = RootItem::instance();

などとして取得することができます。

上で例として挙げたアイテムツリーも、ルートアイテムに追加されていなければGUI上で扱うことはできません。例えば先の

.. code-block:: text

 + itemA
   + itemB
   + itemC

というツリーについて、 ::

 RootItem::instance()->addChildItem(itemA);

とすることでルートアイテムと接続できます。この場合ツリーは ::

 + RootItem::instance()
   + itemA
     + itemB
     + itemC

という状態になり、itemA以降のツリーが :ref:`basics_mainwindow_itemtreeview` 上に表示されます。逆に言えば、既にGUI上に読み込まれているアイテムは、このようにルートアイテムと接続された状態になっています。

この状態だと、itemAのサブツリーに含まれる全てのアイテムについて、メンバ関数 isConnectedToRoot() がtrueを返します。ルートアイテムに接続されていないときはこれがfalseを返します。

このようにアイテムがルートアイテムに接続されているかどうかはChoreonoidでは重要な要素となりますので、プラグラミングの際に留意してください。

.. _plugin-dev-item-basic-attributes:

アイテムの基本属性
------------------

Itemクラスはアイテムの基本属性に関する情報を持っています。基本属性の項目はItemクラスの列挙型Attributeにて、以下が定義されています。

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - 属性
   - 内容
 * - SubItem
   - 複合アイテムの一部となるアイテム
 * - Attached
   - 親アイテムからの切り離しを禁止
 * - Temporal
   - 一時的に生成されるアイテム
 * - LoadOnly
   - ロードのみ可能なアイテム

これらの属性は、Itemクラスの以下のメンバ関数で設定・参照ができます。

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - 関数
   - 処理
 * - void setAttribute(Attribute attribute)
   - 指定した属性を設定する 
 * - void unsetAttribute(Attribute attribute)
   - 指定した属性を解除する
 * - bool hasAttribute(Attribute attribute)
   - 指定した属性が設定されているか確認する

SubItem属性は、アイテムが :ref:`basics_composite_item` の構成要素であるかどうかを表す属性です。この属性が付いているアイテムは「サブアイテム」と呼びます。サブアイテムはそれ単体で親子／兄弟関係を変えたり、保存や読み込みを行ったり、削除するといったことができません。常に複合アイテムの本体アイテムと一体化して処理されるようになります。

このSubItem属性については、専用のメンバ関数 isSubItem でも判定できます。またアイテムを親アイテムに追加する際に、addChildItem関数の代わりにaddSubItem関数を用いると、アイテムはサブアイテムとして追加されます。

Attached属性は、アイテムの親アイテムからの切り離しを禁止するための属性です。これはSubItem属性と似た属性ではありますが、これが設定されていてもアイテムのデータや処理は親アイテムからは独立しており、例えば独立して保存や読み込みを行うこともできます。その上で、GUI上で親から切り離すという操作だけが禁止されます。これは元々は複合アイテムのように一体化しているわけではないものの、アイテムを親アイテムと必ずまとめて使用したいときに、設定されることになります。

Temporal属性は、アイテムが一時的に生成されたものであることを示すものです。この属性が付与されていると、プロジェクト全体を保存する場合に、該当するアイテムがあたかも存在しないかのように扱われます。つまり、プロジェクトファイルには保存されないので、保存したプロジェクトを再度読み込む際にこのアイテムは復元されませんし、ファイルに保存されることもありません。

これは例えばシミュレーション結果のログデータを格納するアイテムに適用されます。ログデータはその場でシミュレーション結果を再生し直すのに使用されますが、必ずしもプロジェクトの一部として保存する必要はありません。シミュレーションを同じ条件で実行しなおせば同じログが得られるからです。また、ログデータはサイズが巨大になることも多いため、保存しようとするとかえって運用が面倒になってしまいます。このためログデータはTemporal属性を付与して一時的なデータとして扱うようになっています。

LoadOnly属性についてはアイテムがファイルからの読み込みのみをサポートしていて、保存することがない場合に設定されます。この属性はアイテムの実装時に考慮すべきもので、アイテムの操作においては特に考慮する必要はありません。

.. _plugin-dev-item-operations-item-list:

ItemListクラス
--------------

複数のItemを格納するコンテナとして、 `ItemList <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemList.html>`_ というテンプレートクラスが定義されています。これはアイテムのポインタ（正確にはスマートポインタ）を格納する一種の配列ですが、テンプレート引数に与えた型のアイテムのみを選別的に格納する配列となっています。

これは複数アイテムを返す関数に対して、指定した型のアイテムのみを取り出す目的で使用します。

使用にあたってまずこのクラスのヘッダをインクルードしておきます。 ::

 #include <cnoid/ItemList>

ここでは想定上の関数として ::

 ItemList<Item> getItemList();

という関数があるとしましょう。この関数の戻り値はItem型を対象としたItemListになっています。つまりこのリストには全てのアイテム型が格納可能となっています。ちなみにItemListのテンプレート引数はデフォルト値がItemクラスになっているので、この例のようにItemクラスを対象としたリストは ItemList<> と記述することができます。以下ではその書き方を用います。

ここで ::

 ItemList<> items = getItemList();

と記述すると、itemsにはgetItemListが返すアイテム集合と同じ結果が入ります。しかしここで ::

 ItemList<BodyItem> bodyItems = getItemlist();

とすると、getItemListが返すアイテムのうち、BodyItem型に該当するものだけがbodyItemsに格納されることになります。

このようにして、特定の型を指定したItemListを用いることで、その型にマッチするアイテムのみを選別して取得することができます。ItemList同士では、対象とする型が異なっていても、コピーコンストラクタや代入オペレータを相互に使用することができます。そしてその場合実際にコピーされるのは新たに生成されるItemListや代入先のItemListが対象とするアイテム型のみになるというわけです。

Choreonoid SDKではItemListを返す関数がいくつもあり、実際の利用において便利に使用することができます。以下ではその具体例を紹介します。

.. note:: ItemListの要素は指定した型のアイテムを保持するref_ptrとなります。従ってアイテムがItemListに含まれる間は必ず生存が維持されます。

.. _plugin-dev-item-detection:

アイテムの検索
--------------

アイテムツリー上のアイテムについて、与えた条件にマッチするものを取得することができます。

よく利用する例としては、あるアイテム型に適合するアイテムをツリーの中から抽出するという操作です。例えば ::

 ItemList<BodyItem> bodyItems = item->descendantItems<BodyItem>();

とすると、itemを起点とするアイテムツリーの中からBodyItem型のアイテムを抽出して、ItemListとして返します。descendantItemsは実行対象のアイテムを起点とするサブツリーの要素を取得する関数です。このテンプレート引数に特定のアイテム型を指定することで、そのアイテム型のアイテムだけが抽出されます。この例は、サブツリー内の全てのBodyItemに対してある一定の操作を行いたい、という場合に利用できます。起点をルートアイテムにすればプロジェクトに存在する全てのBodyItemが抽出されます。

アイテムを名前で検索することもできます。検索の起点となるアイテムに対して ::

 Item* robotItem = item->findChildItem("Robot");

とすると、itemの子アイテムの中から"Robot"という名前をもつ最初のアイテムを検索し、存在すればそのアイテムを返します。アイテムが存在しない場合はnullptrが返ります。

この関数では階層化した名前も指定できます。例えば ::

 Item* controllerItem = item->findChildItem("World/Robot/Controller");

とすると、itemを起点として以下の構造でControllerまでたどり着ければ、それを返します。ここで指定している階層化された名前は「検索パス」と言い換えることができます。

.. code-block:: text

 + item
   + "World"
     + "Robot"
       + "Controller"

この関数もテンプレート版があり、例えば ::

 BodyItem* robotItem = item->findChildItem<BodyItem>("Robot");

とすると、"Robot"という名前を持つBodyItem型の子アイテムがある場合に、それを返します。この場合同じ名前のアイテムがあっても型が異なればnullptrが返ります。

findChildItemと同様ですが少し挙動の異なるfindItemという関数もあります。そちらは検索パスについて必ずしも対象のアイテムを起点としていなくてもマッチします。例えば先程の検索パスについて、 ::

 Item* controllerItem = item->findChildItem("Robot/Controller");

とした場合はitem直下の子アイテムにRobotというアイテムはありませんのでnullptrが返りますが、これを ::

 Item* controllerItem = item->findItem("Robot/Controller");

とすると、Controllerが返ります。

このfindItem関数も、findChildItemと同様にテンプレート版があります。使用方法も同じです。

上記の関数では、それぞれの基本となる検索方法をベースに、任意の検索条件を指定することもできます。その場合検索条件はアイテムを引数にとりbool値を返す関数オブジェクトとして与えます。

例えば ::

 ItemList<> = item->descendantItems([](Item* item){ return item->isSubItem(); });

とすると、item以下でSubItem属性のあるアイテムを返します。findChildItemやfindItemにおいても検索パスを任意の検索条件で置き換えたり、検索条件を追加の引数で指定することができます。詳細はやはり `Itemクラスのリファレンス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ を参照ください。

これまであるアイテムを起点としたサブツリー内を検索する機能を紹介しましたが、逆にあるアイテムの親アイテムを辿って検索する機能として、findOwnerItemという関数があります。

例えばあるBodyItemがどのWorldItemに属しているか知りたい場合は ::

 WorldItem* worldItem = bodyItem->findOwnerItem<WorldItem>();

とします。するとbodyItemから親アイテムを探索していき、WorldItem型のアイテムが見つかった時点でそれを返します。見つからなければやはりnullptrを返します。

この機能は実際にChoreonoidの各種機能の実装において頻繁に使用されています。
Chorenoidは基本的にアイテム間の関係性をその親子関係で判断する設計となっていますので、あるアイテムの処理をその親（や祖先）のアイテムと連携して行うことも多く、その場合に上記の方法で連携するアイテムを検索することができます。

アイテムの選択／チェック状態
----------------------------

アイテムには「選択状態」と「チェック状態」があります。Choreonoidの使用経験があれば既にこれらの状態についてはどのようなものかお分かりかと思います。もしそうでない場合は :ref:`basics_itemtree_management` - :ref:`basics_selection_and_check` を参照してください。

これらの状態はもちろんプログラムから設定・取得することが可能です。まずItemクラスのメンバ関数として以下を利用できます。

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - 関数
   - 処理
 * - bool isSelected() const
   - 現在の選択状態を返す
 * - void setSelected(bool on, bool isCurrent = false)
   - onで指定した選択状態に切り替える
 * - void setSubTreeItemsSelected(bool on)
   - 自身を含むサブツリーに含まれるアイテムの選択状態を一括して切り替える
 * - bool isChecked(int checkId = PrimaryCheck) const
   - 現在の選択状態を返す
 * - void setChecked(bool on)
   - onで指定したチェック状態に切り替える

これらの関数により、アイテムの選択／チェック状態を設定したり取得できます。プログラムから上記の設定関数を呼び出すと、GUI上の状態も直ちに切り替わります。

.. note:: 上記関数の引数isCurrentやcheckIdはやや高度な利用方法で使用する引数で、特に理由がなければデフォルト引数で使用していただければOKです。

アイテムツリー全体の中での選択状態を取得したい場合はRootItemの関数を使用します。

まずアイテムツリー上の全ての選択アイテムを取得する場合は ::

 ItemList<> selectedItems = RootItem::instance()->selectedItems();

とします。この関数はアイテム型を指定するテンプレート版もあります。それを用いて、例えば ::

 ItemList<BodyItem> selectedBodyItems = RootItem::instance()->selectedItems<BodyItem>();

とすることで、選択中のBodyアイテムを取得することができます。

チェック状態についても同様です。 ::

 ItemList<> checkedItems = RootItem::instance()->checkedItems();

としたり、アイテム型を指定して ::

 ItemList<BodyItem> checkedBodyItems = RootItem::instance()->checkedItems<BodyItem>();

とすることができます。

アイテムの選択状態やチェック状態の参照は、各機能の操作対象とするアイテムを決定するのによく利用されています。独自プラグインの開発においてもこれらの状態を適切に活用してください。

.. _plugin-dev-item-operations-signals:

アイテム関連シグナル
--------------------

アイテムツリーの変化やアイテムの状態変化について、対応するシグナルが定義されています。それらのシグナルを用いることで、アイテムに対して適切なタイミングで適切な処理を行ったり、各機能の対象となるアイテムを適切に切り替えることが可能となります。

まず個々のアイテムの変化を知らせるシグナルが、Itemクラスのメンバとして定義されています。以下に利用可能なものを挙げます。いずれもSignalProxyを返すメンバ関数として定義されています。

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - シグナル
   - 送出のタイミング
 * - SignalProxy<void(const std::string& oldName)> sigNameChanged()
   - 自身の名前が変更されたとき
 * - SignalProxy<void()> sigTreePathChanged()
   - アイテムツリーにおけるルートから自身へのパスが（追加、移動、削除等により）変化したとき
 * - SignalProxy<void()> sigTreePositionChanged()
   - アイテムツリーにおける自身の位置が（追加、移動、削除等により）変化したとき。位置の変化には、パスの変化に加えて、兄弟アイテム間の順序の変化も含まれる。
 * - SignalProxy<void()> sigSubTreeChanged()
   - 自身のサブツリーの構成が（アイテムの追加、移動、削除等により）変化したとき
 * - SignalProxy<void()> sigDisconnectedFromRoot()
   - 自身とルートアイテムとの接続が絶たれたとき
 * - SignalProxy<void(bool on)> sigSelectionChanged()
   - 自身の選択状態が変化したとき
 * - SignalProxy<void(bool on)> sigCheckToggled(int checkId = PrimaryCheck)
   - 自身のチェック状態が変化したとき
 * - SignalProxy<void()> sigUpdated()
   - 自身の内容が更新されたとき

またRootItemクラスに定義されている以下のシグナルも利用可能です。いずれもルートアイテムを起点とするアイテムツリー内での変化をしらせるもので、ルートアイテムに接続されていないアイテムについてはこれらのシグナルの対象となりません。

.. list-table::
 :widths: 50,50
 :header-rows: 1

 * - シグナル
   - 送出のタイミング
 * - SignalProxy<void(Item* item)> sigSubTreeAdded()
   - itemを起点とするサブツリーが追加されたとき
 * - SignalProxy<void(Item* item)> sigItemAdded()
   - アイテムが追加されたとき。sigSubTreeAddedと同じタイミングで、追加されたサブツリー内の各アイテムごとに本シグナルが送出される。
 * - SignalProxy<void(Item* item)> sigSubTreeMoved()
   - itemを起点とするサブツリーが移動したとき
 * - SignalProxy<void(Item* item)> sigItemMoved();
   - アイテムが移動したとき。sigSubTreeMovedと同じタイミングで、移動したサブツリー内の各アイテムごとに本シグナルが送出される。
 * - SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoving()
   - itemを起点とするサブツリーが削除される直前。このシグナルはサブツリー移動時にも送出される。その場合sigSubTreeMovedの前に送出され、isMovingがtrueとなる。
 * - SignalProxy<void(Item* item, bool isMoving)> sigSubTreeRemoved()
   - itemを起点とするサブツリーが削除されたとき。このシグナルは完全に削除されたときのみ送出され、移動時には送出されない。
 * - SignalProxy<void(Item* item, const std::string& oldName)> sigItemNameChanged()
   - ItemクラスのsigNameChangedと同様のシグナル。こちらは対象アイテムを限定せずにスロットを接続できる
 * - SignalProxy<void(const ItemList<>& selectedItems)> sigSelectedItemsChanged()
   - 選択状態のアイテムが変化したとき
 * - SignalProxy<void(Item* item, bool on)> sigCheckToggled(int checkId = PrimaryCheck)
   - チェック状態のアイテムが変化したとき

どれもChoreonoid本体の実装で活用されているシグナルです。

いくつかのシグナルについて、利用の指針を紹介します。

まずItemクラスのsigTreePathChangedは、対象アイテムの上位（親側）のアイテムとの関係が処理内容に影響する場合に、処理の準備や後始末を行うために利用します。これはルートアイテムに接続するタイミングでも送出されるので、その際にアイテムの初期化を行うこともよくあります。

一方で対象アイテムの下位（子側）のアイテムとの関係が処理内容に影響する場合は、sigSubTreeChangedによってそれを関係の変化を検知し、処理の準備や後始末を行うために利用します。

ItemクラスのsigDisconnectedFromRootは、それ以降そのアイテムはChoreonoid上での操作対象でなくなることを意味するので、アイテムの後始末などに利用します。例えばアイテムがChoreooidのオブジェクトやOSのリソースなどを利用している場合に、それらの解放処理を行います。

アイテムの選択状態やチェック状態についても、シグナルによって状態検知を行うことが多いです。ItemクラスのsigSelectionChangedやsigCheckToggledは特定のインスタンスの状態変化を検知するのに使えますし、アイテムツリー全体の中で変化したアイテムを検知したいのであれば、RootItemクラスのsigSelectedItemsChangedやsigCheckToggledを使用します。

RootItemのsigSelectedItemsChangedでは選択されているアイテムの一覧がItemListとして与えられます。これについては、このリストを特定のアイテム型用のItemListに渡すことで、そのアイテム型を対象とした選択状態の変化を検知することが可能となります。例えば選択状態の変化に対する処理を行う関数を ::

 void onSelectedBodyItemsChanged(ItemList<BodyItem> selectedBodyItems)
 {
     ...
 }

と定義しておき、 ::

 RootItem::instance()->sigSelectedItemsChanged().connect(
     [](const ItemList<>& selectedItems){
         onSelectedBodyItemsChanged(selectedItems);
     });

とすれば、onSelectedBodyIemsChangedには選択されているBodyItemのみを格納したItemListが渡されます。

.. TargetItemPickerの説明もここでする？
