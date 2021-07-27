==============================================
プロジェクトアイテム生成／入出力のカスタマイズ
==============================================

.. contents:: 目次
   :local:

.. highlight:: cpp

概要
----

プロジェクトアイテムの生成やファイル入出力に関わるインタフェースは、アイテム型ごとにカスタマイズすることが可能です。
これにより、あるアイテム型に特有の初期化処理やオプションを加えることができ、利用時の利便性を高めることができます。
本節ではこのカスタマイズの方法について解説します。

.. _plugin-dev-item-creation-panel-implementation:

ItemCreationPanelによる生成ダイアログのカスタマイズ
---------------------------------------------------

:doc:`new-item-type` では、 :ref:`plugin-dev-item-creation-panel-registration` を行うことで、「ファイル」−「新規作成」のメニューからアイテムを生成できるようになることを示しました。ただしそこで紹介した方法はデフォルトの生成パネルを使うもので、その場合生成時に設定できる項目はアイテムの名前だけであり、それ以外の要素は全てデフォルトの状態で生成されていました。しかしながら、アイテム型によってはその生成時に名前以外の項目についても予め設定しておきたい場合がありますし、何らかの初期化処理を行いたい場合もあります。これは独自の生成パネルを登録することで実現できます。

生成パネルは `ItemCreationPanelクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemCreationPanel.html>`_ を継承して実装します。このクラスはItemManagerヘッダで以下のように定義されています。 ::

 class ItemCreationPanel : public QWidget
 {
 public:
     ItemCreationPanel();
     virtual bool initializeCreation(Item* protoItem, Item* parentItem) = 0;
     virtual bool updateItem(Item* protoItem, Item* parentItem) = 0;
 };

この定義で示されるように、ItemCreationPanelはQtのQWidgetクラスを継承しており、これが生成パネルのGUI要素をのせる土台として機能します。
またアイテムの生成を処理するためのvirtual関数が定義されており、派生クラスでこれらの関数を実装することで、生成パネルに任意の設定項目や初期化処理を加えることができます。

各virtual関数の第一引数として定義されているprotoItemは、アイテムの「プロトタイプ」に対応します。
これは対象とするアイテム型のインスタンスで、通常はアイテムのデフォルトコンストラクタを用いて生成されます。
プロトタイプは各生成パネルごとに保持されており、アイテムの生成はこのプロトタイプを複製することで実現しています。
プロトタイプの設定を変えることで、生成されるアイテムにもそれが反映されることになります。
繰り返し生成が行われる場合でも、同じプロトタイプのインスタンスが保持され、プロトタイプの最後の設定が次の生成のデフォルトして引き継がれることになります。

このプロトタイプアイテムを軸として、生成パネルは以下の流れで処理を行います。

1. コンストラクタでパネルのGUIを構築する
2. パネルが表示されるタイミングでinitializeCreation関数が呼ばれるので、そこでプロトタイプアイテムや設定項目のGUIを初期化する
3. アイテム生成の直前でupdateItem関数が呼ばれるので、設定項目のGUIへの入力を参照してプロトタイプアイテムの設定を更新する
4. プロトタイプアイテムを複製することでアイテムを生成する

ItemCreationPanelを継承したクラスで上記1〜3の処理を実装します。2と3の各関数については成功した場合はtrueを、失敗した場合はfalseを返すようにします。

なお、対象とするアイテム型に対してパネルを実装しやすくするため、以下のテンプレートクラス `ItemCreationPanelBase <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemCreationPanelBase.html>`_ が定義されています。 ::

 template<class ItemType>
 class ItemCreationPanelBase : public ItemCreationPanel
 {
 protected:
     ItemCreationPanelBase() { }
     virtual bool initializeCreation(ItemType* protoItem, Item* parentItem) = 0;
     virtual bool updateItem(ItemType* protoItem, Item* parentItem) = 0;
 };

このテンプレートを用いると、オーバーライドするvirtual関数について、プロトタイプアイテムの引数を対象アイテム型にすることができます。
例えばFooItemの生成パネルを実装する場合は以下のようにします。 ::

 class FootItemCreationPanel : public ItemCreationPanelBase<FooItem>
 {
 public:
     FootItemCreationPanel();
 protected:
     virtual bool initializeCreation(FooItem* protoItem, Item* parentItem) override;
     virtual bool updateItem(FooType* protoItem, Item* parentItem) override;
 };
     
この場合上記virtual関数のprotoItem引数の型がProtoItemへのポインタとなります。
     
生成パネルの登録については、:ref:`plugin-dev-item-type-registration` でも紹介したように、 `ItemManagerクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemCreationPanel.html>`_ の以下の関数を使用します。 ::

 template <class ItemType>
 ItemManager& addCreationPanel(ItemCreationPanel* panel = nullptr);

引数panelに生成パネルのインスタンスを指定します。デフォルト値のnullptrの場合はデフォルトの生成パネルが使用されますが、ここに上記の要領で実装した独自の生成パネルを指定すると、そちらが使用されるようになります。

.. _plugin-dev-itemfileio:

ItemFileIOによる入出力のカスタマイズ
------------------------------------

:doc:`item-file-io` で示したファイル入出力用のダイアログは、対象のファイルを指定するだけのものでした。
しかしながら、ファイル入出力の際に他の要素に関わる追加の指定を行いたい場合があります。
例えば、あるファイル形式について仕様が厳密に定まっていない部分があり、その部分をどう扱うかについて、ユーザからの指示を仰ぎたい場合があります。そのためには、ファイル入出力用のダイアログにオプション設定用のGUI要素を追加し、それに対する操作をファイル入出力処理に反映できるようにする必要があります。

そのようなファイル入出力のカスタマイズは、 :ref:`plugin-dev-item-io-function-registration` の代わりに `ItemFileIO <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemFileIO.html>`_ の登録を行うことで実現できます。ItemFileIOはアイテムのファイル入出力を実装するためのクラスで、これを使用することで :ref:`plugin-dev-item-io-function-registration` よりも詳細な入出力の実装が可能となります。実はファイル入出力について内部では全てこのItemFileIOとして処理されており、ファイル入出力関数も実際には登録時に内部でItemFileIOに変換されています。

ItemFileIOはBaseモジュールで定義されており、同名のヘッダが提供されています。入出力の実装はItemFileIOを継承したクラスで行います。
この実装の流れについて以下に示します。

1. ItemFileIOを継承した独自のItemFileIOクラスを対象アイテム型に対して定義する
2. コンストラクタでファイル入出力の基本属性を設定する
3. ファイルの入力（読み込み）に対応する場合は、読み込み用の関数をオーバーライドして実装する
4. ファイルの出力（保存）に対応する場合は、保存用の関数をオーバーライドして実装する
5. 入出力のオプションを提供する場合は、関連する関数をオーバーライドして実装する
6. ItemManagerのregisterFileIO関数で登録する   

以下では上記の各項目について説明します。

独自ItemFileIOクラスの定義
~~~~~~~~~~~~~~~~~~~~~~~~~~

上記項目の1は基本的にはItemFileIOを継承して以下のように定義します。 ::

 class FooItemFileIO : public ItemFileIO
 {
 public:
     FooItemFileIO();
     ...
 };

ここではFooItemを対象としたItemFileIOを想定しています。

`ItemFileIoBaseテンプレート <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemFileIoBase.html>`_ を用いると、あるアイテム型に特化したItemFileIOを実装しやすくなります。これは以下のようにして使用します。 ::

 class FooItemFileIO : public ItemFileIoBase<FooItem>
 {
 public:
     FooItemFileIO();
     ...
 };
  
この場合、対象アイテムを引数にとるvirtual関数について、引数の型が対象アイテム型へのポインタとなります。
通常はこちらの方法を用いるのがおすすめです。

少し特殊なケースになりますが、既存のItemFileIOを拡張したものを作成することも可能です。
この場合は `ItemFileIoExtenderテンプレート <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1ItemFileIoExtender.html>`_ を用いて以下のようにします。 ::

基本属性の設定
~~~~~~~~~~~~~~

上記の項目2に示したように、独自ItemFileIOクラスのコンストラクタで、ファイル入出力の基本属性を設定します。

まず基底クラス（ItemFileIOもしくはItemFileIoBaseテンプレート）のコンストラクタが以下のように定義されているので、この引数でファイル形式とサポートするAPIを指定します。 ::

  ItemFileIO::ItemFileIO(const std::string& format, int api);

  ItemFileIoBase::ItemFileIoBase(const std::string& format, int api)

formatにはファイル形式を表す文字列（識別子）を指定します。これは :ref:`plugin-dev-item-io-function-registration` で使用するものと同じです。また、apiにはItemFileIOクラスの列挙型 "API" で定義されている以下のシンボルの組み合わせによって、対応するAPIを指定します。

* **Load**

  * ファイルの読み込みをサポート

* **Save**

  * ファイルの保存をサポート

* **Options**

  * オプションをサポート

* **OptionPanelForLoading**

  * ファイル読み込みダイアログにおけるオプション設定パネルをサポート

* **OptionPanelForSaving**

  * ファイル保存ダイアログにおけるオプション設定パネルをサポート

例えばItemFileIoBaseテンプレートを継承したFooItemFileIOについて以下のように指定します。 ::

 FooItemFileIO::FooItemFileIO()
     : ItemFileIoBase<FooItem>("FOO-DATA-FILE", Load | Options | OptionPanelForLoading)
 {
     ...
 }

この場合ファイル形式は "FOO-DATA-FILE" となり、読み込みとオプション、および読み込みダイアログのオプション設定パネルがサポートされます。
保存をサポートする場合は同様に Save や OptionPanelForSaving を指定します。
最低限LoadかSaveのどちらかのAPIをサポートする必要がありますが、残りのAPIのサポートは任意となります。全てのAPIをサポートすることも可能です。
 
コンストラクタの実装においては、ItemFileIOクラスの以下の関数を使用して各種属性を設定できます。

* **void setCaption(const std::string& caption)**

  * 入出力のキャプションを設定します。入出力用ダイアログのタイトルなどで使用されます。

  * 基本的には「入出力する内容が何であるか」を表すものとし、ファイル形式には必ずしも依存しません。例えばボディモデルとして読み込み可能なファイル形式は複数ありますが、いずれの形式もこの関数で設定するキャプションは "Body" となります。

* **void setFileTypeCaption(const std::string& caption)**

  * ファイルタイプのキャプションを設定します。入出力用ダイアログでファイルタイプの選択肢として表示されます。

  * この関数による設定がない場合はsetCaptionで設定された内容がファイルタイプのキャプションとしても使用されます。

* **void setExtension(const std::string& extension)**

  * 対象ファイル形式のファイル拡張子を設定します。

* **void setExtensions(const std::vector<std::string>& extensions)**

  * 複数のファイル拡張子を設定します。

* **void setInterfaceLevel(InterfaceLevel level)**

  * 利用される際のインタフェースのレベルを設定します。

  * 値はItemFileIOクラスの列挙型 "InterfaceLevel" で定義されている以下のいずれかになります。

  * **Standard**
 
    * 標準レベルです。ファイルの読み込み／保存の項目に加わります。デフォルトではこの設定になります。
       
  * **Conversion**

    * 変換レベルです。ファイルのインポート／エクスポートの項目に加わります。
	 
  * **Internal**

    * 内部利用レベルです。ユーザがメニュー等から直接利用することはできず、プログラムコードからの利用に限定されます。

* **void addFormatAlias(const std::string& format)**

  * ファイル形式のエイリアスを追加します。

  * ファイル形式の識別子を変更した場合でも、以前の識別子をエイリアスとして登録しておくことで、以前の識別子で保存されたプロジェクトファイルを読み込むことが可能となります。主に後方互換性の確保のために利用します。


ファイル読み込み関数の実装
~~~~~~~~~~~~~~~~~~~~~~~~~~

上記の項目3に対応する作業として、ItemFileIOのAPIにLoadが含まれる場合は、読み込み用の関数を実装する必要があります。

ItemFileIOクラスを直接継承している場合は、以下の2つの関数を実装します。

* **virtual Item* createItem()**
* **virtual bool load(Item* item, const std::string& filename)**
 
createItem関数は以下のようにして対象アイテム型のインスタンスを生成するようにします。 ::

 Item* FooItem::createItem()
 {
     return new FooItem;
 }

ファイル読み込みの際にこの関数で生成されるインスタンスが利用されます。

対象アイテム型が :ref:`シングルトンアイテム <plugin-dev-singleton-item-registration>` の場合は、シングルトンインスタンスを返す必要があります。これはItemFileIOのfindSingletonItemInstance関数を使用して、以下のように実装できます。 ::

 Item* FooItem::createItem()
 {
     return findSingletonItemInstance();
 }

ItemFileIoBaseテンプレートを継承している場合は、createItem関数はテンプレートで実装されますので、継承先で実装する必要はありません。
またload関数については第一引数の型がテンプレートパラメータで指定したアイテム型へのポインタとなります。
例えばFooItemの場合は以下の定義となります。

* **virtual bool load(FooItem* item, const std::string& filename)**

いずれの場合もload関数に読み込みの処理を実装する必要があります。
これは :ref:`plugin-dev-item-io-function-registration` で解説したローダ関数と同様に実装します。
その際ItemFileIOの以下の関数を使用することができます。

* **Item* parentItem()**

  * 読み込み成功後に親となるアイテムを返します。

* **int currentInvocationType() const**

  * 読み込み関数の呼び出しを引き起こした操作のタイプを返します。

  * 値はItemFileIOクラスの列挙型 "InvocationType" で定義されている以下のいずれかになります。

  * **Direct** : プログラムコードからの直接的な呼び出し

  * **Dialog** : 読み込み用ダイアログからの呼び出し

  * **DragAndDrop** : ドラッグ＆ドロップ操作による呼び出し

  * デフォルトでDirectが設定されています。
    
* **std::ostream& os()**

  * 出力ストリームを返します。読み込み時のメッセージはここに出力します。

* **void putWarning(const std::string& message)**

  * 警告メッセージを出力します。

* **void putError(const std::string& message)**

  * エラーメッセージを出力します。

ファイル保存関数の実装
~~~~~~~~~~~~~~~~~~~~~~

上記の項目4に対応する作業として、ItemFileIOのAPIにSaveが含まれる場合は、保存用の関数を実装する必要があります。
ItemFileIOクラスを直接継承している場合は、以下の関数を実装します。

* **virtual bool save(Item* item, const std::string& filename)**

読み込みの場合と同様に、ItemFileIoBaseテンプレートを継承している場合は、上記関数の第一引数の型がテンプレートパラメータで指定したアイテム型へのポインタとなります。いずれの場合も、やはり :ref:`plugin-dev-item-io-function-registration` で解説したセーブ関数と同様に実装します。またItemFileIOのメッセージ出力用の関数は読み込みのときと同様に使用することができます。

入出力オプションの実装
~~~~~~~~~~~~~~~~~~~~~~

ItemFileIOで処理するファイル入出力に追加の設定項目（オプション）を設けたい場合は、APIに "Options" を含めるようにします。
その上で、上記の項目5に対応する作業として、入出力のオプションを処理する以下の関数を実装します。

* **virtual void resetOptions()**

  * オプションをリセットします。
    
* **virtual void storeOptions(Mapping* options)**

  * 現在設定されているオプションを引数のMappingに出力します。
    
* **virtual bool restoreOptions(const Mapping* options)**

  * 引数のMappingからオプションを入力します。

ファイル入出力用オプションの設定内容は、ItemFileIO内に任意の形式で保持すればOKです。ただしその設定内容をプロジェクトアイテムやプロジェクトファイルに記録するために、 :ref:`plugin-dev-yaml-structured-data-classes` によるデータと相互変換を実装する必要があります。このデータはMappingを起点とするもので、上記のstoreOptionsとrestoreOptionsでこのデータと内部状態の変換を処理します。restoreOptionsについてはbool型の戻り値で処理の成否を返すようにします。またresetOptionsでは内部状態をリセットするようにします。

ItemFileIOのload、save関数では、対応する読み込みや保存の処理にオプションの設定内容を反映させるようにします。

これでオプション対応の基盤が整います。あとはそもそもどのようにしてオプションを設定するかという問題になります。

これについて、`Itemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ のload、save関数についてはオプションを直接設定できます。これらの関数にはoptionsという引数があることを :doc:`item-file-io` の :ref:`plugin-dev-item-file-io-function-program-use` で示しました。実はこのoptions引数が上記のoptionsに対応するものです。この引数にオプションの設定内容を渡すと、それが上記のrestoreOptionsでItemFileIOに反映された上で、ItemFileIOのload、save関数が実行されます。これによりoptions引数に設定した内容でファイルの読み込みや保存が行われます。

最後に行われたファイル読み込み／保存で使用されたオプションは `Itemクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Item.html>`_ のインスタンスに記録されます。また記録されているオプションを参照することもできます。それらの操作はItemクラスの以下の関数で処理できます。

* **void updateFileInformation(const std::string& filename, const std::string& format, Mapping* options = nullptr)**

  * アイテムの読み込み／保存対象となるファイルの情報を更新します。options引数に与えたオプションがアイテムに記録されます。

* **const Mapping* Item::fileOptions() const**

  * 上記関数で記録されたオプションを返します。

オプションをファイルダイアログから設定したり、プロジェクトファイルに保存する場合は、そのための実装も必要となります。以下ではその方法を解説します。

入出力オプションパネルの実装
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ファイル入出力のオプションを入出力用のファイルダイアログから設定するためには、まずItemFileIOのAPIに "OptionPanelForLoading" や "OptionPanelForSaving" を追加する必要があります。それぞれ読み込み用ダイアログと保存用ダイアログに対応します。その上で、読み込み、保存のそれぞれについて、ItemFileIOの以下のvirtual関数を実装します。

* 読み込み

  * **virtual QWidget* getOptionPanelForLoading()**

    * 読み込み用のオプションパネルをQWidgetとして返します。
    
  * **virtual void fetchOptionPanelForLoading()**

    * 読み込み用オプションパネルの現在の内容をItemFileIOのオプション設定に反映します。

* 保存

  * **virtual QWidget* getOptionPanelForSaving(Item* item)**

    * 保存用のオプションパネルをQWidgetとして返します。
    
  * **virtual void fetchOptionPanelForSaving()**

    * 保存用オプションパネルの現在の内容をItemFileIOのオプション設定に反映します。

getOptionPanelForLoading / getOptionPanelForSaving では、ファイルダイアログに表示する設定用GUIまとめたQWidgetオブジェクトを作成し、そのポインタを返すようにします。するとそのウィジェットがファイルダイアログの所定の領域に挿入されて、ユーザがオプションを編集できるようになります。設定用GUIの作成に特に制限はありませんが、ファイルダイアログ上で表示するにあたって適切なサイズになるようにまとめてください。

fetchOptionPanelForLoading / fetchOptionPanelForSaving では、ItemFileIOが内部で管理するオプション設定を、設定用GUIの内容と一致するように更新します。例えばある設定を整数値のIDで管理していて、その設定用にコンボボックスを使用している場合は、コンボボックスで選択されているインデックスでIDの変数を更新します。この処理により、ユーザがダイアログ上で行ったオプション設定をファイル読み込みや保存の処理に反映できるようになります。

ダイアログを用いてファイル読み込みを行う場合のオプションの処理の流れを以下に示します。      

1. ファイルダイアログの "Files of type" コンボで選択されたItemFileIOに対して、getOptionPanelForLoading関数で読み込み用オプションパネルを取得し、ダイアログ上に表示する。

2. ユーザはオプションパネルを操作してオプションの設定を行う。

3. ユーザがファイルを選択して保存ボタンを押すと、ItemFileIOのfetchOptionPanelForLoading関数が実行され、オプションパネルの設定内容がItemFileIOのオプション設定に反映される。

4. 反映されたオプション設定でファイルの読み込みを行う。

5. 読み込みに使用したオプション設定がItemFileIOのstoreOptions関数によって取得され、それがアイテムのupdateFileInformation関数でアイテムに記録される。

ファイル保存の場合の処理の流れは以下になります。

1. ファイルダイアログの "Files of type" コンボで選択されたItemFileIOに対して、restoreOptions関数を用いてアイテムの最終オプションをセットする（最終オプションはアイテムのfileOptions関数で取得する。）また、ItemFileIOのgetOptionPanelForLoading関数で保存用のオプションパネルを取得し、ダイアログ上に表示する。

2. ユーザはオプションパネルを操作してオプションの設定を行う。

3. ユーザがファイルを選択して保存ボタンを押すと、ItemFileIOのfetchOptionPanelForSaving関数が実行され、オプションパネルの設定内容がItemFileIOのオプション設定に反映される。

4. 反映されたオプション設定でファイルの保存を行う。

5. 保存に使用したオプション設定がItemFileIOのstoreOptions関数によって取得され、それがアイテムのupdateFileInformation関数でアイテムに記録される。


プロジェクト保存／読み込みとの連携
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ファイル入出力のオプション設定が可能なアイテムについては、 :ref:`plugin-dev-item-file-info-project-save` において、オプションの情報も記録する必要があります。プロジェクト復帰においてアイテムがファイルを読み込む際に、ファイルを最後に読込／保存した時と同じオプションでファイルを読み込まないと、アイテムの内容を同じにできないからです。

まずプロジェクト保存について、store関数を以下のように実装することでファイル情報を記録できることを :ref:`plugin-dev-item-file-info-project-save` で解説しました。 ::

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         if(archive.writeRelocatablePath("file", filePath())){
             archive.write("format", fileFormat());
             stored = true;
         }
     }
     return stored;
 }

ここではファイルパスとファイル形式の情報を記録していたわけですが、これにオプションの情報も追加すればよいわけです。
そしてオプションの情報は上述のようにItemクラスのfileOptions関数で取得できます。
これを用いて上記関数を以下のように修正することで、目的を達成できます。

.. code-block:: cpp
 :emphasize-lines: 7,8,9

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         if(archive.writeRelocatablePath("file", filePath())){
             archive.write("format", fileFormat());
	     if(auto fileOptions = item->fileOptions()){
                 archive.insert(fileOptions);
	     }
             stored = true;
         }
     }
     return stored;
 }

ここで強調表示している部分が先のコードに対して追加されています。
最後に使用されたファイル入出力のオプションが有効である場合は、それをarchiveに書き込んでいます。
fileOptionsが返すデータは :ref:`plugin-dev-yaml-structured-data-classes` のデータなので、このようにarchiveにも直接出力できます。

そして実はここに示したオプション情報の記録は、 :ref:`plugin-dev-item-file-info-project-save` で示したwriteFileInformation関数内で処理されるようになっており、そこで示した以下のコードで既に達成されています。 ::

 bool FooItem::store(Archive& archive)
 {
     bool stored = false;
     if(overwrite()){
         stored = archive.writeFileInformation(this);
     }
     return stored;
 }

ファイル入力のみサポートするアイテム型の場合は以下の実装でよい点も同じです。 ::

 bool FooItem::store(Archive& archive)
 {
     return archive.writeFileInformation(this);
 }

プロジェクト復帰のためのrestore関数については、逆にプロジェクトファイルからオプション情報を取り出して、それをファイル読み込みに使用する必要があります。その処理を :ref:`plugin-dev-item-file-info-project-save` で紹介したrestore関数の実装に対して追加したコードは以下になります。
  
.. code-block:: cpp
 :emphasize-lines: 8

 bool FooItem::restore(const Archive& archive)
 {
     bool restored = false;
     string file;
     if(archive.readRelocatablePath("file", file)){
         string format;
         archive.read("format", format);
         restored = load(file, format, archive);
     }
     return restored;
 }
		   
強調表示している行の最後で、load関数の第三引数にarchiveを指定するように修正しています。
これは :ref:`plugin-dev-item-file-loading-function` で示したItemクラスのload関数のoptions引数になります。
ここにオプションデータを指定すると、読み込み時にそれが使用されることになります。
そしてオプションデータはarchiveに記録されていますので、このように直接archiveを指定すればOKです。

そして実はこの処理についても、 :ref:`plugin-dev-item-file-info-project-save` で示したloadFileTo関数に含まれています。
ですからこちらも ::

 bool FooItem::restore(const Archive& archive)
 {
     return archive.loadFileTo(this);
 }

と記述すればオプションも含めて全て処理されることになります。

ItemFileIOの登録
~~~~~~~~~~~~~~~~

:doc:`item-file-io` の :ref:`plugin-dev-item-io-function-registration` ではローダ関数やセーバ関数を登録する方法を紹介しました。
ItemFileIOについても同様にItemManagerを介してシステムに登録できます。
これにはItemManagerの以下のテンプレート関数を使用します。 ::

 template <class ItemType>
 ItemManager& addFileIO(ItemFileIO* fileIO);

テンプレート引数のItemTypeには対象となるアイテム型を指定します。
fileIOには作成した独自のItemFileIOクラスをnewして生成したインスタンスを指定します。
ItemFileIOは :doc:`referenced` であり、そのスマートポインタがItemManagerで保持されるので、登録側が特にポインタを管理する必要はありません。

この関数はやはりプラグインクラスのinitialize関数から使用します。
例えばFooItemFileIOを登録する場合は以下のようにします。 ::

  itemManager().addFileIO<FooItem>(new FooItemFileIO);

あるアイテム型に対して複数のItemFileIOを登録することが可能です。
あるプラグインで定義されているアイテム型に対して、別のプラグインからItemFileIOを追加登録し、アイテムがサポートするファイル形式を追加することも可能です。
