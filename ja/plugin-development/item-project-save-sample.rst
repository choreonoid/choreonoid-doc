============================================
プロジェクトアイテムの状態保存サンプル (S08)
============================================

.. contents:: 目次
   :local:

概要
----

本節では :doc:`item-project-save` に関するサンプルとして、 :doc:`item-property-sample` に対してアイテムの状態をプロジェクトファイルに保存できるようにしたものを提示します。この改良によって、BodyPositionアイテムに記録されている位置やプロパティをプロジェクトとして保存・復帰できるようにしています。

ソースコード
------------

.. highlight:: cpp

今回のソースコードは :doc:`item-property-sample` のコードに :ref:`plugin-dev-state-store-restore-functions` で紹介したstore関数とrestore関数を追加したものとなっています。ファイル構成は同じです。以下では関数を追加したBodyPositionItemクラスのヘッダファイルと実装ファイルを掲載します。ソースコードの大部分はS07と同じで、変更箇所にコメントを付けて強調表示しています。DevGuidePlugin.cppとCMakeLists.txtはS07と同じ内容になります。

BodyPositionItem.h
~~~~~~~~~~~~~~~~~~

.. code-block:: cpp
 :emphasize-lines: 33,34,35

 #ifndef DEVGUIDE_PLUGIN_BODY_POSITION_ITEM_H
 #define DEVGUIDE_PLUGIN_BODY_POSITION_ITEM_H

 #include <cnoid/Item>
 #include <cnoid/RenderableItem>
 #include <cnoid/BodyItem>
 #include <cnoid/SceneGraph>
 #include <cnoid/SceneDrawables>
 #include <cnoid/Selection>

 class BodyPositionItem : public cnoid::Item, public cnoid::RenderableItem
 {
 public:
     static void initializeClass(cnoid::ExtensionManager* ext);

     BodyPositionItem();
     BodyPositionItem(const BodyPositionItem& org);
     void storeBodyPosition();
     void restoreBodyPosition();
     virtual cnoid::SgNode* getScene() override;
     void setPosition(const cnoid::Isometry3& T);
     const cnoid::Isometry3& position() const { return position_; }
     bool setFlagHeight(double height);
     double flagHeight() const { return flagHeight_; }
     enum ColorId { Red, Green, Blue };
     bool setFlagColor(int colorId);
     double flagColor() const { return flagColorSelection.which(); }

 protected:
     virtual Item* doDuplicate() const override;
     virtual void onTreePathChanged() override;
     virtual void doPutProperties(cnoid::PutPropertyFunction& putProperty) override;
     // 以下の2つの関数を追加
     virtual bool store(cnoid::Archive& archive) override;
     virtual bool restore(const cnoid::Archive& archive) override;

 private:
     void createFlag();
     void updateFlagPosition();
     void updateFlagMaterial();

     cnoid::BodyItem* bodyItem;
     cnoid::Isometry3 position_;
     cnoid::SgPosTransformPtr flag;
     double flagHeight_;
     cnoid::Selection flagColorSelection;
     cnoid::SgMaterialPtr flagMaterial;
 };

 typedef cnoid::ref_ptr<BodyPositionItem> BodyPositionItemPtr;

 #endif // DEVGUIDE_PLUGIN_BODY_POSITION_ITEM_H


BodyPositionItem.cpp
~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp
 :emphasize-lines: 7,8,9,227,228,229,230,231,232,233,234,235,236,237,238,239,240,242,243,244,245,246,247,248,249,250,251,252,253
		   
 #include "BodyPositionItem.h"
 #include <cnoid/ItemManager>
 #include <cnoid/MeshGenerator>
 #include <cnoid/EigenUtil>
 #include <cnoid/PutPropertyFunction>

 // 以下のヘッダを追加
 #include <cnoid/Archive>
 #include <cnoid/EigenArchive>

 #include <fmt/format.h>
 
 using namespace std;
 using namespace fmt;
 using namespace cnoid;

 void BodyPositionItem::initializeClass(ExtensionManager* ext)
 {
     ext->itemManager()
	 .registerClass<BodyPositionItem>("BodyPositionItem")
	 .addCreationPanel<BodyPositionItem>();
 }

 BodyPositionItem::BodyPositionItem()
 {
     bodyItem = nullptr;
     position_.setIdentity();
     flagColorSelection.setSymbol(Red, "red");
     flagColorSelection.setSymbol(Green, "green");
     flagColorSelection.setSymbol(Blue, "blue");
     flagColorSelection.select(Red);
     flagHeight_ = 1.8;
 }

 BodyPositionItem::BodyPositionItem(const BodyPositionItem& org)
     : Item(org)
 {
     bodyItem = nullptr;
     position_ = org.position_;
     flagHeight_ = org.flagHeight_;
     flagColorSelection = org.flagColorSelection;
 }

 Item* BodyPositionItem::doDuplicate() const
 {
     return new BodyPositionItem(*this);
 }

 void BodyPositionItem::onTreePathChanged()
 {
     auto newBodyItem = findOwnerItem<BodyItem>();
     if(newBodyItem && newBodyItem != bodyItem){
	 bodyItem = newBodyItem;
	 mvout()
	     << format("BodyPositionItem \"{0}\" has been attached to {1}.",
		       name(), bodyItem->name())
	     << endl;
     }
 }

 void BodyPositionItem::storeBodyPosition()
 {
     if(bodyItem){
	 position_ = bodyItem->body()->rootLink()->position();
	 updateFlagPosition();
	 mvout()
	     << format("The current position of {0} has been stored to {1}.",
		       bodyItem->name(), name())
	     << endl;
     }
 }

 void BodyPositionItem::restoreBodyPosition()
 {
     if(bodyItem){
	 bodyItem->body()->rootLink()->position() = position_;
	 bodyItem->notifyKinematicStateChange(true);
	 mvout()
	     << format("The position of {0} has been restored from {1}.",
		       bodyItem->name(), name())
	     << endl;
     }
 }

 SgNode* BodyPositionItem::getScene()
 {
     if(!flag){
	 createFlag();
     }
     return flag;
 }

 void BodyPositionItem::createFlag()
 {
     if(!flag){
	 flag = new SgPosTransform;
	 updateFlagPosition();
	 flagMaterial = new SgMaterial;
	 updateFlagMaterial();
     } else {
	 flag->clearChildren();
     }

     MeshGenerator meshGenerator;

     auto pole = new SgShape;
     pole->setMesh(meshGenerator.generateCylinder(0.01, flagHeight_));
     pole->getOrCreateMaterial()->setDiffuseColor(Vector3f(0.7f, 0.7f, 0.7f));
     auto polePos = new SgPosTransform;
     polePos->setRotation(AngleAxis(radian(90.0), Vector3::UnitX()));
     polePos->setTranslation(Vector3(0.0, 0.0, flagHeight_ / 2.0));
     polePos->addChild(pole);
     flag->addChild(polePos);

     auto ornament = new SgShape;
     ornament->setMesh(meshGenerator.generateSphere(0.02));
     ornament->getOrCreateMaterial()->setDiffuseColor(Vector3f(1.0f, 1.0f, 0.0f));
     auto ornamentPos = new SgPosTransform;
     ornamentPos->setTranslation(Vector3(0.0, 0.0, flagHeight_ + 0.01));
     ornamentPos->addChild(ornament);
     flag->addChild(ornamentPos);

     auto banner = new SgShape;
     banner->setMesh(meshGenerator.generateBox(Vector3(0.002, 0.3, 0.2)));
     banner->setMaterial(flagMaterial);
     auto bannerPos = new SgPosTransform;
     bannerPos->setTranslation(Vector3(0.0, 0.16, flagHeight_ - 0.1));
     bannerPos->addChild(banner);
     flag->addChild(bannerPos);
 }

 void BodyPositionItem::updateFlagPosition()
 {
     if(flag){
	 auto p = position_.translation();
	 flag->setTranslation(Vector3(p.x(), p.y(), 0.0));
	 auto rpy = rpyFromRot(position_.linear());
	 flag->setRotation(AngleAxis(rpy.z(), Vector3::UnitZ()));
	 flag->notifyUpdate();
     }
 }

 void BodyPositionItem::updateFlagMaterial()
 {
     if(flagMaterial){
	 switch(flagColorSelection.which()){
	 case Red:
	     flagMaterial->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
	     break;
	 case Green:
	     flagMaterial->setDiffuseColor(Vector3f(0.0f, 1.0f, 0.0f));
	     break;
	 case Blue:
	     flagMaterial->setDiffuseColor(Vector3f(0.0f, 0.0f, 1.0f));
	     break;
	 default:
	     break;
	 }
	 flagMaterial->notifyUpdate();
     }
 }        

 void BodyPositionItem::setPosition(const Isometry3& T)
 {
     position_ = T;
     updateFlagPosition();
     notifyUpdate();
 }

 bool BodyPositionItem::setFlagHeight(double height)
 {
     if(height <= 0.0){
	 return false;
     }
     flagHeight_ = height;
     if(flag){
	 createFlag();
	 flag->notifyUpdate();
     }
     notifyUpdate();
     return true;
 }

 bool BodyPositionItem::setFlagColor(int colorId)
 {
     if(!flagColorSelection.select(colorId)){
	 return false;
     }
     updateFlagMaterial();
     notifyUpdate();
     return true;
 }

 void BodyPositionItem::doPutProperties(PutPropertyFunction& putProperty)
 {
     auto p = position_.translation();
     putProperty("Translation", format("{0:.3g} {1:.3g} {2:.3g}", p.x(), p.y(), p.z()),
		 [this](const string& text){
		     Vector3 p;
		     if(toVector3(text, p)){
			 position_.translation() = p;
			 setPosition(position_);
			 return true;
		     }
		     return false;
		 });

     auto r = degree(rpyFromRot(position_.linear()));
     putProperty("Rotation", format("{0:.0f} {1:.0f} {2:.0f}", r.x(), r.y(), r.z()),
		 [this](const string& text){
		     Vector3 rpy;
		     if(toVector3(text, rpy)){
			 position_.linear() = rotFromRpy(radian(rpy));
			 setPosition(position_);
			 return true;
		     }
		     return false;
		 });

     putProperty.min(0.1)("Flag height", flagHeight_,
		 [this](double height){ return setFlagHeight(height); });

     putProperty("Flag color", flagColorSelection,
		 [this](int which){ return setFlagColor(which); });
 }

 // 以下の2つの関数を追加
 bool BodyPositionItem::store(Archive& archive)
 {
     write(archive, "translation", Vector3(position_.translation()));
     write(archive, "rotation", degree(rpyFromRot(position_.linear())));
     archive.write("flag_height", flagHeight_);
     archive.write("flag_color", flagColorSelection.selectedSymbol());
     return true;
 }

 bool BodyPositionItem::restore(const Archive& archive)
 {
     Vector3 v;
     if(read(archive, "translation", v)){
	 position_.translation() = v;
     }
     if(read(archive, "rotation", v)){
	 position_.linear() = rotFromRpy(radian(v));
     }
     archive.read("flag_height", flagHeight_);
     string color;
     if(archive.read("flag_color", color)){
	 flagColorSelection.select(color);
     }
     return true;
 }

 

利用方法
--------

今回のサンプルでは、プロジェクト保存を行うと、BodyPositionItemに記録されている位置と、旗の高さ・色のプロパティが、プロジェクトファイルに保存されます。そして保存したプロジェクトを読み込み直すと、記録位置、旗の高さ、色が保存時と同じ状態に戻ります。実際に :ref:`basics_project_save` と :ref:`basics_project_load` を操作して、挙動を確認してみてください。

例えば :doc:`item-property-sample` の :ref:`plugin-dev-item-property-sample-howto` で示したように、PA10Pickupのプロジェクトを読み込んでBodyPositionItemを導入し、以下の状態にしたとします。

.. image:: images/flags-example.png
    :scale: 50%

これまでのサンプルでは、この状態にしてプロジェクト保存をしても、それを読み込み直した時に記録位置は全て原点に戻ってしまい、旗の高さや色もデフォルトの状態に戻っていました。これではせっかく導入したBodyPositionItemもあまり使い物にならなかったと言えるでしょう。しかし今回のサンプルでは同じ操作を行うと、各BodyPositionItemの状態が元に戻ります。このようにプロジェクトファイルを用いて状態の保存と復帰ができるようになって初めて、ユーザはこの機能を安心して使えるようになるかと思います。

ソースコードの解説
------------------

ヘッダファイルは関数定義を追加しているだけなので、実装ファイルの追加部分を対象に解説します。

まず ::

 #include <cnoid/Archive>

によって `Archiveクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1Archive.html>`_ を使えるようにしています。これはstore関数とrestore関数の実装で必要となります。 ::

 #include <cnoid/EigenArchive>

Choreonoid SDKのUtilライブラリのヘッダで、Eigenの行列やベクトルを :ref:`plugin-dev-yaml-structured-data-classes` と連携させるための関数が定義されています。こちらもstore関数とrestore関数の実装で使用します。 ::

 bool BodyPositionItem::store(Archive& archive)
 {
     ...
     return true;
 }

アイテムの状態をプロジェクトファイルに保存するためのstore関数です。Itemクラスでvirtual関数として定義されているものをオーバーライドして実装します。保存に成功した場合はtrueを返すようにします。以下でこの関数内のコードを解説します。 ::

 write(archive, "translation", Vector3(position_.translation()));

BodyPositionItemに記録されている位置はメンバ変数position_に格納されています。これはEigenのIsometry3型で、4x4の同次変換行列に相当するものです。ここではその要素である並進成分を "translation" というキーでarchiveに書き込んでいます。ここで使用しているwrite関数はEigenArchiveヘッダで定義されているテンプレート関数で、以下のように定義されています。 ::

 template<typename Derived>
 Listing& write(Mapping& mapping, const std::string& key, const Eigen::MatrixBase<Derived>& x);

このテンプレートによって、Eigenの任意の行列・ベクトル型をMappingノードに出力することが可能です。この場合キーに対応する値はListingノードとなり、その要素としてベクトルの要素が入ります。これはYAMLで記述すると

.. code-block:: yaml

 translation: [ x, y, z ]

となります。ベクトルの要素は最終的にはこのようにフロースタイルで出力されます。

なお、このwrite関数を使わない場合は、同じ処理を以下のように記述できます。 ::

 auto translation = archive.createFlowStyleListing();
 translation->append(position_.translation().x());
 translation->append(position_.translation().y());
 translation->append(position_.translation().z());

これと比べるとEigenArchiveのwrite関数を使用したほうが簡潔に書けることが分かります。

次に記録位置の回転成分を以下のコードで出力しています。 ::

 write(archive, "rotation", degree(rpyFromRot(position_.linear())));

rpyFromRotはEigenUtilヘッダで定義されている関数で、3x3回転行列を入力とし、対応するロールピッチヨー（RPY）の回転成分を三次元ベクトルVector3型で返すものです。この場合の各成分の単位はラジアンになるのですが、ここではさらにdegree関数を用いてこれを度に変換しています。（degree関数もEigenUtilヘッダで定義されています。）ラジアンでそのまま出力してもよいのですが、YAMLで記述したときの読性を考慮して度としています。そのRPYの値を先ほどと同様にwrite関数で "rotation" というキーワードを付けて出力しています。 ::

 archive.write("flag_height", flagHeight_);

Archiveが継承しているMapping型の通常の関数を用いて、メンバ変数flagHeight_の値を "flag_height" というキーで出力しています。 ::

 archive.write("flag_color", flagColorSelection.selectedSymbol());

メンバ変数flagColorSelectionで選択されている選択肢について、文字列として出力しています。ここは ::

 archive.write("flag_color", flagColorSelection.which());

として選択項目のインデックス値（整数値）で出力することも考えられますが、YAMLで記述したときの分かりやすさを考慮してここでは文字列としています。

以上で状態保存は完了となります。実際にこのコードで出力されたYAMLは以下のようになります。

.. code-block:: yaml

 translation: [ 0.9, 0, 0.035 ]
 rotation: [ 0, -0, 90 ]
 flag_height: 0.5
 flag_color: Blue

これがプロジェクトファイルの中で該当アイテムの状態を記述する箇所に出力されます。

.. note:: ここでは旗の高さと色について、キーを "flag_height"、"flag_color" というように、「小文字のみで単語を記述してアンダースコアで区切る」命名としています。これは「スネークケース」と呼ばれる記法になります。一方でこれを "FlagHeight" や "flagHeight" とする「キャメルケース」の記法とすることも考えられます。ChoreonoidではYAMLの記述に従来キャメルケースを使用してきたのですが、最近のバージョンではスネークケースへの切り替えを進めており、今後はスネークケースでキーやシンボルを記述することを推奨します。

次に状態復帰のためのrestore関数を実装しています。 ::

 bool BodyPositionItem::restore(const Archive& archive)
 {
     ...
     return true;
 }

ここではstoreで出力したデータを読めるようにすればOKです。まず ::

 Vector3 v;
 if(read(archive, "translation", v)){
     position_.translation() = v;
 }

の部分で記録位置の並進成分を読んでいます。ここで使用しているread関数もEigenArchiveで定義されているテンプレート関数で、writeとは逆に指定したキーの値をベクトル値として読み込むものです。読み込みに成功するとtrueを返すので、その場合に変数vに読み込んだ値をメンバ変数position_の並進成分に代入しています。

これについても、EigenArchiveの関数を使わない場合は、以下のようなやや複雑なコードが必要となります。 ::

 auto translation = archive.findListing("translation");
 if(translation->isValid() && translation->size() == 3){
     for(int i=0; i < 3; ++i){
         position_.translation()[i] = translation->at(i)->toDouble();
     }
 }

つぎに以下のコードで回転成分を読み込んでいます。 ::

 if(read(archive, "rotation", v)){
     position_.linear() = rotFromRpy(radian(v));
 }

ここではstoreの逆の処理として、まず読み込んだRPYの値をラジアンに変換し、そのRPY値からrotFromRpy関数によって回転行列を得て、それをメンバ変数position_の回転成分に代入しています。ここで使用しているradianとrotFromRpyもEigenUtilで定義されている関数です。 ::

 archive.read("flag_height", flagHeight_);

旗の高さを読み込んでいます。 ::

  string color;
  if(archive.read("flag_color", color)){
      flagColorSelection.select(color);
  }

旗の色を読み込んでいます。色の保存は文字列で行うようにしたので、それに合わせてここでもまず文字列で読み込んでいます。それをメンバ変数flagColorSelectionで選択するようにしています。

この部分はMappingのget関数を使用して以下のように記述することも出来ます。 ::

 flagColorSelection.select(archive.get("flag_color", flagColorSelection.selectedSymbol()));

get関数はread関数とは異なり、読み込んだ値を戻り値として返します。ただしキーが存在しない場合は読み込めないので、その場合は第2引数に指定したデフォルト値を返すようになっています。ここではデフォルト値として現在の設定値を指定しています。状況によってはこちらの関数の方がより簡潔に記述できるかと思います。
