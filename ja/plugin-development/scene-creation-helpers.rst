====================
シーン構築の補助機能
====================

.. contents:: 目次
   :local:

.. highlight:: cpp  

概要
----

:ref:`plugin-dev-scene-node-classes` で紹介したクラスは、多様な形状やシーンを表現する能力を有していますが、メッシュ等のデータを含むシーンの全てを直接コードで記述していくのは多くの場合現実的ではありません。
この解決方法として、比較的シンプルな形状やシーンであればプリミティブ形状を利用することが考えられますし、複雑な形状やシーンについては、専用のツールで作成されたモデルのファイルを読み込むのが一般的です。
本節ではそれらの手法をChoreonoidで利用する方法について紹介します。

.. _plugin-dev-use-primitive-shapes:

プリミティブ形状の利用
----------------------

グラフィックスや物理計算のシステムで直接生成・編集可能な幾何形状を「幾何プリミティブ」もしくは「プリミティブ形状」と呼びます。プリミティブ形状は一般的に直方体、球、円柱など少数のパラメータで規定されるシンプルな形状が該当します。シーン内のオブジェクトとしてプリミティブ形状をそのまま使用することもありますし、組み合わせることでより複雑な形状を表現することもあります。Choreonoidのシーングラフでもプリミティブ形状を使用することが可能です。

.. _plugin-dev-mesh-generator:

MeshGenerator
~~~~~~~~~~~~~

Choreonoid SDKでは `MeshGeneratorクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1MeshGenerator.html>`_ を用いてシーングラフ用のプリミティブ形状を生成できます。このクラスはUtilライブラリで定義されていて、MeshGeneratorヘッダをインクルードすると使えるようになります。

生成可能な形状はボックス（直方体）、球、シリンダ（円柱）、円錐、カプセル、アロー（矢印）、トーラスです。それぞれMeshGeneratorの以下のメンバ関数で対応するメッシュ（SgMeshオブジェクト）を生成できます。

* **SgMesh* generateBox(const Vector3& size)**

  * ボックスを生成する

  * パラメータとしてX、Y、Z軸方向の長さを指定

* **SgMesh* generateSphere(double radius)**

  * 球を生成する

  * パラメータとして半径を指定

* **SgMesh* generateCylinder(double radius, double height)**

  * シリンダを生成する

  * パラメータとして半径と高さ指定

* **SgMesh* generateCone(double radius, double height)**

  * 円錐を生成する

  * パラメータとして半径と高さ指定

* **SgMesh* generateCapsule(double radius, double height)**

  * カプセルを生成する

  * パラメータとして半径と高さ指定

* **SgMesh* generateArrow (double cylinderRadius, double cylinderHeight, double coneRadius, double coneHeight)**

  * アローを生成する

  * パラメータとして胴体部分のシリンダのサイズと先端部分の円錐のサイズを指定
  
生成したメッシュはローカル座標において原点が形状の中心となります。
またシリンダ、コーン、カプセル、アローについては、高さ方向がY軸と一致する向きになります。
形状の中心や向きを変更したい場合は :ref:`plugin-dev-scenegraph-sgpostransform` を用いるか、 `SgMesh <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SgMesh.html>`_ のtransform、translate、rotate関数を使用します。

.. note:: カプセルは円柱と2つの半球を組み合わせた形状で、円柱の両端を丸めたものです。指定する半径と長さのパラメータは、円柱と半球に設定されます。両端の半球の半径は円柱の半径と一致します。またアローはシリンダと円錐を組み合わせて矢印状にしたもので、座標軸の表示などに使うことができます。

ボックス以外のプリミティブは、本来球面になる部分もメッシュで近似的に表現したものとなります。その部分のメッシュをどれだけ細かく分けるかについて、divisionNumber（分割数）というパラメータで調整できます。divisionNumberはデフォルトで20になっていて、MeshGeneratorの以下の関数で設定・取得できます。

* **void setDivisionNumber(int n)**

  メッシュの分割数をnに設定します。

* **int divisionNumber() const**

  メッシュの分割数を返します。

分割数は球であれば緯度、経度に相当する方向の分割数になります。シリンダや円錐については、断面となる円の外周の分割数になります。分割数を細かくすることでより滑らかな表示となりますが、その分データや描画が重くなりますので、必要に応じて適切な値となるよう調整します。

プリミティブ形状を生成するコードの例を以下に示します。 ::

 #include <cnoid/MeshGenerator>
 #include <cnoid/EigenUtil>

 using namespace cnoid;
 
 SgNode* generateCone()
 {
     MeshGenerator meshGenerator;
     meshGenerator.setDivisionNumber(36);
     auto shape = new SgShape;
     shape->setMesh(meshGenerator.generateCone(0.5, 1.0));
     auto cone = new SgPosTransform;
     cone->setTranslation(Vector3(0.0, 0.0, 0.5));
     // The radian function is defined in EigenUtil
     cone->setRotation(AngleAxis(radian(90.0), Vector3::UnitX()));
     cone->addChild(shape);
     return cone;
 }
 
ここではMeshGeneratorで生成した円錐のメッシュに座標変換を付与して、円錐の底がXY平面と一致するようにしています。generateCone関数が返すノードをシーンに追加すると以下の表示となります。

.. image:: images/cone.png
    :scale: 60%

.. note:: MeshGeneratorは他にも押し出し形状を生成するgenerateExtrusion関数や、凹凸のあるグリッド形状を生成するgenerateElevationGrid関数を備えています。それらの形状はプリミティブ形状というよりは一般のメッシュ形状となりますが、必要に応じてそちらも使用することができます。それらは `VRML97 <https://tecfa.unige.ch/guides/vrml/vrml97/spec/>`_ のExtrusionノードおよびElevationGridノードに該当するものですので、そちらを参考にしてお使いください。

.. _plugin-dev-sgmesh-primitive-information:
	  
SgMeshのプリミティブ情報
~~~~~~~~~~~~~~~~~~~~~~~~

メッシュの情報を格納するSgMeshクラスでは同時にプリミティブ情報も付与できます。
これはメッシュの内容がプリミティブ形状に対応するものである場合に、プリミティブの種類と寸法を記録しておけるというものです。
現在対応しているプリミティブはボックス、球、シリンダ、円錐、カプセルの5つになります。

MeshGeneratorで生成したメッシュが上記のいずれかのプリミティブである場合は、SgMeshのプリミティブ情報も同時に付与されます。

設定可能なプリミティブの種類はSgMeshクラスで以下の列挙型として定義されています。 ::

 enum PrimitiveType {
    // 一般のメッシュ（必ずしもプリミティブではない）
    MeshType,
    // ボックス
    BoxType,
    // 球
    SphereType,
    // シリンダ
    CylinderType,
    // 円錐
    ConeType,
    // カプセル
    CapsuleType
 };

現在設定されいているプリミティブはSgMeshの以下の関数で取得できます。

* **const int primitiveType() const**

各プリミティブのパラメータは、SgMeshクラスで定義されている以下の入れ子クラス（プリミティブ型）の値として格納されます。 ::

 // ボックス
 class Box {
 public:
     Box() : Box(Vector3(1.0, 1.0, 1.0)) { }
     Box(Vector3 size) : size(size) { }
     Vector3 size;
 };

 // 球
 class Sphere {
 public:
     Sphere() : Sphere(1.0) { }
     Sphere(double radius) : radius(radius) { }
     double radius;
 };

 // シリンダ
 class Cylinder {
 public:
     Cylinder() : Cylinder(1.0, 1.0) { }
     Cylinder(double radius, double height)
         : radius(radius), height(height), top(true), bottom(true), side(true) { }
     double radius;
     double height;
     bool top;
     bool bottom;
     bool side;
 };

 // 円錐
 class Cone {
 public:
     Cone() : Cone(1.0, 1.0) { }
     Cone(double radius, double height)
         : radius(radius), height(height), bottom(true), side(true) { }
     double radius;
     double height;
     bool bottom;
     bool side;
 };

 // カプセル
 class Capsule {
    public:
        Capsule() : Capsule(1.0, 1.0) { }
        Capsule(double radius, double height)
            : radius(radius), height(height) { }
        double radius;
        double height;
    };

 // 一般のメッシュであることを示す型で、プリミティブ情報をクリアする際に使用する
 class Mesh { };

.. 英訳指示： 上のコードで各プリミティブクラスに日本語のコメントをつけていますが、これは英語にするとクラス名そのものなので、英訳の際にはコメントは省いてください。ただし最後のMeshクラスのコメントは英訳して残してください。

SgMeshオブジェクトは、これらのプリミティブ型のうちprimitiveTypeに対応するクラスの値のみ格納しています。
これを取り出すには以下の関数を使用します。

* **template<class TPrimitive> const TPrimitive& primitive() const**

  * 指定したプリミティブ型の設定値を返します

例えばプリミティブがボックスである場合にその情報を取得するコードは以下のようになります。 ::

 if(mesh->primitiveType() == SgMesh::BoxType){
     auto& box = mesh->primitive<SgMesh::Box>();
     double x = box.size().x();
     ...
 }

primitiveTypeの値とは異なるプリミティブ型の情報を取り出そうとする例外がスローされますのでご注意ください。

上述のように、サポートされているプリミティブ形状をMeshGeneratorを用いて生成する場合は、生成されたSgMeshオブジェクトにこの情報も付与されます。
一方で、独自に構築したメッシュにプリミティブ情報を付与することもできます。
その場合は以下の関数でプリミティブ情報を設定します。

* **void setPrimitive(Primitive prim)**

ここで引数のprimには上記のプリミティブ型のいずれかの型の値を指定できます。例えばSgMesh型のオブジェクトmeshにボックスのプリミティブ情報を設定する場合は、以下のようにします。 ::

 mesh->setPrimitive(SgMesh::Box(1.0, 2.0, 3.0));

なお、SgMeshのプリミティブ情報はあくまで補助的なもので、それを設定する場合でもメッシュ本体の（頂点等の）データは必ず必要です。
そしてメッシュ情報はメッシュ本体のデータとは独立して設定するものなので、それがメッシュ本体と一致することは必ずしも保証されるものではありません。
そこはSgMeshオブジェクトを構築する側の責任であり、利用側は両者が一致していることを信じて利用することになります。

プリミティブの情報は一般的に干渉検出やモデル編集などの処理で利用されます。
それらの処理ではプリミティブであることが分かっている形状に対して処理の効率化や簡潔化を実現できますが、メッシュ本体のデータからそれを得ることは容易ではなく、メッシュオブジェクトがプリミティブの情報も有していることは大きな手助けとなります。

モデルファァイルの読み込み
--------------------------

シーンの要素が比較的シンプルであったり動的に構築する必要がある場合は、上記のプリミティブ形状なども活用しながらプログラムコードで構築するのが適しているかもしれません。一方で、予め決まったモデルを使用する場合は、専用のツールで作成されたモデルをファイルから読み込むのが適しています。Choreonoid SDKでもこれを行うことが可能です。なお、三次元モデルをメッシュ形式でのみ格納するファイルは「メッシュファイル」と呼ばれることもありますが、ここではそれも含めて三次元モデルデータのファイルを「モデルファイル」と呼ぶことにします。

SceneLoader
~~~~~~~~~~~

モデルファイルの読み込みは、 `SceneLoaderクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1SceneLoader.html>`_ を用いて行います。このクラスもUtilライブラリで定義されていて、SceenLoaderヘッダをインクルードすることで使えるようになります。デフォルトコンストラクタで生成して、以下の関数を使用できます。

* **void setMessageSink(std::ostream& os)**

  読み込み時のエラーなどのメッセージを出力をする標準出力ストリームを設定します。
  デフォルトではどこにも出力されないようになっています。

* **void setDefaultDivisionNumber(int n)**

  プリミティブ読み込み時のデフォルトの分割数を設定します。初期設定はMeshGeneratorのデフォルト値となっています。

* **void setDefaultCreaseAngle(double theta)**

  法線を自動生成する場合のデフォルトの折り目角度を設定します。

* **SgNode* load(const std::string& filename)**

  filenameで指定したモデルファイルを読み込みます。読み込まれたモデルのルートとなるシーンノードを返します。読み込みに失敗した場合はnullptrを返します。

* **SgNode* load(const std::string& filename, bool& out_isSupportedFormat)**

  上記関数と同様に読み込みを試みます。その上で、指定したファイルがサポートされている形式であるかどうかをout_isSupportedFormatに返します。

基本的にload関数を用いれば読み込むことが可能で、他の関数による設定は必要に応じて実行すればOKです。
例えば "model.obj" というモデルファイルを読み込む場合は以下のようにします。 ::

  #include <cnoid/SceneLoader>
  ...

  SceneLoader sceneLoader;

  // メッセージがあればメッセージビューに出力する
  sceneLoader.setMessageSink(MessageView::instance()->cout());

  auto node = sceneLoader.load("model.obj");
  
これでmodel.objの内容がnodeに入ります。
ロードに失敗した場合はnodeがnullptrとなり、（場合によっては）失敗の原因がメッセージビューに出力されます。

SceneLoaderのサポートするファイル形式
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SceneLoaderは標準で以下のファイル形式（括弧内は標準のファイル拡張子）をサポートしています。

* STL (.stl)
* Wavefront OBJ (.obj)
* VRML97 (.wrl)
* Collada (.dae)
* DirectX .x file (.x)
* Blender (.blend)
* DXF (.dxf)

STL、OBJ、VRML97についてはChoreonoidがネイティブのローダ実装を持っています。それらはUtilライブラリで実装されています。
Collada以下のファイル形式は、モデルファイル読み込み用ライブラリである `Open Asset Import Library (Assimp) <https://github.com/assimp/assimp>`_ によってロードされます。このためこれらのファイル形式についてはAssimpプラグインを導入しておくことが必要です。

これらに加えて、Choreonoid標準のモデルファイル形式も定義されており、そちらのファイルもロードできます。これは「標準シーンファイル」と呼んでいて、拡張子は .scen になります。これについてはChoreonoid固有のもので広く使われているわけではないので、ここでは詳細を省きます。

なお、プリミティブ形状をサポートしているファイル形式については、ファイルにプリミティブ形状が含まれていて、それが :ref:`plugin-dev-sgmesh-primitive-information` でサポートされている形状である場合、プリミティブ情報も読み込まれます。例えば上記のファイル形式の中ではVRML97がこれをサポートしていて、SceneLoaderでプリミティブ情報も読み込まれるようになっています。もちろん「標準シーンファイル」もプリミティブ情報をサポートしています。

各ファイル形式の読み込みは、実際には専用のローダクラスで行われます。このためのクラス階層として、まず基盤となる `AbstractSceneLoaderクラス <https://choreonoid.org/ja/documents/reference/latest/classcnoid_1_1AbstractSceneLoader.html>`_ が定義されています。これは抽象クラスとして定義されていて、ファイル形式ごとにこのクラスを継承したローダクラスが実装されています。Choreonoidでは上記のファイル形式に対応する以下のローダクラスが実装されています。

* STLSceneLoader
* ObjSceneLoader
* VRMLSceneLoader
* AssimpSceneLoader

SceneLoaderもAbstractSceneLoaderを継承したローダクラスですが、そちらは特定のファイル形式を対象とせず、実際のロード処理は内部で上記のローダを使用するようになっています。load関数に与える拡張子でファイル形式を判別し、実際に使用するローダを選択します。ほとんどの場合はこのSceneLoaderを使用すればよいですが、特定形式のファイルを読み込む場合はそれ専用のローダも使うことができます。
  
他のファイル形式についても、追加のローダを実装することで対応できるようになります。ここでは詳細には触れませんが、AbstractSceneLoaderを継承したローダクラスを作成し、それをSceneLoaderのstatic関数registerLoaderで登録すればOKです。

.. note:: Assimpライブラリ自体は上に挙げた形式以外にもかなりの数のファイル形式をサポートしているのですが、AssimpSceneLoaderについては現状でChoreonoidのシーングラフとして正常に読み込めることを確認できた形式だけをロードできるようにしています。
