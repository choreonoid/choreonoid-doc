
xacro（URDF用マクロ）
======================

.. contents::
   :local:
   :depth: 3

概要
----

xacro (XML macro)は、XMLのためのマクロ言語です。xacroを用いることで、XMLを短く、読みやすく書くことができます。Choreonoidで利用可能なロボットモデル記法 `URDF (Unified Robot Description Format) <modelfile-urdf.html>`_ はXMLを用いています。したがって、xacroはロボットモデルの記述の簡略化に貢献します。

最新版のChoreonoidではxacroをバンドルしているため、ROS環境の有無を問わず、xacroの機能を利用することが可能です（現在はLinux版でのみ対応）。

なお、xacroファイルでは、robotタグに xmlns:xacro="http://www.ros.org/wiki/xacro" と属性を記述する必要があります。

以下では、 `xacroの公式ドキュメント <http://wiki.ros.org/xacro>`_ に基づいて、xacroの機能を簡単に紹介します。

.. _xacro-file-reference-properties:

変数
----

propertyマクロは、XMLドキュメント内で、変数の役割を果たします．propertyマクロの使い方は2つあります。

1つ目は、数や文字列を変数として扱う使い方です。まずxacro:propertyタグで、変数名（name属性）と値（value属性）によって変数を定義します。そして、使いたい場所で、dollared-bracesと呼ばれる記法により、${変数名} として、その変数を利用します。

次の例では、円筒状の幾何形状を、radiusとlengthという2つの変数を介して指定しています。

例::

    <xacro:property name="radius" value="2.1"/>
    <xacro:property name="length" value="4.5"/>

    <geometry type="cylinder" radius="${radius}" length="${length}"/>

2つ目は、propertyブロックと呼ばれる方法です。propertyブロックでは、valueを指定する代わりに、xacro:propertyタグで囲った部分を変数のように扱うことができます。ただし、これを使う際には、xacro:insert_blockタグを使わなければなりません。

次の例では、xacro:insert_blockタグを用いて、予め定義しておいたoriginタグを適用しています。同じ内容のタグを色々なところで使う場合などで、propertyブロックは有効です。

例::

    <xacro:property name="link_origin">
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
    </xacro:property>

    <link name="sample_link">
      <inertial>
        <xacro:insert_block name="link_origin"/>
        ...
      </inertial>
    </link>

辞書・リスト・YAMLファイル
~~~~~~~~~~~~~~~~~~~~~~~~~~

xacroでは、変数の定義において、Pythonの辞書形式やリストを利用できます。

例（Python記法）::

    <xacro:property name="props" value="${dict(a=1, b=2, c=3)}"/>
    <xacro:property name="props_alt" value="${dict([('1a',1), ('2b',2), ('3c',3)])}"/>
    <xacro:property name="numbers" value="${[1,2,3,4]}"/>

また、YAMLファイルを読み込んで、YAMLファイルから直接変数を定義できます。このとき、YAMLファイル内の変数は、全て辞書内に格納されます。

例（YAMLファイルの読み込み）::

    <xacro:property name="yaml_file" value="$(find package)/config/props.yaml" />
    <xacro:property name="props" value="${load_yaml(yaml_file)}"/>

YAMLファイル::

    a: 1
    b: 2
    c: 3

辞書内の変数の利用例::

    <xacro:property name="a" value="${props['a']}"/>

.. _xacro-file-reference-mathematical-expression:

数式
----

xacroでは、XML内で計算を行わせることも可能です。dollared-braces (${})の中で書かれた四則演算や比較、Pythonのmathモジュールで定義された定数（円周率πなど）および関数（三角関係など）を利用できます。

例::

    <xacro:property name="R" value="2"/>
    <xacro:property name="alpha" value="${sin(30/180*pi)}"/>

    <limit lower="${radians(-90)}" upper="${radians(90)}" effort="0" velocity="${radians(75)}"/>

.. _xacro-file-reference-conditional-blocks:

条件分岐
--------

xacroでは、条件分岐が使えます。条件分岐のために、xacro:ifタグとxacro:unlessタグが用意されています。

xacro:ifタグは、自身のvalue属性が1もしくはtrueとなるときだけ、タグで囲んだ部分（ブロック内部）を有効にします。逆にxacro:unlessタグは、自身のvalue属性が0もしくはfalseとなるときだけ、タグで囲んだ部分（ブロック内部）を有効にします。どちらのタグも、value属性が1/0とtrue/falseのどちらでもない値を取ったときには、エラーを返すことに注意が必要です。

value属性の中で数学マクロを使うことで、複雑な条件での分岐を行わせることもできます。

例::

    <xacro:property name="var" value="useit"/>
    <xacro:if value="${var == 'useit'}"/>
    <xacro:if value="${var.startswith('use') and var.endswith('it')}"/>

    <xacro:property name="allowed" value="${[1,2,3]}"/>
    <xacro:if value="${1 in allowed}"/>

.. _xacro-file-reference-ros-commands:

ROSコマンド
----------------

xacroではdollared-parentheses ($())の中で、ROS (Robot Operating System) で使われているコマンドの一部を利用できます。

Choreonoidで利用可能なコマンドは以下の通りです。これらはいずれもROS環境の有無に関わらず利用可能です。

envコマンド
~~~~~~~~~~~

$(env ENVIRONMENT_VARIABLE) とすることで 環境変数 ENVIRONMENT_VARIABLE の値を取得します。環境変数が存在しなかった場合、エラーを返します。

optenvコマンド
~~~~~~~~~~~~~~
$(optenv ENVIRONMENT_VARIABLE) とすることで、環境変数 ENVIRONMENT_VARIABLE の値を取得します。環境変数が存在しなかった場合、空文字列を返します。

また $(optenv ENVIRONMENT_VARIABLE default_value) とすれば、環境変数が存在しなかった場合、デフォルト値として default_value を返します。

findコマンド
~~~~~~~~~~~~

$(find pkg) とすると、環境変数 ROS_PACKAGE_PATH の中で末尾が pkg となるパスを探索します。もしそのようなパスが存在しない場合は、コマンドはエラーを返します。

ROS環境では、環境変数 ROS_PACKAGE_PATH が設定され、インストールされているパッケージを参照できます。一方でROS環境がない場合でも、環境変数 ROS_PACKAGE_PATH を設定することで、このコマンドを利用できます。

argコマンド
~~~~~~~~~~~

$(arg arg1) とすることで、xacro:argタグで与えられた引数 arg1 を利用できます。

例::

    <xacro:arg name="link_name" default="default_link"/>

    <link name="$(link_name)">

evalコマンド
~~~~~~~~~~~~

$(eval <expression>) とすることで、通常のdollared-braces (${})では扱えない複雑な表現 expression を評価できます。

例（コマンドの併用と文字列の連結）::

    <xacro:property name="paths" value="$(eval env('PATH') + ':' + find('pkg')">

.. _xacro-file-reference-macro:

マクロ
------

xacroの最も強力な機能はマクロです．マクロはxacro:macroタグを用いて定義します。name属性でマクロの名前を、params属性でマクロのパラメータ（関数の引数に相当）を指定します。パラメータが複数あるときは、空白を挟んで並べます。

各パラメータは標準で文字列を取りますが、タグやブロック（あるタグによって囲まれた複数のタグ）をパラメータとすることも可能です。パラメータ名の前に*（アスタリスク）を1つ付けることでXMLのタグを、アスタリスクを2つ付けることでXMLブロックを、パラメータとして与えることができるようになります。

タグやブロックをパラメータはとして与えるときには、パラメータ名と与えるタグ名・ブロック名を対応付ける必要はありません。ただし、複数のタグやブロックをパラメータとして与える場合、パラメータの順番と記述した順番が対応することになりますので、注意が必要です。以下の例では、2つのブロック b0 と abc が、それぞれ block0 と block1 に対応して展開されています。

例::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:macro name="sample_macro" params="prefix *tag **block0 **block1">
        <link name="${prefix}_link">
          <inertial>
            <xacro:insert_block name="tag"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>

          <xacro:insert_block name="block0"/>

          <xacro:insert_block name="block1"/>
        </link>
      </xacro:macro>

      <xacro:sample_macro prefix="sample">
        <mass value="1.0"/>
        <b0>
          <collision>
            <geometry>
              <box size="1.0 1.0 1.0"/>
            </geometry>
          </collision>
          <!-- memo -->
        </b0>
        <abc>
          <visual>
            <geometry>
              <box size="0.5 0.5 0.5"/>
            </geometry>
            <material>
              <color rgba="1.0 0.0 0.0 1.0"/>
            </material>
          </visual>
        </abc>
      </xacro:sample_macro>
    </robot>


xacroの出力結果::

    <robot name="sample">
      <link name="sample_link">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
        <collision>
          <geometry>
            <box size="1.0 1.0 1.0"/>
          </geometry>
        </collision>
        <!-- memo -->
        <visual>
          <geometry>
            <box size="0.5 0.5 0.5"/>
          </geometry>
          <material>
            <color rgba="1.0 0.0 0.0 1.0"/>
          </material>
        </visual>
      </link>
    </robot>

マクロ内のマクロ
~~~~~~~~~~~~~~~~

マクロはその内部に他のマクロを持つことができます。ただし、内部のマクロは事前に定義されている必要があります。

例::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <link name="a">
        <xacro:macro name="mass" params="value">
          <mass value="${value}"/>
        </xacro:macro>

        <xacro:macro name="inertial">
          <inertial>
            <xacro:mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>
        </xacro:macro>

        <xacro:inertial/>
      </link>
    </robot>

xacroの出力結果::

    <robot name="sample">
      <link name="a">
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </robot>

スコープ
~~~~~~~~

変数やマクロのスコープは基本的にマクロの内部となります。つまり、マクロの中で定義された変数やマクロは、原則としてそのマクロの内部でしか利用できません。

外部の変数やマクロをどうしても参照したいときは、それらの変数やマクロを定義するタグに scope="parent" と属性を追加することで、一つ上の（親の）階層までスコープを広げることができます。あるいは、 scope="global" と属性を追加することで、自身のスコープをグローバル、すなわち全体に広げることができます。ただし、スコープを広げるほど名前の管理が複雑になるため、これらのスコープ拡張の利用には注意が必要です。

パラメータのデフォルト値
~~~~~~~~~~~~~~~~~~~~~~~~

マクロの標準（文字列の）パラメータは、デフォルト値を取ることができます。デフォルト値は、マクロの定義時に、パラメータ名に := に続いて設定します。

また、^ （サーカムフレックス）を用いて、パラメータ名に :=^ と続けることで、同じ名前の外部の変数を読み込むことができます。さらに、 :=^| 1.0 のようにすると、まず外部変数を探し、それが見つからなかった場合に、バーティカルバーに続く値（ここでは1.0）がデフォルト値として使われます。

例::

    <robot name="sample" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <link name="a">
        <xacro:property name="y" value="10.0"/>

        <xacro:macro name="inertial" params="x:=1.0 y:=^ z:=^|3.0">
          <inertial>
            <origin xyz="${x} ${y} ${z}"/>
            <mass value="1.0"/>
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
          </inertial>
        </xacro:macro>

        <xacro:inertial/>
      </link>
    </robot>

xacroの出力結果::

    <robot name="sample">
      <link name="a">
        <inertial>
          <origin xyz="1.0 10.0 3.0"/>
          <mass value="1.0"/>
          <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
        </inertial>
      </link>
    </robot>

.. _xacro-file-reference-loading-files:

ファイルの読み込み
------------------

xacro:include タグを利用すると、他のxacroファイルを読み込めます。

例::

    <xacro:include filename="$(find package)/other_file.xacro"/>

マクロや変数の名前が衝突しないように、読み込み時に ns 属性で名前空間を付与することも可能です。

例（名前空間の付与）::

    <xacro:include filename="$(find package)/other_file.xacro" ns="namespace/>

.. _xacro-file-reference-processing-order:

処理順
------

xacroコマンドは、与えられたファイルを上から順に読み込み、逐次的に処理・評価を実施します。

過去の仕様との比較
~~~~~~~~~~~~~~~~~~

当初、xacroコマンドは、以下の順番でxacroファイルを処理していました。

1. ファイル読み込み（xacro:includeタグの展開）
2. 変数 (property) およびマクロの定義
3. マクロの展開
4. 数式やマクロ等の評価

評価が最後の処理となるため、 if や unless による条件分岐は、変数やマクロの定義に影響を与えませんでした。そこで、処理順をこの旧仕様から現在の仕様に変更することにより、以下の利点が実現されました。

- ファイルの読み込みや変数、マクロの定義を条件分岐によって実行できる。そのため、必要なファイル読み込み、変数・マクロの定義だけを実行できる。
- 読み込むファイルの名前を、変数やマクロ等を利用して指定できる。
- ファイルの途中で変数の値を変更すると、それより後の部分にだけその変更が反映される。
- ローカルなマクロや変数を、他のスコープの同名変数に影響を与えること無く定義できる。
