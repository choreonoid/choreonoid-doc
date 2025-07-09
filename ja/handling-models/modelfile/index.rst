
モデルファイル
==============

Choreonoid上で扱うロボットや環境物体のモデルは通常、あらかじめファイルとして記述されたものをボディアイテムとして読み込むことで生成します。ファイルはモデルの形状、色などの光学情報、関節構造、物理パラメータ、センサ情報等が記述されたもので、これを「モデルファイル」と呼びます。

モデルファイルの形式はロボットのソフトウェアシステムによって様々なものが定義されています。Choreonoidでは現在のところ以下の形式を標準でサポートしています。

* Body形式モデルファイル（Bodyファイル）
* Unified Robotics Description形式モデルファイル（URDFファイル）
* OpenHRP形式モデルファイル

Choreonoidの現バージョンではBody形式が標準のモデルファイル形式となっていますので、特に理由がなければこの形式を用いるようにしてください。

.. Choreonoidではプラグインによって他の形式のモデルファイルのインポート機能を追加できる設計となっており、OpenHRP以外の形式についても利用の多いものは今後対応を進めていきたく思っています。その一環として現在のところ "Collada Robot Extensions" 形式の読み込み機能も実装されています。ただしこの実装はまだ完全ではなく、うまく読み込めない場合もあるため、今のところは特別な理由がなければOpenHRP形式を使うようにしてください。

.. 英訳指示：英語版では modelfile-blender への参照を省いてください。toctreeから modelfile-blender の行を削除してください。

.. toctree::
   :maxdepth: 2

   modelfile-newformat
   tank
   yaml-reference
   modelfile-yaml
   tank-blender
   modelfile-blender
   modelfile-urdf
   modelfile-xacro
   modelfile-openhrp

..   modelfile-sr1
