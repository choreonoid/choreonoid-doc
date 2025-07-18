ソースコードからのビルドとインストール (Ubuntu Linux編)
=======================================================

.. sectionauthor:: 中岡 慎一郎 <s.nakaoka@aist.go.jp>

Linuxには様々なディストリビューションがありますが、現在のところChoreonoidが公式にサポートしているディストリビューションは Ubuntu Linux になります。Choreonoidの最新の開発版では、Ubuntuバージョン24.04, 22.04, 20.04について、x64アーキテクチャ(64ビット）でのビルドと動作を確認しています。本ドキュメントでは、Ubuntu Linux におけるChoreonoidのソースコードからのビルド方法について説明します。

.. contents::
   :local:

.. highlight:: sh

ソースコードの取得
------------------

開発版
~~~~~~

Choreonoidの開発は `github <https://github.com/>`_ 上で行われており、以下のリポジトリから最新のソースコードを取得可能です。

- https://github.com/choreonoid/choreonoid

ソースコードは `Git <http://git-scm.com/>`_ リポジトリとして管理されており、利用にあたってはgitコマンドが必要です。Ubuntuでは以下のコマンドでGitをインストールできます。 ::

 sudo apt install git

Choreonoidのリポジトリは以下のコマンドを実行することで取得できます。 ::

 git clone https://github.com/choreonoid/choreonoid.git

これによってリポジトリを格納した "choreonoid" というディレクトリが生成されます。このディレクトリ内で ::

 git pull

などとすることにより、その時点での最新のソースコードにアップデートできます。
Gitの使用方法の詳細についてはGitのマニュアルや解説記事を参照してください。

ソースコードはこのようにGitリポジトリとして管理される場合もあれば、そうでない場合もあります。どちらの場合でもソースコードを格納したディレクトリを **「ソースディレクトリ」** とも呼ぶことにします。


リリース版
~~~~~~~~~~

Choreonoidのリリース版のソースコードは、 `ダウンロード <http://choreonoid.org/ja/downloads.html>`_ のページからダウンロードすることが可能です。このページにある「ソースパッケージ」の該当するバージョンをダウンロードしてください。ファイルはZIPファイルになっていますので、適当なディレクトリで ::

 unzip choreonoid-2.2.0.zip

などとして展開してください。

展開すると choreonoid-2.2.0 といったディレクトリが生成されます。このディレクトリもソースコード一が格納された **「ソースディレクトリ」** となります。

.. note:: リリース版については、開発版を対象とした本マニュアルの手順が異なる場合があります。例えば、2.0.0以前のバージョンでは、Boost C++ Librariesのインストールも必要となります。リリース版のインストール方法については、　`各リリース版のマニュアル <http://choreonoid.org/ja/documents/index.html>`_ を参照するようにしてください。

開発ツールと依存ソフトウェアのインストール
------------------------------------------

開発ツール
~~~~~~~~~~

Choreonoidをソースコードからビルドするためには、以下の開発ツールが必要になります。

- C/C++標準開発ツール一式: C/C++コンパイラ、Make等の標準開発ツール一式が必要です。Ubuntuであれば "build-essential" というパッケージで一式インストールできます。C/C++コンパイラに関しては通常GCCを用いますが、Clang/LLVMも利用可能です。
- `CMake <http://www.cmake.org/>`_ :  ビルドツールです。本ツール独自の記述から、Make や Visual Studio といった標準ビルドツールのファイルを生成します。多くの環境に対応したビルド記述を効率的に行うことが可能です。

依存ライブラリ
~~~~~~~~~~~~~~  
  
基本機能をビルドするにあたって以下のライブラリも必要になります。

* `Eigen <https://eigen.tuxfamily.org>`_ : 行列・ベクトル・線形代数演算のための高速・高機能なテンプレートライブラリです。
* `Qt <https://www.qt.io/>`_ : GUIツールキットを含むフレームワークライブラリです。
* `gettext <http://www.gnu.org/s/gettext/>`_ : 表示を多国語対応とするためのツール・ライブラリです。
* `fmtlib <https://github.com/fmtlib/fmt>`_ : 書式付き文字列を出力するライブラリです。
* `libjpeg <http://libjpeg.sourceforge.net/>`_ : JPEG形式の画像ファイルを読み込むためのライブラリです。
* `libpng <http://www.libpng.org/pub/png/libpng.html>`_ : PNG形式の画像ファイルを読み込むためのライブラリです。
* `libzip <https://libzip.org/>`_ : ZIP形式のファイルを読み書きするためのライブラリです。
* `LibYAML <http://pyyaml.org/wiki/LibYAML>`_ : YAML形式テキストのパーサです。
* `FreeType <http://freetype.org/>`_ : フォントを描画するためのライブラリです。3D画像にテキストを描画するために使用します。
* `Assimp <http://assimp.sourceforge.net/>`_ : 様々な形式の3Dモデルファイルを読み込むためのライブラリです。

.. note:: 以前のバージョンではBoost C++ Librariesにも依存していましたが、2024年3月11日のコミットf40ea6fcよりBoost C++ Librariesは不要となりました。ただし、Choreonoid本体とは別に配布されているプラグインがBoost C++ Librariesを必要とする場合はあるかもしれません。

また、オプションの機能をビルドする際には、以下のようなソフトウェアも追加で必要となってきます。

* `Python <https://www.python.org/>`_ : プログラミング言語Pythonを用いてChoreonoidを操作するための「Pythonプラグイン」を利用する際に必要となります。通常Pythonは標準でインストールされていますが、プラグインをビルドする際に開発用のライブラリが必要となります。
* `Numpy <http://www.numpy.org/>`_ : 各種科学技術計算を行うためのPythonライブラリです。こちらもPythonプラグインで必要になります。
* `Open Dynamics Engine (ODE) <http://www.ode.org/>`_ : 物理計算ライブラリです。この物理計算によるシミュレーションを行うための「ODEプラグイン」を利用する際に必要です。
* `GStreamer <http://gstreamer.freedesktop.org/>`_ : メディアファイルを扱うためのライブラリです。音声ファイルや動画ファイルをChoreonoid上で再生するための「Mediaプラグイン」を利用する際に必要です。
* `PulseAudio <http://www.freedesktop.org/wiki/Software/PulseAudio/>`_ : 音声出力を行うためのシステムです。Ubuntuでは標準でインストールされていますが、Mediaプラグインをビルドする場合には別途開発用ライブラリが必要になります。
* `libsndfile <http://www.mega-nerd.com/libsndfile/>`_ : 音声ファイルを読み込むためのライブラリです。Mediaプラグインを利用する際に必要です。

.. _build-ubuntu-install-packages:

依存パッケージのインストール
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
Ubuntuの場合、"misc/script" 以下にある "install-requisites-ubuntu-x.x.sh" というスクリプトを用いることにより、上記のソフトウェアのほとんどを簡単にインストールすることができます。x.xはUbuntuのバージョンに対応します。例えば Ubuntu 24.04 であれば ::

 misc/script/install-requisites-ubuntu-24.04.sh

を実行すると、sudoのパスワードが求められるので入力してください。すると、パッケージシステム経由で、必要なパッケージが自動でインストールされます。

なお、Qtについてはバージョン6と5に対応しています。Qtバージョン4はChoreonoid 1.7までは対応していましたが、最新版では対応していません。

.. _build-ubuntu-cmake:
	  
CMakeによるビルド設定
---------------------

まず、ビルドに使用するディレクトリを作成します。Choreonoidのソースディレクトリ上で ::

 mkdir build

などとして作成してください。作成したディレクトリを **ビルドディレクトリ** と呼びます。ここでは "build" というディレクトリ名を使用していますが、名前は何でも結構です。複数のビルドディレクトリを作成して、それぞれ異なる設定でビルドすることも可能です。

次にビルドディレクトリ上でCMakeを実行します。 ::

 cd build
 cmake ..

このコマンドにより、コンパイラのバージョンや依存ライブラリのチェックなどが行われ、ビルドに必要なMakefileが生成されます。cmakeコマンドの後はピリオドが２つになりますので、ここも間違わないようにしてください。これはcmakeの対象となるソースがひとつ上のディレクトリにあることを示しています。

対象バージョンのUbuntuにおいて上述の説明通りに作業を進めていれば問題なくMakefileが生成されるはずですが、必要なライブラリが所定の場所にインストールされていなかったりすると、cmake実行の際にエラーが出ることがあります。その場合には、適切にインストールを行うか、CMakeによるビルド設定を修正することが必要になります。ビルド設定はcmakeコマンドを用いてコマンドラインから行うことも可能ですし、ccmakeコマンドを ::

 ccmake ..

と実行することにより、各種設定をメニュー形式で行うことも可能です。詳しくはCMakeのマニュアルを参照してください。

Choreonoidは、上記のデフォルトではビルドされないオプション機能もいくつか備えています。それらの概要を :doc:`options` にまとめてありますので、希望する機能がある場合はCMakeの設定で有効にしてください。例えば、Open Dynamics Engine によるシミュレーション機能を使いたい場合は、 **BUILD_ODE_PLUGIN** を "ON" にしておきます。


Clangの設定
-----------

通常はGCCコンパイラを用いてビルドされますが、Clangを用いてビルドすることも可能です。その場合はClangをインストールした上で、環境変数CC、CXXを設定するか、CMakeのCMAKE_C_COMPIER、CMAKE_CXX_COMPIERの各変数を設定します。

Clangは以下のようにしてインストールできます。 ::

 sudo apt install clang

環境変数でClangの使用を設定する場合は、それぞれ以下のように設定します。

 * CC: clang
 * CXX: clang++

これはCMake実行時に設定されていればよいので、例えばCMakeを実行する際に ::

 CC=clang CXX=clang++ cmake ..

としてもよいですし、予め ::

 export CC=clang
 export CXX=clang++

としておいてもよいです。

あるいは、上記の環境変数はCMakeの変数CMAKE_C_COMPIER、CMAKE_CXX_COMPIERに対応しているので、 ::

 cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=gcc ..

などとしてもよいです。

NVIDIAのドライバをインストールしている場合、ドライバのビルド用に特定のバージョンのGCCが追加でインストールされることがあります。その場合は上記の方法ではClangの標準C++ライブラリが使えなくなることがあります。これに対処するためには、まず ::

 clang --verbose

として表示される ::

 Selected GCC installation: /usr/bin/../lib/gcc/x86_64-linux-gnu/12

といった表示を確認します。ここで表示されている最後の数字がClangを用いたビルドに必要なGCCのバージョンとなります。このバージョンに対応する標準C++ライブラリを ::

 sudo apt install libstdc++-12-dev

などとしてインストールします。 ( `stack overflow の参考ページ <https://stackoverflow.com/questions/74543715/usr-bin-ld-cannot-find-lstdc-no-such-file-or-directory-on-running-flutte>`_  )

.. note:: Clangでビルドする場合、環境やClangのバージョンによってはRange sensorのシミュレーションがうまくいかない不具合が発生しますのでご注意ください。Ubuntu 22.04でClang14を使用してビルドする場合、この不具合は発生しないようです。

.. _install_build-ubuntu_build:

Choreonoidのビルド
------------------

CMakeの実行に成功すると、ビルドのためのMakefile一式がビルドディレクトリ内に生成されます。ビルドディレクトリで ::

 make

を実行することで、Choreonoidのビルドが行われます。

マルチコアCPUであれば、"-j" オプションにより並列ビルドを行うことでビルド時間を短縮できます。例えば、 ::

 make -j8

とすると、最大で8つのビルドプロセスが同時に実行されることになります。通常は論理コア数と同じプロセス数を指定することで、CPU能力を最大限に活かした並列ビルドとなります。

なお、CMakeが生成したMakefileによるmakeでは、実行コマンドの詳細は表示されず、ビルド過程がすっきりとまとまった表示で出力されます。これはビルドの進行を確認する際には大変見やすくてよいのですが、GCCに与えている細かなコンパイルオプションなどは確認できません。その必要があるときには、 ::

 make VERBOSE=1

というように VERBOSE変数をオンにしてmakeを行うことで、全てのコマンド実行文の詳細を出力させることも可能です。

makeコマンドの代わりに、CMakeのコマンドでビルドすることもできます。この場合は ::

 cmake --build ビルドディレクトリ

とします。 ::

 cmake --build ビルドディレクトリ --parallel 並列数

とすると並列ビルドを行います。並列数を省略するとコンパイラのデフォルト値が使用されます。環境変数CMAKE_BUILD_PARALLEL_LEVELに並列数をセットしておくと、--parallelオプションを入力しなくても並列ビルドを行いますので、これを .bashrc などに記述しておくとよいでしょう。

また "-v" オプションをつけると、"make VERBOSE=1" のときと同様に実行されるコマンドの詳細が出力されるようになります。

.. _build-ubuntu_install:

インストール
------------

ChoreonoidをUbuntuで使用する場合は、ビルドディレクトリ内に生成される実行ファイルをそのまま実行することが可能です。ビルドに成功すれば、ビルドディレクトリ内の"bin"というディレクトリの下に "choreonoid" という実行ファイルが生成されていますので、これを実行してください。 ::

 bin/choreonoid

ビルドに問題がなければ、Choreonoidのメインウィンドウが起動します。

このようにインストール作業なしに実行できるのは便利なので、特に問題がなければこの形態で使用してもよいかと思います。

一方で指定したディレクトリへのインストールを行うこともできます。この場合ソフトウェアの実行に必要なバイナリファイルやデータファイルのみが一箇所にまとめられることになります。このためソフトウェアをシステム全体で共有したり、パッケージ化したり、他のソフトウェアと連携して使用する場合などは、インストール作業を行います。

これを行うためには、ビルドディレクトリ上で ::

 make install

を実行します。すると、実行に必要なファイル一式が所定のディレクトリにインストールされます。

Ubuntuではデフォルトのインストール先は "/usr/local" となっています。このディレクトリへの書き込みは通常はroot権限が必要ですので、 ::

 sudo make install

とする必要があります。

/usr/localの場合は実行ファイルを格納する/usr/local/binにデフォルトでパスが通っているので、カレントディレクトリがどこにあっても、単に ::

 choreonoid

とすることでChoreonoidを実行できます。

インストール先は、CMakeの **CMAKE_INSTALL_PREFIX** の設定で変更することも可能です。複数のアカウントで利用する必要がなければ、ホームディレクトリのどこかをインストール先にしてもOKです。この場合、インストール時にsudoをする必要もなくなります。ただし/usr/local/binと同様にパスが通っている必要がある場合は、インストール先のbinディレクトリに自前でパスを通すようにしてください。

.. note:: デフォルトのインストール先である/usr/localにインストールすることは **お勧めできません** 。このディレクトリはデフォルトのインストール先として一般的ではあるのですが、これは便宜的なものだと考えたほうがよいです。ソフトウェアをソースコードから自前でビルド・インストールする場合、OSのパッケージ管理システムでは管理されないのが一般的です。つまり管理も自前で行う必要がありますが、そのようなものが/usr/localという同一のディレクトリにごちゃまぜにインストールされると、ある特定のソフトウェアのアップグレードにおいて不必要になったファイルを除去したり、特定のソフトウェアだけアンインストールするといったことが、大変困難になります。従って/usr/localにはインストールせず、ホームディレクトリ上に各ソフトウェアごとに専用のディレクトリを用意してそこにインストールするのがよいかと思います。

.. note:: Choreonoidのように共有ライブラリを含むソフトウェアの場合、一般的には共有ライブラリをインストールするlibディレクトリに共有ライブラリパスが通っている必要があります。これについても/usr/local/libについてはデフォルトでパスが通っていますが、そうでないディレクトリの場合は自前でパスを通す必要があります。ただしChoreonoidではRPATHという仕組みで共有ライブラリパスを設定しなくても動作するようになっていますので、通常この設定は必要ありません。Choreonoidの共有ライブラリを外部のソフトウェアからライブラリとして利用する際は、この設定が必要になる場合があります。なおRPATHについてはCMakeのAdvancedオプションで **ENABLE_INSTALL_RPATH** をOFFにすることで無効化できます。これはデフォルトでONになっており、特に無効化する理由がなければ変更しないようにしてください。

なお、インストールの操作もMakeの代わりにCMakeのコマンドでも実行できます。 ::

 cmake --install ビルドディレクトリ

とすると **CMAKE_INSTALL_PREFIX** に設定されているディレクトリにインストールします。インストール先は ::

 cmake --install ビルドディレクトリ --prefix インストール先

として指定することも可能です。

オプション機能のビルド
----------------------

コレオノイドでは、上記手順のデフォルト状態で有効になるもの以外にも、いくつかのモジュールやプラグイン、サンプル等があります。それらは :doc:`options` にまとめてあります。

オプション機能を有効にする手順は、基本的に以下のようになります。

1. （必要に応じて）依存ライブラリをインストールする
2. CMakeのビルド設定で該当するオプションを有効化する
3. Choreonoidのビルドを再度実行する

2については、オプションに対応するCMakeの変数がありますので、そちらを "ON" に設定します。

変数はコマンドラインからcmakeコマンドで設定してもよいですし、ccmakeコマンドで表示されるメニュー画面から設定することも可能です。

例えばChoreonoidの動作振り付け機能に対応する「PoseSeqプラグイン」と「バランサープラグイン」は以下のようにして有効化できます。 ::

 cd ビルドディレクトリ
 cmake .. -DBUILD_POSE_SEQ_PLUGIN=ON -DBUILD_BALANCER_PLUGIN=ON

逆にあるオプションを無効化する場合は、対応する変数に "OFF" を設定します。例えば ::

 cmake .. -DENABLE_SAMPLES=OFF

とすることで、サンプルをビルドしないように設定することができます。

"-D" オプションで設定した内容はビルドディレクトリ内に保存されるので、変更したい変数だけを追加で設定することが可能です。
もちろん複数の変数をまとめて設定してもOKで、cmakeの初期化時に全ての設定を行ってもOKです。

設定の変更後に再度ビルドの操作を行うことにより、オプション機能がビルドされ利用できるようになります。

その他の環境整備
----------------

Choreonoid本体のビルドとインストールが完了しましたら、より快適な利用環境の確保のため、以下の内容についても確認することをおすすめします。

* :doc:`setup-gpu`
* :doc:`setup-qt`
