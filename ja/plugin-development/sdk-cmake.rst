===============
CMakeの記述方法
===============

.. list-table::
 :widths: 40,60
 :header-rows: 1

 * - 変数
   - 内容
 * - CHOREONOID_DEFINITIONS
   - コンパイルオプション
 * - CHOREONOID_INCLUDE_DIRS
   - ヘッダファイルのディレクトリ
 * - CHOREONOID_LIBRARY_DIRS
   - ライブラリファイルのディレクトリ
 * - CHOREONOID_UTIL_LIBRARIES
   - Utilモジュール使用時にリンクすべきライブラリ
 * - CHOREONOID_BASE_LIBRARIES
   - Baseモジュール使用時にリンクすべきライブラリ
 * - CHOREONOID_PLUGIN_DIR
   - プラグインファイルをインストールするディレクトリ

次に、find_packageによって取得された情報を以下のように使用しています。 ::

 add_definitions(${CHOREONOID_DEFINITIONS})
 include_directories(${CHOREONOID_INCLUDE_DIRS})
 link_directories(${CHOREONOID_LIBRARY_DIRS})

この記述により、コンパイルオプション、インクルードパス、リンクパスが適切に設定されます。 ::

 set(target CnoidHelloWorldPlugin)

プラグイン名を変数targetに設定しています。 ::

 add_library(${target} SHARED HelloWorldPlugin.cpp)

プラグインは共有ライブラリになりますので、CMake標準のadd_libraryコマンドでビルドを行うことができます。

:ref:`hello-world-build-together` では、add_libraryを拡張したchoreonoid_add_pluginというコマンドでプラグインをビルドしましたが、プラグインを単体でビルドする場合は直接add_libaryを使用するようにします。 ::

 target_link_libraries(${target} ${CHOREONOID_BASE_LIBRARIES})

プラグインにリンクすべきライブラリを指定しています。find_packageで取得されたCHOREONOID_BASE_LIBRARIES変数を使用することで、プラグインの基盤となるライブラリ一式をリンクすることができます。 ::

 install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})

ビルドしたプラグインのファイルをChoreonoidのプラグインディレクトリにインストールするための設定です。インストール先はこのようにCHOREONOID_PLUGIN_DIR変数で指定することができます。



なお、CMakeでは、同一のプロジェクトで定義されているライブラリをtarget_link_librariesで指定すると、そのライブラリが依存している全てのライブラリへのリンクも行われるようになります。例えば、CnoidBaseはQtのライブラリにも依存しているため、上記の記述でHelloWorldプラグインにもQtのライブラリがリンクされるようになります。このように、本手法ではリンクすべきライブラリについてあまり細かい部分まで気にせずに完結に記述することができます。

