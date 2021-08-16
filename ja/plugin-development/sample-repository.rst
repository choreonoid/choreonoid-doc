==================
サンプルリポジトリ
==================

本ガイドで作成するサンプルプラグインのソースコードはGithub上でリポジトリとして公開しています。アドレスは https://github.com/choreonoid/plugin-dev-guide.git です。このリポジトリを「サンプルリポジトリ」と呼ぶことにします。これは本ガイドで作成する全てのサンプルのソースコードを含むもので、それぞれの通し番号に対応するディレクトリに分けて格納しています。

このリポジトリではどのサンプルもプラグイン名は同一の"DevGuidePlugin"としています。これはサンプルごとに別のプラグインとなってガイドを進める度にそれが増えてくると、プラグイン間で競合を起こしてしまうからです。このためプラグイン名は同一とし、同時にビルド・実行するサンプルは常にひとつに限定されるようにしています。

サンプルはChoreonoid本体のビルド環境でビルドすることが可能です。その場合はChoreonoid本体のextディレクトリにリポジトリをクローンしてください。するとChoreonoid本体のCMakeの設定で以下の項目が追加されます。

* DEV_GUIDE_SAMPLE_INDEX

ここにビルド・実行するサンプルの通し番号を設定します。するとその通し番号のサンプルがビルド・インストールされるようになります。

各サンプルの通し番号について以下に示します。

* 01: :doc:`minimum-sample`
* 02: :doc:`signal-sample`
* 03: :doc:`item-operation-sample`
* 04: :doc:`toolbar-sample`
* 05: :doc:`new-item-type-sample`
* 06: :doc:`item-scene-sample`
* 07: :doc:`item-property-sample`
* 08: :doc:`item-project-save-sample`
* 09: :doc:`item-file-io-sample`
* 10: :doc:`item-creation-io-customization-sample`
* 11: :doc:`create-view-sample`

サンプルを紹介する各セクションの見出しには "（S通し番号）" という表記でサンプルの番号が分かるようにしています。
