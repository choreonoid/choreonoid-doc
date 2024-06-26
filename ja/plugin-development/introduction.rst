========
はじめに
========

Choreonoidは :doc:`../basics/plugin` を備えており、プラグインと呼ばれるソフトウェアモジュールによって機能の拡張や追加ができるようになっています。プラグインはChoreonoid本体と同様にC++で記述されたプログラムであり、Choreonoidの提供する各種クラスを利用して、様々な処理を実装することが可能です。

Choreonoid本体が提供する機能もその多くがプラグインとして実装されています。例えば、ロボットや物体のモデルを扱うための基本的な機能を提供するBodyプラグインや、動作振り付け関係の機能を提供するPoseSeqプラグイン、メディア関係の機能をまとめたMediaプラグインなどがあります。そのことからも分かるように、プラグインは特定の機能の拡張に限定されるものではなく、多用な機能を実装できるものです。

もちろんプラグインはユーザが独自に開発することが可能です。Choreonoidはプラグインによって機能拡張することを前提とした設計となっており、C++でプログラミングのできる方であれば比較的容易に機能拡張を行うことができます。そして、プラグインを開発をすることでユーザーごとのニーズに応えられるようになり、ロボットの研究開発や運用にChoreonoidを活用できる幅が広がるかと思います。

本ガイドでは新たにプラグインを開発する方法について解説します。

なお、プラグインの開発方法を学ぶことで、Choreonoidの設計や実装についても理解が深まり、Choreonoid本体の開発やバグ修正にも手を出しやすくなるのではないかと思います。Choreonoidはオープンソースソフトウェアであり、どなたでも開発に参加することが可能ですので、Choreonoidをもっと改良したいという方はぜひ開発にご参加ください。
