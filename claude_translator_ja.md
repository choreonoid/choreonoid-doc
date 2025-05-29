### 温度パラメータの調整

翻訳の決定性を制御できます：

```bash
# 完全に決定的な翻訳（デフォルト）
python claude_translator.py ja en --temperature 0.0

# 少し柔軟性を持たせた翻訳
python claude_translator.py ja en --temperature 0.2

# より多様な表現を許可（通常は推奨しない）
python claude_translator.py ja en --temperature 0.5
```

技術文書の翻訳では、一貫性を保つため低い温度（0.0-0.2）を推奨します。## 技術仕様

### 使用モデル
- Claude Sonnet 4 (claude-sonnet-4-20250514)
  - 最大出力トークン: 64,000
- Claude Opus 4 (claude-opus-4-20250514)
  - 最大出力トークン: 32,000
- Temperature: 0.0（デフォルト、完全に決定的な出力）

### ファイル処理
- ファイルごとに独立したAPI呼び出し
- 各ファイルの翻訳は完全に独立（文脈の共有なし）
- ルートディレクトリのファイルを優先的に処理

### URL変換
- choreonoid.orgドメインのURLで`/ja/`パスを自動的に`/en/`に変換
- その他のURLは変更しない## 利用可能なモデル

### Claude Sonnet 4（デフォルト）
- 高速でバランスの取れた性能
- ほとんどのユースケースに推奨
- コスト効率が良い

### Claude Opus 4
- 最高品質の翻訳
- より正確な文法と表現
- 重要な文書や品質を最優先する場合に推奨

```bash
# Opus 4を使用して翻訳
python claude_translator.py ja en --model opus

# 特定のファイルをOpus 4で翻訳
python claude_translator.py ja en --single-file ja/install/build-ubuntu.rst --model opus --force
```# Choreonoid Documentation Translator 使用方法

## 概要

このスクリプトは、Choreonoidの技術文書を日本語から英語に翻訳するためのツールです。Anthropic Claude API (Claude 4)を使用して、reStructuredText形式のドキュメントを高品質に翻訳します。

## 必要な環境

- Python 3.7以上
- Anthropic Claude APIキー
- インターネット接続

## セットアップ

### 1. 必要なパッケージのインストール

```bash
pip install anthropic pyyaml
```

### 2. APIキーの設定

環境変数として設定する方法（推奨）：

```bash
export ANTHROPIC_API_KEY="your-api-key-here"
```

または、実行時にコマンドラインオプションで指定することも可能です。

### 3. Choreonoidドキュメントリポジトリの準備

```bash
# リポジトリのクローン
git clone https://github.com/choreonoid/choreonoid-doc.git
cd choreonoid-doc

# 翻訳スクリプトをリポジトリのルートディレクトリにコピー
cp /path/to/claude_translator.py .
```

## 基本的な使用方法

### 初回の全体翻訳

```bash
python claude_translator.py ja en --force
```

このコマンドは、`ja`ディレクトリ内のすべての`.rst`ファイルを`en`ディレクトリに翻訳します。

### 変更されたファイルのみ翻訳（通常使用）

```bash
python claude_translator.py ja en
```

ファイルのハッシュ値を比較し、前回の翻訳以降に変更されたファイルのみを翻訳します。

### 特定のディレクトリのみ翻訳

特定のセクションだけを翻訳したい場合、ディレクトリを直接指定できます：

```bash
# installセクションのみ翻訳
python claude_translator.py ja/install en/install

# basicsセクションのみ翻訳
python claude_translator.py ja/basics en/basics

# plugin-developmentセクションのみ翻訳
python claude_translator.py ja/plugin-development en/plugin-development
```

**注意**: 進捗管理（`translation_progress.json`）とログ（`translation.log`）は、どのディレクトリを対象にしても共通で使用されます。これにより、部分的な翻訳作業を積み重ねることができます。

## 高度な使用方法

### テスト実行（ファイル数を制限）

```bash
# 最初の3ファイルのみ翻訳
python claude_translator.py ja en --max-files 3 --force
```

### 特定のファイルのみ翻訳

```bash
python claude_translator.py ja en --single-file ja/install/build-ubuntu.rst
```

### 中断した翻訳の再開

```bash
python claude_translator.py ja en --resume
```

### 画像ファイルのみコピー（翻訳なし）

```bash
python claude_translator.py ja en --copy-images
```

### APIキーをコマンドラインで指定

```bash
python claude_translator.py ja en --api-key "your-api-key-here"
```

## コマンドラインオプション

| オプション | 説明 |
|----------|------|
| `source_dir` | ソースディレクトリ（例：`ja`） |
| `target_dir` | ターゲットディレクトリ（例：`en`） |
| `--api-key` | Anthropic APIキー（環境変数の代替） |
| `--force` | すべてのファイルを強制的に翻訳（変更チェックをスキップ） |
| `--max-files N` | 翻訳するファイルの最大数を指定（テスト用） |
| `--single-file` | 単一ファイルのみを翻訳 |
| `--resume` | 前回の進捗から翻訳を再開 |
| `--copy-images` | 画像ファイルのみコピー（翻訳はスキップ） |
| `--terminology` | 用語集YAMLファイルを指定（デフォルト: choreonoid_terminology.yaml） |
| `--model` | 使用するモデルを選択: sonnet（デフォルト）, opus |
| `--temperature` | 応答生成の温度パラメータ（デフォルト: 0.0）。低い値で決定的、高い値で多様性が増加 |

## 翻訳の仕組み

### ファイル変更検出

各ファイルのSHA256ハッシュ値を`translation_progress.json`に記録し、次回実行時に比較することで変更を検出します。

### 画像ファイルの処理

RSTファイル内の画像参照（`.. image::`、`.. figure::`）を自動検出し、必要に応じて画像ファイルをコピーします。英語版ディレクトリに既に存在する画像はスキップされます。

### 翻訳品質

- reStructuredTextの構造を完全に保持
- ディレクティブやロールは翻訳せず、本文のみを翻訳
- コードブロック内の日本語コメントは翻訳対象
- ソフトウェアマニュアルとして適切な文体を維持
- 括弧内の内容も省略せずに翻訳
- choreonoid.orgのURLは自動的に`/ja/`から`/en/`に変換

### 検証機能

翻訳後、以下の項目を自動的にチェックします：
- RST構造の保持（ディレクティブ数）
- コードブロックの数の一致
- 不一致がある場合は警告を表示（翻訳は継続）

## 進捗管理

翻訳の進捗は`translation_progress.json`ファイルに記録されます：

```json
{
  "file_hashes": {
    "ja/install/build-ubuntu.rst": "hash_value..."
  },
  "completed_files": ["ja/install/build-ubuntu.rst"],
  "failed_files": [],
  "translation_stats": {
    "total_characters": 12345,
    "total_time_seconds": 300,
    "files_per_directory": {
      "ja/install": 5,
      "ja/basics": 3
    }
  }
}
```

## ログ出力

翻訳の詳細なログは以下に出力されます：

- `translation.log` - ファイルに保存される詳細ログ
- コンソール出力 - リアルタイムの進捗表示

## 用語の統一

特定の用語を統一して翻訳したい場合は、`choreonoid_terminology.yaml`ファイルを作成して用語集を定義できます。

### 用語集ファイルの形式

`choreonoid_terminology.yaml`:
```yaml
# 日本語: 英語 の形式で記述
プロジェクト: project
シミュレーション: simulation
アイテム: item
コントローラ: controller
ボディ: body
リンク: link
```

### 用語集の使用方法

```bash
# デフォルトのファイル名（choreonoid_terminology.yaml）を使用
python claude_translator.py ja en

# カスタムファイル名を指定
python claude_translator.py ja en --terminology my_custom_terms.yaml
```

**注意**: 
- 用語集ファイルが存在しない場合や空の場合でも、スクリプトは正常に動作します
- 用語集はあくまでガイドラインとして使用され、文脈に応じて適切な翻訳が選択されます

## トラブルシューティング

### Rate Limit エラーが発生する場合

APIのレート制限に達した場合、スクリプトは自動的に待機してリトライします。頻繁に発生する場合は、翻訳間隔を調整するか、APIの利用制限を確認してください。

### 翻訳が中断された場合

`--resume`オプションを使用して、中断した箇所から再開できます：

```bash
python claude_translator.py ja en --resume
```

### RST構造の警告が表示される場合

```
WARNING - Code block count mismatch: 25 vs 24
WARNING - RST structure validation failed for ja/install/build-windows.rst
```

このような警告は、翻訳前後でコードブロックやディレクティブの数が異なることを示しています。翻訳は完了していますが、念のため該当ファイルを確認することを推奨します。

### 特定のファイルで問題が発生する場合

1. `translation.log`で詳細なエラー内容を確認
2. 問題のあるファイルを`--single-file`オプションで個別に翻訳してデバッグ
3. 必要に応じて、該当ファイルを手動で修正

## 推奨される使用フロー

1. **初回翻訳**
   ```bash
   python claude_translator.py ja en --force
   ```

2. **日常的な更新**
   ```bash
   # リポジトリの更新
   git pull
   
   # 変更されたファイルのみ翻訳
   python claude_translator.py ja en
   ```

3. **特定のセクションのみ更新**
   ```bash
   # インストールガイドのみ更新
   python claude_translator.py ja/install en/install
   
   # 複数のセクションを順次更新
   python claude_translator.py ja/basics en/basics
   python claude_translator.py ja/simulation en/simulation
   ```

4. **段階的な全体翻訳**
   大量のファイルがある場合、セクションごとに分けて翻訳することで、作業を管理しやすくなります：
   ```bash
   # 1日目: インストールガイド
   python claude_translator.py ja/install en/install
   
   # 2日目: 基本操作
   python claude_translator.py ja/basics en/basics
   
   # 3日目: その他の変更分をすべて処理
   python claude_translator.py ja en
   ```

## 注意事項

- 大量のファイルを翻訳する場合、APIの利用料金が発生します
- 翻訳には時間がかかるため、初回の全体翻訳は数時間を要する場合があります
- ネットワーク接続が不安定な環境では、`--resume`オプションの使用を推奨します

## サポート

問題が発生した場合は、以下を確認してください：

1. `translation.log`のエラーメッセージ
2. APIキーが正しく設定されているか
3. ネットワーク接続が安定しているか
4. Pythonとanthropicパッケージのバージョンが適切か