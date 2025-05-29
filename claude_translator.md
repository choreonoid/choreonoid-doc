# Choreonoid Documentation Translator

## Setup

### 1. Install Required Libraries

```bash
pip install anthropic pathlib
```

### 2. API Key Configuration

Obtain an Anthropic API key and configure it using one of the following methods:

```bash
# Set as environment variable (recommended)
export ANTHROPIC_API_KEY="your-api-key-here"

# Or specify as argument
python choreonoid_translator.py --api-key "your-api-key-here" ...
```

## Usage

### Basic Usage

```bash
# Translate all files (hash-based detection, automatic image copying)
python choreonoid_translator.py /path/to/choreonoid-doc/ja /path/to/choreonoid-doc/en

# Force translate all files (recommended for first run)
python choreonoid_translator.py /path/to/source /path/to/target --force

# Copy images only (no translation)
python choreonoid_translator.py /path/to/source /path/to/target --copy-images

# Show current usage cost
python choreonoid_translator.py /path/to/source /path/to/target --show-cost

# Translate maximum 10 files (for testing)
python choreonoid_translator.py /path/to/source /path/to/target --max-files 10

# Translate single file only
python choreonoid_translator.py /path/to/source /path/to/target --single-file /path/to/source/basics/launch.rst
```

### Resume Functionality

```bash
# Resume interrupted translation (changed files only)
python choreonoid_translator.py /path/to/source /path/to/target --resume

# Force resume all files
python choreonoid_translator.py /path/to/source /path/to/target --resume --force
```

## Practical Examples

### 1. Initial Translation (All Files)

```bash
# Clone repository
git clone https://github.com/choreonoid/choreonoid-doc.git
cd choreonoid-doc

# Force translate all files (first run)
python choreonoid_translator.py ja en --force
```

### 2. Image Processing Workflow

```bash
# Copy all images first
python choreonoid_translator.py ja en --copy-images

# Then run translation (images already copied)
python choreonoid_translator.py ja en --force
```

### 3. Daily Update Translation

```bash
# Edit Japanese files
vim ja/basics/launch.rst

# Translate changed files only (automatic image processing)
python choreonoid_translator.py ja en
# → Only files with changed content will be translated
# → New images will be automatically copied
```

### 4. Incremental Translation

```bash
# Step 1: Translate basic operations
python choreonoid_translator.py ja/basics en/basics --force

# Step 2: Translate installation guide
python choreonoid_translator.py ja/install en/install --force

# Step 3: Translate simulation guide
python choreonoid_translator.py ja/simulation en/simulation --force
```

## Features

### Main Features

1. **Hash-based Detection**: Accurately detects file content changes (regardless of Git status)
2. **Automatic Image Processing**: Detects image references in RST and copies images as needed
3. **Real-time Cost Tracking**: Displays and records API usage and costs in real-time
4. **Progress Management**: Automatically saves translation progress in `translation_progress.json`
5. **Incremental Translation**: Skips files with unchanged content
6. **Force Translation**: `--force` option to forcibly translate all files
7. **Error Handling**: Handles API limits and network errors
8. **Term Consistency**: Uses Choreonoid-specific terminology dictionary
9. **Logging**: Detailed logs in `translation.log`

### Real-time Cost Tracking

**Cost display during translation**:
```bash
# Example log during translation
INFO - API Usage - Input: 1,234 tokens, Output: 2,567 tokens, Cost: $0.0421
INFO - Total Cost so far: $1.2345

# Final summary
API Usage Summary:
Input tokens: 45,678
Output tokens: 123,456
Total tokens: 169,134
Estimated cost: $2.3456
Estimated cost (JPY): ¥352
```

**Cost check command**:
```bash
# Check current cumulative cost
python choreonoid_translator.py ja en --show-cost
```

**Records in progress file**:
```json
{
  "total_input_tokens": 45678,
  "total_output_tokens": 123456,
  "estimated_cost": 2.3456
}
```

### Automatic Image Processing

**Supported image reference formats**:
```rst
.. image:: images/screenshot.png
.. figure:: diagrams/flow.svg
:image: `icons/warning.png`
```

**Processing logic**:
1. Automatically detect image references in RST
2. Check if same-named image exists in `en` directory
3. If not found, copy corresponding image from `ja` directory
4. Maintain relative path structure

**Practical example**:
```bash
# ja/basics/images/screenshot.png exists
# en/basics/images/screenshot.png does not exist
# → Automatically copied

# en/basics/images/screenshot.png already exists
# → Skip copying (prioritize English version)
```

### Hash-based Detection Benefits

**Problems with traditional timestamp method**:
```bash
git clone repo  # All files get same timestamp
```

**New hash-based method**:
```bash
# Uses SHA256 hash of file content for detection
# → Detects even single character changes
# → Accurate regardless of Git status
```

**Practical example**:
```bash
# Edit file (not yet committed)
vim ja/basics/launch.rst

# Run translation
python choreonoid_translator.py ja en
# → Detects content change and runs translation
# → Can translate latest content before commit
```

### Terminology Customization

You can customize the terminology dictionary by editing the `self.glossary` dictionary in the script:

```python
self.glossary = {
    "Choreonoid": "Choreonoid",
    "プロジェクト": "project",
    "新しい用語": "new term",
    # Add additional terms
}
```

### Translation Style Adjustment

You can adjust the translation style by editing the prompt in the `_create_translation_prompt()` method.

### Cost Management Best Practices

1. **Pre-estimation**:
```bash
# Check unit cost with small-scale test
python choreonoid_translator.py ja en --max-files 3 --force
python choreonoid_translator.py ja en --show-cost
# → Estimate total cost from 3-file cost
```

2. **Incremental Execution**:
```bash
# Execute important sections first
python choreonoid_translator.py ja/basics en/basics --force
python choreonoid_translator.py ja en --show-cost  # Check cost

python choreonoid_translator.py ja/simulation en/simulation --force  
python choreonoid_translator.py ja en --show-cost  # Check again
```

3. **Budget Control**:
   - Set budget before running script
   - Regularly check with `--show-cost`
   - Stop process if approaching budget limit

## Troubleshooting

### API Rate Limit Error

```
anthropic.RateLimitError: Rate limit exceeded
```

→ Script automatically waits 60 seconds and retries

### Character Encoding Error

```
UnicodeDecodeError: 'utf-8' codec can't decode
```

→ File encoding may not be UTF-8. Re-save file as UTF-8.

### Memory Issues

If memory shortage occurs with large files:

1. Reduce `max_tokens` value
2. Split files for processing
3. Use `--max-files` to limit number of files processed

## Log Files and Debugging

### Log Files

- `translation.log`: Detailed execution log
- `translation_progress.json`: Translation progress status

### Debug Mode

For more detailed logs, change `level` in `logging.basicConfig` to `logging.DEBUG` in the script.

## Important Notes

1. **API Key Protection**: Use environment variables instead of hardcoding API keys
2. **Backup**: Backup important files before running translation
3. **Quality Review**: Always have humans review translation results
4. **Copyright**: Handle translation results with appropriate copyright considerations
