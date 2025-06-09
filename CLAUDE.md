# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is the official documentation repository for Choreonoid, a robot simulation software. The documentation is written in reStructuredText format and built using Sphinx.

## Build Commands

### Building Documentation
```bash
# Build English documentation
cd en/
make html

# Build Japanese documentation  
cd ja/
make html

# Alternative build method (outputs to external directory)
cd en/
./make.sh

cd ja/
./make.sh
```

### Clean Build
```bash
cd en/  # or ja/
make clean
make html
```

## Documentation Structure

The repository maintains parallel English (`en/`) and Japanese (`ja/`) documentation with identical directory structures:

- **basics/** - Getting started with Choreonoid interface
- **install/** - Installation guides for various platforms
- **handling-models/** - Working with robot models and kinematics
- **simulation/** - Physics simulation tutorials and concepts
- **plugin-development/** - Creating Choreonoid plugins
- **ros/** and **ros2/** - ROS integration guides
- **agxdynamics/** - AGX Dynamics physics engine integration
- **tips/** - Advanced usage tips and troubleshooting

## Key Technical Details

### Sphinx Configuration
- Configuration file: `en/conf.py` and `ja/conf.py`
- Theme: "nature"
- Version tracking: "master" branch
- Custom templates in `_templates/` for navigation and search

### Image Management
- Images stored in `images/` subdirectories within each topic folder
- Formats: PNG, SVG, JPG
- SVG source files often included for diagrams

### Cross-Language Consistency
When updating documentation:
1. Changes should typically be made to both English and Japanese versions
2. Image files are duplicated between languages (not shared)
3. File structure must remain identical between `en/` and `ja/`

### Deployment
- Japanese docs have an upload script: `upload-ja.sh`
- Uses rsync to deploy to web server
- Built documentation goes to `_build/html/`

## Common Tasks

### Adding a New Page
1. Create `.rst` file in appropriate section directory
2. Add to `index.rst` toctree in that section
3. Create corresponding file in other language directory
4. Add any images to the section's `images/` folder

### Updating Existing Content
1. Edit the `.rst` file directly
2. If adding images, place them in the section's `images/` directory
3. Remember to update both language versions

### Testing Changes Locally
```bash
cd en/  # or ja/
make html
# Open _build/html/index.html in a browser
```

## Translation Guidelines

### Japanese to English Translation Instructions

When translating Choreonoid documentation from Japanese to English, follow these guidelines:

1. **This is reStructuredText (RST) format documentation**
2. **Translate ONLY the natural language content** (paragraphs, descriptions, explanations)
3. **Do NOT translate**:
   - RST directives (like `.. toctree::`, `.. code-block::`, `.. image::`, etc.)
   - RST roles (like `:doc:`, `:ref:`, `:class:`, etc.)
   - URLs and file paths (EXCEPT for Choreonoid documentation URLs - see point 7)
   - Code examples themselves (but DO translate Japanese comments within code blocks)
   - Command-line instructions
   - Option names and parameter names
4. **IMPORTANT: DO translate RST comments** (lines starting with `.. ` followed by text that is not a directive)
   - For example: `.. これはコメントです` should be translated to `.. This is a comment`
   - Comments are different from directives - they contain explanatory text that should be translated
   - **Special translation instructions**: Comments starting with `.. 英訳指示：` contain specific translation instructions that must be followed
     - These instructions can specify:
       - Exact English text for UI messages, error messages, or software-generated text
       - Preferred English expressions or terminology
       - Any other translation-related guidance
     - Example: `.. 英訳指示：上のメッセージはLoading Body model "/usr/local/share/choreonoid-1.6/PA10/PA10.body" -> Completed!としてください。`
     - When encountering these instructions, follow them precisely instead of creating your own translation
     - **IMPORTANT: Do NOT include the `.. 英訳指示：` comments themselves in the translated output** - they are instructions only
5. **Maintain the exact RST structure, indentation, and formatting**
   - **CRITICAL: Never change RST formatting styles** (e.g., if the Japanese version uses bullet points for function descriptions, do NOT change them to code blocks or any other format)
   - Preserve RST structural elements (headers, lists, code blocks, directives) exactly as they appear in the source
   - This applies to RST markup structure only - the actual English text content should still be written naturally
6. **Follow standard software user manual writing style**:
   - Clear and concise technical language
   - Consistent terminology throughout
   - Appropriate level of formality for technical documentation
   - Ensure grammatical accuracy
   - When direct translation would make English sentences unclear, prioritize natural and comprehensible English over literal translation
7. **Translate explanations in parentheses without omitting them**
8. **IMPORTANT: For URLs pointing to choreonoid.org, change language codes**:
   - Replace `/ja/` with `/en/` in all choreonoid.org URLs
   - For example: `https://choreonoid.org/ja/documents/` → `https://choreonoid.org/en/documents/`
   - This applies to any path component, not just at the beginning
9. **Quality of existing translations**: If existing English translations are not natural or appropriate, you should improve them rather than maintaining consistency with poor translations. Prioritize clear, natural English that follows standard technical documentation conventions

### Image File Handling

When translating RST files, images referenced in the source file need to be properly handled:

1. **Extract image references** from RST directives like `.. image::` and `.. figure::`
2. **Check if the image already exists** in the target directory (e.g., `en/handling-models/images/`)
3. **If the image exists** in the target directory, keep the image path as-is (the English version takes precedence)
4. **If the image doesn't exist** in the target directory:
   - Copy the image file from the Japanese source directory to the English target directory
   - Maintain the same relative path structure
   - Use the Bash tool with `cp` command to copy image files
5. **Image formats to handle**: .png, .jpg, .jpeg, .gif, .svg, .bmp, .webp

### Translation Process

1. **Read the entire source file first** to understand context
2. **Check if choreonoid_terminology.yaml exists** in the repository root - if it does, use it for consistent terminology
3. **Extract and handle images** as described above before saving the translated content
4. **Preserve all RST markup** exactly as it appears
5. **Validate RST structure** after translation
6. **Ensure completeness** - translate the entire content without truncation

### Example Translation Request

To request translation, use phrases like:
- "ja/basics/index.rstを英語に翻訳してください"
- "このファイルを翻訳して、en/ディレクトリに保存してください"
- "ja/install/ディレクトリのすべてのファイルを翻訳"

## Git Commit Guidelines

When creating git commits:
- Use simple, concise commit messages
- Do not include Claude Code attribution or co-author information
- Focus on what was changed, not how it was done
- Keep messages brief but descriptive enough to understand the change

## Important Notes

- This is documentation only - no Choreonoid source code
- Git status shows some untracked files for new/incomplete pages
- Current branch: master (primary branch for PRs)