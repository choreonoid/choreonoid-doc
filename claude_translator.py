#!/usr/bin/env python3
"""
Choreonoid Documentation Translator
Translates Choreonoid documentation from Japanese to English using Anthropic Claude API.

Repository structure:
- choreonoid-doc/
  ├── ja/           # Japanese source files
  │   ├── install/
  │   ├── basics/
  │   ├── handling-models/
  │   ├── simulation/
  │   ├── plugin-development/
  │   └── ...
  └── en/           # English target files (same structure as ja/)

GitHub repository: https://github.com/choreonoid/choreonoid-doc
"""

import os
import json
import hashlib
import argparse
import logging
import shutil
import time
import re
import yaml
from pathlib import Path
from typing import Dict, List, Set, Optional, Tuple
from datetime import datetime
import anthropic
from anthropic import Anthropic, APIError, APIConnectionError, RateLimitError

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('translation.log', encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Default terminology file name
DEFAULT_TERMINOLOGY_FILE = "choreonoid_terminology.yaml"

# Available models with their configurations
AVAILABLE_MODELS = {
    "sonnet": {
        "model_id": "claude-sonnet-4-20250514",  # Claude Sonnet 4
        "max_tokens": 64000,  # Sonnet 4 supports up to 64K output tokens
        "description": "Fast and balanced (default)"
    },
    "opus": {
        "model_id": "claude-opus-4-20250514",  # Claude Opus 4
        "max_tokens": 32000,  # Opus 4 supports up to 32K output tokens
        "description": "Highest quality, more accurate"
    }
}

# Default model
DEFAULT_MODEL = "sonnet"

# Default temperature
DEFAULT_TEMPERATURE = 0.0


def load_terminology(terminology_file: str = DEFAULT_TERMINOLOGY_FILE) -> Dict[str, str]:
    """Load terminology from YAML file."""
    if not os.path.exists(terminology_file):
        logger.info(f"Terminology file '{terminology_file}' not found. Proceeding without custom terminology.")
        return {}
    
    try:
        with open(terminology_file, 'r', encoding='utf-8') as f:
            content = f.read().strip()
            if not content:
                logger.info(f"Terminology file '{terminology_file}' is empty. Proceeding without custom terminology.")
                return {}
            
            terminology = yaml.safe_load(content)
            if not isinstance(terminology, dict):
                logger.warning(f"Terminology file should contain a dictionary. Found: {type(terminology)}. Proceeding without custom terminology.")
                return {}
            
            # Validate and convert all values to strings
            valid_terms = {}
            for ja, en in terminology.items():
                if isinstance(ja, str) and en is not None:
                    valid_terms[ja] = str(en)
                else:
                    logger.warning(f"Skipping invalid term: {ja} -> {en}")
            
            logger.info(f"Loaded {len(valid_terms)} terms from '{terminology_file}'")
            return valid_terms
            
    except yaml.YAMLError as e:
        logger.error(f"Error parsing YAML file '{terminology_file}': {e}")
        return {}
    except Exception as e:
        logger.error(f"Error loading terminology file '{terminology_file}': {e}")
        return {}


# Image file extensions
IMAGE_EXTENSIONS = {'.png', '.jpg', '.jpeg', '.gif', '.svg', '.bmp', '.webp'}

# reStructuredText image directives patterns
IMAGE_DIRECTIVE_PATTERNS = [
    re.compile(r'^\s*\.\.\s+image::\s+(.+)$', re.MULTILINE),
    re.compile(r'^\s*\.\.\s+figure::\s+(.+)$', re.MULTILINE)
]

# reStructuredText elements that should not be translated
RST_PRESERVE_PATTERNS = []


class TranslationProgress:
    """Manages translation progress tracking."""
    
    def __init__(self, progress_file: str = "translation_progress.json"):
        self.progress_file = progress_file
        self.data = self._load_progress()
    
    def _load_progress(self) -> Dict:
        """Load progress from JSON file."""
        if os.path.exists(self.progress_file):
            try:
                with open(self.progress_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
                # Validate and fix data structure
                if not isinstance(data, dict):
                    logger.warning(f"Invalid progress file format, starting fresh")
                    return self._create_empty_progress()
                    
                # Ensure all required fields are present and have correct types
                required_fields = {
                    "file_hashes": dict,
                    "completed_files": list,
                    "failed_files": list,
                    "translation_stats": dict
                }
                
                for field, expected_type in required_fields.items():
                    if field not in data or not isinstance(data[field], expected_type):
                        logger.warning(f"Invalid or missing field '{field}' in progress file, resetting to default")
                        data[field] = expected_type()
                
                # Ensure translation_stats has proper structure
                if "translation_stats" in data:
                    stats = data["translation_stats"]
                    if not isinstance(stats.get("files_per_directory"), dict):
                        stats["files_per_directory"] = {}
                    if not isinstance(stats.get("total_characters"), (int, float)):
                        stats["total_characters"] = 0
                    if not isinstance(stats.get("total_time_seconds"), (int, float)):
                        stats["total_time_seconds"] = 0
                
                return data
                    
            except json.JSONDecodeError:
                logger.warning(f"Invalid JSON in progress file {self.progress_file}, starting fresh")
                return self._create_empty_progress()
            except Exception as e:
                logger.warning(f"Error loading progress file: {e}, starting fresh")
                return self._create_empty_progress()
        return self._create_empty_progress()
    
    def _create_empty_progress(self) -> Dict:
        """Create empty progress structure."""
        return {
            "file_hashes": {},
            "completed_files": [],  # This must be a list, not an integer
            "failed_files": [],
            "last_updated": None,
            "total_files": 0,
            "completed_count": 0,  # This is the count as integer
            "translation_stats": {
                "total_characters": 0,
                "total_time_seconds": 0,
                "files_per_directory": {}
            }
        }
    
    def save(self):
        """Save progress to JSON file."""
        self.data["last_updated"] = datetime.now().isoformat()
        with open(self.progress_file, 'w', encoding='utf-8') as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)
    
    def get_file_hash(self, filepath: str) -> str:
        """Calculate SHA256 hash of file content."""
        with open(filepath, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    
    def needs_translation(self, filepath: str) -> bool:
        """Check if file needs translation based on content hash."""
        if "file_hashes" not in self.data:
            self.data["file_hashes"] = {}
            
        current_hash = self.get_file_hash(filepath)
        stored_hash = self.data["file_hashes"].get(filepath)
        return current_hash != stored_hash
    
    def mark_completed(self, filepath: str, char_count: int = 0, duration: float = 0):
        """Mark file as successfully translated."""
        # Ensure all required keys exist
        if "file_hashes" not in self.data:
            self.data["file_hashes"] = {}
        if "completed_files" not in self.data:
            self.data["completed_files"] = []
        if "failed_files" not in self.data:
            self.data["failed_files"] = []
        if "translation_stats" not in self.data:
            self.data["translation_stats"] = {
                "total_characters": 0,
                "total_time_seconds": 0,
                "files_per_directory": {}
            }
        
        self.data["file_hashes"][filepath] = self.get_file_hash(filepath)
        if filepath not in self.data["completed_files"]:
            self.data["completed_files"].append(filepath)
        if filepath in self.data["failed_files"]:
            self.data["failed_files"].remove(filepath)
        self.data["completed_count"] = len(self.data["completed_files"])
        
        # Update statistics
        self.data["translation_stats"]["total_characters"] += char_count
        self.data["translation_stats"]["total_time_seconds"] += duration
        
        # Track files per directory
        dir_name = os.path.dirname(filepath)
        if dir_name not in self.data["translation_stats"]["files_per_directory"]:
            self.data["translation_stats"]["files_per_directory"][dir_name] = 0
        self.data["translation_stats"]["files_per_directory"][dir_name] += 1
        
        self.save()
    
    def mark_failed(self, filepath: str):
        """Mark file as failed translation."""
        if "failed_files" not in self.data:
            self.data["failed_files"] = []
            
        if filepath not in self.data["failed_files"]:
            self.data["failed_files"].append(filepath)
        self.save()
    
    def get_statistics(self) -> str:
        """Get translation statistics summary."""
        if "translation_stats" not in self.data:
            return "No translation statistics available yet."
            
        stats = self.data["translation_stats"]
        total_chars = stats.get("total_characters", 0)
        total_time = stats.get("total_time_seconds", 0)
        
        if total_time > 0:
            chars_per_second = total_chars / total_time
            chars_per_minute = chars_per_second * 60
        else:
            chars_per_minute = 0
        
        summary = f"""
Translation Statistics:
- Total files translated: {self.data.get('completed_count', 0)}
- Total characters: {total_chars:,}
- Total time: {total_time/60:.1f} minutes
- Average speed: {chars_per_minute:.0f} characters/minute
- Files per directory:"""
        
        files_per_dir = stats.get("files_per_directory", {})
        for dir_name, count in sorted(files_per_dir.items()):
            summary += f"\n  - {dir_name}: {count} files"
        
        return summary


class ChoreonoidTranslator:
    """Main translator class for Choreonoid documentation."""
    
    def __init__(self, api_key: str, source_lang: str = "ja", target_lang: str = "en", 
                 terminology_file: str = DEFAULT_TERMINOLOGY_FILE, model: str = DEFAULT_MODEL,
                 temperature: float = DEFAULT_TEMPERATURE):
        self.client = Anthropic(api_key=api_key)
        self.source_lang = source_lang
        self.target_lang = target_lang
        self.progress = TranslationProgress()
        self.term_dict = load_terminology(terminology_file)
        self.temperature = temperature
        
        # Set model
        if model in AVAILABLE_MODELS:
            self.model_config = AVAILABLE_MODELS[model]
            self.model = self.model_config["model_id"]
            self.max_tokens = self.model_config["max_tokens"]
            self.model_name = model
            logger.info(f"Using model: {model} ({self.model}, max_tokens: {self.max_tokens})")
        else:
            logger.warning(f"Unknown model '{model}', using default: {DEFAULT_MODEL}")
            self.model_config = AVAILABLE_MODELS[DEFAULT_MODEL]
            self.model = self.model_config["model_id"]
            self.max_tokens = self.model_config["max_tokens"]
            self.model_name = DEFAULT_MODEL
    
    def find_rst_files(self, source_dir: Path) -> List[Path]:
        """Find all .rst files in source directory."""
        rst_files = []
        for root, _, files in os.walk(source_dir):
            for file in files:
                if file.endswith('.rst'):
                    file_path = Path(root) / file
                    rst_files.append(file_path)
                    logger.debug(f"Found RST file: {file_path}")
        
        # Sort with custom key: prioritize files by depth (root files first)
        # Then alphabetically within each depth level
        def sort_key(path: Path) -> tuple:
            # Count the depth from source_dir
            try:
                relative_path = path.relative_to(source_dir)
                depth = len(relative_path.parts) - 1  # -1 because the file itself is a part
                return (depth, str(relative_path).lower())
            except ValueError:
                # If path is not relative to source_dir, put it at the end
                return (999, str(path).lower())
        
        sorted_files = sorted(rst_files, key=sort_key)
        
        # Log files in root directory specifically
        root_files = [f for f in sorted_files if f.parent == source_dir]
        if root_files:
            logger.info(f"Found {len(root_files)} RST files in root directory: {[str(f) for f in root_files]}")
        
        return sorted_files
    
    def get_relative_path(self, file_path: Path, base_dir: Path) -> Path:
        """Get relative path from base directory."""
        return file_path.relative_to(base_dir)
    
    def create_target_path(self, source_file: Path, source_dir: Path, target_dir: Path) -> Path:
        """Create corresponding target file path."""
        relative_path = self.get_relative_path(source_file, source_dir)
        return target_dir / relative_path
    
    def extract_images_from_rst(self, content: str) -> List[str]:
        """Extract image references from RST content using regex."""
        images = []
        
        for pattern in IMAGE_DIRECTIVE_PATTERNS:
            matches = pattern.findall(content)
            for match in matches:
                # Clean up the image path
                image_path = match.strip()
                if image_path:
                    images.append(image_path)
        
        return images
    
    def copy_image_if_needed(self, image_path: str, source_rst: Path, source_dir: Path, target_dir: Path):
        """Copy image from source to target if it doesn't exist in target."""
        try:
            # Resolve image path relative to RST file
            rst_dir = source_rst.parent
            source_image = (rst_dir / image_path).resolve()
            
            # Check if it's within the source directory
            if not str(source_image).startswith(str(source_dir.resolve())):
                logger.warning(f"Image path outside source directory: {source_image}")
                return
            
            if not source_image.exists():
                logger.warning(f"Image not found: {source_image}")
                return
            
            # Calculate target image path
            relative_rst_path = self.get_relative_path(source_rst, source_dir)
            target_rst_dir = target_dir / relative_rst_path.parent
            target_image = (target_rst_dir / image_path).resolve()
            
            # Check if image already exists in target
            if target_image.exists():
                logger.debug(f"Image already exists in target, skipping: {target_image}")
                return
            
            # Create target directory if needed
            target_image.parent.mkdir(parents=True, exist_ok=True)
            
            # Copy image
            shutil.copy2(source_image, target_image)
            logger.info(f"Copied image: {source_image} -> {target_image}")
            
        except Exception as e:
            logger.error(f"Failed to copy image {image_path}: {e}")
    
    def preserve_rst_elements(self, content: str) -> Tuple[str, Dict[str, str]]:
        """This method is no longer needed as we rely on Claude's understanding of RST."""
        return content, {}
    
    def restore_rst_elements(self, content: str, preserved: Dict[str, str]) -> str:
        """This method is no longer needed as we rely on Claude's understanding of RST."""
        return content
    
    def build_translation_prompt(self, content: str) -> str:
        """Build translation prompt with Choreonoid-specific instructions."""
        terms_list = "\n".join([f"- {ja} → {en}" for ja, en in self.term_dict.items()])
        terms_section = ""
        if terms_list:
            terms_section = f"\nUse the following terminology consistently:\n{terms_list}\n"
        
        prompt = f"""Translate the following Choreonoid technical documentation from Japanese to English.

Important guidelines:
1. This is reStructuredText (RST) format documentation
2. Translate ONLY the natural language content (paragraphs, descriptions, explanations)
3. Do NOT translate:
   - RST directives (like .. toctree::, .. code-block::, .. image::, etc.)
   - RST roles (like :doc:, :ref:, :class:, etc.)
   - URLs and file paths (EXCEPT for Choreonoid documentation URLs - see point 7)
   - Code examples themselves (but DO translate Japanese comments within code blocks)
   - Command-line instructions
   - Option names and parameter names
4. IMPORTANT: DO translate RST comments (lines starting with .. followed by text that is not a directive)
   - For example: ".. これはコメントです" should be translated to ".. This is a comment"
   - Comments are different from directives - they contain explanatory text that should be translated
5. Maintain the exact RST structure, indentation, and formatting
6. Follow standard software user manual writing style:
   - Clear and concise technical language
   - Consistent terminology throughout
   - Appropriate level of formality for technical documentation
   - Ensure grammatical accuracy
7. Translate explanations in parentheses without omitting them
8. IMPORTANT: For URLs pointing to choreonoid.org, change language codes:
   - Replace "/ja/" with "/en/" in all choreonoid.org URLs
   - For example: https://choreonoid.org/ja/documents/ → https://choreonoid.org/en/documents/
   - This applies to any path component, not just at the beginning
{terms_section}
Content to translate:
---
{content}
---

Provide only the translated content without any additional explanation. Ensure the COMPLETE content is translated without truncation."""
        
        return prompt
    
    def translate_content(self, content: str, max_retries: int = 3) -> str:
        """Translate content using Claude API with retry logic."""
        prompt = self.build_translation_prompt(content)
        
        for attempt in range(max_retries):
            try:
                response = self.client.messages.create(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    messages=[
                        {
                            "role": "user",
                            "content": prompt
                        }
                    ],
                    timeout=600.0  # 10 minutes timeout for long translations
                )
                
                # Handle the response content properly
                if hasattr(response, 'content') and response.content:
                    if isinstance(response.content, list) and len(response.content) > 0:
                        translated = response.content[0].text
                    elif isinstance(response.content, str):
                        translated = response.content
                    else:
                        raise ValueError(f"Unexpected response content type: {type(response.content)}")
                else:
                    raise ValueError("Empty response from API")
                
                # Check if translation was cut off
                if '[Translation continues' in translated or \
                   (translated.strip().endswith('...') and 'つづく' not in content and '...' not in content):
                    logger.warning("Translation appears to be truncated. This might be a Claude safety feature.")
                    logger.info(f"Original content length: {len(content)} chars")
                    logger.info(f"Translated content length: {len(translated)} chars")
                
                return translated
                
            except RateLimitError as e:
                wait_time = min(60 * (2 ** attempt), 300)  # Exponential backoff, max 5 minutes
                logger.warning(f"Rate limit hit, waiting {wait_time} seconds...")
                time.sleep(wait_time)
                
            except (APIError, APIConnectionError) as e:
                logger.error(f"API error on attempt {attempt + 1}: {e}")
                if attempt < max_retries - 1:
                    time.sleep(10 * (attempt + 1))
                else:
                    raise
            except Exception as e:
                logger.error(f"Unexpected error on attempt {attempt + 1}: {e}", exc_info=True)
                if attempt < max_retries - 1:
                    time.sleep(5)
                else:
                    raise
        
        raise Exception("Max retries exceeded")
    
    def validate_rst_structure(self, original: str, translated: str) -> bool:
        """Basic validation to ensure RST structure is preserved."""
        try:
            # Check if major RST elements are preserved
            original_directives = len(re.findall(r'^\s*\.\.\s+\w+::', original, re.MULTILINE))
            translated_directives = len(re.findall(r'^\s*\.\.\s+\w+::', translated, re.MULTILINE))
            
            if abs(original_directives - translated_directives) > 2:  # Allow small differences
                logger.warning(f"Directive count mismatch: {original_directives} vs {translated_directives}")
                return False
            
            # Check if code blocks are preserved
            original_codeblocks = len(re.findall(r'::\s*\n', original))
            translated_codeblocks = len(re.findall(r'::\s*\n', translated))
            
            if original_codeblocks != translated_codeblocks:
                logger.warning(f"Code block count mismatch: {original_codeblocks} vs {translated_codeblocks}")
                return False
            
            return True
        except Exception as e:
            logger.warning(f"Error during RST validation: {e}")
            return True  # Don't fail translation due to validation errors
    
    def translate_file(self, source_file: Path, source_dir: Path, target_dir: Path, 
                      force: bool = False, copy_images_only: bool = False) -> bool:
        """Translate a single RST file."""
        start_time = time.time()
        
        try:
            # Check if translation is needed
            if not force and not self.progress.needs_translation(str(source_file)):
                logger.info(f"Skipping unchanged file: {source_file}")
                return True
            
            # Read source content
            with open(source_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            char_count = len(content)
            
            # Extract and copy images
            images = self.extract_images_from_rst(content)
            for image in images:
                self.copy_image_if_needed(image, source_file, source_dir, target_dir)
            
            if copy_images_only:
                return True
            
            # Skip empty files
            if not content.strip():
                logger.warning(f"Skipping empty file: {source_file}")
                return True
            
            # Translate content
            logger.info(f"Translating: {source_file} ({char_count} characters)")
            translated_content = self.translate_content(content)
            
            # Validate translation
            if not self.validate_rst_structure(content, translated_content):
                logger.warning(f"RST structure validation failed for {source_file}")
            
            # Create target file path
            target_file = self.create_target_path(source_file, source_dir, target_dir)
            target_file.parent.mkdir(parents=True, exist_ok=True)
            
            # Write translated content
            with open(target_file, 'w', encoding='utf-8') as f:
                f.write(translated_content)
            
            # Calculate duration
            duration = time.time() - start_time
            
            # Mark as completed
            self.progress.mark_completed(str(source_file), char_count, duration)
            logger.info(f"Successfully translated: {source_file} -> {target_file} ({duration:.1f}s)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to translate {source_file}: {e}", exc_info=True)
            self.progress.mark_failed(str(source_file))
            return False
    
    def translate_directory(self, source_dir: str, target_dir: str, 
                          force: bool = False, max_files: Optional[int] = None,
                          single_file: Optional[str] = None, resume: bool = False,
                          copy_images_only: bool = False):
        """Translate all RST files in directory."""
        source_path = Path(source_dir)
        target_path = Path(target_dir)
        
        if not source_path.exists():
            logger.error(f"Source directory does not exist: {source_dir}")
            return
        
        # Create target directory
        target_path.mkdir(parents=True, exist_ok=True)
        
        # Find files to translate
        if single_file:
            rst_files = [Path(single_file)]
        else:
            rst_files = self.find_rst_files(source_path)
        
        # Apply max_files limit if specified
        if max_files:
            rst_files = rst_files[:max_files]
        
        # Filter files if resuming
        if resume and not force:
            rst_files = [f for f in rst_files if str(f) not in self.progress.data["completed_files"]]
        
        # Update total files count
        self.progress.data["total_files"] = len(rst_files)
        self.progress.save()
        
        logger.info(f"Found {len(rst_files)} RST files to process")
        
        # Log directory structure
        directories = set(str(f.parent.relative_to(source_path)) for f in rst_files)
        logger.info(f"Directories to process: {sorted(directories)}")
        
        # Translate each file
        success_count = 0
        for i, rst_file in enumerate(rst_files, 1):
            logger.info(f"\nProcessing file {i}/{len(rst_files)}: {rst_file}")
            
            if self.translate_file(rst_file, source_path, target_path, force, copy_images_only):
                success_count += 1
            
            # Small delay to avoid hitting rate limits
            if not copy_images_only and i < len(rst_files):
                time.sleep(1)
        
        # Print statistics
        logger.info(f"\nTranslation completed: {success_count}/{len(rst_files)} files successful")
        logger.info(self.progress.get_statistics())


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Translate Choreonoid documentation from Japanese to English",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Initial full translation
  python claude_translator.py ja en --force
  
  # Translate only changed files
  python claude_translator.py ja en
  
  # Test with limited files
  python claude_translator.py ja en --max-files 3 --force
  
  # Copy images only
  python claude_translator.py ja en --copy-images
  
  # Translate single file
  python claude_translator.py ja en --single-file ja/install/build-ubuntu.rst
  
  # Resume interrupted translation
  python claude_translator.py ja en --resume
  
  # Use custom terminology file
  python claude_translator.py ja en --terminology my_terms.yaml
  
  # Use Opus model for higher quality
  python claude_translator.py ja en --single-file ja/install/build-ubuntu.rst --model opus
  
Repository structure:
  The script expects the Choreonoid documentation repository structure:
  - choreonoid-doc/
    ├── ja/     # Japanese source files
    └── en/     # English target files (created automatically)
        """
    )
    
    parser.add_argument('source_dir', help='Source directory (e.g., "ja")')
    parser.add_argument('target_dir', help='Target directory (e.g., "en")')
    parser.add_argument('--api-key', help='Anthropic API key (or set ANTHROPIC_API_KEY env var)')
    parser.add_argument('--force', action='store_true', help='Force translate all files')
    parser.add_argument('--max-files', type=int, help='Maximum number of files to translate (for testing)')
    parser.add_argument('--single-file', help='Translate only a single file')
    parser.add_argument('--resume', action='store_true', help='Resume from previous progress')
    parser.add_argument('--copy-images', action='store_true', help='Copy images only, skip translation')
    parser.add_argument('--terminology', default=DEFAULT_TERMINOLOGY_FILE, 
                       help=f'YAML file containing terminology mappings (default: {DEFAULT_TERMINOLOGY_FILE})')
    parser.add_argument('--model', default=DEFAULT_MODEL, 
                       choices=list(AVAILABLE_MODELS.keys()),
                       help=f'Model to use for translation (default: {DEFAULT_MODEL}). '
                            f'Options: sonnet (fast, default), opus (highest quality)')
    parser.add_argument('--temperature', type=float, default=DEFAULT_TEMPERATURE,
                       help=f'Temperature for response generation (default: {DEFAULT_TEMPERATURE}). '
                            'Lower values (0.0) for more deterministic output, '
                            'higher values (0.5+) for more variation')
    
    args = parser.parse_args()
    
    # Get API key
    api_key = args.api_key or os.environ.get('ANTHROPIC_API_KEY')
    if not api_key and not args.copy_images:
        logger.error("API key required. Use --api-key or set ANTHROPIC_API_KEY environment variable")
        return 1
    
    # Create translator instance
    translator = ChoreonoidTranslator(api_key, terminology_file=args.terminology, 
                                     model=args.model, temperature=args.temperature)
    
    try:
        translator.translate_directory(
            source_dir=args.source_dir,
            target_dir=args.target_dir,
            force=args.force,
            max_files=args.max_files,
            single_file=args.single_file,
            resume=args.resume,
            copy_images_only=args.copy_images
        )
        return 0
    except KeyboardInterrupt:
        logger.info("\nTranslation interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Translation failed: {e}")
        return 1


if __name__ == "__main__":
    exit(main())