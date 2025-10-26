import os
import shutil

# Define folders
coding_folder = "Coding huh"
docs_folder = "Reports and Documents"
media_folder = "Photos and Videos"
ignore_folders = {"Deep Docs", "Designs"}

# Create folders if they don't exist
os.makedirs(coding_folder, exist_ok=True)
os.makedirs(docs_folder, exist_ok=True)
os.makedirs(media_folder, exist_ok=True)

# File type categories
coding_exts = {".py", ".cpp", ".exe"}
doc_exts = {".docx", ".pdf"}
media_exts = {
    ".jpg", ".jpeg", ".png", ".gif", ".bmp", ".tiff",
    ".mp4", ".mkv", ".avi", ".mov", ".wmv", ".flv"
}

# Get current directory
base_dir = os.getcwd()

for root, dirs, files in os.walk(base_dir):
    # Skip ignored folders
    if any(ignored in root for ignored in ignore_folders):
        continue

    for file in files:
        src_path = os.path.join(root, file)
        _, ext = os.path.splitext(file)
        ext = ext.lower()

        # Skip files already in target folders
        if any(folder in root for folder in [coding_folder, docs_folder, media_folder]):
            continue

        # Decide where to move
        if ext in coding_exts:
            shutil.move(src_path, os.path.join(coding_folder, file))
        elif ext in doc_exts:
            shutil.move(src_path, os.path.join(docs_folder, file))
        elif ext in media_exts:
            shutil.move(src_path, os.path.join(media_folder, file))
