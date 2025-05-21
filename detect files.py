import os

def track_extensions_with_git_lfs(folder_path):
    extensions = set()

    # Recursively walk through all subdirectories and files
    for root, _, files in os.walk(folder_path):
        for file in files:
            _, ext = os.path.splitext(file)
            if ext:  # Only process files with an extension
                ext = ext.lower().lstrip(".")  # Normalize: lowercase, remove leading dot
                extensions.add(ext)

    # Print git lfs track commands for each unique extension
    for ext in sorted(extensions):
        print(f'git lfs track "*.{ext}"')

# Path to the root folder (Designs)
folder = "Designs"
track_extensions_with_git_lfs(folder)
