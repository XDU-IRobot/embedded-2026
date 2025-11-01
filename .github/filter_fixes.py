#!/usr/bin/env python3
"""
Filter clang-tidy fixes.yaml to exclude issues from external libraries.
Only keep diagnostics from project source files.
"""

import sys
import os
import yaml
from pathlib import Path


def should_include_diagnostic(diagnostic, project_root, include_patterns, exclude_patterns):
    """
    Check if a diagnostic should be included based on file path patterns.

    Args:
        diagnostic: The diagnostic entry from YAML
        project_root: Root directory of the project
        include_patterns: List of path patterns to include (e.g., ['app/'])
        exclude_patterns: List of path patterns to exclude (e.g., ['libs/', 'Drivers/'])

    Returns:
        bool: True if diagnostic should be included
    """
    file_path = diagnostic.get('DiagnosticMessage', {}).get('FilePath', '')

    # Skip empty file paths
    if not file_path:
        return False

    # Convert to relative path if absolute
    try:
        rel_path = os.path.relpath(file_path, project_root)
    except ValueError:
        # If relative path fails (different drives on Windows), use absolute
        rel_path = file_path

    # Check exclude patterns first (higher priority)
    for pattern in exclude_patterns:
        if pattern in rel_path:
            return False

    # Check include patterns
    if not include_patterns:
        # If no include patterns specified, include everything not excluded
        return True

    for pattern in include_patterns:
        if pattern in rel_path:
            return True

    return False


def filter_fixes_yaml(input_file, output_file, project_root, include_patterns, exclude_patterns):
    """
    Filter a clang-tidy fixes YAML file.

    Args:
        input_file: Path to input YAML file
        output_file: Path to output YAML file
        project_root: Root directory of the project
        include_patterns: List of path patterns to include
        exclude_patterns: List of path patterns to exclude
    """
    try:
        with open(input_file, 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Error: File not found: {input_file}", file=sys.stderr)
        return False
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}", file=sys.stderr)
        return False

    if not data or 'Diagnostics' not in data:
        print("Warning: No diagnostics found in input file", file=sys.stderr)
        return True

    original_count = len(data['Diagnostics'])

    # Filter diagnostics
    filtered_diagnostics = [
        d for d in data['Diagnostics']
        if should_include_diagnostic(d, project_root, include_patterns, exclude_patterns)
    ]

    data['Diagnostics'] = filtered_diagnostics
    filtered_count = len(filtered_diagnostics)

    # Write output
    try:
        with open(output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    except Exception as e:
        print(f"Error writing output file: {e}", file=sys.stderr)
        return False

    print(f"Filtered {original_count} diagnostics -> {filtered_count} diagnostics")
    print(f"Removed {original_count - filtered_count} diagnostics from external libraries")

    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: filter-fixes.py <input.yaml> [output.yaml]")
        print("  If output.yaml is not specified, will overwrite input file")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file

    # Get project root (assuming script is in project root)
    project_root = os.path.dirname(os.path.abspath(__file__))

    # Define include/exclude patterns
    # Only include diagnostics from app/ directory
    include_patterns = [
        'app/',
    ]

    # Exclude external libraries and system headers
    exclude_patterns = [
        'libs/librm/libs/',  # External libraries in librm
        'libs/librm/src/librm/core/',  # Core librm (you might want to include this)
        'Drivers/',  # STM32 HAL drivers
        'Middlewares/',  # ST middleware
        'Core/',  # STM32Cube generated code
        'USB_DEVICE/',  # USB device library
        '/usr/',  # System headers
        'arm-none-eabi',  # ARM toolchain headers
    ]

    print(f"Filtering {input_file}...")
    print(f"Project root: {project_root}")
    print(f"Include patterns: {include_patterns}")
    print(f"Exclude patterns: {exclude_patterns}")
    print()

    success = filter_fixes_yaml(input_file, output_file, project_root, include_patterns, exclude_patterns)

    if success:
        print(f"✓ Filtered results written to: {output_file}")
        sys.exit(0)
    else:
        print("✗ Failed to filter fixes", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
