#!/bin/bash
# Script to run clang-tidy on the project with correct configuration
# Usage: ./run-clang-tidy.sh [--export]
#   --export: Export results to clang-tidy-result/ directory

BUILD_DIR="./build/Debug"
COMPILE_DB="$BUILD_DIR/compile_commands.json"
EXPORT_RESULTS=false
RESULT_DIR="clang-tidy-result"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --export)
            EXPORT_RESULTS=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--export]"
            echo "  --export: Export results to $RESULT_DIR/ directory"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if compile_commands.json exists
if [ ! -f "$COMPILE_DB" ]; then
    echo "Error: compile_commands.json not found at $COMPILE_DB"
    echo "Please build the project first with: cmake --build build/Debug"
    exit 1
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}Running clang-tidy on project files...${NC}"

# Find all C++ source files (not headers, they will be checked when included)
CPP_FILES=$(find ./app -type f \( -name "*.cc" -o -name "*.cpp" \) 2>/dev/null)

if [ -z "$CPP_FILES" ]; then
    echo -e "${RED}No C++ source files found in ./app${NC}"
    exit 1
fi

echo -e "${YELLOW}Found $(echo "$CPP_FILES" | wc -l) C++ source files${NC}"
echo ""

# Detect ARM GCC toolchain include paths automatically
echo -e "${YELLOW}Detecting ARM GCC toolchain include paths...${NC}"

# Check if arm-none-eabi-g++ is available
if ! command -v arm-none-eabi-g++ &> /dev/null; then
    echo -e "${RED}Error: arm-none-eabi-g++ not found in PATH${NC}"
    echo "Please install ARM GCC toolchain"
    exit 1
fi

# Get the include paths from the compiler
INCLUDE_PATHS=$(arm-none-eabi-g++ -E -x c++ - -v < /dev/null 2>&1 | \
    sed -n '/#include <...> search starts here:/,/End of search list./p' | \
    grep '^ ' | \
    sed 's/^[[:space:]]*//' | \
    grep -v '^End of search list')

if [ -z "$INCLUDE_PATHS" ]; then
    echo -e "${RED}Error: Failed to detect ARM GCC include paths${NC}"
    exit 1
fi

# Build extra arguments for clang-tidy
EXTRA_ARGS=""
while IFS= read -r path; do
    if [ -d "$path" ]; then
        EXTRA_ARGS="$EXTRA_ARGS --extra-arg=-isystem$path"
        echo -e "${GREEN}  Found: $path${NC}"
    fi
done <<< "$INCLUDE_PATHS"

if [ -z "$EXTRA_ARGS" ]; then
    echo -e "${RED}Error: No valid include paths found${NC}"
    exit 1
fi

echo ""

# Prepare export directory if needed
if [ "$EXPORT_RESULTS" = true ]; then
    echo -e "${YELLOW}Export mode enabled. Results will be saved to $RESULT_DIR/${NC}"
    rm -rf "$RESULT_DIR"
    mkdir -p "$RESULT_DIR"
    FIXES_FILE="$RESULT_DIR/fixes.yaml"
    REPORT_FILE="$RESULT_DIR/report.txt"
    echo "Clang-Tidy Analysis Report" > "$REPORT_FILE"
    echo "Generated: $(date)" >> "$REPORT_FILE"
    echo "======================================" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
fi

# Run clang-tidy on each file
ERROR_COUNT=0
WARNING_COUNT=0
FILE_COUNT=0

for file in $CPP_FILES; do
    FILE_COUNT=$((FILE_COUNT + 1))
    echo -e "${YELLOW}Checking [$FILE_COUNT/$(echo "$CPP_FILES" | wc -l)]: $file${NC}"
    
    # Prepare clang-tidy command
    TIDY_CMD="clang-tidy -p $BUILD_DIR $EXTRA_ARGS"
    
    # Add export-fixes option if exporting
    if [ "$EXPORT_RESULTS" = true ]; then
        FILE_BASENAME=$(basename "$file" .cc)
        FILE_BASENAME=$(basename "$FILE_BASENAME" .cpp)
        FIXES_YAML="$RESULT_DIR/${FILE_BASENAME}.yaml"
        TIDY_CMD="$TIDY_CMD --export-fixes=$FIXES_YAML"
    fi
    
    # Run clang-tidy and capture output
    OUTPUT=$($TIDY_CMD "$file" 2>&1)
    EXIT_CODE=$?
    
    # Count errors and warnings
    FILE_ERRORS=$(echo "$OUTPUT" | grep -c "error:" || true)
    FILE_WARNINGS=$(echo "$OUTPUT" | grep -c "warning:" || true)
    
    # Save to report file if exporting
    if [ "$EXPORT_RESULTS" = true ]; then
        echo "=== $file ===" >> "$REPORT_FILE"
        echo "$OUTPUT" >> "$REPORT_FILE"
        echo "" >> "$REPORT_FILE"
    fi
    
    if [ $EXIT_CODE -ne 0 ] || [ $FILE_ERRORS -gt 0 ]; then
        ERROR_COUNT=$((ERROR_COUNT + 1))
        echo -e "${RED}✗ Found issues${NC}"
        if [ "$EXPORT_RESULTS" = false ]; then
            echo "$OUTPUT"
        fi
        echo ""
    elif [ $FILE_WARNINGS -gt 0 ]; then
        WARNING_COUNT=$((WARNING_COUNT + 1))
        echo -e "${YELLOW}⚠ Warnings found${NC}"
        if [ "$EXPORT_RESULTS" = false ]; then
            echo "$OUTPUT"
        fi
        echo ""
    else
        echo -e "${GREEN}✓ OK${NC}"
        echo ""
    fi
done

echo -e "${GREEN}=== Summary ===${NC}"
echo -e "Files checked: $(echo "$CPP_FILES" | wc -l)"
echo -e "${RED}Files with errors: $ERROR_COUNT${NC}"
echo -e "${YELLOW}Files with warnings: $WARNING_COUNT${NC}"

if [ "$EXPORT_RESULTS" = true ]; then
    echo ""
    echo -e "${GREEN}Results exported to:${NC}"
    echo -e "  Report: ${YELLOW}$REPORT_FILE${NC}"
    
    # Merge all YAML fixes into one file
    YAML_FILES=$(find "$RESULT_DIR" -name "*.yaml" -type f 2>/dev/null)
    if [ -n "$YAML_FILES" ]; then
        echo "---" > "$FIXES_FILE"
        echo "MainSourceFile: ''" >> "$FIXES_FILE"
        echo "Diagnostics:" >> "$FIXES_FILE"
        
        for yaml_file in $YAML_FILES; do
            if [ -s "$yaml_file" ]; then
                # Extract diagnostics and remove document separators (--- and ...)
                sed -n '/^Diagnostics:/,$ p' "$yaml_file" | \
                    tail -n +2 | \
                    grep -v '^\.\.\.$' | \
                    grep -v '^---$' >> "$FIXES_FILE"
            fi
            rm "$yaml_file"
        done
        
        # Add YAML document end marker
        echo "..." >> "$FIXES_FILE"
        
        echo -e "  Fixes:  ${YELLOW}$FIXES_FILE${NC}"
        
        # Filter fixes to exclude external libraries
        echo ""
        echo -e "${YELLOW}Filtering diagnostics to exclude external libraries...${NC}"
        
        if command -v python3 &> /dev/null; then
            FIXES_FILE_UNFILTERED="$RESULT_DIR/fixes-unfiltered.yaml"
            mv "$FIXES_FILE" "$FIXES_FILE_UNFILTERED"
            
            if python3 .github/filter-fixes.py "$FIXES_FILE_UNFILTERED" "$FIXES_FILE"; then
                echo -e "${GREEN}✓ Filtered fixes saved to: $FIXES_FILE${NC}"
                echo -e "  Unfiltered version: ${YELLOW}$FIXES_FILE_UNFILTERED${NC}"
            else
                echo -e "${RED}✗ Filtering failed, keeping unfiltered version${NC}"
                mv "$FIXES_FILE_UNFILTERED" "$FIXES_FILE"
            fi
        else
            echo -e "${YELLOW}⚠ Python3 not found, skipping filtering${NC}"
        fi
        
        echo ""
        echo -e "${CYAN}To apply fixes, run:${NC}"
        echo -e "  clang-apply-replacements $RESULT_DIR"
    else
        echo -e "  ${GREEN}No fixes needed${NC}"
    fi
fi

if [ $ERROR_COUNT -gt 0 ]; then
    exit 1
fi
