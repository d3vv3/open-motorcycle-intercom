#!/bin/bash
# OMI Development Helper Script
# Builds, flashes to both ESP32 boards, and monitors both outputs

set -e  # Exit on error

# Configuration
BOARD1_PORT="/dev/ttyACM0"
BOARD2_PORT="/dev/ttyACM1"
BAUD_RATE=115200

# Colors for output
COLOR_RESET="\033[0m"
COLOR_GREEN="\033[0;32m"
COLOR_BLUE="\033[0;34m"
COLOR_YELLOW="\033[0;33m"
COLOR_RED="\033[0;31m"
COLOR_CYAN="\033[0;36m"

# Function to print colored messages
print_msg() {
    echo -e "${2}${1}${COLOR_RESET}"
}

# Function to check if devices exist
check_devices() {
    print_msg "Checking for ESP32 devices..." "$COLOR_CYAN"
    
    if [ ! -e "$BOARD1_PORT" ]; then
        print_msg "ERROR: Board 1 not found at $BOARD1_PORT" "$COLOR_RED"
        exit 1
    fi
    
    if [ ! -e "$BOARD2_PORT" ]; then
        print_msg "ERROR: Board 2 not found at $BOARD2_PORT" "$COLOR_RED"
        exit 1
    fi
    
    print_msg "✓ Both boards detected" "$COLOR_GREEN"
}

# Function to setup ESP-IDF environment
setup_idf() {
    print_msg "Setting up ESP-IDF environment..." "$COLOR_CYAN"
    
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        source "$HOME/esp/esp-idf/export.sh" > /dev/null 2>&1
        print_msg "✓ ESP-IDF environment loaded" "$COLOR_GREEN"
    else
        print_msg "ERROR: ESP-IDF not found at $HOME/esp/esp-idf" "$COLOR_RED"
        exit 1
    fi
}

# Function to build the project
build_project() {
    print_msg "Building project..." "$COLOR_CYAN"
    idf.py build
    print_msg "✓ Build complete" "$COLOR_GREEN"
}

# Function to flash a board
flash_board() {
    local port=$1
    local board_name=$2
    
    print_msg "Flashing $board_name ($port)..." "$COLOR_YELLOW"
    idf.py -p "$port" flash
    print_msg "✓ $board_name flashed successfully" "$COLOR_GREEN"
}

# Function to monitor both boards
monitor_both() {
    print_msg "Starting dual monitor (Ctrl+C to exit)..." "$COLOR_CYAN"
    echo ""
    
    # Kill any existing cat processes on these ports
    pkill -f "cat $BOARD1_PORT" 2>/dev/null || true
    pkill -f "cat $BOARD2_PORT" 2>/dev/null || true
    
    # Reset serial ports
    stty -F "$BOARD1_PORT" "$BAUD_RATE" raw -echo 2>/dev/null || true
    stty -F "$BOARD2_PORT" "$BAUD_RATE" raw -echo 2>/dev/null || true
    
    # Monitor both boards with colored output
    (
        while IFS= read -r line; do
            echo -e "${COLOR_BLUE}[BOARD1]${COLOR_RESET} $line"
        done < "$BOARD1_PORT"
    ) &
    local pid1=$!
    
    (
        while IFS= read -r line; do
            echo -e "${COLOR_GREEN}[BOARD2]${COLOR_RESET} $line"
        done < "$BOARD2_PORT"
    ) &
    local pid2=$!
    
    # Wait for Ctrl+C
    trap "kill $pid1 $pid2 2>/dev/null; exit 0" INT TERM
    wait
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  all       - Build, flash both boards, and monitor (default)"
    echo "  build     - Build only"
    echo "  flash     - Flash both boards (no build)"
    echo "  flash1    - Flash board 1 only"
    echo "  flash2    - Flash board 2 only"
    echo "  monitor   - Monitor both boards"
    echo "  help      - Show this help"
    echo ""
    echo "Configuration:"
    echo "  Board 1: $BOARD1_PORT"
    echo "  Board 2: $BOARD2_PORT"
    echo "  Baud:    $BAUD_RATE"
}

# Main script
main() {
    local command="${1:-all}"
    
    print_msg "=== OMI Development Script ===" "$COLOR_CYAN"
    echo ""
    
    case "$command" in
        all)
            check_devices
            setup_idf
            build_project
            echo ""
            flash_board "$BOARD1_PORT" "Board 1"
            echo ""
            flash_board "$BOARD2_PORT" "Board 2"
            echo ""
            sleep 2  # Give boards time to boot
            monitor_both
            ;;
        
        build)
            setup_idf
            build_project
            ;;
        
        flash)
            check_devices
            setup_idf
            flash_board "$BOARD1_PORT" "Board 1"
            echo ""
            flash_board "$BOARD2_PORT" "Board 2"
            ;;
        
        flash1)
            check_devices
            setup_idf
            flash_board "$BOARD1_PORT" "Board 1"
            ;;
        
        flash2)
            check_devices
            setup_idf
            flash_board "$BOARD2_PORT" "Board 2"
            ;;
        
        monitor)
            check_devices
            monitor_both
            ;;
        
        help|--help|-h)
            show_usage
            ;;
        
        *)
            print_msg "ERROR: Unknown command '$command'" "$COLOR_RED"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

# Run main function
main "$@"
