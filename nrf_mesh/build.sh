#!/bin/bash
# Build and flash script for nRF52840 mesh firmware
# Usage: ./build.sh [build|flash|clean|monitor]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
BOARD="xiao_ble"

# nRF Connect SDK location (adjust if different)
NCS_BASE="${NCS_BASE:-$HOME/ncs}"

# Source Zephyr environment
if [ -f "$NCS_BASE/zephyr/zephyr-env.sh" ]; then
    source "$NCS_BASE/zephyr/zephyr-env.sh"
else
    echo "Error: Zephyr not found at $NCS_BASE/zephyr"
    echo "Set NCS_BASE to your nRF Connect SDK location"
    exit 1
fi

# Find and set Zephyr SDK
ZEPHYR_SDK_DIR=$(ls -d ~/zephyr-sdk-* 2>/dev/null | head -1)
if [ -n "$ZEPHYR_SDK_DIR" ]; then
    export ZEPHYR_SDK_INSTALL_DIR="$ZEPHYR_SDK_DIR"
    export ZEPHYR_TOOLCHAIN_VARIANT="zephyr"
fi

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_help() {
    echo "nRF52840 Mesh Firmware Build Script"
    echo ""
    echo "Usage: ./build.sh [command]"
    echo ""
    echo "Commands:"
    echo "  build     Build the firmware (default)"
    echo "  flash     Build and flash to device"
    echo "  clean     Clean build directory"
    echo "  monitor   Open serial monitor (921600 baud)"
    echo "  help      Show this help"
}

do_build() {
    echo -e "${GREEN}Building nRF52840 mesh firmware...${NC}"
    cd "$SCRIPT_DIR"
    west build -b "$BOARD" -- -DBOARD_ROOT="$SCRIPT_DIR"
    echo -e "${GREEN}Build complete!${NC}"
    echo "Firmware: $BUILD_DIR/zephyr/zephyr.hex"
}

do_flash() {
    do_build
    echo -e "${YELLOW}Flashing to $BOARD...${NC}"
    
    # Try west flash first (works with J-Link, nrfjprog)
    if west flash 2>/dev/null; then
        echo -e "${GREEN}Flash complete!${NC}"
        return
    fi
    
    # Fallback: Try UF2 for XIAO bootloader
    UF2_FILE="$BUILD_DIR/zephyr/zephyr.uf2"
    if [ -f "$UF2_FILE" ]; then
        # Find the XIAO-SENSE block device
        XIAO_DEV=$(lsblk -rno NAME,LABEL 2>/dev/null | awk '$2=="XIAO-SENSE" {print "/dev/"$1}')
        if [ -z "$XIAO_DEV" ]; then
            echo -e "${RED}XIAO not in bootloader mode.${NC}"
            echo "Double-tap reset button to enter bootloader, then run again."
            exit 1
        fi

        # Check if already mounted, if not mount it
        XIAO_MOUNT=$(lsblk -rno MOUNTPOINT "$XIAO_DEV" 2>/dev/null)
        if [ -z "$XIAO_MOUNT" ]; then
            echo -e "${YELLOW}Mounting XIAO bootloader drive...${NC}"
            udisksctl mount -b "$XIAO_DEV" 2>&1
            # Re-read mount point after mounting
            XIAO_MOUNT=$(lsblk -rno MOUNTPOINT "$XIAO_DEV" 2>/dev/null)
        fi

        if [ -n "$XIAO_MOUNT" ] && [ -d "$XIAO_MOUNT" ]; then
            echo -e "${YELLOW}Copying UF2 to $XIAO_MOUNT ...${NC}"
            cp "$UF2_FILE" "$XIAO_MOUNT/"
            sync
            echo -e "${GREEN}Flash complete! Device will reboot.${NC}"
        else
            echo -e "${RED}Failed to find mount point for $XIAO_DEV${NC}"
            exit 1
        fi
    else
        echo -e "${RED}Flash failed. No programmer found and no UF2 available.${NC}"
        echo "Options:"
        echo "  1. Connect J-Link or use nrfjprog"
        echo "  2. Double-tap reset to enter XIAO bootloader mode"
        exit 1
    fi
}

do_clean() {
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
    echo -e "${GREEN}Clean complete!${NC}"
}

do_monitor() {
    # Find serial port
    PORT=$(ls /dev/ttyACM* 2>/dev/null | head -1 || true)
    if [ -z "$PORT" ]; then
        PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1 || true)
    fi
    
    if [ -z "$PORT" ]; then
        echo -e "${RED}No serial port found${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}Opening serial monitor on $PORT (921600 baud)${NC}"
    echo "Press Ctrl+C to exit"
    
    # Use screen, minicom, or python
    if command -v screen &>/dev/null; then
        screen "$PORT" 921600
    elif command -v minicom &>/dev/null; then
        minicom -D "$PORT" -b 921600
    elif command -v python3 &>/dev/null; then
        python3 -m serial.tools.miniterm "$PORT" 921600
    else
        echo -e "${RED}No serial terminal found. Install screen, minicom, or pyserial.${NC}"
        exit 1
    fi
}

# Main
case "${1:-build}" in
    build)
        do_build
        ;;
    flash)
        do_flash
        ;;
    clean)
        do_clean
        ;;
    monitor)
        do_monitor
        ;;
    help|--help|-h)
        print_help
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        print_help
        exit 1
        ;;
esac
