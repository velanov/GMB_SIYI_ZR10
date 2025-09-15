#!/bin/bash
# Start the modern PySide6 Gimbal application

# Navigate to project directory
cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

# Check if we have a display
if [ -z "$DISPLAY" ]; then
    echo "No display detected. If running over SSH, try:"
    echo "ssh -X username@hostname"
    echo "or run with: DISPLAY=:0 $0"
    exit 1
fi

# Check for required packages
if ! command -v python3 &> /dev/null; then
    echo "Python3 not found. Please install Python3."
    exit 1
fi


# Try xcb with system plugins first (this shows actual GUI windows)
echo "Trying Qt platform: xcb with system plugins"
if QT_IM_MODULE="" QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt6/plugins QT_QPA_PLATFORM=xcb python3 run_pyside6_app.py; then
    echo "Success with xcb platform!"
    exit 0
fi

echo "xcb failed, trying minimal (headless but stable)..."

# Try minimal as backup (no GUI but stable)
echo "Trying Qt platform: minimal (no GUI window)"
if QT_QPA_PLATFORM=minimal python3 run_pyside6_app.py; then
    echo "App running in headless mode - no GUI window visible"
    exit 0
fi

echo "All stable platforms failed, trying problematic ones with short timeout..."

# Last resort: try xcb and wayland with very short timeouts
for platform in xcb wayland; do
    echo "Trying Qt platform: $platform (short timeout)"
    if timeout 5s bash -c "QT_QPA_PLATFORM=$platform python3 run_pyside6_app.py"; then
        echo "Success with platform: $platform"
        break
    else
        echo "Platform $platform failed quickly"
        pkill -f "python3 run_pyside6_app.py" 2>/dev/null || true
    fi
done