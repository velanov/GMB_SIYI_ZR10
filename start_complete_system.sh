#!/bin/bash
# Complete System Startup Script
# Starts PX4 SITL, Gimbal Control Application, and QGroundControl

echo "=========================================="
echo "Starting Complete Gimbal System"
echo "=========================================="

# Navigate to project directory
cd "$(dirname "$0")"
PROJECT_DIR=$(pwd)

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo "=========================================="
    echo "Shutting down all processes..."
    echo "=========================================="
    
    # Kill PX4 processes
    echo "Stopping PX4..."
    pkill -f "px4" 2>/dev/null || true
    pkill -f "gazebo" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    
    # Kill QGroundControl
    echo "Stopping QGroundControl..."
    pkill -f "QGroundControl" 2>/dev/null || true
    pkill -f "qgroundcontrol" 2>/dev/null || true
    
    # Kill Gimbal app and terminals
    echo "Stopping Gimbal Application..."
    pkill -f "run_pyside6_app.py" 2>/dev/null || true
    pkill -f "main_app_pyside.py" 2>/dev/null || true
    pkill -f "Gimbal Control Application" 2>/dev/null || true
    
    echo "All processes stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check for required commands
check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo "ERROR: $1 not found. Please install $1 first."
        exit 1
    fi
}

echo "Checking required software..."

# Check for Python
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python3 not found. Please install Python3."
    exit 1
fi

# Set PX4 path
PX4_DIR="/home/zagros"

if [ -d "$PX4_DIR" ]; then
    echo "Found PX4 at: $PX4_DIR"
else
    echo "ERROR: PX4 directory not found at: $PX4_DIR"
    exit 1
fi

# Set QGroundControl path
QGC_PATH="/home/zagros/QGroundControl-x86_64.AppImage"

if [ -f "$QGC_PATH" ] && [ -x "$QGC_PATH" ]; then
    echo "Found QGroundControl at: $QGC_PATH"
else
    echo "ERROR: QGroundControl not found at: $QGC_PATH"
    exit 1
fi

# Check if gimbal app exists
if [ ! -f "$PROJECT_DIR/run_pyside6_app.py" ]; then
    echo "ERROR: Gimbal application (run_pyside6_app.py) not found in project directory."
    echo "Project directory: $PROJECT_DIR"
    exit 1
fi

echo ""
echo "=========================================="
echo "Starting Services..."
echo "=========================================="

# 1. Start Gimbal Application FIRST
echo "1. Starting Gimbal Control Application..."
cd "$PROJECT_DIR"

# Check if virtual environment exists
if [ -d "venv" ]; then
    echo "   Activating virtual environment..."
    source venv/bin/activate
fi

# Start applications in terminal with tabs
echo "   Opening terminal with multiple tabs..."

# Create the terminal with the first tab (Gimbal Application)
gnome-terminal \
    --tab --title="Gimbal App" -- bash -c "cd '$PROJECT_DIR'; source venv/bin/activate 2>/dev/null || true; python3 run_pyside6_app.py; echo 'Gimbal app closed. Press Enter to close tab.'; read" \
    --tab --title="PX4 SITL" -- bash -c "sleep 5; cd '$PX4_DIR'; export PX4_HOME_LAT=47.4085; export PX4_HOME_LON=8.5490; export PX4_HOME_ALT=500; make px4_sitl gz_rc_cessna; echo 'PX4 closed. Press Enter to close tab.'; read" \
    --tab --title="QGroundControl" -- bash -c "sleep 20; '$QGC_PATH'; echo 'QGC closed. Press Enter to close tab.'; read" &

TERMINAL_PID=$!
echo "   Terminal with tabs started (PID: $TERMINAL_PID)"

# Give a moment for the terminal to start
sleep 2

# Get actual process PIDs by finding the running processes
echo "   Finding process PIDs..."
sleep 3

# Find gimbal app PID
GIMBAL_PID=$(pgrep -f "run_pyside6_app.py" | head -1)
if [ -n "$GIMBAL_PID" ]; then
    echo "   Gimbal Application running (PID: $GIMBAL_PID)"
else
    echo "   Gimbal Application starting..."
    GIMBAL_PID="starting"
fi

sleep 12  # Wait for PX4 to start

# Find PX4 PID
PX4_PID=$(pgrep -f "px4" | head -1)
if [ -n "$PX4_PID" ]; then
    echo "   PX4 SITL running (PID: $PX4_PID)"
else
    echo "   PX4 SITL starting..."
    PX4_PID="starting"
fi

sleep 8  # Wait for QGC to start

# Find QGC PID
QGC_PID=$(pgrep -f "QGroundControl" | head -1)
if [ -n "$QGC_PID" ]; then
    echo "   QGroundControl running (PID: $QGC_PID)"
else
    echo "   QGroundControl starting..."
    QGC_PID="starting"
fi

echo ""
echo "=========================================="
echo "System Startup Complete!"
echo "=========================================="
echo "Services started:"
echo "  ✓ Gimbal Application: Tab 1"
echo "  ✓ PX4 SITL: Tab 2" 
echo "  ✓ QGroundControl: Tab 3"
echo ""
echo "All applications are running in separate tabs."
echo "Press Ctrl+C to stop all services"
echo "=========================================="

# Simply wait for user input instead of monitoring processes
echo ""
echo "Applications are starting in the terminal tabs above."
echo "To stop all services:"
echo "  - Press Ctrl+C in this terminal, OR"
echo "  - Close the terminal window with the tabs"
echo ""

# Wait indefinitely for Ctrl+C
while true; do
    sleep 1
done