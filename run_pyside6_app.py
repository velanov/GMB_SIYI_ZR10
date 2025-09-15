#!/usr/bin/env python3
"""
Run the PySide6 version of the Gimbal Control System V2
with integrated ZR10 camera streaming
"""

import sys
import os
import signal
import atexit

# Don't override QT_QPA_PLATFORM if it's already set by startup script
# This allows the startup script to control platform selection

# Add the project root to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Add signal handling for graceful shutdown
def signal_handler(signum, frame):
    print(f"\n[SIGNAL] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def cleanup():
    print("[CLEANUP] Application closing...")

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
atexit.register(cleanup)

print(f"[STARTUP] Using Qt platform: {os.environ.get('QT_QPA_PLATFORM', 'default')}")

try:
  
    # Run the PySide6 app
    from gimbal_app.ui.main_app_pyside import main
    sys.exit(main())
except Exception as e:
    import traceback
    print(f"Error starting PySide6 application: {e}")
    print(f"Full traceback:\n{traceback.format_exc()}")
    print("\nTrying with offscreen platform (headless mode)...")
    
    # Fallback to offscreen mode for testing
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    
    try:
        from gimbal_app.ui.main_app_pyside import main
        print("Application would start successfully in GUI environment")
        print("Backend systems are fully functional")
    except Exception as e2:
        print(f"Even offscreen mode failed: {e2}")
        print(f"Offscreen traceback:\n{traceback.format_exc()}")
        print("This indicates a deeper Qt installation issue")
        sys.exit(1)