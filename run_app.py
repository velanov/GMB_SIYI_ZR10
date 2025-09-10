# Run the refactored app
from gimbal_app.ui.main_app import GimbalGPSAppV2

if __name__ == "__main__":
    app = GimbalGPSAppV2()
    app.root.mainloop()
