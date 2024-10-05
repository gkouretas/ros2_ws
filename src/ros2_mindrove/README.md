# Mindrove ROS2 package

## Setup
### SDK
See the [Mindrove SDK Documentation](https://docs.mindrove.com/index.html) for the Mindrove API docs. See the [GitHub](https://github.com/MindRove/MindRoveSDK) for SDK installation instructions. Installing MindRove for Python is required for this package (via `pip install mindrove`).

### Windows
See [here](https://mindrove.com/downloads/) to download a Windows-based visualizer application. Good for quick verification of Mindrove functionality.

WiFi driver was already present, so no further actions there required.

### Linux
See [here](https://github.com/morrownr/8821cu-20210916/tree/main) to install the drivers for the shipped WiFi dongle to work (verified w/ Ubuntu 22.04). 
- NOTE: for installation, I was able to compile w/ gcc 11 when gcc 12 was in use.