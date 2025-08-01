
# Environment
+ Ubuntu 24.04 LTS
+ Python 3.12

# WSL2 Serial Port Setting

## `usbipd`

Install [`usbipd`](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) on Windows. 

``` bash
# in powershell
usbipd list
```

Find the device that has to be bind and attached to wsl.

``` bash
# in powershell
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```
To detach the usb port.
```bash
usbipd detach --busid <busid>
```
## WSL Config
Start WSL.
``` bash
lsusb
```


# Packages
+ Python 3+ python3-pip python3-venv
+ vim git 
```
sudo apt update
sudo apt upgrade
sudo apt install 
sudo apt install python3 python3-pip python3-venv swig jupyter
```

# Linux Setting

Clone the repository from the original building
``` bash
git clone https://github.com/natsnyder1/CauchyFriendly.git
```
## Python Virtual Environment

Direct to `*/CauchyFriendly` folder. Setup python virtual environment in Python.
```bash
python3 -m venv .venv
```
Activate python virtual environment
``` bash
# pwd: */CauchyFriendly 
source .venv/bin/activate
# deactivate
deactivate
```
Install python packages

> numpy scipy requests matplotlib

``` bash
pip install numpy
```



## Python Config

Install the required python libraries
``` bash
pip install -r ./scripts/requirements.txt
```
Run the `auto_config.py`
``` bash
# pwd: */CauchyFriendly
python ./scripts/auto_config.py
```

## Swig

Under `*/CauchyFriendly/scripts/swig/cauchy` modify `swigit_unix.sh`
+ Include `PYTHON_INC_FILE="-I/*/CauchyFriendly/.venv/bin/python"`
+ Include `INCLUDE_FILE=${FILE_NAME}.hpp`
``` bash
# pwd: */CauchyFriendly/scripts/swig/cauchy
source swigit_unix.sh
```

