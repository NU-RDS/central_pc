#############################################################################################################################################################################
##
## Description :  This codes is based on gs-usb 0.2.9 demo from pypi.org released on Feb 25, 2022. Much appreciated to the original author.   
##                You could test all series innomaker usb2can device(usb2can module/usb2can-c/usb2can-x2/usb2can-core) by this python code     
##                 
##                
##                
##
## Author      :  Calvin (calvin@inno-maker.com)/ www.inno-maker.com
##
##                
## Date        :  2021.07.02
##
## Environment :  Hardware            ----------------------  Computer and innomaker usb2can module
##                System              ----------------------  Windows 10
##                Version of Python   ----------------------  Python 3.7 64bit
##
## reference   :  gs-usb              ----------------------  pip install gs-usb
##                libusb              ----------------------  https://sourceforge.net/projects/libusb/
##                Winusb              ----------------------  https://udomain.dl.sourceforge.net/project/libusb-win32/libusb-win32-releases/1.2.6.0/libusb-win32-bin-1.2.6.0.zip  
##
## Notes       :  1. Use the libusb and zadig and do not install the libusb-win32 on your computer. libusb-win32 will case the device incorrectly recognize
##
##                2.  If this demo can't check the usb2can device after you install all driver. Download the libusb-1.0.20 packet from below link
##                      https://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.20/libusb-1.0.20.7z/download
##                      Copy the MS64\dll\libusb-1.0.dll to your computer C:\Windows\System32 
##
## support:          Any problem and suggestion, feel free to contact support@inno-maker.com/calvin@inno-maker.com

##############################################################################################################################################################################


#note: I had to copy the DLL file like they said in order for the demo to start
#note 2: I modified the original script to make it read-only; see the full script here:
#https://github.com/INNO-MAKER/usb2can/blob/master/For%20Windows/Python/usb2can.py

import time

from gs_usb.gs_usb import GsUsb
from gs_usb.gs_usb_frame import GsUsbFrame
from gs_usb.constants import (
    CAN_EFF_FLAG,
    CAN_ERR_FLAG,
    CAN_RTR_FLAG,
)

# gs_usb general also can import from gs_usb_structures.py
GS_USB_ECHO_ID = 0
GS_USB_NONE_ECHO_ID = 0xFFFFFFFF

#below macro is for gs_usb 0.3.0 and above
from gs_usb.gs_usb import (
    GS_CAN_MODE_NORMAL,
    GS_CAN_MODE_LISTEN_ONLY,
    GS_CAN_MODE_LOOP_BACK,
)

def main():
    devs = GsUsb.scan()
    if len(devs) == 0:
        print("Can not find gs_usb device")
        return

    #set can device handle from 0 to 1,2,3...  choosing the right device serial number accroding to the print
    #I found defualt usb2can device is devs[0] with gs_usb 0.2.9 ,and devs[1] with gs_usb 0.3.0
    #dev = devs[0] 
    dev = devs[1]
    print(dev)   
    # Close before Start device in case the device was not properly stop last time
    # If do not stop the device, bitrate setting will be fail.
    #dev.stop() 

    # Configuration Modify the Baudrate you want here
    if not dev.set_bitrate(1000000):
        print("Can not set bitrate for gs_usb")
        return
    
    # Start device, If you have only one device for test, pls use the loop-back mode, otherwise you will get a lot of error frame
    # If you have already connect to a aviailable CAN-BUS, you could set as NORMAL mode
    dev.start(GS_CAN_MODE_NORMAL)
    
    # Read all the time and send message in each second
    while True:
        
        iframe = GsUsbFrame()
        if dev.read(iframe, 1):
            print("RX  {}".format(iframe))

    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
