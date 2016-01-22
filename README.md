## **HID (Human Interface Device) Api for Windows and Linux Based Systems**

This project serves an api for HIDs to end users who use Windows or Linux based operating systems. It helps you to communicate with HIDs.

It comes with functions like standart file operation functions (read, write, open, close etc). On the both operating system, these functions and call sytles are same.

It determines your operating system automatically and generates codes according to it. But the high level api usage won't be different so users can compile and use its own main code on both platform.

----------

### **Classes and Hierarchies**

- HidError
- HidApi
	- HidDeviceMonitoringThread
- HidDevice
	- HidDeviceReaderThread
- HidDeviceList

----------
#### **HidError** class
HidError class is a container for errors which occur run time. It can be handled in error callback functions. It has got these functions:

- getErrorCode()
- getErrorString()

----------
#### **HidApi** class
HidApi class should be instantiated once in whole program (maybe this can be turned into singleton later versions). It can be used for scan HIDs and register callbacks. It runs a backgroud operation also to recognize the devices which are plugged or unplugged. This operation is managed by HidApi class and users are not responsible for managing it. It has got these functions:

- scanDevices()
- getDeviceList()
- isInitialized()
- registerDeviceAddCallback()
- registerDeviceRemoveCallback()
- registerApiErrorCallback()
- registerDeviceErrorCallback()

----------
#### **HidDeviceMonitoringThread** class
HidDeviceMonitoringThread is a class which executes a background operation to recognize the devices which are plugged or unplugged. This operation is managed by HidApi class and users are not responsible for managing it. Its behaviour is different on Windows and Linux.

----------
#### **HidDevice** class
HidDevice class is the interface for interacting with HIDs. Users should not instantiate an object from this class. Because it instantiates with empty values and this is meaningless. HidDevice class objects are generally used for being the elements of the HidDeviceList and HidDeviceList will be filled with HidApi::scanDevices() function. So users don't need to instantiate HidDevice objects unless they try to get a HidDeviceList element to the local or global variable. It runs a backgroud operation also to read data from device asynchronously and these are stored in the internal FIFO buffer. This operation is managed by HidDevice class and users are not responsible for managing it. This background operation starts when device opened and finishes when device closed. It has got these functions:

- open()
- close()
- flush()
- readAvailable()
- read()
- write()
- getIndexedString()
- sendFeatureReport()
- recvFeatureReport()
- isInitialized()
- isOpened()
- registerDeviceErrorCallback()
- getPath()
- getSerial()
- getManufacturer()
- getProductString()
- getVendorId()
- getProductId()
- getRelease()
- getUsagePage()
- getUsage()
- getInterface()

> **Important Notes:**

> - All functions except open(), close() isInitialized(),  registerDeviceErrorCallback() and isOpened(), will be accessible if device is **opened**.
> - If you use device functions directly over the HidDeviceList object with device index, you should remember this; in every calls of scanDevices() function, all devices will be closed. You should re-open device which you want.
> - Background reader system starts with HidDevice::open() function and finishes with HidDevice::close() function. These functions clear internal FIFO buffer.
> - If any error occured, the callback function which registered with HidApi::registerDeviceErrorCallback() function will be called.

----------
#### **HidDeviceReaderThread** class
HidDeviceReaderThread is a class which executes a background operation to read data from device asynchronously and these are stored in the internal FIFO buffer.  This operation is managed by HidDevice class and users are not responsible for managing it. This background operation starts when device opened and finishes when device closed. Its behaviour is different on Windows and Linux.

----------

### **Dependencies**
The C++11 support is necessery for both Windows and Linux based systems.

On Windows OS, the library uses **setupapi**, so the "**-lsetupapi**" linker flag must be added to linker librariers.

On Linux based OS, the library uses [Posix Threads](https://en.wikipedia.org/wiki/POSIX_Threads) and [UDEV library (libudev)](https://en.wikipedia.org/wiki/Udev). So you must install UDEV library with this command.
```bash
$ sudo apt-get install libudev-dev
```
And the "**-lpthread**" and "**-ludev**" linker flags must be added to linker librariers.

----------

### **USAGE**

This library is header-only so you must just include the HidApi.h file to your code. That's it.
```c++
#include "HidApi.h"
```

----------

This project is based on a Signal11's [hidapi](https://github.com/signal11/hidapi) project. Thanks for their affords.
