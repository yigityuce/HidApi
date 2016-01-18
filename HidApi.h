/*
 ####################################################################################
 #  C++ HidApi Library                                                              #
 #  Copyright (C) 2015-2016 by Yigit YUCE                                           #
 ####################################################################################
 #                                                                                  #
 #  This file is part of C++ HidApi Library.                                        #
 #                                                                                  #
 #  C++ HidApi Library is free software: you can redistribute it and/or modify      #
 #  it under the terms of the GNU Lesser General Public License as published by     #
 #  the Free Software Foundation, either version 3 of the License, or               #
 #  (at your option) any later version.                                             #
 #                                                                                  #
 #  C++ HidApi Library is distributed in the hope that it will be useful,           #
 #  but WITHOUT ANY WARRANTY; without even the implied warranty of                  #
 #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    #
 #  GNU Lesser General Public License for more details.                             #
 #                                                                                  #
 #  You should have received a copy of the GNU Lesser General Public License        #
 #  along with this program. If not, see <http://www.gnu.org/licenses/>.            #
 #                                                                                  #
 #  For any comment or suggestion please contact the creator of C++ HidApi Library  #
 #  at ygtyce@gmail.com                                                             #
 #                                                                                  #
 ####################################################################################
 */



#ifndef HIDAPI_H
#define HIDAPI_H



/*!
 * Determines and defines operating system
 */
#if defined _WIN32 && !(defined __gnu_linux__ || defined __linux__ || defined __unix__ || defined __APPLE__ || defined __MACH__)
#define OS_WINDOWS
#elif defined __gnu_linux__ || defined __linux__ || defined __unix__ && !(defined _WIN32 || defined __APPLE__ || defined __MACH__)
#define OS_LINUX
#elif defined __APPLE__ || defined __MACH__ && !(defined _WIN32 || defined __gnu_linux__ || defined __linux__ || defined __unix__)
#define OS_MAC
#endif



/*!
 * Windows specific structs, include files and macros
 */
#ifdef OS_WINDOWS
    #include <windows.h>

    #ifdef __MINGW32__
        #include <winbase.h>
    #endif

    #include <ntdef.h>
    #include <setupapi.h>
    #include <winioctl.h>
    #include <stdlib.h>
    #include <dbt.h>

    #define sleep(x)   Sleep(x*1000)
    #define msleep(x)  Sleep(x)

    typedef struct _HIDD_ATTRIBUTES
    {
        ULONG Size;
        USHORT VendorID;
        USHORT ProductID;
        USHORT VersionNumber;
    } HIDD_ATTRIBUTES, *PHIDD_ATTRIBUTES;

    typedef struct _HIDP_CAPS
    {
        USHORT Usage;
        USHORT UsagePage;
        USHORT InputReportByteLength;
        USHORT OutputReportByteLength;
        USHORT FeatureReportByteLength;
        USHORT Reserved[17];
        USHORT fields_not_used_by_hidapi[10];
    } HIDP_CAPS, *PHIDP_CAPS;

    struct HidDllFunctions
    {
        BOOLEAN  (WINAPI *GetAttributes)(HANDLE, PHIDD_ATTRIBUTES);
        BOOLEAN  (WINAPI *GetSerialNumberString)(HANDLE, PVOID, ULONG);
        BOOLEAN  (WINAPI *GetManufacturerString)(HANDLE, PVOID, ULONG);
        BOOLEAN  (WINAPI *GetProductString)(HANDLE, PVOID, ULONG);
        BOOLEAN  (WINAPI *SetFeature)(HANDLE, PVOID, ULONG);
        BOOLEAN  (WINAPI *GetFeature)(HANDLE, PVOID, ULONG);
        BOOLEAN  (WINAPI *GetIndexedString)(HANDLE, ULONG, PVOID, ULONG);
        BOOLEAN  (WINAPI *GetPreparsedData)(HANDLE, void**);
        BOOLEAN  (WINAPI *FreePreparsedData)(void *);
        NTSTATUS (WINAPI *GetCaps)(void*, HIDP_CAPS*);
        BOOLEAN  (WINAPI *SetNumInputBuffers)(HANDLE, ULONG);
    };
#endif



/*!
 * Linux specific include files and macros
 */
#ifdef OS_LINUX
    #include <locale.h>
    #include <errno.h>
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
    #include <sys/utsname.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <linux/hidraw.h>
    #include <linux/version.h>
    #include <linux/input.h>
    #include <libudev.h>
    #include <pthread.h>

    #define msleep(x) usleep(x*1000)
#endif



/*!
 * Common include files and defines
 */
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <queue>
#include <algorithm>
#include <utility>
#include <ctime>

#define DEFAULT_INTERNAL_BUFFER_SIZE     64             /*!< devices' default internal buffer size. It changes at enumaration(scan) stage on Windows */
#define POLLING_TIME_MS                  10             /*!< defines how long the reading operation takes time at single try */
#define DEVICE_READ_INTERVAL_MS          1              /*!< defines the reading loop's sleep time between two try loop */
#define API_CHECK_DEVICES_INTERVAL_MS    500            /*!< defines the device monitoring loop's sleep time between two check loop */
#define ANY                              0              /*!< uses for assigning the parameter value to NULL or 0 */



/*!
 * Class prototypes
 */
class HidApi;
class HidDevice;
class HidDeviceList : public std::vector<HidDevice> {};



/*!
 * Errors' human readable explanations. They are related to HidErrorCodes enum
 */
const static std::string errorStrings[] =
{
    "Api not initialized",                              /*!< explanation of HidError::HidErrorCodes::API_INITIALIZED enumaration */
    #ifdef OS_LINUX
        "Api udew_new failed",                          /*!< explanation of HidError::HidErrorCodes::API_UDEV_NEW enumaration */
    #endif
    "Api monitoring thread creating failed",            /*!< explanation of HidError::HidErrorCodes::API_MONITOR_THREAD_CREATE enumaration */
    "Api monitoring setup error",                       /*!< explanation of HidError::HidErrorCodes::API_MONITOR_THREAD_SETUP enumaration */
    "Api monitoring thread stopped",                    /*!< explanation of HidError::HidErrorCodes::API_MONITOR_THREAD_STOP enumaration */
    "Api new device could not add",                     /*!< explanation of HidError::HidErrorCodes::API_ADD enumaration */
    "Api removed device could not found in list",       /*!< explanation of HidError::HidErrorCodes::API_REMOVE enumaration */
    "Device not initialized",                           /*!< explanation of HidError::HidErrorCodes::DEVICE_INITIALIZED enumaration */
    "Device could not open",                            /*!< explanation of HidError::HidErrorCodes::DEVICE_OPEN enumaration */
    "Device could not close",                           /*!< explanation of HidError::HidErrorCodes::DEVICE_CLOSE enumaration */
    "Device not opened",                                /*!< explanation of HidError::HidErrorCodes::DEVICE_OPENED enumaration */
    "Device write failed",                              /*!< explanation of HidError::HidErrorCodes::DEVICE_WRITE enumaration */
    "Device flush failed",                              /*!< explanation of HidError::HidErrorCodes::DEVICE_FLUSH enumaration */
    "Device get index string failed",                   /*!< explanation of HidError::HidErrorCodes::DEVICE_GET_INDEXED enumaration */
    "Device report send failed",                        /*!< explanation of HidError::HidErrorCodes::DEVICE_SEND_REPORT enumaration */
    "Device report receive failed",                     /*!< explanation of HidError::HidErrorCodes::DEVICE_RECV_REPORT enumaration */
    "Device reader thread creating failed",             /*!< explanation of HidError::HidErrorCodes::DEVICE_READER_THREAD_CREATE enumaration */
    "Device reader thread's parent device is null",     /*!< explanation of HidError::HidErrorCodes::DEVICE_READER_THREAD_SETUP enumaration */
    "Device reader thread stopped"                      /*!< explanation of HidError::HidErrorCodes::DEVICE_READER_THREAD_STOP enumaration */
};



/*!
 * This class is a container class of errors. It is used for being an error callback functions' parameter.
 * Error callback functions are called with this class instantiates. Errors should be evaluated in callback
 * functions according to error code. Errors' causes are described below:
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_INITIALIZED
 *
 *        This error can be occured when the HidApi::scanDevices() function or HidApi class constructor is
 *        called. HidApi's initialized status are determined at HidApi class'es constructor. On the Linux
 *        based operating systems' initialized status is always true. Error casuse is Windows hid.dll library's
 *        couldn't be load properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_UDEV_NEW
 *
 *        This error only can be occured when the HidApi::scanDevices() function is called on the Linux based
 *        operating systems. Error cause is udev_new() function.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_MONITOR_THREAD_CREATE
 *
 *        This error only can be occured when the HidApi constructor is called. This is only triggered at
 *        HidApi::HidDeviceMonitoringThread::run() function. Error cause is CreateThread() on Windows or
 *        pthread_create() function on Linux.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_MONITOR_THREAD_SETUP
 *
 *        This error only can be occured when the HidApi constructor is called. This is only triggered at
 *        HidApi::HidDeviceMonitoringThread::onStartHandler() function. Error cause is preparation operations
 *        of HidApi device monitoring system can't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_MONITOR_THREAD_STOP
 *
 *        This error can be occured when the HidApi's device monitoring system couldn't be started or its
 *        execution is stopped. If device monitoring thread couldn't be started, API_MONITOR_SETUP error
 *        occured before this error.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_ADD
 *
 *        This error can be occured if the newly plugged device couldn't add the device list.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::API_REMOVE
 *
 *        This error can be occured if the unplugged device couldn't remove from device list.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_INITIALIZED
 *
 *        This error can be occured when the HidDevice::read() or HidDevice::open() or HidDevice::close() or
 *        HidDevice::write() or HidDevice::getIndexedString() or HidDevice::sendFeatureReport() or
 *        HidDevice::recvFeatureReport() or HidDevice::flush() function is called. On the Linux based
 *        operating systems' initialized status is always true. Error cause is Windows hid.dll library's
 *        couldn't be load properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_OPEN
 *
 *        This error only can be occured when the HidDevice::open() function is called. Error cause is the
 *        file opening operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_CLOSE
 *
 *        This error only can be occured when the HidDevice::close() function is called. This function can be
 *        called when assigning an exist HidDevice object to new or exist HidDevice object. Error cause is the
 *        file closing operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_OPENED
 *
 *        This error can be occured when the HidDevice::read() or HidDevice::write() or HidDevice::getIndexedString()
 *        or HidDevice::sendFeatureReport() or HidDevice::recvFeatureReport() or HidDevice::flush() function
 *        is called. Error cause is HidDevice::open() function isn't called before or when it is called it
 *        couldn't be done the file opening operation properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_WRITE
 *
 *        This error only can be occured when the HidDevice::write() function is called. Error cause is the
 *        writing operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_FLUSH
 *
 *        This error only can be occured when the HidDevice::flush() function is called. Error cause is the
 *        flushing operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_GET_INDEXED
 *
 *        This error only can be occured when the HidDevice::getIndexedString() function is called. Error
 *        cause is the getting indexed string operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_SEND_REPORT
 *
 *        This error only can be occured when the HidDevice::sendFeatureReport() function is called. Error
 *        cause is the sending report operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_RECV_REPORT
 *
 *        This error only can be occured when the HidDevice::recvFeatureReport() function is called. Error
 *        cause is the receiving report operation's couldn't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_READER_THREAD_CREATE
 *
 *        This error only can be occured when the HidDevice::open() function is called. This is only triggered
 *        at HidDevice::HidDeviceReaderThread::run() function. Error cause is CreateThread() on Windows or
 *        pthread_create() function on Linux.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_READER_THREAD_SETUP
 *
 *        This error only can be occured when the HidDevice::open() function is called. Error cause is preparation
 *        operations of HidDevice backgroud reader system can't be done properly.
 *
 *  - getErrorCode() == HidError::HidErrorCodes::DEVICE_READER_THREAD_STOP
 *
 *        This error can be occured when the HidDevice's backgroud reader system execution is stopped.
 *
 */
class HidError
{
    public:
        enum HidErrorCodes
        {
            API_INITIALIZED = 0,
            #ifdef OS_LINUX
                API_UDEV_NEW,
            #endif
            API_MONITOR_THREAD_CREATE,
            API_MONITOR_THREAD_SETUP,
            API_MONITOR_THREAD_STOP,
            API_ADD,
            API_REMOVE,
            DEVICE_INITIALIZED,
            DEVICE_OPEN,
            DEVICE_CLOSE,
            DEVICE_OPENED,
            DEVICE_WRITE,
            DEVICE_FLUSH,
            DEVICE_GET_INDEXED,
            DEVICE_SEND_REPORT,
            DEVICE_RECV_REPORT,
            DEVICE_READER_THREAD_CREATE,
            DEVICE_READER_THREAD_SETUP,
            DEVICE_READER_THREAD_STOP
        };

        /*!
         * HidError class constructor. It takes two parameters but the second parameter can be empty.
         * If the second parameter is empty, it will get error's string value from the errorStrings
         * array.
         */
                      HidError(HidErrorCodes code, std::string str = "");

        /*!
         * HidError class destructor. It does nothing.
         */
        virtual      ~HidError();

        /*!
         * Returns the error code as enumaration.
         */
        HidErrorCodes getErrorCode();

        /*!
         * Returns the error explanation as string.
         */
        std::string   getErrorString();

    private:
        HidErrorCodes errorCode;       /*!< stores the error code */
        std::string   errorString;     /*!< stores the error explanation */
};



 /*!
  * This class stores device informations and the device control operations like write, read,
  * send-receive report etc. are realized over this class.
  *
  * Important notes:
  *
  * - Before any control operation function is called, HidDevice::open() function had been called.
  *
  * - If any error occured, the callback function which registered with HidApi::registerDeviceErrorCallback()
  *   function will be called.
  */
class HidDevice
{
    private:

        /*!
         * This class executes background reading operation and it saves read values to HidDevice's
         * internal fifo buffer. It starts after the HidDevice::open() function is called. Background
         * reading system is controled by HidDevice class and users can't access to it.
         */
        class HidDeviceReaderThread
        {
            private:
                HidDevice              *device;            /*!< stores pointer of parent device which its owner. */
                bool                    running;           /*!< stores system's running state. System keeps going or stops according to this value */

                /*!
                 * Thread function executes this function. This function tries to read something during
                 * POLLING_TIME_MS and then waits DEVICE_READ_INTERVAL_MS long for doing this again. If
                 * it can read something, it adds it to fifo buffer. This loop runs until system stops.
                 */
                void                    onStartHandler();

                #ifdef OS_WINDOWS
                    HANDLE              threadHandle;      /*!< stores the thread handler of system if the OS is Windows. */
                    HANDLE              runningMutex;      /*!< stores the mutex which is used for synchronisation if the OS is Windows. */

                    /*!
                     * Function which callable by Windows's thread api. It takes casted HidDeviceReaderThread
                     * class pointer. This function recasts it to HidDeviceReaderThread class pointer and
                     * calls HidDeviceReaderThread::onStartHandler() function.
                     */
                    static DWORD WINAPI threadFunc(void* classPtr);
                #endif

                #ifdef OS_LINUX
                    pthread_t           threadHandle;      /*!< stores the thread handler of system if the OS is Linux. */
                    pthread_mutex_t     runningMutex;      /*!< stores the mutex which is used for synchronisation if the OS is Linux. */

                    /*!
                     * Function which callable by Linux's posix thread api. It takes casted HidDeviceReaderThread
                     * class pointer. This function recasts it to HidDeviceReaderThread class pointer and
                     * calls HidDeviceReaderThread::onStartHandler() function.
                     */
                    static void*        threadFunc(void* classPtr);
                #endif


            public:

                /*!
                 * HidDeviceReaderThread class constructor.
                 */
                                        HidDeviceReaderThread(HidDevice *dev = NULL);

                /*!
                 * HidDeviceReaderThread class destructor. It stops the background reader system.
                 */
                virtual                ~HidDeviceReaderThread();

                /*!
                 * This function sets the parent device of background reader system. It calls
                 * from HidApi::open() function. If it hadn't been called before the HidDeviceReaderThread::run()
                 * function is called, the HidError::HidErrorCodes::DEVICE_READER_THREAD_SETUP
                 * error occurs.
                 */
                void                    setParent(HidDevice *dev);

                /*!
                 * This function creates and starts the background reader system if the parent device
                 * is initialized and opened. Then returns immediately.
                 */
                void                    run();

                /*!
                 * This function stops the background reader system. It waits until the thread goes to
                 * sleep state.
                 */
                void                    stop();

                /*!
                 * Returns the system's running state.
                 */
                bool                    isRunning();
        };


        #ifdef OS_WINDOWS
            struct HidDllFunctions     *HidDllFuncs;       /*!< stores the hid.dll library function pointers struct if the operating system is Windows. */
            HANDLE                      devHandle;         /*!< stores the file handler of device if the operating system is Windows. */
        #endif

        #ifdef OS_LINUX
            int devHandle;                                 /*!< stores the file handler of device if the operating system is Linux. */
        #endif

        std::string                     path;                                        /*!< stores the device's path. */
        std::wstring                    serial;                                      /*!< stores the device's serial number. */
        std::wstring                    manufacturer;                                /*!< stores the device's manufacturer. */
        std::wstring                    product;                                     /*!< stores the device's product string. */
        unsigned short                  vendorId;                                    /*!< stores the device's vendor id. */
        unsigned short                  productId;                                   /*!< stores the device's product id. */
        unsigned short                  release;                                     /*!< stores the device's relase number. */
        unsigned short                  usagePage;                                   /*!< stores the device's usage page. */
        unsigned short                  usage;                                       /*!< stores the device's usage. */
        int                             interfaceNumber;                             /*!< stores the device's interface number. */
        unsigned short                  internalReadBufferSize;                      /*!< stores the device's read buffer size. */
        unsigned short                  internalWriteBufferSize;                     /*!< stores the device's write buffer size. */
        mutable bool                    opened;                                      /*!< stores the device file's status. */
        std::queue<std::string>         readFifoBuffer;                              /*!< internal read fifo buffer. */
        HidDeviceReaderThread          *backgroundReader;                            /*!< backgroud reader system. */
        void                          (*deviceErrorCallback) (HidDevice, HidError);  /*!< error callback function's pointer. */


    public:

        /*!
         * HidDevice class constructor. It sets initial values to member variables. Internal buffer size
         * variables are set to DEFAULT_INTERNAL_BUFFER_SIZE
         */
                        HidDevice();

        /*!
         * HidDevice class copy constructor. It will be called when "HidDevice newDevice = oldDevice;" like
         * code is executed. It stops the "oldDevice"s background reader system and copies whole things from
         * "oldDevice" to "newDevice" except background reader system.
         */
                        HidDevice(const HidDevice& cpyCtor);

        /*!
         * HidDevice class move constructor. It stops the "oldDevice"s background reader system, copies whole
         * things from "oldDevice" to "newDevice" except background reader system and resets the "oldDevice"s
         * member variables to default.
         */
                        HidDevice(HidDevice&& mvCtor);

        /*!
         * HidDevice assignment operator overload function. It will be called when "newDevice = oldDevice;"
         * like code is executed. It stops the "oldDevice"s background reader system and copies whole things
         * from "oldDevice" to "newDevice" except background reader system.
         */
        HidDevice&      operator= (const HidDevice& cpAssgnmnt);

        /*!
         * Equal compare operator overload function. It will be called when "newDevice == oldDevice"
         * like code is executed. If the both HidDevice object's path variables are equal it returns
         * true, else false.
         */
        bool            operator==(const HidDevice& isEq);

        /*!
         * HidDevice class destructor. It calls HidDevice::close() function.
         */
        virtual        ~HidDevice();

        /*!
         * This function tries to close device file. If the background reader system is running, this
         * function stops it also. If the closing process fails, the HidError::HidErrorCodes::DEVICE_OPEN
         * error occurs and this function returns false, else it returns true.
         */
        bool            close()                              const;

        /*!
         * This function tries to open device file. If the opening process is executed successfully, it
         * clears the internal read fifo buffer and if the background reader system is running, this
         * function stops and re-runs it. If the opening process fails, this function returns false and
         * the HidError::HidErrorCodes::DEVICE_OPEN error occurs, else it returns true.
         */
        bool            open();

        /*!
         * This function tries to flush device file. If the flushing process fails, this function returns
         * false and the HidError::HidErrorCodes::DEVICE_FLUSH error occurs, else it returns true.
         */
        bool            flush();

        /*!
         * This function tries to read something from internal fifo buffer which is filling by background
         * reader system. The behaviour of function changes according to the parameter's value like below:
         *
         * - timeout  < 0 : waits until read value is available (BLOCKING)
         * - timeout == 0 : returns IMMEDIATELY with or without value
         * - timeout  > 0 : waits until read value is available or TIME is OUT
         */
        std::string     read(int timeout = 0 );

        /*!
         * Returns the internal fifo buffer's size.
         */
        int             readAvailable()                      const;

        /*!
         * This function tries to write something to device file. If the writing process fails, the
         * HidError::HidErrorCodes::DEVICE_WRITE error occurs and this function returns with -1, else
         * the size of bytes that written is returned.
         */
        int             write(std::string data)              const;

        /*!
         * This function tries to get indexed string from device file. If the getting process fails, the
         * HidError::HidErrorCodes::DEVICE_GET_INDEXED error occurs and this function returns with empty
         * string, else it returns with received string. On Linux this function does nothing and it always
         * returns with empty string.
         */
        std::wstring    getIndexedString(int index)          const;

        /*!
         * This function tries to send report to device file. If the sending process fails, this function
         * returns false and the HidError::HidErrorCodes::DEVICE_SEND_REPORT error occurs, else it returns
         * true.
         */
        bool            sendFeatureReport(std::string* data) const;

        /*!
         * This function tries to receive report from device file. If the receiving process fails, this
         * function returns false and the HidError::HidErrorCodes::DEVICE_RECV_REPORT error occurs, else
         * it returns true.
         */
        bool            recvFeatureReport(std::string* data) const;

        /*!
         * If the hid.dll library is loaded properly this functions returns true on Windows.
         * This function always returns true on Linux.
         */
        bool            isInitialized()                      const;

        /*!
         * Returns device file's status.
         */
        bool            isOpened()                           const;

        /*!
         * Saves the function pointer to HidDevice::deviceErrorCallback member variable. Function prototype
         * must be like this: void func_name(HidDevice dev, HidError err);
         */
        void            registerDeviceErrorCallback(void (*fptr)(HidDevice,HidError));

        std::string     getPath()                            const;   /*!< Returns HidDevice::path variable value. */
        std::wstring    getSerial()                          const;   /*!< Returns HidDevice::serial variable value. */
        std::wstring    getManufacturer()                    const;   /*!< Returns HidDevice::manufacturer variable value. */
        std::wstring    getProductString()                   const;   /*!< Returns HidDevice::product variable value. */
        unsigned short  getVendorId()                        const;   /*!< Returns HidDevice::vendorId variable value. */
        unsigned short  getProductId()                       const;   /*!< Returns HidDevice::productId variable value. */
        unsigned short  getRelease()                         const;   /*!< Returns HidDevice::release variable value. */
        unsigned short  getUsagePage()                       const;   /*!< Returns HidDevice::usagePage variable value. */
        unsigned short  getUsage()                           const;   /*!< Returns HidDevice::usage variable value. */
        int             getInterface()                       const;   /*!< Returns HidDevice::path variable value. */



        friend class    HidApi;
        friend class    HidDevice::HidDeviceReaderThread;
};



/*!
 * This class has got two missions. First one is scan(or enumarate) the hid devices and the second one is
 * monitor(or check) the devices which are plugged or unplugged from the system.
 *
 * Important notes:
 *
 * - HidApi::getDeviceList() function returns the list of last scan and this can not be updated.
 *
 * - If the device error callback had never been registered with HidApi::registerDeviceErrorCallback() before
 *   the HidApi::scanDevices() function is called, the devices which are stored in list haven't got any error
 *   callback mechanism.
 *
 * - But if the device error callback is registered even once, this will be used at HidApi::scanDevices() function
 *   when device informations are being generated.
 *
 * - If the api error callback function's pointer is not passed with HidApi class constructor, nothing happens
 *   when the HidError::HidErrorCodes::API_MONITOR_THREAD_SETUP and HidError::HidErrorCodes::API_MONITOR_THREAD_CREATE
 *   errors are occured. So the best practice is to pass an api error callback function's pointer when instantiate
 *   HidApi object.
 */
class HidApi
{
    private:

        /*!
         * This class executes background monitoring operation and calls callback functions when the add or
         * remove actions are occured. It starts with the HidApi object. Background monitoring system is
         * controled by HidApi class and users can't access to it.
         */
        class HidDeviceMonitoringThread
        {
            private:
                HidApi                     *parent;              /*!< stores pointer of api which its owner. */
                bool                        running;             /*!< stores system's running state. System keeps going or stops according to this value */

                /*!
                 * Thread function executes this function. This function checks the system every
                 * API_CHECK_DEVICES_INTERVAL_MS milisecond. If add(plug) or remove(unplug) action is occured,
                 * the callback function which related with the action is called from this function on Linux.
                 *
                 * But on the Windows these are different. This function only catches the device related messages
                 * from system and it sends to another window and this window executes
                 * HidDeviceMonitoringThread::onMessageReceivedHandler() function.
                 *
                 * This loop runs until the system stops.
                 */
                void                        onStartHandler();

                #ifdef OS_WINDOWS
                    HANDLE                  threadHandle;        /*!< stores the thread handler of system if the OS is Windows. */
                    HANDLE                  runningMutex;        /*!< stores the mutex which is used for synchronisation if the OS is Windows. */

                    /*!
                     * This function will be executed when the system sent message about the device actions
                     * on Windows. If add(plug) or remove(unplug) action is occured, the callback function
                     * which related with the action is called from this function.
                     */
                    static LRESULT CALLBACK onMessageReceivedHandler(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

                    /*!
                     * Function which callable by Windows's thread api. It takes casted HidDeviceMonitoringThread
                     * class pointer. This function recasts it to HidDeviceMonitoringThread class pointer and
                     * calls HidDeviceMonitoringThread::onStartHandler() function.
                     */
                    static DWORD WINAPI     threadFunc(void* classPtr);
                #endif

                #ifdef OS_LINUX
                    pthread_t               threadHandle;        /*!< stores the thread handler of system if the OS is Linux. */
                    pthread_mutex_t         runningMutex;        /*!< stores the mutex which is used for synchronisation if the OS is Linux. */
                    struct udev_monitor    *mon;                 /*!< stores the udev monitor pointer if the OS is Linux. */


                    /*!
                     * Function which callable by Linux's posix thread api. It takes casted HidDeviceMonitoringThread
                     * class pointer. This function recasts it to HidDeviceMonitoringThread class pointer and
                     * calls HidDeviceMonitoringThread::onStartHandler() function.
                     */
                    static void*            threadFunc(void* classPtr);
                #endif

            public:

                /*!
                 * HidDeviceMonitoringThread class constructor.
                 */
                                            HidDeviceMonitoringThread(HidApi *pr);

                /*!
                 * HidDeviceReaderThread class destructor. It stops the background monitoring system.
                 */
                virtual                    ~HidDeviceMonitoringThread();

                /*!
                 * This function creates and starts the background monitoring system if the parent api
                 * is initialized. Then returns immediately.
                 */
                void                        run();

                /*!
                 * This function stops the background monitoring system. It waits until the thread goes to
                 * sleep state.
                 */
                void                        stop();
        };

        #ifdef OS_WINDOWS
            HMODULE                         libHandle;         /*!< stores the hid.dll library handle if the operating system is Windows. */
            struct HidDllFunctions         *HidDllFuncs;       /*!< stores the hid.dll library function pointers struct if the operating system is Windows. */
        #endif

        bool                                initialized;       /*!< stores api's initialization status. */
        HidDeviceList                       devices;           /*!< internal container of devices. */
        HidDeviceMonitoringThread          *monitorThread;     /*!< backgroud monitoring system. */

        void                              (*deviceAddCallback)   (int, HidDevice);       /*!< device added callback function's pointer. */
        void                              (*deviceRemoveCallback)(int, HidDevice);       /*!< device removed callback function's pointer. */
        void                              (*apiErrorCallback)    (HidError);             /*!< api error callback function's pointer. */
        void                              (*deviceErrorCallback) (HidDevice, HidError);  /*!< device error callback function's pointer. */

    public:

        /*!
         * HidApi class constructor. It sets initial values to member variables and runs the background
         * monitoring system. Additionaly it loads hid.dll library on Windows.
         */
                            HidApi(void (*apiErrCb)(HidError) = NULL);

        /*!
         * HidApi class destructor. It stops the background monitoring system.
         */
        virtual            ~HidApi();

        /*!
         * This function scans(enumarates) the devices according to filter parameters. Filter parameters
         * can be ANY(#define ANY 0) or the value according to parameter types. The usagePage and usage
         * filtering operations are ignored on Linux systems.
         */
        HidDeviceList       scanDevices(unsigned short _vendorId     = ANY,
                                        unsigned short _productId    = ANY,
                                        const wchar_t* _serial       = ANY,
                                        const wchar_t* _manufacturer = ANY,
                                        const wchar_t* _product      = ANY,
                                        unsigned short _release      = ANY,
                                        unsigned short _usagePage    = ANY,
                                        unsigned short _usage        = ANY);

        /*!
         * Returns the list of devices which are found and saved at the last scan.
         */
        HidDeviceList       getDeviceList();

        /*!
         * Returns the api's initialization status.
         */
        bool                isInitialized();

        /*!
         * Saves the function pointer to HidApi::deviceAddCallback member variable. Function prototype
         * must be like this: void func_name(int index, HidDevice dev);
         */
        void                registerDeviceAddCallback(void (*fptr)(int,HidDevice));

        /*!
         * Saves the function pointer to HidApi::deviceRemoveCallback member variable. Function prototype
         * must be like this: void func_name(int index, HidDevice dev);
         */
        void                registerDeviceRemoveCallback(void (*fptr)(int, HidDevice));

        /*!
         * Saves the function pointer to HidApi::apiErrorCallback member variable. Function prototype
         * must be like this: void func_name(HidError err);
         */
        void                registerApiErrorCallback(void (*fptr)(HidError));

        /*!
         * Saves the function pointer to HidApi::deviceErrorCallback member variable. Function prototype
         * must be like this: void func_name(HidDevice dev, HidError err);
         */
        void                registerDeviceErrorCallback(void (*fptr)(HidDevice,HidError));

        /*!
         * Converts c-style wide char array to std::string. If size is -1, it determines the size of
         * wide char array.
         */
        static std::string  wcharArrayToString(const wchar_t* arr, int size = -1);

        /*!
         * Converts c-style wide char array to std::wstring. If size is -1, it determines the size of
         * wide char array.
         */
        static std::wstring wcharArrayToWString(const wchar_t* arr, int size = -1);

        /*!
         * Converts std::string to std::wstring.
         */
        static std::wstring stringToWString(std::string str);

        /*!
         * Converts c-style char array to std::wstring.
         */
        static std::wstring charArrayToWString(const char *utf8);



        friend class        HidApi::HidDeviceMonitoringThread;
};



#endif // HIDAPI_H
