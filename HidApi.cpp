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



#include "HidApi.h"



// ######################################### COMMON HIDERROR CLASS FUNCTION DEFINITIONS BEGIN ######################################### //
HidError::HidError(HidErrorCodes code, std::string str)
{
    this->errorCode   = code;
    this->errorString = ( str == "" ? errorStrings[code] : str );
}

HidError::~HidError()
{
}

HidError::HidErrorCodes HidError::getErrorCode()
{
    return this->errorCode;
}

std::string HidError::getErrorString()
{
    return this->errorString;
}

// ########################################## COMMON HIDERROR CLASS FUNCTION DEFINITIONS END ########################################## //





// ######################################## COMMON HIDDEVICE CLASS FUNCTION DEFINITIONS BEGIN ######################################### //

void           HidDevice::registerDeviceErrorCallback(void (*fptr)(HidDevice,HidError))        { this->deviceErrorCallback  = fptr;     }
std::string    HidDevice::getPath()                                                      const { return this->path;                     }
std::wstring   HidDevice::getSerial()                                                    const { return this->serial;                   }
std::wstring   HidDevice::getManufacturer()                                              const { return this->manufacturer;             }
std::wstring   HidDevice::getProductString()                                             const { return this->product;                  }
unsigned short HidDevice::getVendorId()                                                  const { return this->vendorId;                 }
unsigned short HidDevice::getProductId()                                                 const { return this->productId;                }
unsigned short HidDevice::getRelease()                                                   const { return this->release;                  }
unsigned short HidDevice::getUsagePage()                                                 const { return this->usagePage;                }
unsigned short HidDevice::getUsage()                                                     const { return this->usage;                    }
int            HidDevice::getInterface()                                                 const { return this->interfaceNumber;          }
bool           HidDevice::isOpened()                                                     const { return this->opened;                   }
int            HidDevice::readAvailable()                                                const { return this->readFifoBuffer.size();    }

std::string    HidDevice::read(int timeout)
{
    std::string ret = "";

    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return ret;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return ret;
    }

    if( timeout < 0 )
    {
        while( this->readAvailable() <= 0 ){ msleep(1); }

        ret = this->readFifoBuffer.front();
        this->readFifoBuffer.pop();
    }
    else if( timeout == 0 )
    {
        if( this->readAvailable() > 0 )
        {
            ret = this->readFifoBuffer.front();
            this->readFifoBuffer.pop();
        }
    }
    else
    {
        std::clock_t start  = std::clock();
        double end          = (double)(timeout*((double)(CLOCKS_PER_SEC / 1000)));

        while( (std::clock()-start) < end )
        {
            if( this->readAvailable() > 0 )
            {
                ret = this->readFifoBuffer.front();
                this->readFifoBuffer.pop();
                break;
            }
        }
    }

    return ret;
}

// ########################################## COMMON HIDDEVICE CLASS FUNCTION DEFINITIONS END ######################################### //





// ###################################### OS SPECIFIC HIDDEVICE CLASS FUNCTION DEFINITIONS BEGIN ###################################### //

HidDevice::HidDevice()
{
    #ifdef OS_WINDOWS
        this->HidDllFuncs            = NULL;
        this->devHandle              = INVALID_HANDLE_VALUE;
    #endif

    #ifdef OS_LINUX
        this->devHandle              = -1;
    #endif

    this->path                       = "";
    this->serial                     = L"";
    this->manufacturer               = L"";
    this->product                    = L"";
    this->vendorId                   = 0;
    this->productId                  = 0;
    this->release                    = 0;
    this->usagePage                  = 0;
    this->usage                      = 0;
    this->interfaceNumber            = 0;
    this->internalReadBufferSize     = DEFAULT_INTERNAL_BUFFER_SIZE;
    this->internalWriteBufferSize    = DEFAULT_INTERNAL_BUFFER_SIZE;
    this->opened                     = false;
    this->deviceErrorCallback        = NULL;
    this->backgroundReader           = new HidDevice::HidDeviceReaderThread();
}

HidDevice::HidDevice(const HidDevice& cpyCtor)
{
    if(this != &cpyCtor)
    {
        cpyCtor.close();

        #ifdef OS_WINDOWS
            this->HidDllFuncs            = cpyCtor.HidDllFuncs;
            this->devHandle              = INVALID_HANDLE_VALUE;
        #endif

        #ifdef OS_LINUX
            this->devHandle              = -1;
        #endif

        this->path                       = cpyCtor.path;
        this->serial                     = cpyCtor.serial;
        this->manufacturer               = cpyCtor.manufacturer;
        this->product                    = cpyCtor.product;
        this->vendorId                   = cpyCtor.vendorId;
        this->productId                  = cpyCtor.productId;
        this->release                    = cpyCtor.release;
        this->usagePage                  = cpyCtor.usagePage;
        this->usage                      = cpyCtor.usage;
        this->interfaceNumber            = cpyCtor.interfaceNumber;
        this->internalReadBufferSize     = cpyCtor.internalReadBufferSize;
        this->internalWriteBufferSize    = cpyCtor.internalWriteBufferSize;
        this->readFifoBuffer             = cpyCtor.readFifoBuffer;
        this->deviceErrorCallback        = cpyCtor.deviceErrorCallback;
        this->opened                     = false;
        this->backgroundReader           = new HidDevice::HidDeviceReaderThread();
    }
}

HidDevice::HidDevice(HidDevice&& mvCtor)
{
    mvCtor.close();

    #ifdef OS_WINDOWS
        this->HidDllFuncs             = mvCtor.HidDllFuncs;
        this->devHandle               = INVALID_HANDLE_VALUE;
        mvCtor.HidDllFuncs            = NULL;
        mvCtor.devHandle              = INVALID_HANDLE_VALUE;
    #endif

    #ifdef OS_LINUX
        this->devHandle               = -1;
        mvCtor.devHandle              = -1;
    #endif

    this->path                        = mvCtor.path;
    this->serial                      = mvCtor.serial;
    this->manufacturer                = mvCtor.manufacturer;
    this->product                     = mvCtor.product;
    this->vendorId                    = mvCtor.vendorId;
    this->productId                   = mvCtor.productId;
    this->release                     = mvCtor.release;
    this->usagePage                   = mvCtor.usagePage;
    this->usage                       = mvCtor.usage;
    this->interfaceNumber             = mvCtor.interfaceNumber;
    this->internalReadBufferSize      = mvCtor.internalReadBufferSize;
    this->internalWriteBufferSize     = mvCtor.internalWriteBufferSize;
    this->readFifoBuffer              = mvCtor.readFifoBuffer;
    this->deviceErrorCallback         = mvCtor.deviceErrorCallback;
    this->opened                      = false;
    this->backgroundReader            = new HidDevice::HidDeviceReaderThread();


    mvCtor.path                       = "";
    mvCtor.serial                     = L"";
    mvCtor.manufacturer               = L"";
    mvCtor.product                    = L"";
    mvCtor.vendorId                   = 0;
    mvCtor.productId                  = 0;
    mvCtor.release                    = 0;
    mvCtor.usagePage                  = 0;
    mvCtor.usage                      = 0;
    mvCtor.interfaceNumber            = 0;
    mvCtor.internalReadBufferSize     = 0;
    mvCtor.internalWriteBufferSize    = 0;
    mvCtor.readFifoBuffer             = std::queue<std::string>();
    mvCtor.deviceErrorCallback        = NULL;
    mvCtor.backgroundReader           = NULL;

}

HidDevice& HidDevice::operator= (const HidDevice& cpAssgnmnt)
{
    if(this != &cpAssgnmnt)
    {
        cpAssgnmnt.close();
        this->close();

        #ifdef OS_WINDOWS
            this->HidDllFuncs            = cpAssgnmnt.HidDllFuncs;
            this->devHandle              = INVALID_HANDLE_VALUE;
        #endif

        #ifdef OS_LINUX
            this->devHandle              = -1;
        #endif

        this->path                       = cpAssgnmnt.path;
        this->serial                     = cpAssgnmnt.serial;
        this->manufacturer               = cpAssgnmnt.manufacturer;
        this->product                    = cpAssgnmnt.product;
        this->vendorId                   = cpAssgnmnt.vendorId;
        this->productId                  = cpAssgnmnt.productId;
        this->release                    = cpAssgnmnt.release;
        this->usagePage                  = cpAssgnmnt.usagePage;
        this->usage                      = cpAssgnmnt.usage;
        this->interfaceNumber            = cpAssgnmnt.interfaceNumber;
        this->internalReadBufferSize     = cpAssgnmnt.internalReadBufferSize;
        this->internalWriteBufferSize    = cpAssgnmnt.internalWriteBufferSize;
        this->readFifoBuffer             = cpAssgnmnt.readFifoBuffer;
        this->deviceErrorCallback        = cpAssgnmnt.deviceErrorCallback;
        this->backgroundReader           = new HidDevice::HidDeviceReaderThread();
    }
    return *this;
}

bool HidDevice::operator==(const HidDevice& isEq)
{
    return (this->path == isEq.path);
}

HidDevice::~HidDevice()
{
    this->close();
}

bool HidDevice::open()
{
    if( this->isOpened() )
    {
        return true;
    }

    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return false;
    }

    #ifdef OS_WINDOWS
        this->devHandle = CreateFile(&(this->path[0]), (GENERIC_WRITE|GENERIC_READ), 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

        if(this->devHandle == INVALID_HANDLE_VALUE)
        {
            this->opened = false;
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPEN));
            return false;
        }

        HIDP_CAPS caps;
        void * pp_data = NULL;

        // TODO error handling operations will be done for these
        if(!this->HidDllFuncs->SetNumInputBuffers(this->devHandle, 64))
        {
        }

        if(!this->HidDllFuncs->GetPreparsedData(this->devHandle, &pp_data))
        {
        }

        if(this->HidDllFuncs->GetCaps(pp_data, &caps) != 0x110000 )
        {
        }

        this->internalReadBufferSize  = caps.InputReportByteLength;
        this->internalWriteBufferSize = caps.OutputReportByteLength;
        this->HidDllFuncs->FreePreparsedData(pp_data);

    #endif

    #ifdef OS_LINUX
        this->devHandle = ::open(this->path.c_str(), O_RDWR);
        if( this->devHandle < 0 )
        {
            this->opened = false;
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPEN));
            return false;
        }
    #endif



    this->opened          = true;
    this->backgroundReader->stop();
    this->backgroundReader->setParent(this);
    this->readFifoBuffer  = std::queue<std::string>(); // flush
    this->backgroundReader->run();

    return this->opened;
}

bool HidDevice::close() const
{
    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return false;
    }

    if( this->isOpened() )
    {
        #ifdef OS_WINDOWS
            bool closed = (CloseHandle(this->devHandle) ? true : false);
        #endif
        #ifdef OS_LINUX
            bool closed = ((::close(this->devHandle) == 0 ) ? true : false );
        #endif

        if( closed )
        {
            if( this->backgroundReader )
            {
                if( this->backgroundReader->isRunning() ) this->backgroundReader->stop();
            }
            this->opened = false;
        }
        else
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_CLOSE));
        }
        return closed;
    }
    else
    {
        if( this->backgroundReader )
        {
            if( this->backgroundReader->isRunning() ) this->backgroundReader->stop();
        }
        return true;
    }
}

int HidDevice::write(std::string data) const
{
    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return -1;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return -1;
    }

    #ifdef OS_WINDOWS
        DWORD       bytesWritten = -1;
        std::string tempData     = data;
        OVERLAPPED  ol;
        memset(&ol, 0, sizeof(ol));
        tempData.resize(this->internalWriteBufferSize,0);

        if( !WriteFile(this->devHandle, &(tempData[0]), tempData.size(), NULL, &ol) )
        {
            if( GetLastError() != ERROR_IO_PENDING )
            {
                bytesWritten = -1;
                if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_WRITE));
                return bytesWritten;
            }
        }

        if( !GetOverlappedResult(this->devHandle, &ol, &bytesWritten, TRUE/*wait*/) )
        {
            bytesWritten = -1;
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_WRITE));
            return bytesWritten;
        }

        return bytesWritten;
    #endif

    #ifdef OS_LINUX
        int bytesWritten = ::write(this->devHandle, &data[0], data.size());

        if( bytesWritten < 0 )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_WRITE));
        }
        return bytesWritten;
    #endif
}

bool HidDevice::flush()
{
    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return false;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return false;
    }

    #ifdef OS_WINDOWS
        if( ! FlushFileBuffers(this->devHandle) )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_FLUSH));
            return false;
        }
    #endif

    #ifdef OS_LINUX
        if( fsync(this->devHandle) != 0 )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_FLUSH));
            return false;
        }
    #endif

    return true;
}

std::wstring HidDevice::getIndexedString(int index)  const
{
    std::wstring retVal = L"";
    retVal.resize(this->internalReadBufferSize);

    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return retVal;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return retVal;
    }

    #ifdef OS_WINDOWS
        if( !this->HidDllFuncs->GetIndexedString(this->devHandle, index, &(retVal[0]), retVal.size()) )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_GET_INDEXED));
            retVal = L"";
        }
    #endif

    #ifdef OS_LINUX
        retVal = L"";
    #endif


    return retVal;
}

bool HidDevice::sendFeatureReport(std::string* data) const
{
    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return false;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return false;
    }

    #ifdef OS_WINDOWS
        if( this->HidDllFuncs->SetFeature(this->devHandle, (PVOID)(&((*data)[0])), (*data).size()) )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_SEND_REPORT));
            return false;
        }
        return true;
    #endif

    #ifdef OS_LINUX
        if( ioctl(this->devHandle, HIDIOCSFEATURE(data->size()), &((*data)[0])) < 0 )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_SEND_REPORT));
            return false;
        }
        return true;
    #endif
}

bool HidDevice::recvFeatureReport(std::string* data) const
{
    if( !this->isInitialized() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_INITIALIZED));
        return false;
    }

    if( !this->isOpened() )
    {
        if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_OPENED));
        return false;
    }

    #ifdef OS_WINDOWS
        DWORD bytesReturned;
        OVERLAPPED ol;
        memset(&ol, 0, sizeof(ol));

        if( !DeviceIoControl(this->devHandle,
                             CTL_CODE(FILE_DEVICE_KEYBOARD, (100), METHOD_OUT_DIRECT, FILE_ANY_ACCESS),
                             NULL, 0, &((*data)[0]), (*data).size(), &bytesReturned, &ol))
        {
            if(GetLastError() != ERROR_IO_PENDING)
            {
                if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_RECV_REPORT));
                return false;
            }
        }

        if( !GetOverlappedResult(this->devHandle, &ol, &bytesReturned, TRUE) )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_RECV_REPORT));
            return false;
        }

        bytesReturned++;
        (*data).resize(bytesReturned);
        return true;
    #endif

    #ifdef OS_LINUX

        if( ioctl(this->devHandle, HIDIOCGFEATURE(data->size()), &((*data)[0])) < 0 )
        {
            if( this->deviceErrorCallback ) (*(this->deviceErrorCallback))(*this,HidError(HidError::DEVICE_RECV_REPORT));
            return false;
        }
        return true;
    #endif
}

bool HidDevice::isInitialized() const
{
    #ifdef OS_WINDOWS
        return ( this->HidDllFuncs != NULL );
    #endif

    #ifdef OS_LINUX
        return true;
    #endif
}

// ###################################### OS SPECIFIC HIDDEVICE CLASS FUNCTION DEFINITIONS END ###################################### //





// ################################ HIDDEVICE::HIDDEVICEREADERTHREAD CLASS FUNCTION DEFINITIONS BEGIN ############################### //

HidDevice::HidDeviceReaderThread::HidDeviceReaderThread(HidDevice *dev)
{
    #ifdef OS_WINDOWS
        this->threadHandle  = INVALID_HANDLE_VALUE;
        this->runningMutex  = CreateMutex(NULL, FALSE, NULL);
    #endif

    #ifdef OS_LINUX
        this->threadHandle  = 0;
        pthread_mutex_init( &(this->runningMutex), NULL);
    #endif

    this->running           = false;
    this->device            = dev;
}

HidDevice::HidDeviceReaderThread::~HidDeviceReaderThread()
{
    this->stop();

    #ifdef OS_WINDOWS
        CloseHandle(this->runningMutex);
    #endif

    #ifdef OS_LINUX
        pthread_mutex_destroy( &(this->runningMutex) );
    #endif
}

void HidDevice::HidDeviceReaderThread::setParent(HidDevice *dev)
{
    this->device = dev;
}

void HidDevice::HidDeviceReaderThread::run()
{
    if( this->device != NULL )
    {
        if( !this->device->isInitialized() )
        {
            if( this->device->deviceErrorCallback ) (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_INITIALIZED));
            return;
        }

        if( !this->device->isOpened() )
        {
            if( this->device->deviceErrorCallback ) (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_OPENED));
            return;
        }


        #ifdef OS_WINDOWS
            this->threadHandle = CreateThread( NULL,                                           // default security attributes
                                               0,                                              // use default stack size
                                               &HidDevice::HidDeviceReaderThread::threadFunc,  // thread function name
                                               (void*)this,                                    // argument to thread function
                                               0,                                              // use default creation flags
                                               0                                               // pointer of thread identifier
                                             );

            if( (this->threadHandle == INVALID_HANDLE_VALUE) and this->device->deviceErrorCallback )
            {
                (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_READER_THREAD_CREATE));
            }

        #endif

        #ifdef OS_LINUX
            int res = pthread_create( &(this->threadHandle),
                                      NULL,
                                      &HidDevice::HidDeviceReaderThread::threadFunc,
                                      (void*)(this)
                                    );

            if( (res != 0) and this->device->deviceErrorCallback )
            {
                (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_READER_THREAD_CREATE));
            }
        #endif
    }
    else
    {
        if( this->device->deviceErrorCallback )
        {
            (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_READER_THREAD_SETUP));
        }
    }
}

void HidDevice::HidDeviceReaderThread::stop()
{
    #ifdef OS_WINDOWS
        WaitForSingleObject( this->runningMutex, INFINITE);
        this->running = false;
        ReleaseMutex(this->runningMutex);
    #endif

    #ifdef OS_LINUX
        pthread_mutex_lock( &(this->runningMutex));
        this->running = false;
        pthread_mutex_unlock( &(this->runningMutex));
    #endif
}

void HidDevice::HidDeviceReaderThread::onStartHandler()
{
    if( this->device != NULL )
    {
        std::string tempBuf;

        #ifdef OS_WINDOWS

            WaitForSingleObject( this->runningMutex, INFINITE);
            this->running = true;
            ReleaseMutex(this->runningMutex);

            DWORD       bytesRead;
            OVERLAPPED  readOL;

            while(this->running)
            {
                WaitForSingleObject( this->runningMutex, INFINITE);
                if( this->device->isInitialized() and this->device->isOpened() )
                {
                    memset(&(readOL), 0, sizeof(readOL));
                    readOL.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
                    bytesRead     = 0;
                    tempBuf       = "";
                    tempBuf.resize(this->device->internalReadBufferSize, 0);

                    ResetEvent(readOL.hEvent);

                    if( !ReadFile(this->device->devHandle, &(tempBuf[0]), tempBuf.size(), &bytesRead, &readOL) )
                    {
                        if (GetLastError() != ERROR_IO_PENDING)
                        {
                            CancelIo(this->device->devHandle);
                            CloseHandle(readOL.hEvent);
                            ReleaseMutex(this->runningMutex);
                            msleep(1);
                            continue;
                        }
                    }

                    if( WaitForSingleObject(readOL.hEvent, POLLING_TIME_MS) != WAIT_OBJECT_0 )
                    {
                        CancelIo(this->device->devHandle);
                        CloseHandle(readOL.hEvent);
                        ReleaseMutex(this->runningMutex);
                        msleep(DEVICE_READ_INTERVAL_MS);
                        continue;
                    }

                    if( GetOverlappedResult(this->device->devHandle, &readOL, &bytesRead, TRUE) )
                    {
                        if( bytesRead > 0 ) this->device->readFifoBuffer.push(tempBuf);
                    }

                    CloseHandle(readOL.hEvent);
                }
                ReleaseMutex(this->runningMutex);
                msleep(DEVICE_READ_INTERVAL_MS);
            }
        #endif

        #ifdef OS_LINUX

            pthread_mutex_lock( &(this->runningMutex));
            this->running = true;
            pthread_mutex_unlock( &(this->runningMutex));

            struct timeval tv;
            tv.tv_sec  = 0;
            tv.tv_usec = POLLING_TIME_MS*1000;
            fd_set fds;

            while(this->running)
            {
                pthread_mutex_lock( &(this->runningMutex));
                FD_ZERO(&fds);
                FD_SET(this->device->devHandle, &fds);
                if( (select(this->device->devHandle+1, &fds, NULL, NULL, &tv) > 0) and FD_ISSET(this->device->devHandle, &fds) )
                {
                    tempBuf       = "";
                    tempBuf.resize(128, 0);

                    int bytesRead = ::read(this->device->devHandle, &tempBuf[0], tempBuf.size());
                    if( bytesRead > 0 )
                    {
                        tempBuf.resize(bytesRead);
                        this->device->readFifoBuffer.push( tempBuf );
                    }
                }
                pthread_mutex_unlock( &(this->runningMutex));
                msleep(DEVICE_READ_INTERVAL_MS);
            }
        #endif

        if( this->device->deviceErrorCallback )
        {
            (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_READER_THREAD_STOP));
        }
    }
    else
    {
        if( this->device->deviceErrorCallback )
        {
            (*(this->device->deviceErrorCallback))(*(this->device),HidError(HidError::DEVICE_READER_THREAD_SETUP));
        }
    }
}

bool HidDevice::HidDeviceReaderThread::isRunning()
{
    return this->running;
}

#ifdef OS_WINDOWS
    DWORD WINAPI HidDevice::HidDeviceReaderThread::threadFunc(void* classPtr)
    {
        ((HidDevice::HidDeviceReaderThread*)classPtr)->onStartHandler();
        return 0;
    }
#endif
#ifdef OS_LINUX
    void* HidDevice::HidDeviceReaderThread::threadFunc(void* classPtr)
    {
        ((HidDevice::HidDeviceReaderThread*)classPtr)->onStartHandler();
        return NULL;
    }
#endif

// ################################# HIDDEVICE::HIDDEVICEREADERTHREAD CLASS FUNCTION DEFINITIONS END ################################ //





// ######################################### COMMON HIDAPI CLASS FUNCTION DEFINITIONS BEGIN ######################################### //
bool          HidApi::isInitialized()                                               { return this->initialized;          }
void          HidApi::registerDeviceAddCallback(void (*fptr)(int,HidDevice))        { this->deviceAddCallback    = fptr; }
void          HidApi::registerDeviceRemoveCallback(void (*fptr)(int,HidDevice))     { this->deviceRemoveCallback = fptr; }
void          HidApi::registerApiErrorCallback(void (*fptr)(HidError))              { this->apiErrorCallback     = fptr; }
void          HidApi::registerDeviceErrorCallback(void (*fptr)(HidDevice,HidError)) { this->deviceErrorCallback  = fptr; }

HidDeviceList HidApi::getDeviceList()
{
    for( size_t i=0 ; i<this->devices.size() ; i++)
    {
        this->devices[i].registerDeviceErrorCallback(this->deviceErrorCallback);
    }
    return this->devices;
}

std::string HidApi::wcharArrayToString(const wchar_t* arr, int size)
{
    std::string result = "";
    if( !arr ) return result;

    if( size < 0 )
    {
        size = 0;
        while( arr[size] != 0 ){ ++size; }
    }

    result.resize(size);
    std::wcstombs(&result[0],arr,size);

    return result;
}

std::wstring HidApi::wcharArrayToWString(const wchar_t* arr, int size)
{
    if( !arr ) return L"";

    if( size < 0 )
    {
        size = 0;
        while( arr[size] != 0 ){ ++size; }
    }

    return std::wstring(arr,size);
}

std::wstring HidApi::stringToWString(std::string str)
{
    std::wstring wc;
    wc.resize( str.size()+1 );
    mbstowcs( &wc[0], &str[0], str.size()+1 );

    return wc;
}

std::wstring HidApi::charArrayToWString(const char *utf8)
{
    wchar_t *ret = NULL;

    if (utf8)
    {
        size_t wlen = mbstowcs(NULL, utf8, 0);
        if ((size_t) -1 == wlen) {
            return std::wstring( L"" );
        }
        ret = (wchar_t *)calloc(wlen+1, sizeof(wchar_t));
        mbstowcs(ret, utf8, wlen+1);
        ret[wlen] = 0x0000;
    }

    return HidApi::wcharArrayToWString(ret);
}

// ########################################## COMMON HIDAPI CLASS FUNCTION DEFINITIONS END ########################################## //





// ####################################### OS SPECIFIC HIDAPI CLASS FUNCTION DEFINITIONS BEGIN ###################################### //

HidApi::HidApi(void (*apiErrCb)(HidError))
{
    this->deviceAddCallback    = NULL;
    this->deviceRemoveCallback = NULL;
    this->deviceErrorCallback  = NULL;
    this->apiErrorCallback     = apiErrCb;

    #ifdef OS_WINDOWS
        this->libHandle    = LoadLibraryA("hid.dll");
        this->HidDllFuncs  = new struct HidDllFunctions;

        if (this->libHandle)
        {
            this->HidDllFuncs->GetAttributes         = (BOOLEAN  (WINAPI *)(HANDLE, PHIDD_ATTRIBUTES))    GetProcAddress(this->libHandle, "HidD_GetAttributes");
            this->HidDllFuncs->GetSerialNumberString = (BOOLEAN  (WINAPI *)(HANDLE, PVOID, ULONG))        GetProcAddress(this->libHandle, "HidD_GetSerialNumberString");
            this->HidDllFuncs->GetManufacturerString = (BOOLEAN  (WINAPI *)(HANDLE, PVOID, ULONG))        GetProcAddress(this->libHandle, "HidD_GetManufacturerString");
            this->HidDllFuncs->GetProductString      = (BOOLEAN  (WINAPI *)(HANDLE, PVOID, ULONG))        GetProcAddress(this->libHandle, "HidD_GetProductString");
            this->HidDllFuncs->SetFeature            = (BOOLEAN  (WINAPI *)(HANDLE, PVOID, ULONG))        GetProcAddress(this->libHandle, "HidD_SetFeature");
            this->HidDllFuncs->GetFeature            = (BOOLEAN  (WINAPI *)(HANDLE, PVOID, ULONG))        GetProcAddress(this->libHandle, "HidD_GetFeature");
            this->HidDllFuncs->GetIndexedString      = (BOOLEAN  (WINAPI *)(HANDLE, ULONG, PVOID, ULONG)) GetProcAddress(this->libHandle, "HidD_GetIndexedString");
            this->HidDllFuncs->GetPreparsedData      = (BOOLEAN  (WINAPI *)(HANDLE, void **))             GetProcAddress(this->libHandle, "HidD_GetPreparsedData");
            this->HidDllFuncs->FreePreparsedData     = (BOOLEAN  (WINAPI *)(void *))                      GetProcAddress(this->libHandle, "HidD_FreePreparsedData");
            this->HidDllFuncs->GetCaps               = (NTSTATUS (WINAPI *)(void *, HIDP_CAPS *))         GetProcAddress(this->libHandle, "HidP_GetCaps");
            this->HidDllFuncs->SetNumInputBuffers    = (BOOLEAN  (WINAPI *)(HANDLE, ULONG))               GetProcAddress(this->libHandle, "HidD_SetNumInputBuffers");
        }
        else
        {
            this->HidDllFuncs->GetAttributes         = NULL;
            this->HidDllFuncs->GetSerialNumberString = NULL;
            this->HidDllFuncs->GetManufacturerString = NULL;
            this->HidDllFuncs->GetProductString      = NULL;
            this->HidDllFuncs->SetFeature            = NULL;
            this->HidDllFuncs->GetFeature            = NULL;
            this->HidDllFuncs->GetIndexedString      = NULL;
            this->HidDllFuncs->GetPreparsedData      = NULL;
            this->HidDllFuncs->FreePreparsedData     = NULL;
            this->HidDllFuncs->GetCaps               = NULL;
            this->HidDllFuncs->SetNumInputBuffers    = NULL;
        }



        this->initialized = ((this->HidDllFuncs->GetAttributes)         and
                             (this->HidDllFuncs->GetSerialNumberString) and
                             (this->HidDllFuncs->GetManufacturerString) and
                             (this->HidDllFuncs->GetProductString)      and
                             (this->HidDllFuncs->SetFeature)            and
                             (this->HidDllFuncs->GetFeature)            and
                             (this->HidDllFuncs->GetIndexedString)      and
                             (this->HidDllFuncs->GetPreparsedData)      and
                             (this->HidDllFuncs->FreePreparsedData)     and
                             (this->HidDllFuncs->GetCaps)               and
                             (this->HidDllFuncs->SetNumInputBuffers));

    #endif

    #ifdef OS_LINUX
        if(!setlocale(LC_CTYPE, NULL)) setlocale(LC_CTYPE, "");
        this->initialized   = true;
    #endif

    if( this->initialized )
    {
        this->monitorThread = new HidApi::HidDeviceMonitoringThread(this);
        this->monitorThread->run();
    }

}

HidApi::~HidApi()
{
    #ifdef OS_WINDOWS
        if( this->libHandle ) FreeLibrary(this->libHandle);
        this->libHandle = NULL;
    #endif

    #ifdef OS_LINUX
    #endif

    this->monitorThread->stop();
    this->initialized = false;
}

HidDeviceList HidApi::scanDevices(unsigned short _vendorId,
                                  unsigned short _productId,
                                  const  wchar_t* _serial,
                                  const wchar_t* _manufacturer,
                                  const wchar_t* _product,
                                  unsigned short _release,
                                  unsigned short _usagePage,
                                  unsigned short _usage)
{
    this->devices.clear();

    if( !this->initialized )
    {
        if( this->apiErrorCallback ) (*(this->apiErrorCallback))(HidError(HidError::API_INITIALIZED));
        return this->devices;
    }

    #ifdef OS_WINDOWS
        BOOL                                res;
        int                                 deviceIndex = 0 ;
        SP_DEVINFO_DATA                     devinfoData;
        SP_DEVICE_INTERFACE_DATA            deviceInterfaceData;
        SP_DEVICE_INTERFACE_DETAIL_DATA_A * deviceInterfaceDetailData = NULL;
        GUID                                InterfaceClassGuid = {0x4d1e55b2, 0xf16f, 0x11cf, {0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30} };

        memset(&devinfoData,           0x0, sizeof(devinfoData));
        memset(&deviceInterfaceData,   0x0, sizeof(deviceInterfaceData));
        devinfoData.cbSize                = sizeof(SP_DEVINFO_DATA);
        deviceInterfaceData.cbSize        = sizeof(SP_DEVICE_INTERFACE_DATA);

        HDEVINFO deviceInfoSet            = SetupDiGetClassDevsA(&InterfaceClassGuid, NULL, NULL, DIGCF_PRESENT|DIGCF_DEVICEINTERFACE);



        while( SetupDiEnumDeviceInterfaces(deviceInfoSet, NULL, &InterfaceClassGuid, deviceIndex, &deviceInterfaceData) )
        {
            HANDLE          writeHandle   = INVALID_HANDLE_VALUE;
            DWORD           requiredSize  = 0;
            HIDD_ATTRIBUTES attrib;

            res = SetupDiGetDeviceInterfaceDetailA(deviceInfoSet, &deviceInterfaceData, NULL, 0, &requiredSize, NULL);

            deviceInterfaceDetailData         = (SP_DEVICE_INTERFACE_DETAIL_DATA_A*) malloc(requiredSize);
            deviceInterfaceDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

            res = SetupDiGetDeviceInterfaceDetailA(deviceInfoSet, &deviceInterfaceData, deviceInterfaceDetailData, requiredSize, NULL, NULL);

            if (!res)
            {
                free(deviceInterfaceDetailData);
                deviceIndex++;
                continue;
            }

            for (int i = 0; ; i++)
            {
                char driverName[256];

                res = SetupDiEnumDeviceInfo(deviceInfoSet, i, &devinfoData);
                if(!res) break;

                res = SetupDiGetDeviceRegistryPropertyA(deviceInfoSet,
                                                        &devinfoData,
                                                        SPDRP_CLASS,
                                                        NULL,
                                                        (PBYTE)driverName,
                                                        sizeof(driverName),
                                                        NULL);
                if(!res) break;

                if(strcmp(driverName, "HIDClass") == 0)
                {
                    res = SetupDiGetDeviceRegistryPropertyA(deviceInfoSet,
                                                            &devinfoData,
                                                            SPDRP_DRIVER,
                                                            NULL,
                                                            (PBYTE)driverName,
                                                            sizeof(driverName),
                                                            NULL);
                    if(res) break;
                }
            }


            if( !res )
            {
                free(deviceInterfaceDetailData);
                deviceIndex++;
                continue;
            }


            writeHandle = CreateFileA(deviceInterfaceDetailData->DevicePath,
                                      0,
                                      (FILE_SHARE_READ|FILE_SHARE_WRITE),
                                      NULL,
                                      OPEN_EXISTING,
                                      FILE_FLAG_OVERLAPPED,
                                      0);


            if( writeHandle == INVALID_HANDLE_VALUE)
            {
                CloseHandle(writeHandle);
                continue;
            }

            attrib.Size = sizeof(HIDD_ATTRIBUTES);

            void      *ppData     = NULL;
            HIDP_CAPS caps;
            wchar_t   serial[256];
            wchar_t   manufa[256];
            wchar_t   produc[256];
            bool      isPreparsed = false;
            bool      isAttr      = false;
            bool      isSerial    = false;
            bool      isManufa    = false;
            bool      isProduc    = false;


            if( this->HidDllFuncs->GetPreparsedData(writeHandle, &ppData) )
            {
                if(this->HidDllFuncs->GetCaps(ppData, &caps) == 0x110000)
                {
                    isPreparsed   = true;
                }

                this->HidDllFuncs->FreePreparsedData(ppData);
            }

            if( this->HidDllFuncs->GetAttributes(writeHandle, &attrib) )                        { isAttr   = true; }
            if( this->HidDllFuncs->GetSerialNumberString(writeHandle, serial, sizeof(serial) )) { isSerial = true; }
            if( this->HidDllFuncs->GetManufacturerString(writeHandle, manufa, sizeof(manufa) )) { isManufa = true; }
            if( this->HidDllFuncs->GetProductString(writeHandle, produc, sizeof(produc) ))      { isProduc = true; }



            if( ((_vendorId      == ANY) or ( isAttr      and ( _vendorId  == attrib.VendorID )      )) and
                ((_productId     == ANY) or ( isAttr      and ( _productId == attrib.ProductID )     )) and
                ((_serial        == ANY) or ( isSerial    and ( wcscmp( _serial, serial)       == 0 ))) and
                ((_manufacturer  == ANY) or ( isManufa    and ( wcscmp( _manufacturer, manufa) == 0 ))) and
                ((_product       == ANY) or ( isProduc    and ( wcscmp( _product, produc)      == 0 ))) and
                ((_release       == ANY) or ( isAttr      and ( _release   == attrib.VersionNumber ) )) and
                ((_usagePage     == ANY) or ( isPreparsed and ( _usagePage == caps.UsagePage )       )) and
                ((_usage         == ANY) or ( isPreparsed and ( _usage     == caps.Usage )           ))
            )
            {
                HidDevice temp;


                temp.path = std::string(deviceInterfaceDetailData->DevicePath);
                std::transform(temp.path.begin(), temp.path.end(), temp.path.begin(), ::toupper);

                if( isAttr )
                {
                    temp.vendorId  = attrib.VendorID;
                    temp.productId = attrib.ProductID;
                    temp.release   = attrib.VersionNumber;
                }

                if( isPreparsed )
                {
                    temp.usagePage = caps.UsagePage;
                    temp.usage     = caps.Usage;
                }

                if( isSerial ) { temp.serial       = HidApi::wcharArrayToWString(serial); }
                if( isManufa ) { temp.manufacturer = HidApi::wcharArrayToWString(manufa); }
                if( isProduc ) { temp.product      = HidApi::wcharArrayToWString(produc); }


                temp.interfaceNumber = 0;
                size_t foundAt = temp.path.find("&mi_");
                if (foundAt != std::string::npos )
                {
                    foundAt += 4;
                    temp.interfaceNumber = strtol(&(temp.path[foundAt]), NULL, 16);
                }
                temp.HidDllFuncs         = this->HidDllFuncs;
                temp.deviceErrorCallback = this->deviceErrorCallback;

                this->devices.push_back(temp);
            }

            CloseHandle(writeHandle);
            free(deviceInterfaceDetailData);
            deviceIndex++;
        }//main while loop

        SetupDiDestroyDeviceInfoList(deviceInfoSet);
    #endif


    #ifdef OS_LINUX
        udev *udev = udev_new();
        if( udev )
        {
            udev_enumerate *enumerate = udev_enumerate_new(udev);
            udev_enumerate_add_match_subsystem(enumerate, "hidraw");
            udev_enumerate_scan_devices(enumerate);
            for( udev_list_entry *i=udev_enumerate_get_list_entry(enumerate); i!=NULL ; i=udev_list_entry_get_next(i) )
            {
                udev_device *raw_dev = udev_device_new_from_syspath(udev, udev_list_entry_get_name(i));
                udev_device *hid_dev = udev_device_get_parent_with_subsystem_devtype(raw_dev, "hid", NULL);

                if( hid_dev )
                {
                    int bus_type            = 0;
                    unsigned short dev_vid  = 0;
                    unsigned short dev_pid  = 0;
                    unsigned short dev_rel  = 0;
                    int            dev_inf  = 0;
                    std::wstring serial     = L"";
                    std::wstring manufa     = L"";
                    std::wstring produc     = L"";

                    bool      isAttr      = false;
                    bool      isSerial    = false;
                    bool      isManufa    = false;
                    bool      isProduc    = false;
                    bool      isReleas    = false;

                    // VID, PID, NAME, BUS TYPE, SERIAL parsing
                    std::string ueventInfo = std::string( strdup(udev_device_get_sysattr_value(hid_dev, "uevent")) );
                    size_t lineBegin = 0;
                    size_t lineEnd   = ueventInfo.find('\n');
                    while( lineEnd!=std::string::npos )
                    {
                        std::string key  = "";
                        std::string line = ueventInfo.substr(lineBegin,lineEnd-lineBegin);

                        size_t eqFound   = line.find("=");
                        if( eqFound != std::string::npos )
                        {
                            key = line.substr(0,eqFound);
                            if( key == "HID_ID" )
                            {
                                size_t colonFound  = line.find(':',eqFound+1);
                                if( colonFound != std::string::npos )
                                {
                                    bus_type = static_cast<int>(strtol(line.substr(eqFound+1,colonFound-eqFound-1).c_str(),NULL,10));
                                }

                                size_t colonFound2 = line.find(':',colonFound+1);
                                if( colonFound2 != std::string::npos )
                                {
                                    dev_vid = static_cast<unsigned short>(strtol(line.substr(colonFound+1,colonFound2-colonFound-1).c_str(),NULL,16));
                                    dev_pid = static_cast<unsigned short>(strtol(line.substr(colonFound2+1,line.size()-1).c_str(),NULL,16));
                                    isAttr  = true;
                                }
                            }
                            else if( key == "HID_NAME" )
                            {
                                std::string temp = line.substr(eqFound+1,line.size()-eqFound-1);
                                produc           = HidApi::stringToWString(temp);
                                isProduc         = true;
                            }
                            else if( key == "HID_UNIQ" )
                            {
                                std::string temp = line.substr(eqFound+1,line.size()-eqFound-1);
                                serial           = HidApi::stringToWString(temp);
                                isSerial         = true;
                            }
                        }

                        lineBegin = lineEnd+1;
                        lineEnd   = ueventInfo.find('\n',lineBegin);
                    }


                    if( bus_type == BUS_USB )
                    {
                        udev_device *usb_dev = udev_device_get_parent_with_subsystem_devtype(raw_dev, "usb", "usb_device");
                        udev_device *if_dev  = udev_device_get_parent_with_subsystem_devtype(raw_dev, "usb", "usb_interface");

                        if(usb_dev)
                        {
                            std::string rel( udev_device_get_sysattr_value(usb_dev, "bcdDevice") );
                            dev_rel      = (rel.empty() ? 0x00 : static_cast<unsigned short>(strtol(rel.c_str(), NULL, 16)) );
                            isReleas     = true;

                            manufa       = HidApi::stringToWString( std::string( udev_device_get_sysattr_value(usb_dev, "manufacturer") ) );
                            isManufa     = true;

                            produc       = HidApi::stringToWString( std::string( udev_device_get_sysattr_value(usb_dev, "product") ) );
                            isProduc     = true;

                            if (if_dev)
                            {
                                std::string interf( udev_device_get_sysattr_value(if_dev, "bInterfaceNumber") );
                                dev_inf = (interf.empty() ? 0 : static_cast<unsigned short>(strtol(interf.c_str(), NULL, 16)) );
                            }
                        }
                    }



                    if(bus_type == BUS_USB or bus_type == BUS_BLUETOOTH)
                    {
                        if( ((_vendorId      == ANY) or ( isAttr      and ( _vendorId  == dev_vid )                    )) and
                            ((_productId     == ANY) or ( isAttr      and ( _productId == dev_pid )                    )) and
                            ((_serial        == ANY) or ( isSerial    and ( wcscmp( _serial, &(serial[0])) == 0 )      )) and
                            ((_manufacturer  == ANY) or ( isManufa    and ( wcscmp( _manufacturer, &(manufa[0])) == 0 ))) and
                            ((_product       == ANY) or ( isProduc    and ( wcscmp( _product, &(produc[0]))      == 0 ))) and
                            ((_release       == ANY) or ( isReleas    and ( _release   == dev_rel )                    ))
                        )
                        {
                            HidDevice temp;
                            temp.path                = std::string(udev_device_get_devnode(raw_dev));
                            temp.vendorId            = dev_vid;
                            temp.productId           = dev_pid;
                            temp.serial              = serial;
                            temp.release             = dev_rel;
                            temp.manufacturer        = manufa;
                            temp.product             = produc;
                            temp.interfaceNumber     = dev_inf;
                            temp.deviceErrorCallback = this->deviceErrorCallback;

                            this->devices.push_back(temp);
                        }
                    }
                }
                udev_device_unref(raw_dev);
            }
            udev_enumerate_unref(enumerate);
        }
        else
        {
            if( this->apiErrorCallback ) (*(this->apiErrorCallback))(HidError(HidError::API_UDEV_NEW));
        }
        udev_unref(udev);
    #endif


    return this->devices;
}

// ######################################## OS SPECIFIC HIDAPI CLASS FUNCTION DEFINITIONS END ####################################### //





// ################################ HIDAPI::HIDDEVICEMONITORINGTHREAD CLASS FUNCTION DEFINITIONS BEGIN ############################## //
HidApi::HidDeviceMonitoringThread::HidDeviceMonitoringThread(HidApi *pr)
{
    this->running               = false;
    this->parent                = pr;

    #ifdef OS_WINDOWS
        this->runningMutex      = CreateMutex(NULL, FALSE, NULL);
        this->threadHandle      = INVALID_HANDLE_VALUE;
    #endif

    #ifdef OS_LINUX
        pthread_mutex_init( &(this->runningMutex), NULL);
        this->threadHandle      = 0;
        struct udev *udev_      = udev_new();
        if( udev_ )
        {
            this->mon           = udev_monitor_new_from_netlink(udev_, "udev");
            if( this->mon )
            {
                udev_monitor_filter_add_match_subsystem_devtype(this->mon, "hidraw", NULL);
                udev_monitor_enable_receiving(this->mon);
            }
        }
    #endif
}

HidApi::HidDeviceMonitoringThread::~HidDeviceMonitoringThread()
{
    this->stop();

    #ifdef OS_WINDOWS
        CloseHandle(this->runningMutex);
    #endif

    #ifdef OS_LINUX
        pthread_mutex_destroy( &(this->runningMutex) );
    #endif
}

void HidApi::HidDeviceMonitoringThread::run()
{
    if( this->parent->initialized )
    {
        #ifdef OS_WINDOWS
            this->threadHandle = CreateThread( NULL,                                                       // default security attributes
                                               0,                                                          // use default stack size
                                               &HidApi::HidDeviceMonitoringThread::threadFunc,             // thread function name
                                               (void*)this,                                                // argument to thread function
                                               0,                                                          // use default creation flags
                                               0                                                           // pointer of thread identifier
                                             );

            if( (this->threadHandle == INVALID_HANDLE_VALUE) and this->parent->apiErrorCallback )
            {
                (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_CREATE));
            }
        #endif

        #ifdef OS_LINUX
            int res = pthread_create( &(this->threadHandle),
                                      NULL,
                                      &HidApi::HidDeviceMonitoringThread::threadFunc,
                                      (void*)(this)
                                    );

            if( (res != 0) and this->parent->apiErrorCallback )
            {
                (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_CREATE));
            }

        #endif
    }
    else
    {
        if( this->parent->apiErrorCallback )
        {
            (*(this->parent->apiErrorCallback))(HidError(HidError::API_INITIALIZED));
        }
    }
}

void HidApi::HidDeviceMonitoringThread::stop()
{
    #ifdef OS_WINDOWS
        WaitForSingleObject( this->runningMutex, INFINITE);
        this->running = false;
        ReleaseMutex(this->runningMutex);
    #endif

    #ifdef OS_LINUX
        pthread_mutex_lock( &(this->runningMutex));
        this->running = false;
        pthread_mutex_unlock( &(this->runningMutex));
    #endif
}

void HidApi::HidDeviceMonitoringThread::onStartHandler()
{
    if( this->parent->initialized )
    {

        #ifdef OS_WINDOWS
            WaitForSingleObject( this->runningMutex, INFINITE);
            this->running = true;
            ReleaseMutex(this->runningMutex);

            WNDCLASSEX  wc;
            wc.cbSize        = sizeof(WNDCLASSEX);
            wc.style         = 0;
            wc.lpfnWndProc   = &HidApi::HidDeviceMonitoringThread::onMessageReceivedHandler;
            wc.cbClsExtra    = 0;
            wc.cbWndExtra    = sizeof(HidApi::HidDeviceMonitoringThread*);
            wc.hInstance     = reinterpret_cast<HINSTANCE>(GetModuleHandle(0));
            wc.hIcon         = 0;
            wc.hCursor       = 0;
            wc.hbrBackground = 0;
            wc.lpszMenuName  = NULL;
            wc.lpszClassName = "MonitoringClassName";
            wc.hIconSm       = NULL;
            RegisterClassEx(&wc);


            HWND monitorWindow = CreateWindowEx( 0,                                               // extra style
                                                 "MonitoringClassName",                           // classname
                                                 "MonitoringClassName",                           // window name
                                                 0,                                               // style
                                                 0, 0, 0, 0,                                      // geometry
                                                 HWND_MESSAGE,                                    // parent (message-only window)
                                                 NULL,                                            // menu handle
                                                 reinterpret_cast<HINSTANCE>(GetModuleHandle(0)), // application handle
                                                 NULL);                                           // windows creation data

            if( monitorWindow )
            {
                SetWindowLongPtr(monitorWindow, 0, reinterpret_cast<LONG_PTR>(this));

                DEV_BROADCAST_DEVICEINTERFACE NotificationFilter ;
                memset(&NotificationFilter, 0, sizeof(DEV_BROADCAST_DEVICEINTERFACE)) ;
                NotificationFilter.dbcc_size        = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
                NotificationFilter.dbcc_devicetype  = DBT_DEVTYP_DEVICEINTERFACE;
                NotificationFilter.dbcc_classguid   = { 0x4d1e55b2, 0xf16f, 0x11cf, {0x88, 0xcb, 0x00, 0x11, 0x11, 0x00, 0x00, 0x30} };
                HDEVNOTIFY monitorNotify            = RegisterDeviceNotification(monitorWindow, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);

                if( monitorNotify )
                {
                    MSG msg;
                    while( this->running )
                    {
                        WaitForSingleObject( this->runningMutex, INFINITE);
                        if( PeekMessage(&msg, monitorWindow, 0, 0,PM_REMOVE) )
                        {
                            TranslateMessage(&msg);
                            DispatchMessage(&msg);
                        }
                        ReleaseMutex(this->runningMutex);
                        msleep(API_CHECK_DEVICES_INTERVAL_MS);
                    }
                }
                else
                {
                    if( this->parent->apiErrorCallback )
                    {
                        (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_SETUP));
                    }

                    WaitForSingleObject( this->runningMutex, INFINITE);
                    this->running = false;
                    ReleaseMutex(this->runningMutex);
                }
            }
            else
            {
                if( this->parent->apiErrorCallback )
                {
                    (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_SETUP));
                }

                WaitForSingleObject( this->runningMutex, INFINITE);
                this->running = false;
                ReleaseMutex(this->runningMutex);
            }
        #endif

        #ifdef OS_LINUX

            pthread_mutex_lock( &(this->runningMutex));
            this->running        = true;
            pthread_mutex_unlock( &(this->runningMutex));


            int fd = udev_monitor_get_fd(this->mon);

            if( fd < 0)
            {
                if( this->parent->apiErrorCallback )
                {
                    (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_SETUP));
                }

                pthread_mutex_lock( &(this->runningMutex));
                this->running        = false;
                pthread_mutex_unlock( &(this->runningMutex));
            }

            udev_device *dev;
            fd_set       fds;
            timeval      tv;
            tv.tv_sec  = 0;
            tv.tv_usec = 10000;

            while( this->running )
            {
                pthread_mutex_lock( &(this->runningMutex));
                FD_ZERO(&fds);
                FD_SET(fd, &fds);
                if( (select(fd+1, &fds, NULL, NULL, &tv) > 0) and FD_ISSET(fd, &fds) )
                {
                    dev = udev_monitor_receive_device(this->mon);
                    if( dev )
                    {
                        std::string node(udev_device_get_devnode(dev));
                        std::string act(udev_device_get_action(dev));
                        udev_device_unref(dev);

                        if( act == "add" )
                        {
                            this->parent->scanDevices();

                            int addedToList = -1;
                            for( size_t i = 0 ; i < this->parent->devices.size() ; i++)
                            {
                                if( this->parent->devices[i].path == node )
                                {
                                    addedToList = static_cast<int>(i);
                                    break;
                                }
                            }

                            if( addedToList >= 0 )
                            {
                                HidDevice added = this->parent->devices[addedToList];
                                if( this->parent->deviceAddCallback )
                                {
                                    (*(this->parent->deviceAddCallback))(addedToList, added);
                                }
                            }
                            else
                            {
                                if( this->parent->apiErrorCallback )
                                {
                                    (*(this->parent->apiErrorCallback))(HidError(HidError::API_ADD));
                                }
                            }
                        }
                        else if( act == "remove" )
                        {
                            int deleteFromList = -1;
                            for( size_t i = 0 ; i < this->parent->devices.size() ; i++)
                            {
                                if( this->parent->devices[i].path == node )
                                {
                                    deleteFromList = static_cast<int>(i);
                                    break;
                                }
                            }

                            if( deleteFromList >= 0 )
                            {
                                HidDevice removed = this->parent->devices[deleteFromList];
                                this->parent->devices.erase(this->parent->devices.begin() + deleteFromList);
                                if( this->parent->deviceRemoveCallback )
                                {
                                    (*(this->parent->deviceRemoveCallback))(deleteFromList, removed);
                                }
                            }
                            else
                            {
                                if( this->parent->apiErrorCallback )
                                {
                                    (*(this->parent->apiErrorCallback))(HidError(HidError::API_REMOVE));
                                }
                            }
                        }
                    }
                }
                pthread_mutex_unlock( &(this->runningMutex));
                msleep(API_CHECK_DEVICES_INTERVAL_MS);
            }
        #endif

        if( this->parent->apiErrorCallback )
        {
            (*(this->parent->apiErrorCallback))(HidError(HidError::API_MONITOR_THREAD_STOP));
        }
    }
    else
    {
        if( this->parent->apiErrorCallback )
        {
            (*(this->parent->apiErrorCallback))(HidError(HidError::API_INITIALIZED));
        }
    }


}

#ifdef OS_WINDOWS
    LRESULT CALLBACK HidApi::HidDeviceMonitoringThread::onMessageReceivedHandler(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
    {
        HidApi::HidDeviceMonitoringThread* _this = reinterpret_cast<HidApi::HidDeviceMonitoringThread*>(GetWindowLongPtr(hwnd, 0));

        if( _this )
        {
            if( message == WM_DEVICECHANGE )
            {
                DEV_BROADCAST_HDR *lpdb = (DEV_BROADCAST_HDR *)lParam;
                if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
                {
                    PDEV_BROADCAST_DEVICEINTERFACE pDevInf = (PDEV_BROADCAST_DEVICEINTERFACE)lpdb;
                    std::string node(pDevInf->dbcc_name);
                    std::transform(node.begin(), node.end(), node.begin(), ::toupper);

                    if( wParam == DBT_DEVICEARRIVAL)
                    {
                        _this->parent->scanDevices();

                        int addedToList = -1;
                        for( size_t i = 0 ; i < _this->parent->devices.size() ; i++)
                        {
                            if( _this->parent->devices[i].path == node )
                            {
                                addedToList = static_cast<int>(i);
                                break;
                            }
                        }

                        if( addedToList >= 0 )
                        {
                            HidDevice added = _this->parent->devices[addedToList];
                            if( _this->parent->deviceAddCallback )
                            {
                                (*(_this->parent->deviceAddCallback))(addedToList, added);
                            }
                        }
                        else
                        {
                            if( _this->parent->apiErrorCallback )
                            {
                                (*(_this->parent->apiErrorCallback))(HidError(HidError::API_ADD));
                            }
                        }
                    }
                    else if( wParam == DBT_DEVICEREMOVECOMPLETE)
                    {
                        int deleteFromList = -1;
                        for( size_t i = 0 ; i < _this->parent->devices.size() ; i++)
                        {
                            if( _this->parent->devices[i].path == node )
                            {
                                deleteFromList = static_cast<int>(i);
                                break;
                            }
                        }

                        if( deleteFromList >= 0 )
                        {
                            HidDevice removed = _this->parent->devices[deleteFromList];
                            _this->parent->devices.erase(_this->parent->devices.begin() + deleteFromList);
                            if( _this->parent->deviceRemoveCallback )
                            {
                                (*(_this->parent->deviceRemoveCallback))(deleteFromList, removed);
                            }
                        }
                        else
                        {
                            if( _this->parent->apiErrorCallback )
                            {
                                (*(_this->parent->apiErrorCallback))(HidError(HidError::API_REMOVE));
                            }
                        }
                    }
                }
            }
        }

        return DefWindowProc(hwnd, message, wParam, lParam);
    }

    DWORD WINAPI HidApi::HidDeviceMonitoringThread::threadFunc(void* classPtr)
    {
        ((HidApi::HidDeviceMonitoringThread*)classPtr)->onStartHandler();
        return 0;
    }
#endif
#ifdef OS_LINUX
    void* HidApi::HidDeviceMonitoringThread::threadFunc(void* classPtr)
    {
        ((HidApi::HidDeviceMonitoringThread*)classPtr)->onStartHandler();
        return NULL;
    }
#endif

// ################################# HIDAPI::HIDDEVICEMONITORINGTHREAD CLASS FUNCTION DEFINITIONS END ############################### //

