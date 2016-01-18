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



#include <iostream>
#include <stdint.h>
#include "HidApi.h"
using namespace std;



// Arzawa Instrument's Frequency Meter device commands
const uint8_t CMD_GETTIME    = 0x10;
const uint8_t CMD_SETGTIME   = 0x11;
const uint8_t CMD_BEGINMEAS  = 0x20;
const uint8_t CMD_ABORTMEAS  = 0x21;
const uint8_t CMD_GETFREQ    = 0x22;
const uint8_t CMD_ENTERDFU   = 0x30;
const uint8_t CMD_GETHWINFO  = 0x40;
const uint8_t CMD_GETSWINFO  = 0x41;

// Arzawa Instrument's Frequency Meter device commands responses
const uint8_t RSP_OK         = 0x00;
const uint8_t RSP_BUSY       = 0x01;
const uint8_t RSP_INVALIDCMD = 0x02;
const uint8_t RSP_BADPARAM   = 0x03;
const uint8_t RSP_ERR0       = 0x04;
const uint8_t RSP_ERR2       = 0x05;
const uint8_t RSP_ERR3       = 0x06;
const uint8_t RSP_ERR4       = 0x07;



// Callback functions' prototypes
void hidApiErrorCb(HidError err);
void deviceAddedCb(int index, HidDevice dev);
void deviceRemovedCb(int index, HidDevice dev);
void hidDeviceErrorCb(HidDevice dev, HidError err);



// Global variables
HidApi        api(&hidApiErrorCb);
HidDeviceList devList;



int main()
{
    // Flushes output stream buffer, after each character inserted
    std::cout.setf( std::ios_base::unitbuf );



    // Register callbacks to HidApi
    api.registerApiErrorCallback( &hidApiErrorCb );
    api.registerDeviceErrorCallback( &hidDeviceErrorCb );
    api.registerDeviceAddCallback( &deviceAddedCb );
    api.registerDeviceRemoveCallback( &deviceRemovedCb );



//    Arzawa Instrument's Frequency Meter device properties
//    --------------------------------------------------
//       Path        : \\?\HID#VID_0483&PID_3256&MI_00#8&6A72FA&0&0000#{4D1E55B2-F16F-11CF-88CB-001111000030}
//       Manufacturer: Arzawa Instruments
//       Product     : Arzawa FrMeter ARZFC1
//       Serial      : 204EB0425642
//       Vendor Id   : 1155
//       Product Id  : 12886
//       Release No  : 512
//       Usage Page  : 255
//       Usage       : 1
//       Interface   : 0
//       --------------------------------------------------



    // Scan(enumarate) devices with filter parameters
    devList = api.scanDevices( ANY,                // vendor id
                               ANY,                // product id
                               L"204EB0425642",    // serial
                               ANY,                // manufacturer
                               ANY,                // product string
                               512,                // release
                               ANY,                // usage page
                               ANY                 // usage
                             );



    // Create empty HidDevice object
    HidDevice freqMeter;




    std::cout << "HidAPi is inited? : " << std::boolalpha << api.isInitialized() << std::endl;
    std::cout << "Found device count: " << devList.size() << std::endl;



    // Prints found devices' properties
    for( size_t i=0 ; i<devList.size() ; i++ )
    {
        std::wcout << i+1 << ". DEVICE\n"
                   << "   Path        : " << devList[i].getPath().c_str()  << std::endl
                   << "   Manufacturer: " << devList[i].getManufacturer()  << std::endl
                   << "   Product     : " << devList[i].getProductString() << std::endl
                   << "   Serial      : " << devList[i].getSerial()        << std::endl
                   << "   Vendor Id   : " << devList[i].getVendorId()      << std::endl
                   << "   Product Id  : " << devList[i].getProductId()     << std::endl
                   << "   Release No  : " << devList[i].getRelease()       << std::endl
                   << "   Usage Page  : " << devList[i].getUsagePage()     << std::endl
                   << "   Usage       : " << devList[i].getUsage()         << std::endl
                   << "   Interface   : " << devList[i].getInterface()     << std::endl
                   << "--------------------------------------------------" << std::endl;



        // If device is Arzawa Instrument's Frequency Meter device, it will be recorded to "freqMeter" object
        if( devList[i].getProductId() == 12886 )
        {
            freqMeter = devList[i];
            std::cout << "Arzawa Instrument's Frequency Meter is found" << std::endl;
        }
    }



    std::cout << std::endl << " <<<<<< OPERATIONS ARE STARTED >>>>>> " << std::endl << std::endl;



    // Generate command
    std::string HidData;
    HidData.resize(65, 0);
    HidData[0]  = 0;
    HidData[1]  = static_cast<char>(CMD_GETHWINFO);



    if( freqMeter.isInitialized() )
    {
        bool isOpen = freqMeter.open();
        std::cout << "open result : " << std::boolalpha << isOpen << std::endl;

        if( isOpen )
        {
            std::cout << "write result: " << freqMeter.write(HidData) << std::endl;
            std::cout << "read result : " << std::endl << freqMeter.read(-1) << std::endl << std::endl; // blocking read
        }
    }


    HidData[1]  = static_cast<char>(CMD_GETSWINFO);

    if( freqMeter.isInitialized() )
    {
        bool isOpen = freqMeter.open();
        std::cout << "open result : " << std::boolalpha << isOpen << std::endl;

        if( isOpen )
        {
            std::cout << "write result: " << freqMeter.write(HidData) << std::endl;
            std::cout << "read result : " << std::endl << freqMeter.read(100) << std::endl << std::endl; // timed read
        }
    }



    HidData[1]  = static_cast<char>(CMD_GETHWINFO);
    int i       = 0;

    // write banging
    for( ; i<8 ; i++ )
    {
        std::cout << "write result " << i+1 << ": " << freqMeter.write(HidData) << std::endl;
    }

    std::cout << std::endl;


    // wait approximately background reader thread's 8 cycle time (DEVICE_READ_INTERVAL_MS + POLLING_TIME_MS)
    msleep( 8 * (DEVICE_READ_INTERVAL_MS + POLLING_TIME_MS) );


    // reads from internal fifo buffer
    while( freqMeter.readAvailable() )
    {
        std::cout << "available data count : " << freqMeter.readAvailable() << std::endl;
        std::cout << "read result          : " << std::endl << freqMeter.read(0) << std::endl << std::endl; // immediate read
    }



    // main thread still running
    while(true)
    {
        std::cout << "." ;
        sleep(1);
    }
    return 0;
}








// ######################################### CALLBACK FUNCTIONS' DEFINITIONS BEGIN ######################################### //

void hidApiErrorCb(HidError err)
{
    std::cout << "Api error occured:     " << err.getErrorCode() << " - " << err.getErrorString() << std::endl;
}

void deviceAddedCb(int index, HidDevice dev)
{
    std::cout << "\nThis device added:   " << dev.getPath() << std::endl;
    devList = api.getDeviceList();
}

void deviceRemovedCb(int index, HidDevice dev)
{
    std::cout << "\nThis device removed: " << dev.getPath() << std::endl;
    devList = api.getDeviceList();
}

void hidDeviceErrorCb(HidDevice dev, HidError err)
{
    std::cout << "Device error occured:  " << err.getErrorCode() << " - " << err.getErrorString() << " - " << dev.getPath() << std::endl;
}

// ########################################## CALLBACK FUNCTIONS' DEFINITIONS END ########################################## //










// ##################################################### EXAMPLE OUTPUT #################################################### //
//
//    HidAPi is inited? : true
//    Found device count: 1
//    1. DEVICE
//    Path        : \\?\HID#VID_0483&PID_3256&MI_00#8&6A72FA&0&0000#{4D1E55B2-F16F-11CF-88CB-001111000030}
//    Manufacturer: Arzawa Instruments
//    Product     : Arzawa FrMeter ARZFC1
//    Serial      : 204EB0425642
//    Vendor Id   : 1155
//    Product Id  : 12886
//    Release No  : 512
//    Usage Page  : 255
//    Usage       : 1
//    Interface   : 0
//    --------------------------------------------------
//    Arzawa Instrument's Frequency Meter is found
//
//    <<<<<< OPERATIONS ARE STARTED >>>>>>
//
//    open result : true
//    write result: 65
//    read result :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    open result : true
//    write result: 65
//    read result :
//    A Sw:Awa FrMeter FC-1
//    SwType:1GHz-32K
//
//
//    write result 1: 65
//    write result 2: 65
//    write result 3: 65
//    write result 4: 65
//    write result 5: 65
//    write result 6: 65
//    write result 7: 65
//    write result 8: 65
//
//    available data count : 8
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 7
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 6
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 5
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 4
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 3
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 2
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    available data count : 1
//    read result          :
//    @ Device:Awa FrMeter FC-1
//    HwType:1GHz-32K
//    SeriNo:12345498
//
//    .......
//    This device removed: \\?\HID#VID_0483&PID_3256&MI_00#8&6A72FA&0&0000#{4D1E55B2-F16F-11CF-88CB-001111000030}
//    ....
//    This device added:   \\?\HID#VID_0483&PID_3256&MI_00#8&6A72FA&0&0000#{4D1E55B2-F16F-11CF-88CB-001111000030}
//    ...............
//
// ##################################################### EXAMPLE OUTPUT #################################################### //
