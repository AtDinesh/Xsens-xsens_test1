/*	Copyright (c) 2003-2015 Xsens Technologies B.V. or subsidiaries worldwide.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1.	Redistributions of source code must retain the above copyright notice,
        this list of conditions and the following disclaimer.

    2.	Redistributions in binary form must reproduce the above copyright notice,
        this list of conditions and the following disclaimer in the documentation
        and/or other materials provided with the distribution.

    3.	Neither the names of the copyright holders nor the names of their contributors
        may be used to endorse or promote products derived from this software without
        specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
    THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
    OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
    TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <fstream>
#include <cstdlib>

#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

int main(int argc, char* argv[])
{
    DeviceClass device;

    try
    {
        // Scan for connected USB devices
        std::cout << "Scanning for USB devices..." << std::endl;
        XsPortInfoArray portInfoArray;
        xsEnumerateUsbDevices(portInfoArray);
        if (!portInfoArray.size())
        {
            std::string portName;
            int baudRate;
#ifdef WIN32
            std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. COM1): " <<
#else
            std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. /dev/ttyUSB0): " <<
#endif
            std::endl;
            std::cin >> portName;
            std::cout << "Please enter baud rate (eg. 115200): ";
            std::cin >> baudRate;

            XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
            portInfoArray.push_back(portInfo);
        }

        // Use the first detected device
        XsPortInfo mtPort = portInfoArray.at(0);

        // Open the port with the detected device
        std::cout << "Opening port..." << std::endl;
        if (!device.openPort(mtPort))
            throw std::runtime_error("Could not open port. Aborting.");

        // Put the device in configuration mode
        std::cout << "Putting device into configuration mode..." << std::endl;
        if (!device.gotoConfig()) // Put the device into configuration mode before configuring the device
        {
            throw std::runtime_error("Could not put device into configuration mode. Aborting.");
        }

        // Request the device Id to check the device type
        mtPort.setDeviceId(device.getDeviceId());

        // Check if we have an MTi / MTx / MTmk4 device
        if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isLegacyMtig() && !mtPort.deviceId().isMtMk4() && !mtPort.deviceId().isFmt_X000())
        {
            throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
        }
        std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

        try
        {
            //variable needed to store data
            std::ofstream data_file;
            std::string fileName;

            // Print information about detected MTi / MTx / MTmk4 device
            std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

            // Configure the device. Note the differences between MTix and MTmk4
            std::cout << "Configuring the device..." << std::endl;
            if (mtPort.deviceId().isMt9c() || mtPort.deviceId().isLegacyMtig())
            {
                XsOutputMode outputMode = XOM_Calibrated; // output calibrated data
                XsOutputSettings outputSettings = XOS_CalibratedMode_AccGyrOnly; // output calibrated accelerometer and gyrometer

                // set the device configuration
                if (!device.setDeviceMode(outputMode, outputSettings))
                {
                    throw std::runtime_error("Could not configure MT device. Aborting.");
                }
            }
            //For MTi-1 series this condition will be used
            else if (mtPort.deviceId().isMtMk4() || mtPort.deviceId().isFmt_X000())
            {
                XsOutputConfiguration acc(XDI_Acceleration, 1000);
                XsOutputConfiguration rate_of_turn(XDI_RateOfTurn, 1000);
                XsOutputConfiguration time(XDI_SampleTimeFine, 1000);
                XsOutputConfigurationArray configArray;
                configArray.push_back(acc);
                configArray.push_back(rate_of_turn);
                configArray.push_back(time);
                if (!device.setOutputConfiguration(configArray))
                {

                    throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
                }

            }
            else
            {
                throw std::runtime_error("Unknown device while configuring. Aborting.");
            }

            //export data to csv ?
            bool export_csv = false;
            std::cout << "Do you ant to export data to csv ? (1 = Yes, else = No)";
            if(!(std::cin >> export_csv) || export_csv==false){
                std::cout << "Not exporting data" << std::endl;
            }
            else{
                std::cout << "Enter the file name : ";
                std::cin >> fileName;
                data_file.open(fileName.c_str());

                //error checking
                    if (!data_file)
                    {
                        std::cerr << "File could not be opened." << std::endl;
                        exit(1);
                    }
                    else
                        data_file << "Timestamp" << ";" <<"acc_X" << ";" << "acc_Y" << ";" << "acc_Z" << ";" << "gyro_X" << ";" << "gyro_Y" << ";" << "gyro_Z" << std::endl;
            }
            std::cin.clear();

            // Put the device in measurement mode
            std::cout << "Putting device into measurement mode..." << std::endl;
            if (!device.gotoMeasurement())
            {
                throw std::runtime_error("Could not put device into measurement mode. Aborting.");
            }

            std::cout << "\nMain loop (press any key to quit)" << std::endl;
            std::cout << std::string(79, '-') << std::endl;

            XsByteArray data;
            XsMessageArray msgs;
            while (!_kbhit())
            {
                device.readDataToBuffer(data);
                device.processBufferedData(data, msgs);
                for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
                {
                    // Retrieve a packet
                    XsDataPacket packet;
                    if ((*it).getMessageId() == XMID_MtData) {
                        LegacyDataPacket lpacket(1, false);
                        lpacket.setMessage((*it));
                        lpacket.setXbusSystem(false);
                        lpacket.setDeviceId(mtPort.deviceId(), 0);
                        lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
                        XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
                    }
                    //For MTi-1 series imu sensors
                    else if ((*it).getMessageId() == XMID_MtData2) {
                        packet.setMessage((*it));
                        packet.setDeviceId(mtPort.deviceId());
                    }

                    //Get timestamp
                    uint32_t timestamp = packet.sampleTimeFine();
                    std::cout << ",timestamp :   " << timestamp;

                    // Get the quaternion data
                    XsVector acceleration = packet.calibratedAcceleration();
                    std::cout << "\r"
                              << "acc_X:" << std::setw(5) << std::fixed << std::setprecision(2) << acceleration[0]
                              << ",acc_Y:" << std::setw(5) << std::fixed << std::setprecision(2) << acceleration[1]
                              << ",acc_Z:" << std::setw(5) << std::fixed << std::setprecision(2) << acceleration[2]
                    ;

                    // Get gyro calibrated measurements
                    XsVector gyro = packet.calibratedGyroscopeData();
                    std::cout << ",gyro_X:" << std::setw(7) << std::fixed << std::setprecision(2) << gyro[0]
                              << ",gyro_Y:" << std::setw(7) << std::fixed << std::setprecision(2) << gyro[1]
                              << ",gyro_Z:" << std::setw(7) << std::fixed << std::setprecision(2) << gyro[2]
                    ;

                    if(data_file) //save data
                    {
                        data_file << timestamp << ";" << acceleration[0] << ";" << acceleration[1] << ";" << acceleration[2] << ";" << gyro[0] << ";" << gyro[1] << ";" << gyro[2] << std::endl;
                    }
                    std::cout << std::flush;
                }
                msgs.clear();
                XsTime::msleep(0);
            }
            _getch(); // user pressed a key to exit
            std::cout << "\n" << std::string(79, '-') << "\n";
            std::cout << std::endl;
            //close file if opened
            if(data_file){
                data_file.close();
                std::cout << "data exported to " << fileName << std::endl;
            }
        }
        catch (std::runtime_error const & error)
        {
            std::cout << error.what() << std::endl;
        }
        catch (...)
        {
            std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        }

        // Close port
        std::cout << "Closing port..." << std::endl;
        device.close();
    }
    catch (std::runtime_error const & error)
    {
        std::cout << error.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    }

    std::cout << "Successful exit." << std::endl;

    std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

    return 0;
}
