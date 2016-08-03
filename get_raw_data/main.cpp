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
#include <unordered_map>
#include <bitset>
#include <sstream>

#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

union{ //BEWARE : THIS MAKES THE CONVERSION COMPILER DEPENDANT !!
    int num;
    float fnum;
} my_union;

//split the message in strings for further analyze
void split_xsmessage_string(XsMessage &src, std::vector<std::string> &dest){
    dest.clear();
    int len_pos = 12;
    int msg_begin = 8;
    uint32_t msg_size = src.getTotalMessageSize();
    XsString msg_b = src.toHexString();

    std::string str_msg = msg_b.toStdString();
    str_msg.erase(std::remove(str_msg.begin(), str_msg.end(), ' '), str_msg.end());
    //msg format : |PRE|BID|DATA_FORMAT(MT_DATA2))|LEN_TOTAL_MSG|MID|LEN|DATA|MID|LEN|DATA|....|MID|LEN|DATA|CS|
    //              1   2       3                      4         5-6   7  ... 
    // we want to sparse the incoming message and put the different XsString in a vector in order to compute then easily
    // 1 byte -> ex FF =~ 0xFF ==> 2 characters needed

    //the structure of the first packet contained in the message is already known
    if (dest.size()==0){
        unsigned int x = strtoul(str_msg.substr(len_pos, 2).c_str(), NULL, 16);
        dest.push_back(str_msg.substr(msg_begin,len_pos+4+(x*2)-msg_begin)); //+4 for MID
        msg_begin = len_pos+2+(x*2);
        len_pos = 4;
    }
    while(msg_begin < str_msg.size()-2){ //-2 CHECK_SUM byte
        unsigned int x = strtoul(str_msg.substr(msg_begin+len_pos, 2).c_str(), NULL, 16);
        dest.push_back(str_msg.substr(msg_begin,4+(x*2)));
        msg_begin = msg_begin + 4+(x*2);
    }
}

//split the message in an unordered_map for further analyze. Main advantage : easy access to data (values) using MID (key string)
void split_xsmessage_map(XsMessage &src, std::unordered_map<std::string, std::string> &dest){
    dest.clear();
    int len_pos = 12;
    int msg_begin = 8;
    uint32_t msg_size = src.getTotalMessageSize();
    XsString msg_b = src.toHexString();

    std::string str_msg = msg_b.toStdString();
    str_msg.erase(std::remove(str_msg.begin(), str_msg.end(), ' '), str_msg.end());
    //msg format : |PRE|BID|DATA_FORMAT(MT_DATA2))|LEN_TOTAL_MSG|MID|LEN|DATA|MID|LEN|DATA|....|MID|LEN|DATA|CS|
    //              1   2       3                      4         5-6   7  ... 
    // we want to sparse the incoming message and put the different XsString in a vector in order to compute then easily
    // 1 byte -> ex FF =~ 0xFF ==> 2 characters needed

    //the structure of the first packet contained in the message is already known
    if (dest.size()==0){
        unsigned int x = strtoul(str_msg.substr(len_pos, 2).c_str(), NULL, 16);
        dest.insert(std::make_pair<std::string,std::string>(str_msg.substr(msg_begin,4),str_msg.substr(msg_begin+4,x*2)));
        msg_begin = len_pos+2+(x*2);
        len_pos = 4;
    }
    while(msg_begin < str_msg.size()-2){ //-2 CHECK_SUM byte
        unsigned int x = strtoul(str_msg.substr(msg_begin+len_pos, 2).c_str(), NULL, 16);
        std::cout << "size of data to order : " << x << std::endl;
        printf("x = %d\n", x);
        dest.insert(std::make_pair<std::string,std::string>(str_msg.substr(msg_begin,4),str_msg.substr(msg_begin+6,(int)x*2)));
        msg_begin = msg_begin +6+(x*2);
    }
}

void extract_accgyro(std::string data_string,std::vector<double> &dest){
    // std::stringstream not working because we need IEEE754 convention... that is also used by compiler
    // we take profit of the fact that most compiler use this convention and we use a union to convert data but makes the conversion COMPILER DEPENDANT
    if (data_string.length() != 24)
        std::cout << "wrong size for data : " << data_string.length() << std::endl;
    std::cout << "data : " << data_string << std::endl;

    char c[11];
    for(int i = 0; i<3; i++){
    strcpy(c,"0x");
    strcat(c, data_string.substr(i*8, 8).c_str());
    //std::cout << "strcat : " << c << std::endl;

    long hex_value = std::strtol(c,0,16);
    //std::cout << "hex value: " << hex_value << std::endl;
    my_union.num = hex_value;
    //printf("%f\n", (my_union.fnum)/9.81);
    dest[i] = (my_union.fnum)/9.81;
    }
    std::cout << "\tdest[0]=" << dest[0] << "\tdest[1]=" << dest[1] << "\tdest[2]=" << dest[2] << std::endl; 
}

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
                XsOutputConfiguration acc(XDI_AccelerationHR, 1000);
                XsOutputConfiguration rate_of_turn(XDI_RateOfTurnHR, 1000);
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

                    XsMessage msg = packet.toMessage();

                    /*XsSize msg_size = msg.getTotalMessageSize();
                    XsXbusMessageId msg_Id = msg.getMessageId();
                    uint8_t msg_data_byte = msg.getDataByte();
                    float msg_float = msg.getDataFloat();
                    XsString msg_b = msg.toHexString();                                        
                    

                    std::cout << "Total size is : " << msg_size << std::endl;
                    std::cout << "Data_Byte : " << std::hex << +msg_data_byte << std::endl;
                    std::cout << "msg_Id : " << msg_Id << std::endl;
                    std::cout << "msg_b: " << msg_b << std::endl;*/
                    
                    //std::vector<std::string> msg_vect;
                    std::unordered_map<std::string,std::string> msg_map;
                    //split_xsmessage_string(msg, msg_vect);
                    split_xsmessage_map(msg, msg_map);

                    // string 1060 : timestamp (SampleTimeFine)
                    // string 4020 : Acceleration
                    // string 8020 : RateOfTurn
                    // string 4040 : AccelerationHR
                    // string 8040 : RateOfTurnHR
                    std::vector<double> Acceleration(3);
                    /*if(msg_map.find("4040") !=  msg_map.end())
                        extract_accgyro(msg_map["4040"], Acceleration);
                    else std::cout << "key 4040 not found" << std::endl;*/
                    
                    //std::cout << "number of messages in map : " << msg_map.size() << std::endl;
                    //std::cout << "number of messages : " << msg_vect.size() << std::endl;
                    //Get timestamp
                    /*uint32_t timestamp = packet.sampleTimeFine();
                    std::cout << ",timestamp :   " << timestamp;*/

                    /*// Get the quaternion data
                    XsVector acceleration = packet.calibratedAcceleration();
                    //std::cout << "number of item in packet : " << packet.itemCount() << std::endl;
                    //XsUShortVector acceleration = packet.rawAcceleration();

                    for(int it=0; it<=2; it++){
                    acceleration[it] = acceleration[it]/9.81;
                    }

                    std::cout << "\r"
                              << "acc_X:" << std::setw(5) << std::fixed << std::setprecision(3) << acceleration[0]
                              << ",acc_Y:" << std::setw(5) << std::fixed << std::setprecision(3) << acceleration[1]
                              << ",acc_Z:" << std::setw(5) << std::fixed << std::setprecision(3) << acceleration[2]
                    ;

                    // Get gyro calibrated measurements
                    XsVector gyro = packet.calibratedGyroscopeData();
                    //XsUShortVector gyro = packet.rawGyroscopeData();

                    for(int it=0; it<=2; it++){
                    gyro[it] = gyro[it]*180.0/3.1415;
                    }

                    std::cout << ",gyro_X:" << std::setw(7) << std::fixed << std::setprecision(3) << gyro[0]
                              << ",gyro_Y:" << std::setw(7) << std::fixed << std::setprecision(3) << gyro[1]
                              << ",gyro_Z:" << std::setw(7) << std::fixed << std::setprecision(3) << gyro[2]
                    ;*/

                    if(data_file) //save data
                    {
                        //data_file << timestamp << ";" << acceleration[0] << ";" << acceleration[1] << ";" << acceleration[2] << ";" << gyro[0] << ";" << gyro[1] << ";" << gyro[2] << std::endl;
                        //data_file << timestamp << ";" << acceleration[0] << ";" << acceleration[1] << ";" << acceleration[2] << std::endl;
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
