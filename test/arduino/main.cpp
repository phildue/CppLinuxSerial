#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>

#include "SerialProtocol.h"

int main() {
    SerialProtocol serial;
    // Create serial port object and open serial port
    serial._serialPort.SetBaudRate(BaudRate::B_9600);
    serial._serialPort.SetDevice("/dev/ttyACM0");
    // SerialPort serialPort("/dev/ttyACM0", 13000);
    serial._serialPort.SetTimeout(100); // Block when reading until any data is received
    serial._serialPort.Open();

    // Write some ASCII datae
    const float T_MS = 100;


    for(int i = 0; i < 1000; i++)
    {
        float setPoint = std::sin((float)i/10.0)*16.0;
        if (setPoint < 4)
        {
            setPoint = 4;
        }

        auto msgOut = std::make_shared<SerialProtocol::MsgCmdVel>(setPoint,-1*setPoint,i);
        std::cout << "Sending:\n" << msgOut->str() << std::endl;

        serial.send(msgOut);
        try{
            serial.read(10);

        }catch(const SerialProtocol::ParseError& e)
        {
            std::cerr << e.what()  << "\n Message: " << e._parsedMessage << std::endl;

        }

        //Handle newest state message
        if(!serial._messagesState.empty())
        {
            auto msg = serial._messagesState[serial._messagesState.size()-1];
            std::cout << msg->str() << std::endl;
        }

        for (const auto& msg : serial._messages)
        {
            std::cout << msg->str() << std::endl;
        }

        serial._messagesState.clear();
        serial._messages.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    // Close the serial port
    serial._serialPort.Close();
}