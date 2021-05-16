#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

#include <chrono>
#include <thread>
#include <cmath>

int main() {
	// Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_9600);
	// SerialPort serialPort("/dev/ttyACM0", 13000);
    serialPort.SetTimeout(10); // Block when reading until any data is received
	serialPort.Open();

	// Write some ASCII datae
	const float T_MS = 100;


	// Read some data back

	for(int i = 0; i < 1000; i++)
    {
	    float setPoint = std::sin((float)i/10.0)*16.0;
	    if (setPoint < 4)
        {
	        setPoint = 4;
        }
	    std::stringstream ss;
	    ss << "v " << setPoint << "," << -1*setPoint << "\n";

	    std::cout << "Setting:\n" << ss.str() << std::endl;

	    serialPort.Write(ss.str());


	    std::string msg;
        std::string readData;
         do{
            serialPort.Read(readData);
            msg += readData;
        }while(readData != "\n");
        std::cout << "Received data: " << msg << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


	// Close the serial port
	serialPort.Close();
}