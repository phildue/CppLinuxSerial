#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>


class SerialProtocol
{
public:


    enum class MsgType{
        UNKNOWN,
        CMD_VEL,
        STATE
    };
    struct Message
    {
    public:
        Message(const std::string& msg, MsgType type = MsgType::UNKNOWN):_str(to_string(type) + " " + msg + "\n"),_type(type){}
        const std::string& serialStr() {return _str;};
        const MsgType& type() {return _type;};
        virtual std::string str() { return _str;}

        static MsgType msgType(const std::string& str)
        {
            if ( str == "S")
            {
                return MsgType::STATE;
            }
            if ( str == "v" )
            {
                return MsgType::CMD_VEL;

            }
            return MsgType::UNKNOWN;
        }
        static std::string to_string(MsgType msgType)
        {
            switch(msgType)
            {
                case MsgType ::CMD_VEL:
                    return "v";
                case MsgType ::STATE:
                    return "S";
                default:
                    return "";
            }
        }
    protected:
        std::string _str;
        MsgType _type;



    };
    struct MsgCmdVel : public Message
    {
        float _vl,_vr;

        MsgCmdVel(const std::string& raw):Message(raw,MsgType::CMD_VEL)
        {
            auto values = split(raw,' ');
            _vl = std::stod(values[0]);
            _vr = std::stod(values[1]);

        }
        MsgCmdVel(float vl, float vr):Message("",MsgType::CMD_VEL),_vl(vl),_vr(vr)
        {
            std::stringstream ss;
            ss << to_string(_type) << " " << vl << " " << vr << "\n";
            _str = ss.str();
        }
    };

    struct State
    {
        float angularVelocityCmd, angularVelocity,wheelTicks,err,dutySet;

    };
    struct MsgState : public Message
    {
        State _stateLeft, _stateRight;
        MsgState(const std::string& raw):Message(raw,MsgType::STATE)
        {
            auto values = split(raw.substr(2),' ');
            _stateLeft.wheelTicks = std::stof(values[0]);
            _stateLeft.angularVelocityCmd = std::stof(values[1]);
            _stateLeft.angularVelocity = std::stof(values[2]);
            _stateLeft.err = std::stof(values[3]);
            _stateLeft.dutySet = std::stof(values[4]);

            _stateRight.wheelTicks = std::stof(values[5]);
            _stateRight.angularVelocityCmd = std::stof(values[6]);
            _stateRight.angularVelocity = std::stof(values[7]);
            _stateRight.err = std::stof(values[8]);
            _stateRight.dutySet = std::stof(values[9]);

        }
        MsgState(const State& stateLeft, const State& stateRight):_stateLeft(stateLeft),_stateRight(stateRight),Message("",MsgType::STATE)
        {
            std::stringstream ss;
            ss << to_string(_type) << " " << stateLeft.wheelTicks << " " << stateLeft.angularVelocityCmd << " " << stateLeft.angularVelocity << " " << stateLeft.err;
            ss << stateRight.wheelTicks << " " << stateRight.angularVelocityCmd << " " << stateRight.angularVelocity << " " << stateRight.err;
            ss << "\n";
            _str = ss.str();
        }
        std::string str() override {
            std::stringstream ss;
            ss.precision(4);
            ss ;
            ss << "       |" << std::setw(6) <<"L" << " |" << std::setw(8) <<"R|\n";
            ss << " ticks |" << std::setw(6) << (double)_stateLeft.wheelTicks << " |" << std::setw(6) << (double)_stateRight.wheelTicks << "|\n";
            ss << " v*    |" << std::setw(6) << _stateLeft.angularVelocityCmd << " |" << std::setw(6) << _stateRight.angularVelocityCmd << "|\n";
            ss << " v     |" << std::setw(6) << _stateLeft.angularVelocity << " |" << std::setw(6) << _stateRight.angularVelocity << "|\n";
            ss << " err   |" << std::setw(6) << _stateLeft.err << " |" << std::setw(6) << _stateRight.err << "|\n";
            ss << " pwm   |" << std::setw(6) << _stateLeft.dutySet << " |" << std::setw(6) << _stateRight.dutySet << "|\n";
            return ss.str();
        }


    };

    std::vector<std::shared_ptr<Message>> _messages;
    std::vector<std::shared_ptr<MsgCmdVel>> _messagesCmdVel;
    std::vector<std::shared_ptr<MsgState>> _messagesState;
    SerialPort _serialPort;

    void read(int nMessages) {
        int nBytes = 0;
        int readMessages = 0;
        do{
            nBytes = _serialPort.Read(_buffer,1);
            if (_buffer.str().back() == '\n')
            {
                auto raw = _buffer.str();
                try{
                    auto msgType = Message::msgType(raw.substr(0,1));
                    switch(msgType)
                    {
                        case MsgType::STATE:
                            _messagesState.push_back(std::make_shared<MsgState>(raw));
                            break;
                        case MsgType::CMD_VEL:
                            _messagesCmdVel.push_back(std::make_shared<MsgCmdVel>(raw));
                            break;
                        default:
                            _messages.push_back(std::make_shared<Message>(raw));
                    }
                }catch(const std::exception& e)
                {
                    std::cerr << e.what()  << std::endl;
                    _messages.push_back(std::make_shared<Message>(raw));
                }

                _buffer = std::stringstream();
                readMessages++;
            }
        }while(nBytes > 0 && readMessages < nMessages);

    }

    void send(const std::shared_ptr<Message>& msg)
    {
        _serialPort.Write(msg->serialStr());
    }

    static std::vector<std::string> split(const std::string& s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }
private:
    std::stringstream _buffer;
};

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
        std::stringstream ss;
        ss << "v " << setPoint << "," << -1*setPoint << "\n";

        std::cout << "Setting:\n" << ss.str() << std::endl;

        serial.send(std::make_shared<SerialProtocol::MsgCmdVel>(setPoint,-1*setPoint));

        serial.read(10);

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