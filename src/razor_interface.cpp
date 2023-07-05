#include <razorcpp/razor_interface.h>

RazorInterface::RazorInterface(std::string &port_name, int &baudrate)
{
    connect(port_name, baudrate);

    deg2rad = M_PI / 180.0;
    gravity_scale = 9.81 / 1000.0;
}

RazorInterface::~RazorInterface()
{
    delete port;
}

void RazorInterface::connect(std::string port_name, int baudrate)
{
    int i = 0;
    bool connected = false;
    while (i < 5 && !connected)
    {
        port = new serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(50));
        if (port->isOpen())
        {
            connected = true;
        }
        else
        {
            connected = false;
        }

        sleep(1);
        i += 1;
        port->flush();
    }

    if (connected)
    {
        std::cout << "Connected: " << port_name << std::endl;
        std::cout << i << " attempts" << std::endl;
    }
    else
    {
        std::cout << "Failed to connect: " << port_name << std::endl;
        std::cout << i << " attempts" << std::endl;
    }
}

bool RazorInterface::readMessage(ImuData &data)
{
    std::string msg;
    msg = port->readline();
    std::vector<std::string> msg_vect;
    boost::split(msg_vect, msg, boost::is_any_of(","));

    if (msg_vect.size() >= 10)
    {
        data.accel.x = std::atof(msg_vect.at(0).c_str()) * gravity_scale;
        data.accel.y = std::atof(msg_vect.at(1).c_str()) * gravity_scale;
        data.accel.z = std::atof(msg_vect.at(2).c_str()) * gravity_scale;

        data.gyro.x = std::atof(msg_vect.at(3).c_str()) * deg2rad;
        data.gyro.y = std::atof(msg_vect.at(4).c_str()) * deg2rad;
        data.gyro.z = std::atof(msg_vect.at(5).c_str()) * deg2rad;

        data.mag.x = std::atof(msg_vect.at(6).c_str());
        data.mag.y = std::atof(msg_vect.at(7).c_str());
        data.mag.z = std::atof(msg_vect.at(8).c_str());

        data.temp = std::atof(msg_vect.at(9).c_str());

        return true;
    }
    else
    {
        return false;
    }
}
