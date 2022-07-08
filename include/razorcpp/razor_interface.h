#include <iostream>
#include <string>
#include <unistd.h>
#include <serial/serial.h>
#include <cmath>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

class RazorInterface
{
    private:
        serial::Serial* port;  

        void connect(std::string port, int baudrate);

        double gravity_scale;
        double deg2rad;

    public:
        typedef struct
        {
            double x;
            double y;
            double z;            
        } Vector3;

        typedef struct
        {
            Vector3 accel;
            Vector3 gyro;
            Vector3 mag;
            double temp;
        } ImuData;

        bool readMessage(ImuData& data);
        RazorInterface(std::string& port, int& baudrate);
        ~RazorInterface();
};