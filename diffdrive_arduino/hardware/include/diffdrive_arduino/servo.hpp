#ifndef DIFFDRIVE_ARDUINO_SERVO_HPP
#define DIFFDRIVE_ARDUINO_SERVO_HPP

#include <string>

class Servo
{
    public:

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    Servo() = default;

    Servo(const std::string &servo_name)
    {
      setup(servo_name);
    }
    
    void setup(const std::string &servo_name)
    {
      name = servo_name;
    }

    void reset()
    {
      cmd = 0;
    }
};


#endif // DIFFDRIVE_ARDUINO_SERVO_HPP
