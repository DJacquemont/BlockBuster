#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cmath>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void read_rpm_values(double &val_1, double &val_2)
  {
    std::string response = send_msg("a\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    // Reading speed values of the wheels in radians per second
    val_1 = std::stod(token_1.c_str())*(2*M_PI)/(60.0*60.0);
    val_2 = std::stod(token_2.c_str())*(2*M_PI)/(60.0*60.0);
  }

  void read_system_values(double &val_1, double &val_2, double &val_3, double &val_4)
  {
    std::string response = send_msg("b\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_2 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    del_pos = response.find(delimiter);
    std::string token_3 = response.substr(0, del_pos);
    response = response.substr(del_pos + delimiter.length());

    std::string token_4 = response;

    val_1 = 0.0;
    val_2 = 0.0;
    val_3 = 0.0;
    val_4 = 0.0;

    val_1 = std::stod(token_1.c_str())*(2*M_PI)/(60.0*60.0);
    val_2 = std::stod(token_2.c_str())*(2*M_PI)/(60.0*60.0);
    val_3 = std::stod(token_3.c_str());
    val_4 = std::stod(token_4.c_str());

    // if (!token_1.empty() && !token_2.empty() && !token_3.empty() && !token_4.empty())
    // {
    //     try
    //     {
    //         val_1 = std::stod(token_1.c_str())*(2*M_PI)/(60.0*60.0);
    //         val_2 = std::stod(token_2.c_str())*(2*M_PI)/(60.0*60.0);
    //         val_3 = std::stod(token_3.c_str());
    //         val_4 = std::stod(token_4.c_str());
    //     }
    //     catch (const std::invalid_argument& e)
    //     {
    //         std::cerr << "Invalid argument for token_1: " << token_1 << std::endl;
    //     }
    // }    
  }

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    // Sending speed values of the wheels in rpm

    ss << "o " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }

  void set_servo_values(int val)
  {
    std::stringstream ss;
    // Sending command to the servo 

    ss << "s " + std::to_string(val) + "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP