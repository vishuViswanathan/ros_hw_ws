#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <cstring>
#include <sys/socket.h>
#include <cstdlib>
#include <serial/serial.h>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  // ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  //     : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  // {  }
 
  void setup(const std::string &serial_device);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);
  void resetEncoder();
  bool connected() const { return isConnected ;}

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

  bool waitForPi();
  void showMsg(const std::string &msg);


private:
    int socket_desc;
    bool isConnected = false;
    char server_reply[2000];
    };

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H