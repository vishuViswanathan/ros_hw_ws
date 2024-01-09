#include "diffdrive_pi_arduino/arduino_pi_comms.h"
// #include <ros/console.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include<sys/socket.h>
#include<arpa/inet.h>	//inet_addr
#include<unistd.h>  // close


void ArduinoComms::setup(const std::string &serial_device)
{  
	struct sockaddr_in server;
    showMsg("Setting socker_desc                 $$$$$$");
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		showMsg("ERROR    Could not create socket $$$$$$$");
	}
		
	server.sin_addr.s_addr = inet_addr(serial_device.c_str());
	server.sin_family = AF_INET;
	server.sin_port = htons( 8888 );
    //Connect to remote server
	if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		showMsg("ERROR connect error $$$$$$");
	}
    else {
        // isConnected = waitForPi();
        isConnected = true;
        showMsg("$$$$$$$\n$$$$$$$$$$$$$\n$$$$$$$$$$$$$$$$$\n Connected $$$$$$$");
    }
}

bool ArduinoComms::waitForPi() {
	int n;
	bool ready = false;
	int msgNum = 0;
	while (!ready) {
 		n = read(socket_desc, server_reply , 2000);
		if (n > 0) {
			msgNum++;
			server_reply[n-1] = '\0';
			// showMsg(server_reply);
			if (!strcasecmp(server_reply, "PI_READY"))
				ready = true;
		}
	}
	return ready;
}

void ArduinoComms::showMsg(const std::string &msg) 
{
    printf("$$$$$$$                                         $$$$$$$");
    for (int n = 1; n < 3; n++)
        printf("$$$$$$                                      $$$$$$");
    printf(" MESSAGE  %s", msg.c_str());
    for (int n = 1; n < 3; n++)
        printf("$$$$$$                                      $$$$$$");
}

void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\n");
}

void ArduinoComms::resetEncoder() 
{
    std::string response = sendMsg("r\n");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\n");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\n";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\n";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{

	if (print_output) {
        printf("messge to send <%s>", msg_to_send.c_str());
    }
    write(socket_desc , msg_to_send.c_str() ,  msg_to_send.length() - 1);
    int n = read(socket_desc, server_reply , 2000);
    if( n >= 0) {
        return std::string(server_reply, server_reply + n);
    }
    return "NO DATA";
}