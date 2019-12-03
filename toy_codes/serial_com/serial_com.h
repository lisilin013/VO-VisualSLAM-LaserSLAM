/*
  ******************************************************************************
  * @file   : serial_com.h
  * @author : Silin Li
  * @version: V1.0
  * @date   : 2018.10.24
  * @note   : a class for serial using Libserial
  * @history:
  *
  ******************************************************************************
*/

#ifndef PROJECT_SERIAL_COM_H
#define PROJECT_SERIAL_COM_H

#include <SerialStream.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <cstring>


/**
  * @code example :

 #include "serial_com.h"
 #include <string>

 using namespace std;

 std::string port_name("/dev/ttyUSB0");
 SerialCom serial_com(port_name, LibSerial::SerialStreamBuf::BAUD_115200);

 // send
 char tx_buff[10] = {...};
 serial_com.write(tx_buff, 10);
 // receive
 char rx_buff[100];
 int rx_len = serial_com.read(rx_buff, 100);

 **/
class SerialCom {
public:
    explicit SerialCom(std::string port_name, LibSerial::SerialStreamBuf::BaudRateEnum baud) : _port_name(port_name),
                                                                                               _baud(baud) {
        _serial_stream = new LibSerial::SerialStream();
        init();
    }


    ~SerialCom() {
        if (_serial_stream) {
            delete _serial_stream;
        }
    }

    void write(const char *buff, size_t len) {
        if (isOpen()) {
            _serial_stream->write(buff, len);
        }
        else {
            std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                      << "Error: serial port is closed! Cannot write!\n";
            exit(1);
        }
    }

    size_t read(char *buff, size_t BUFF_MAX) {
        if (isOpen()) {
            size_t len = 0;
            while (len < BUFF_MAX && _serial_stream->rdbuf()->in_avail() > 0) {
                char data = 0;
                _serial_stream->get(data);
                buff[len++] = data;
            }
        }
        else {
            std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                      << "Error: serial port is closed! Cannot read!\n";
            exit(1);
        }

    }

    void close() {
        _serial_stream->Close();
    }

    bool isOpen() {
        return _serial_stream->IsOpen();
    }

private:
    void init() {
        _serial_stream->Open(_port_name);
        if (!_serial_stream->good()) {
            std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                      << "Error: Could not open serial port."
                      << std::endl;
            exit(1);
        }

        // Set the baud rate of the serial port.
        _serial_stream->SetBaudRate(_baud);
        if (!_serial_stream->good()) {
            std::cerr << "Error: Could not set the baud rate." << std::endl;
            exit(1);
        }

        // Set the number of data bits.
        _serial_stream->SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
        if (!_serial_stream->good()) {
            std::cerr << "Error: Could not set the character size." << std::endl;
            exit(1);
        }

        // Disable parity.
        _serial_stream->SetParity(LibSerial::SerialStreamBuf::PARITY_NONE);
        if (!_serial_stream->good()) {
            std::cerr << "Error: Could not disable the parity." << std::endl;
            exit(1);
        }

        // Set the number of stop bits.
        _serial_stream->SetNumOfStopBits(1);
        if (!_serial_stream->good()) {
            std::cerr << "Error: Could not set the number of stop bits."
                      << std::endl;
            exit(1);
        }

        // Turn off hardware flow control.
        _serial_stream->SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE);
        if (!_serial_stream->good()) {
            std::cerr << "Error: Could not use hardware flow control."
                      << std::endl;
            exit(1);
        }
    }

private:
    LibSerial::SerialStream *_serial_stream;
    LibSerial::SerialStreamBuf::BaudRateEnum _baud;
    std::string _port_name;

};

/**
  * @brief : A status machine demo to decode rx data
  * @retval:
  */
void DecodeRxData(char data) {
    //set param
    const size_t kRxLen = 9;

    //must be unsigned char
    //rx_buff stored a complete frame data
    static unsigned char rx_buff[kRxLen];
    static int pit = 0;
    static int cur_pos = 0;

    //status machine
    //modify these code according to your protocol
    switch (cur_pos) {
        case 0://head
            if (data == 0x01) {
                cur_pos = 1;
                pit = 0;
                memset(rx_buff, 0, kRxLen);
                rx_buff[pit++] = data;
            }
            break;
        case 1:
            if (data == 0x03) {
                cur_pos = 2;
                rx_buff[pit++] = data;
            }
            else {
                cur_pos = 0;
            }
            break;
        case 2:
            if (data == 0x04) {
                cur_pos = 3;
                rx_buff[pit++] = data;
            }
            else {
                cur_pos = 0;
            }
            break;
        case 3://check and get data from a complete frame
            rx_buff[pit++] = data;
            if (pit == 9) {
                cur_pos = 0;
            }
            break;
        default:
            break;
    }
}

#endif //PROJECT_SERIAL_COM_H
