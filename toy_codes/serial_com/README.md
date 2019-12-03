# serial_com

## 1 install libserial

```bash
git clone https://github.com/lisilin013/serial_com.git
tar -xvf libserial-0.6.0rc2.tar.gz
cd libserial-0.6.0rc2
./configure 
make
sudo make install
```

## 2 use SerialCom 
see the code example below:

```cpp
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

```

## 3 decode rx data example

```cpp
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
```
