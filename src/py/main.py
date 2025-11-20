import serial
import struct
import time
import sys

import paho.mqtt.client as mqtt

#ser = serial.Serial(
#    port='/dev/ttyUSB0',
#    baudrate=9600,
#    bytesize=serial.EIGHTBITS,
#    parity=serial.PARITY_NONE,
#    timeout=1 # Read timeout in seconds
#)


def init_all(port_name):
    print("init_all()")
    data0 = b"00000000000000000000000000000000000000000000000000000000000"
    format_string = '>BHHHHHHHHHHHHHHHHHHHHHHHHHHHI'
    ups_data = struct.unpack(format_string, data0)

    tty_port = serial.Serial(
#        port='/dev/ttyUSB0',
        port=port_name,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        timeout=1 # Read timeout in seconds
    )
    time.sleep(0.5) # Give the port some time to initialize
    return tty_port, ups_data

def read_serial_data_EP20(ups_data, tty_port):
    try:
        #print("read_serial_data_EP20()")
        #print(f"ups data befor read: {ups_data}")
        if tty_port.isOpen():
            #print(f"port open{tty_port}")
            #byte_list0 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x07, 0x9c, 0x28] # handshake?
            byte_list1 = [0x0A, 0x03, 0x75, 0x30, 0x00, 0x1B, 0x1E, 0xB9] # responce 59 byte
            #byte_list2 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x0A, 0x5D, 0xED] # responce 25 byte

            tty_port.write(serial.to_bytes(byte_list1))
            #print(f"Sent byte list: {byte_list1}")
            time.sleep(0.1) #0.1 is good value

            # Read binary data
            data1 = tty_port.read(59) # Read up to 59 bytes

            if data1:
                data1_size = sys.getsizeof(data1)
                print(f"Received {data1_size} bytes, data:{data1.hex()}") # Print as hex string
                # B - uint8, H - uint16, I - uint32, Q - uint64
                format_string1 = '>BHHHHHHHHHHHHHHHHHHHHHHHHHHHI'
                ups_data = struct.unpack(format_string1, data1)
                gridVoltage = ups_data[7]*0.1; #V
                gridFreq = ups_data[8]*0.1;    #Hz
                outVoltage = ups_data[9]*0.1;  #V
                outFreq = ups_data[10]*0.1;    #Hz
                print(f"Work State:       {ups_data[4]}")
                print(f"Battery Class:    {ups_data[5]} V")
                print(f"Rated Power:      {ups_data[6]} W")
    #            print(f"Grid Voltage: {gridVoltage:.1f} Volt {hex(ups_data[7])}") #debug example with hex
                print(f"Grid Voltage:     {(ups_data[7]*0.1):.1f} V")
                print(f"Grid Frequence:   {(ups_data[8]*0.1):.1f} Hz")
                print(f"Outзut Voltage:   {(ups_data[9]*0.1):.1f} V")
                print(f"Output Frequency: {(ups_data[10]*0.1):.1f} Hz")

                print(f"Battery Voltage:: {ups_data[16]*.1:.1f} V")
                print(f"Battery Current:  {ups_data[17]*.1:.1f} A")

                print(f"Battery SOC:      {ups_data[19]} %")
                print(f"Transform Temp:   {ups_data[20]} C")

            else:
                print("No data received.")
        else:
            print("Could not open serial port.")

    except:
        print("Serial I/O:something went wrong")
        sys.exit(1)
    finally:
        #print("finally: end of read/write")
        return ups_data


# MQTT Publisher
def publish_message(ups_data, time_interval):
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.loop_start()
    _count = 0;
    while True:
#        message = f"Message {_count}"
        ups_data = read_serial_data_EP20(ups_data, tty_port)
        message = f"Message #{_count}: {ups_data}"
        client.publish("test/topic", message)
        #print(f"Published: {message}")
        _count += 1
        time.sleep(time_interval)
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    time_interval = 5; #sec, for debugging
    tty_port, ups_data = init_all('/dev/ttyUSB0');
    print (tty_port, ups_data);
    publish_message(ups_data, time_interval)
