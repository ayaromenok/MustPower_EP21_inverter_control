import serial
import struct
import time
import sys

import paho.mqtt.client as mqtt


def init_all():
    print("init_all()")
    data0 = b"\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    format_string = '>BHHHHHHHHHHHHHHHHHHHHHHHHHHHI'
    ups_data = struct.unpack(format_string, data0)
    return ups_data

def read_serial_data_EP20(ups_data, port_name):
    try:
        tty_port = serial.Serial(
#            port='/dev/ttyUSB0',
                port=port_name,
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            timeout=1 # Read timeout in seconds
        )
        time.sleep(0.5) # Give the port some time to initialize

        #print("read_serial_data_EP20()")
        #print(f"ups data befor read: {ups_data}")
        if tty_port.isOpen():
            #print(f"port open{tty_port}")
            #byte_list0 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x07, 0x9c, 0x28] # handshake?
            byte_list1 = [0x0A, 0x03, 0x75, 0x30, 0x00, 0x1B, 0x1E, 0xB9] # responce 59 byte
            #byte_list2 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x0A, 0x5D, 0xED] # responce 25 byte

            tty_port.write(serial.to_bytes(byte_list1))
            #print(f"Sent byte list: {byte_list1}")
            time.sleep(0.5) #0.1 is good value

            # Read binary data
            data1 = tty_port.read(59) # Read up to 59 bytes

            if data1:
                data1_size = sys.getsizeof(data1)
                print(f"Received {data1_size} bytes, data:{data1.hex()}") # Print as hex string
                # B - uint8, H - uint16, I - uint32, Q - uint64
                format_string1 = '>BHHHHHHHHHHHHHHHHHHHHHHHHHHHI'

                old_ups_data = ups_data;
                ups_data = struct.unpack(format_string1, data1)
                if (ups_data[20] == 0) & (ups_data[0] != 1): #sometime all zeros came from port
                    print ("use old values")
                    ups_data = old_ups_data;

#                if (ups_data[17] > 300): # when battery discarged - value 65468 appear isntead of 0#
#                    ups_data[17] = 0;
#                print(f"Work State:       {ups_data[4]}")
#                print(f"Battery Class:    {ups_data[5]} V")
#                print(f"Rated Power:      {ups_data[6]} W")
                print(f"Grid Voltage:     {(ups_data[7]*0.1):.1f} V")
                print(f"Grid Frequence:   {(ups_data[8]*0.1):.1f} Hz")
                print(f"Output Voltage:   {(ups_data[9]*0.1):.1f} V")
                print(f"Output Frequency: {(ups_data[10]*0.1):.1f} Hz")
                print(f"Battery Voltage:  {ups_data[16]*.1:.1f} V")
                print(f"Bat Charge Curr:  {ups_data[17]*.1:.1f} A")
                print(f"Bat Load Currrnt: {ups_data[11]} A")
                print(f"Load Power:       {ups_data[12]} W")
                print(f"Load Percent:     {ups_data[14]} %")
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
        if tty_port.isOpen():
            tty_port.close();
        return ups_data


# MQTT Publisher
def publish_message(port_name, ups_data, time_interval):
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.loop_start()
#    _count = 0;
    while True:
#        message = f"Message {_count}"
        ups_data = read_serial_data_EP20(ups_data, port_name)
#ToDo: don't send 0000000(zeros)
        client.publish("EP20/battery_voltage", round(ups_data[16]*0.1,1))
        client.publish("EP20/battery_charge_current", round(ups_data[17]*0.1,1))
        client.publish("EP20/battery_load_current", round(ups_data[11]))
        client.publish("EP20/battery_SOC", ups_data[19])
        client.publish("EP20/load_current", ups_data[11])
        client.publish("EP20/load_power", ups_data[12])
        client.publish("EP20/transformer_temp", ups_data[20])
        client.publish("EP20/grid_voltage", round(ups_data[7]*0.1,1))
        client.publish("EP20/grid_frequency",round( ups_data[8]*0.1,1))
        client.publish("EP20/output_voltage", round(ups_data[9]*0.1,1))
        client.publish("EP20/output_frequency", round(ups_data[10]*0.1,1))
#        _count += 1
        time.sleep(time_interval)
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    time_interval = 5; #sec, for debugging
    ups_data = init_all();
    print (ups_data);
    publish_message('/dev/ttyUSB0',ups_data, time_interval)
