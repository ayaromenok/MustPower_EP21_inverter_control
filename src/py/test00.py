import serial
import struct
import time
import sys
import struct

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
#    stopbits=serial.STOPBITS_ONE,
    timeout=0 # Read timeout in seconds
)

try:
    if ser.isOpen():
        print(f"Serial port {ser.port} opened successfully.")

        byte_list0 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x07, 0x9c, 0x28] # handshake?
        byte_list1 = [0x0A, 0x03, 0x75, 0x30, 0x00, 0x1B, 0x1E, 0xB9] # responce 59 byte
        byte_list2 = [0x0A, 0x03, 0x79, 0x18, 0x00, 0x0A, 0x5D, 0xED] # responce 25 byte

        ser.write(serial.to_bytes(byte_list1))
        print(f"Sent byte list: {byte_list1}")

        time.sleep(0.5)

# Read binary data
        data1 = ser.read(128) # Read up to 59 bytes

        if data1:
            data1_size = sys.getsizeof(data1)
            print(f"Received data: {data1.hex()}") # Print as hex string
            print(f"Data size: {data1_size}") # Print as hex string
            # B - uint8, H - uint16, I - uint32, Q - uint64
            format_string = '>BHHHHHHHHHHHHHHHHHHHHHHHHHHHI'
            unpacked_data1 = struct.unpack(format_string, data1)
            gridVoltage = unpacked_data1[7]*0.1; #V
            gridFreq = unpacked_data1[8]*0.1;    #Hz
            outVoltage = unpacked_data1[9]*0.1;  #V
            outFreq = unpacked_data1[10]*0.1;    #Hz
            print(f"Work State:       {unpacked_data1[4]}")
            print(f"Battery Class:    {unpacked_data1[5]} Volt")
            print(f"Rated Power:      {unpacked_data1[6]} Watt")
#            print(f"Grid Voltage: {gridVoltage:.1f} Volt {hex(unpacked_data1[7])}") #debug example with hex
            print(f"Grid Voltage:     {gridVoltage:.1f} Volt")
            print(f"Grid Frequence:   {gridFreq:.1f} Hertz")
            print(f"Outзut Voltage:   {outVoltage:.1f} Volt")
            print(f"Output Frequency: {outFreq:.1f} Hertz")

        else:
            print("No data received.")

    else:
        print("Could not open serial port.")

except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    if ser.isOpen():
        ser.close()
#        print(f"Serial port {ser.port} closed.")
