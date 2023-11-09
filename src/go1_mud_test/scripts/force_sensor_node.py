#!/usr/bin/env python3

import rospy
import serial
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3


class ATI_sensor:

    def __init__(self):
        print("Initializing object ATI_sensor")
        try:  # The sensor randomly connects to one of these serial ports.
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # open port
        except OSError as e:
            print(e)
            print("Trying now /dev/ttyUSB1")
            self.ser = serial.Serial('/dev/ttyUSB1', 115200)  # or this one

        self.initial_readings = np.array([0, 0, 0, 0, 0, 0])
        self.pub = rospy.Publisher('ati_ft_data',
                                   WrenchStamped,
                                   queue_size=10)
        self.rate = rospy.Rate(150)  # max ~163hz (serial com is the botteneck)
        self.input_ft = "s".encode()
        self.seq = 0
        self.first_read = True

    def twos_complement(self, hexstr, bits):
        # print("In twos_complement")
        """This function converts a string in hexadecimal format into
        a signed decimal integer"""
        value = int(hexstr, bits)
        if value > pow(2, bits - 1):
            value -= pow(2, bits)
        return value

    def stream(self):
        """This function is the main function, it gets readings from
        the extract_data method at a defined frequency, corrects them
        and converts them to Newtons and Newton.meters"""
        # print("In stream")
        while not rospy.is_shutdown():
            try:
                values = self.extract_data()
                if values[0] == -1:
                    continue  # filter out faulty readings
                self.corrected_reading = (values[1:]
                                          - self.initial_readings)/(15.2588 / 1.026)  # 1.026 comes from calibrations I made myself, seems like the 15.2588 is not very accurate anymore.
                data = Wrench(force=Vector3(self.corrected_reading[0],
                                            self.corrected_reading[1],
                                            self.corrected_reading[2]),
                              torque=Vector3(self.corrected_reading[3],
                                             self.corrected_reading[4],
                                             self.corrected_reading[5]))
                msg = WrenchStamped()
                msg.wrench = data
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "ATI_sensor_frame"
                msg.header.seq = self.seq
                self.seq += 1
                self.pub.publish(msg)
                self.rate.sleep()
            except AssertionError as e:
                print(e)

    def initialize_readings(self):
        """This function's goal is to get the default readings of the sensor.
        It takes some readings, averages them and fills the initial_readings
        variable"""
        print("Initializing readings...")
        i = 0
        average_readings = np.array([0, 0, 0, 0, 0, 0])
        while i < 200:
            try:
                values = self.extract_data()
                if values[0] == -1:
                    continue  # filter out faulty readings
                average_readings += values[1:]
                i += 1
                if i % 50 == 0:
                    print('%d sample taken for calibration' % i)
            except AssertionError as e:
                print("e: ", e)
                print("Couldn't read sensor")
        self.initial_readings = average_readings/(i + 1)
        return

    def send_receive_serial(self):
        """This function sends commands and reads back from serial"""
        # print("Reading serial...")
        if self.first_read:
            self.ser.write(self.input_ft)
            self.first_read = False
        line = self.ser.read(55)  # make sure we have one full message
        # line = self.ser.read(29)  # full message is 29 chars (inc \r\n)
        # line = self.ser.readline()
        return line

    def extract_data(self):
        """Since data comes in a '1FFFF00000023000000000000' format,
        this function extracts the individual components of the data
        and outputs them in decimal format."""
        # print("Extracting data")
        successful_line = False
        while successful_line is False:  # The sensor may send empty lines
            line = self.send_receive_serial()
            try:
                # print("trying to get a correct sequence")
                # print("line = ", line)
                # without_r = line.split("\r\n".encode())[1]
                without_r = max(line.split("\r\n".encode()), key=len)
                # print("without_r:  ", without_r)
                # the sensor often sends a \x00 randomly inside a message
                without_x00 = without_r.split("\x00".encode())
                if len(without_x00) > 1:
                    # print("type withoutx00: ", type(without_x00))
                    list_out = without_x00[0] + without_x00[1]
                    # print( " parts of list: ", without_x00[0], "  and  ", without_x00[1])
                    without_x00 = [list_out]
                # print("without x00:  ", without_x00)
                # print("x00 = ", without_x00)
                sequence = without_x00[-25:][0]
                # print("sequence:  ", sequence)
                successful_line = True
            except IndexError as e:
                print(e)

        # print("type sequence:  ", type(sequence))
        # print(len(sequence))
        if len(sequence) == 25:
            [id, Fx, Fy, Fz, Tx, Ty, Tz] = [sequence[0:1],
                                            sequence[1:5],
                                            sequence[5:9],
                                            sequence[9:13],
                                            sequence[13:17],
                                            sequence[17:21],
                                            sequence[21:25]]
            # print("id: ", id)
            self.out_split = np.array([self.twos_complement(data, 16)
                                       for data in
                                       [id, Fx, Fy, Fz, Tx, Ty, Tz]])
            # print(type(self.out_split))
            return self.out_split
        else:
            return np.array([-1, -1, -1, -1, -1, -1, -1])


def main():
    rospy.init_node('force_sensor_node')
    print("Started program for serial communication with ATI F/T sensor")

    ati = ATI_sensor()
    ati.initialize_readings()
    ati.stream()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass