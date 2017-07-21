#!/usr/bin/env python2

import sys
import serial
import csv

import matplotlib.pyplot as plt

def main():
    ser = serial.Serial(sys.argv[1], baudrate=1200, timeout=2)

    t = []
    accelx, accely, accelz, gyrox, gyroy, gyroz, magx, magy, magz = ([], [], [], [], [], [], [], [], [])
    pressure, temp = ([], [])

    setup_done = True

    with open(sys.argv[2], 'w') as f:
        try:
            i = 0
            while True:
                data = str(ser.readline()).strip()
                if not data:
                    continue

                f.write("%s\n" % data)
                print(data)

                if not setup_done:
                    continue

                if data == "OK+FU4":
                    setup_done = True

                try: 
                    tokens = map(float, filter(lambda x: x, data.strip().split('\t')))
                    if len(tokens) < 12:
                        print("Malformed Data Packet!")
                        continue
     
                    t.append(tokens[0])
                    accelx.append(tokens[1])
                    accely.append(tokens[2])
                    accelz.append(tokens[3])
                    gyrox.append(tokens[4])
                    gyroy.append(tokens[5])
                    gyroz.append(tokens[6])
                    magx.append(tokens[7])
                    magy.append(tokens[8])
                    magz.append(tokens[9])
                    pressure.append(tokens[10])
                    temp.append(tokens[11])

                except ValueError as e:
                    print("Malformed Data Packet!")

        except KeyboardInterrupt as e:
            pass
        
        plt.plot(accelx)
        plt.show()

        f.close()

if __name__ == "__main__":
    main()
