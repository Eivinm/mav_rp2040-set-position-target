#!/usr/bin/env python3
# Micropython script for the Raspberry Pi Pico (or any RP2040 based board) for sending ADC
# values via MAVLink connection. The ADC values can then be viewed on a ground station.
# This requires the RP2040's UART (Rx, Tx, Gnd, Pwr) to be connected to a flight controller's
# TELEM ports.

# The TELEM port should be set to MAVLink2, 57600 baud

# In MAVProxy, use "graph NAMED_VALUE_FLOAT[ADC0].value NAMED_VALUE_FLOAT[ADC1].value NAMED_VALUE_FLOAT[ADC2].value"
# to view the values

from machine import Pin, Timer, ADC, UART
import time
import pymavminimal as pymav

MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_GLOBAL_INT = 5

led = Pin('LED', Pin.OUT)
timer = Timer()

uart0 = UART(0, baudrate=57600, tx=Pin(0), rx=Pin(1))

seen_heartbeat = False

#send_count = 0

def sendMessages(timer):
    if seen_heartbeat:
        #global send_count
        
        msgs = []
        msgs.append(mavobj.heartbeat_encode(pymav.MAV_TYPE_ONBOARD_CONTROLLER, 8, 0, 0, 0))
        #msgs.append(mavobj.set_position_target_local_ned_encode(time.ticks_ms(), mavobj.srcSystem, mavobj.srcComponent, MAV_FRAME_LOCAL_NED, 0b0000000111111100, 50, 50, -50, 0, 0, 0, 0, 0, 0, 0, 0))
        msgs.append(mavobj.set_position_target_global_int_encode(time.ticks_ms(), mavobj.srcSystem, mavobj.srcComponent, MAV_FRAME_GLOBAL_INT, 0b0000000111111100, 633200620, 10273286, 20, 0, 0, 0, 0, 0, 0, 0, 0))
        #send_count += 1

        for msg in msgs:
            uart0.write(msg.pack(mavobj))
        print("Sent at {0}".format(time.ticks_ms()))

# Use a timer for sending packets
timer.init(freq=5, mode=Timer.PERIODIC, callback=sendMessages)

# MAVLink
mavobj = pymav.MAVLink()
mavobj.robust_parsing = True
mavobj.srcSystem = 1
mavobj.srcComponent = 1

# Keep looping to receive data
while True:
    num = uart0.any()
    # Receive data and process into MAVLink packets
    if num > 0:
        rxData = uart0.read(num)
        pkts = mavobj.parse_buffer(bytearray(rxData))
        if pkts is not None:
            for pkt in pkts:
                if pkt.get_type() == 'HEARTBEAT' and pkt.type not in [pymav.MAV_TYPE_GCS, pymav.MAV_TYPE_ADSB, pymav.MAV_TYPE_GIMBAL, pymav.MAV_TYPE_ONBOARD_CONTROLLER]:
                    led.toggle()
                    if not seen_heartbeat:
                        print("Got heartbeat from {0}:{1}".format(pkt.get_srcSystem(), pkt.get_srcComponent()))
                        mavobj.srcSystem = pkt.get_srcSystem()
                        mavobj.srcComponent = pkt.get_srcComponent() #158 #MAV_COMP_ID_PERIPHERAL
                        seen_heartbeat = True
    time.sleep(0.01)
