#!/usr/bin/env python2
import time
import math
import rospy
import numpy
import os
import threading

import socket
import serial
import serial.tools.list_ports

from mavlink import MAVLink as mav
import mavlink

import math as m

import json
import paho.mqtt.client as mqtt

OPEN_UDP_FORWARD = 1

if OPEN_UDP_FORWARD == 1:
    LOCAL_IP = '192.168.2.115'
    REMOTE_IP = '192.168.2.110'
    LOCAL_PORT = 14510
    REMOTE_PORT = 14510


def send_to_uart(mavmsg, *args, **kwargs):
        __send(mavmsg.get_msgbuf())

t_file = open('temp.rec', 'wb')
uart_mav = mav(t_file, 1, 0)
uart_mav.set_send_callback(send_to_uart)

class bypass_thread():
    global OPEN_UDP_FORWARD
    if OPEN_UDP_FORWARD == 1:
        def __init__(self):
                self.socket_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket_udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_udp.bind((LOCAL_IP,LOCAL_PORT))
                self.SOCKET_RECV_IP_PORT = (REMOTE_IP,REMOTE_PORT)
                self.socket_udp.settimeout(0.1)
                self.socket_udp.setblocking(False)

                self.t_file = open('temp_udp.rec', 'wb')
                self.udp_mav = mav(t_file, 1, 0)
                self.udp_mav.set_send_callback(self.sendUPD)

                # MQTT configuration
                self.mqtt_broker = "192.168.2.115"  # Change to your MQTT broker address
                self.mqtt_port = 1883
                self.mqtt_topic = "mavlink"

                # Create an MQTT client instance
                self.mqtt_client = mqtt.Client()

                # Connect to the MQTT broker
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)

    # Function to convert MAVLink message to JSON
    def mavlink_message_to_json(self, message):
        message_dict = message.to_dict()
        message_json = json.dumps(message_dict)
        return message_json


    if OPEN_UDP_FORWARD == 1:
        def send_to_udp(self,bs):
            try:
                print("fuck udp:")
                print(bs.get_msgbuf())
                print("fuck after")
                # self.socket_udp.sendto(bs, (BROADCAST_IP,REMOTE_PORT))
                self.socket_udp.sendto(bs, (REMOTE_IP,REMOTE_PORT))
            except Exception as e:
                print("fuck send error:")
                print(e)

        def sendUPD(self, mavmsg, *args, **kwargs):
            self.__send_udp(mavmsg.get_msgbuf())

        def __send_udp(self,c):
            try:
                self.socket_udp.sendto(c,(REMOTE_IP,REMOTE_PORT))

            except Exception as e:
                print(e)

    def recv_udp(self):
        try:
            c,self.SOCKET_RECV_IP_PORT = self.socket_udp.recvfrom(1024)
            if len(c) > 0:
                return c
        except:
            pass

    def run(self):
        global uart_mav
        udp_bytes = self.recv_udp()
        try:
            if udp_bytes > 0:
                msgs = uart_mav.parse_buffer(udp_bytes)

                if msgs is not None:
                    for msg in msgs:
                        dict = msg.to_dict()
                        message_json = json.dumps(dict)
                        topic = dict['mavpackettype']

                        pub_topic = self.mqtt_topic +  "/" + topic

                        result = self.mqtt_client.publish(pub_topic, message_json, qos=0, retain=False)
                        result.wait_for_publish()
                        print("publish topics:",{pub_topic})

        except Exception as e:
                print("Exception MSGS:")
                print(e)

udp_thread = bypass_thread()

def run_udp_thread():
    print("udp thread start")
    rate = rospy.Rate(100) #100hz

    while rospy.is_shutdown() is False:
        udp_thread.run()
        rate.sleep()


def main():
    rospy.init_node('mavlink',anonymous=True)
    thread_mav = threading.Thread(target=run_udp_thread)
    thread_mav.start()
    rospy.spin()

if __name__ == '__main__':
    main()


