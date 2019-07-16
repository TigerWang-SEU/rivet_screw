#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket
import Open_Protocol.open_protocol as open_protocol
import time
import threading

op = open_protocol.OpCon()


class opProtocol:
    IP = None
    PORT = None
    BUFFER = None

    def connect(self, IP, PORT):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setblocking(0)
        self.s.settimeout(0.5)
        try:
            self.s.connect((IP,PORT))
            self.s.send(op.message('Communication_start','001',' ',' ','  '))
        except:
            print "Something's wrong with %s:%s." % (IP, PORT)
        print "Socket Connected"
        try:
            response = self.s.recv(1024)
            print "Communication Start: %s" % (response)
            x = str(response[4:8])
            if x == op.MID['Communication_start_acknowledge']:
                print "Connected to %s:%s" % (IP,PORT)
        except:
            print "Recieve Error %s:%s." % (IP, PORT)

    def sendMID(self, msg):
        try:
            self.s.send(msg)
        except:
            pass
        o = self.s.recv(1024)
        return o

    def recvData(self):
        try:
            recieve='None'
            recieve = self.s.recv(65565)
        except:
            print "Error Recieve %s" % (recieve)
        return recieve

    def keepAlive(self, INTERVAL=8):
        while True:
            try:
                self.s.send(op.message('Keep_Alive','','','',''))
                response_keepalive = self.s.recv(self.BUFFER)
                if response_keepalive == op.message('Keep_Alive','','','',''):
                    break
                else:
                    print "Keep Alive Error: %s" % (response_keepalive)
            except:
                print "Keep Alive Except Error: %s" % (response_keepalive)

            time.sleep(INTERVAL)

    def close(self):
        try:
            self.s.send(op.message('Communication_stop','001',' ',' ','  '))
            self.s.close()
        except:
            print "Error closing Socket: %s" 

    t = threading.Thread(target=keepAlive)
