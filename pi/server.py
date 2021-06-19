#!/usr/bin/env python3

from __future__ import print_function

import time, sys
from struct import unpack
from struct import Struct
from RF24 import *
from RF24Network import *
from array import array
from datetime import datetime

# radio setup
radio = RF24(22, 0)
network = RF24Network(radio)

radio.begin();
network.begin(2, 0o0);
radio.setRetries(2, 2);
radio.setAutoAck(True);
radio.setPALevel(RF24_PA_MAX);
radio.setDataRate(RF24_1MBPS);
radio.setCRCLength(RF24_CRC_8);
radio.printPrettyDetails()
print('Running ...')
time.sleep(1.0)

if __name__ == '__main__':
    CH4 = 0
    CO2 = 0
    NH3 = 0
    while True:
        try:
            network.update()
            
            while network.available():
                try:
                    header, payload = network.read(16)
                    s = Struct('f f f f')
                    fields = s.unpack(bytes(payload))
                    newCO2, newCH4, newNH3, nodeId = fields[:4]
                    newCO2 = float(newCO2)
                    newNH3 = float(newNH3)
                    newCH4 = float(newCH4)
                    nodeId = int(nodeId)
                    
                    print(datetime.now().strftime('%H:%M:%S-%d/%m/%Y'), 'Received payload: CH4:', newCH4, 'CO2:', newCO2, 'NH3:', newNH3, 'node Id', nodeId, 'from node:', nodeId)
                except Exception as e:
                    print(e)
        
        except KeyboardInterrupt:
            radio.powerDown()
            sys.exit()
