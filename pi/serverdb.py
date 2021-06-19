#!/usr/bin/env python3

from __future__ import print_function

import time, sys, pymysql
from struct import unpack
from struct import Struct
from RF24 import *
from RF24Network import *
from array import array
from datetime import datetime
from firebase import Firebase #pip3 install firebase python-jwt gcloud sseclient requests-toolbelt

config = {
  "apiKey": "AIzaSyBk0RIfTPuNx8LHOYx9XOywhw2aOYP2w7s",
  "authDomain": "ycmlg-2021.firebaseapp.com",
  "databaseURL": "https://ycmlg-2021-default-rtdb.firebaseio.com/",
  "storageBucket": "ycmlg-2021.appspot.com"
}
firebase = Firebase(config)
fdb = firebase.database()
#dbLogPath = db.child("jtd").child("TPS").child("logs")

# Database setup..
## Create connection to MySQL server
print("Connecting ...")
try:
    connection = pymysql.connect(host='localhost',
                                 user='pi',
                                 password='raspberry',
                                 db='pi',
                                 charset='utf8',
                                 cursorclass=pymysql.cursors.DictCursor)
except pymysql.err.Error as msg:
    print("Connection error: ", msg)
    exit()
## Create cursor
cc = connection.cursor()
#----------------------------------------------------------------------------------------->

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
                    newCO2 = int(newCO2)
                    newNH3 = int(newNH3)
                    newCH4 = int(newCH4)
                    nodeId = int(nodeId)
                    
                    if newCH4 != CH4:                        
                        try:
                            fdb.child("Lova").child("Node" + str(nodeId)).update({
                                "Met" + str(nodeId): newCH4,
                            })
                            
                            sqlCH4 = """
                                    UPDATE `pantautps`
                                        SET `Nilai` = '{0}',
                                        `Waktu` = current_timestamp()
                                    WHERE `pantautps`.`Node` = {1}
                                        AND
                                        `pantautps`.`Nama` LIKE 'ch4'
                                        ;
                                         """.format(newCH4, nodeId)
                            cc.execute(sqlCH4)
                            connection.commit()
                            CH4 = newCH4
                        except pymysql.err.InternalError as msg:
                            print("Command skipped: ", msg)
                    
                    if newCO2 != CO2:
                        try:
                            fdb.child("Lova").child("Node" + str(nodeId)).update({
                                "Car" + str(nodeId): newCO2,
                            })
                            
                            sqlCO2 = """
                                    UPDATE `pantautps`
                                        SET `Nilai` = '{0}',
                                        `Waktu` = current_timestamp()
                                    WHERE `pantautps`.`Node` = {1}
                                        AND
                                        `pantautps`.`Nama` LIKE 'co2'
                                     """.format(newCO2, nodeId)
                            cc.execute(sqlCO2)
                            connection.commit()
                            CO2 = newCO2
                        except pymysql.err.InternalError as msg:
                            print("Command skipped: ", msg)
                    
                    if newNH3 != NH3:
                        try:
                            fdb.child("Lova").child("Node" + str(nodeId)).update({
                                "Amo" + str(nodeId): newNH3,
                            })
                            
                            sqlNH3 = """
                                    UPDATE `pantautps`
                                        SET `Nilai` = '{0}',
                                        `Waktu` = current_timestamp()
                                    WHERE `pantautps`.`Node` = {1}
                                        AND
                                        `pantautps`.`Nama` LIKE 'nh3'
                                     """.format(newNH3, nodeId)
                            cc.execute(sqlNH3)
                            connection.commit()
                            NH3 = newNH3
                        except pymysql.err.InternalError as msg:
                            print("Command skipped: ", msg)
                    
                    print(datetime.now().strftime('%H:%M:%S-%d/%m/%Y'), 'Received payload: CH4:', CH4, 'CO2:', CO2, 'NH3:', NH3, 'node Id', nodeId, 'from node:', nodeId)
                except Exception as e:
                    print(e)
        
        except KeyboardInterrupt:
            radio.powerDown()
            connection.close()
            sys.exit()
