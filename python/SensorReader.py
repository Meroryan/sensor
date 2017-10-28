# -*- coding: utf-8 -*-
from bluepy.btle import UUID, Scanner, Peripheral, Characteristic
#from SensorBase import SensorNode
import paho.mqtt.client as mqtt




AddrWhitelist = [ \
                "cb:ab:c8:3a:6f:13",\
                "cb:ab:c8:3a:6f:1a",\
                "cb:ab:c8:3a:6f:1b",\
                "cb:ab:c8:3a:6f:1c",\
                "cb:ab:c8:3a:6f:1d"\
                ]

MQTTTable = []
PrevConnTable = []
                
DevAddr1 = "cb:ab:c8:3a:6f:1b"
scanner = Scanner()

Char_DeviceName = ""
Char_Appearance = ""
Char_PPCP = ""
Char_Battery = 0
Char_ManufNameString = 0
Char_SerialNoString = 0
Char_HWRev = 0
Char_FWRev = 0
Char_SWRev = 0
Char_SensorLocation = 0

def MQTT_OnSubscribe(client, userdata, mid, granted_qos):
    print "MQTT subscription successful"

def MQTT_OnResetMsg(client, userdata, message):
    print("%s %s" % (message.topic, message.payload))
    del PrevConnTable[:]

def MQTT_OnConnect(client, userdata, flags, rc):
    print "MQTT connected, subscribing"
    client.subscribe("/Control")

def MQTT_OnPublish(c, userdata, mid):
    if userdata != 0:
        do_publish(c)
    else:
        MQTTTable = []
        print "MQTT publish finished"

def do_publish(c):
    #c.publish(MQTTTable[0]['topic'], MQTTTable[0]['payload'])
    #c.publish(MQTTTable.pop())

    m = MQTTTable.pop()
    if type(m) is dict:
        topic = m['topic']
        try:
            payload = m['payload']
        except KeyError:
            payload = None
        try:
            qos = m['qos']
        except KeyError:
            qos = 0
        try:
            retain = m['retain']
        except KeyError:
            retain = False
    elif type(m) is tuple:
        (topic, payload, qos, retain) = m
    else:
        raise ValueError('message must be a dict or a tuple')
    c.user_data_set(len(MQTTTable))
    c.publish(topic, payload, qos, retain)

def MQTT_Add(Topic, Payload):
    MQTTTable.append({'topic':Topic, 'payload':Payload})

def MQTT_PrepareTable(Node):
    DISData = Node.DIS.GetData()
    GenAcc = Node.GenAcc.GetData()

    if(DISData != 0):
        MQTT_Add("Test/DeviceInfo/ManufacturerName", DISData[0])
        MQTT_Add("Test/DeviceInfo/SerialNumber", DISData[1])

    if (GenAcc != 0):
        MQTT_Add("Test/DeviceInfo/DeviceName", GenAcc[0])
        MQTT_Add("Test/DeviceInfo/Appearance", str(GenAcc[1]))
        MQTT_Add("Test/DeviceInfo/MinConnInterval", str(GenAcc[2]))
        MQTT_Add("Test/DeviceInfo/MaxConnInterval", str(GenAcc[3]))
        MQTT_Add("Test/DeviceInfo/SlaveLatency", str(GenAcc[4]))
        MQTT_Add("Test/DeviceInfo/ConnSupTimeoutMul", str(GenAcc[5]))

    MQTT_Add("Test/EnvSensor/Temperature", Node.Temperature.GetData())
    MQTT_Add("Test/EnvSensor/Humidity", Node.Humidity.GetData())
    MQTT_Add("Test/EnvSensor/Pressure", Node.Pressure.GetData())

def MQTT_Publish(SrcNode, Client):
    MQTT_PrepareTable(SrcNode)
    #publish.multiple(MQTTTable, hostname="localhost")
    Client.user_data_set(len(MQTTTable))
    do_publish(Client)


def bin(s):
    return str(s) if s <= 1 else bin(s >> 1) + str(s & 1)

def main():

 #   found = 0
  #  print "Subscribing to MQTT control topic..."
    #subscribe.callback(MQTT_OnResetMsg, "/Control/Reset", hostname="localhost")
   # callback_userdata = {
    #        'callback':MQTT_OnResetMsg,
     #       'topics':"/Control/Reset",
      #      'qos':0,
       #     'userdata':None}
  #  client = mqtt.Client(client_id="",
   #                      userdata=callback_userdata, transport="tcp")
  #  client.on_connect = MQTT_OnConnect
   # client.on_message = MQTT_OnResetMsg
    #client.on_subscribe = MQTT_OnSubscribe
   # client.on_publish = MQTT_OnPublish

   # client.connect_async("localhost", 1883, 60)
    #client.loop_start()
    print "Scanning for devices..."
    while 1:

        devices = scanner.scan(10.0)
        for dev in devices:
            AdvDataStr = dev.getValueText(9)
            if AdvDataStr != None:
                print "Full Adv. Name String: %s\n" % AdvDataStr
                AdvDataInt = int(AdvDataStr,16)
                print "Adv. Data int: %d\n" % AdvDataInt
                Temp = (AdvDataInt >> (8*8)+5) - 500
                TempFloat = float(float(Temp)/10)
                print "Temperature:\t%2.1f °C" % TempFloat
                #print "Temp: %i" % Temp
                Hum = (AdvDataInt >> ((7*8)+3)) & 0x3FF
                HumFloat = float(float(Hum)/10)
                print "Humidity:\t%2.1f %%rH" % HumFloat
                Press = (AdvDataInt >> ((5 * 8) + 5)) & 0x3FFF
                PressFloat = float(float(Press)/10)
                print "Pressure:\t%2.1f hPa" % PressFloat
                Quali = (AdvDataInt >> ((4 * 8) + 4)) & 0x1FF
                print "Quality:\t%d " % Quali
                Illu = (AdvDataInt >> ((2 * 8) + 2)) & 0x3FFFF
                print "Luminosity:\t%d lux" % Illu
                SoilHum = ((AdvDataInt >> 2) & 0xFFFF) * 10
                print "SoilHumidity:\t%d Hz" % SoilHum
                Battery = (AdvDataInt & 0x03) * 25
                print "Battery:\t%d  %%" % Battery

            #print "Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi)
            #for (adtype, desc, value) in dev.getScanData():
             #   print "  %s = %s" % (desc, value)


                #for addr in AddrWhitelist:
             #   if dev.addr == addr:
              #      print "Found whitelist device:"
               #     print "\n\tAddr: %s; AddrType: %s; RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi)
                #    for pc in PrevConnTable:
                 #       if pc == addr:
 #                          found = 1
  #
   #                 tag = SensorNode(dev)
    #                if found == 0:
     #                   #Diesen Abschnitt nur einmalig bei unbekannten Devices ausführen
      #                  print "\tDevice not found in prev. conn. list, reading DevInfo and GenAcc"
       #                 tag.enableAll()
        #                tag.readAll()
         #
          #              tag.DIS.printData()
           #             tag.GenAcc.printData()
#
 #                       PrevConnTable.append(addr)
  #                  else:
   #                     tag.enableKnownDevice()
    #                    tag.readKnownDevice()
     #
      #              tag.battery.PrintBatteryLevel()
       #             tag.Temperature.PrintTemperature()
        #            tag.Humidity.PrintHumidity()
         #           tag.Pressure.PrintPressure()
          #          tag.SoilMoisture.PrintMoistureData()
           #         tag.Luminosity.PrintLuminosity()
            #        print "\n\tdisconnecting..."
             #       print "\n\tMQTT publish..."
              #      MQTT_Publish(tag, client)
#



      #              tag.disconnect()
       #             del tag
#
                    #publish.single("test/test1","testdaten", hostname="localhost")
 #                   #os.system("")
            #print "\nScanning for devices..."
#

if __name__ == "__main__":
    main()
		