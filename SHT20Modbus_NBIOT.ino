#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <ModbusMaster.h>
#include "REG_CONFIG.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"


#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;



String deviceToken = "jGmVGPqcZKlXt8ppagBT";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";

ModbusMaster node;
void t1CallgetMeter();

void t2CallsendViaNBIOT();
//TASK
Task t1(28000, TASK_FOREVER, &t1CallgetMeter);

Task t2(30000, TASK_FOREVER, &t2CallsendViaNBIOT);
Scheduler runner;

struct Meter
{
  String temp;
  String hum;
 
};
 
Meter meter[10] ;
signal meta ;

void setup()
{

  Serial.begin(115200);
  SerialBT.begin("SHT20 TEMP/HUM_RS485"); //Bluetooth device name
  SerialBT.println("SHT20 TEMP/HUM_RS485");
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println(F("Starting... SHT20 TEMP/HUM_RS485 Monitor"));
  SerialBT.println(F("Starting... SHT20 TEMP/HUM_RS485 Monitor"));

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);

  Serial.println();
  Serial.println(F("***********************************"));


  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");

  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();

  String nccid = AISnb.getNCCID();
  Serial.print("getNCCID:");
  Serial.println(nccid);
  SerialBT.print("getNCCID:");
  SerialBT.println(nccid);
   
//  if (deviceToken.length() < 1 )
//    ESP.restart();

}


void t2CallsendViaNBIOT () {

  meta = AISnb.getSignal();

  Serial.print("RSSI:"); Serial.println(meta.rssi);
  Serial.print("NCCID:");Serial.println(AISnb.getNCCID());


  String json = "";
//  for(int i = 0; i< 1;i++) {
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"temp\":");
  json.concat(meter[0].temp);
  json.concat(",\"hum\":");
  json.concat(meter[0].hum);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);

  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  SerialBT.print("rssi:");
  SerialBT.println(meta.rssi);
}

void readMeter() {

   
  read_Modbus(data_) ;
   
  delay(2000);
}


void t1CallgetMeter() {     // Update read all data
  readMeter();
}

void loop()
{
  runner.execute();


}

void read_Modbus(uint16_t  REG)
{
 
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint16_t dat[2];
  uint32_t value = 0;
 
  
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readInputRegisters(REG, 2);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
  }
  for (int a = 0; a < 2; a++)
    {
      Serial.print(data[a]);
      Serial.print("\t");
    }
    Serial.println("");
     meter[0].temp = data[0];
     meter[0].hum = data[1];
    Serial.println("----------------------");
 
  
}
