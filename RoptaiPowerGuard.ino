/*
  Project powermonitor van externe stroombron
  acties :
  1) verbind met het ethernet netwerk en verkrijg een ip adres.
  2) verbind met de mqtt server
  3) start hoofdprogramma
  --

  hoofdprogramma:
  Bewaken van 2 parameters :
  1) temperatuur van de aanvoerende stroomdraad (ds18b20)
  2) stroomsterkte en andere parameters (modbus via pzem-016)


  --

  debug info op usb, serial1

  --
  gebruikte pinnen: 
  motbus: 2,5,14,15
  relais: 32 en 33
  onewire: 17
  voordeurbel: 34


*/

#include <ETH.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>

HardwareSerial Pzemserial(2);



//Modbus tranciever gebruikt seriele verbinding + 2 pinnen voor data enable en reciever enable

#define RXD2 33 // reciever output voor de max485
#define TXD2 15 // Data in voor de max485

#define RXD1 36 //Gpio pins Serial2
#define TXD1 4

#define MAX485_DE      16  // We're using a MAX485-compatible RS485 Transceiver. The Data Enable and Receiver Enable pins are hooked up as follows:
#define MAX485_RE_NEG  32 


struct pzemvalues {
  float voltage;
  float current;
  float power;
  float energy;
  float hz;
  float pf;
  bool error;
  
};


static uint8_t pzemSlaveAddr = 0x01;

const char* mqtt_server = "192.168.10.3";
const int relay1 = 13;
const int relay2 = 14;

const int f1power = 39;
const int f2power = 36;
const int f3power = 35;

int errorcounter = 0;
int errorTempCounter = 0;
uint8_t reconnectCounter = 0;  // als de reconnectcounter boven de 10 komt, dan rebooten we de zaak.

static bool eth_connected = false;
int peakcounter = 0;
char buf[20];
String incoming = "";

//pzemteller
int i=1;


//vars voor timer millis 

unsigned long nextrun = 0;



ModbusMaster node;
WiFiClient espClient;
PubSubClient client(espClient);


void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
  case SYSTEM_EVENT_ETH_START:
    Serial.println("ETH Started");
    ETH.setHostname("esp32-ethernet");
    break;
  case SYSTEM_EVENT_ETH_CONNECTED:
    Serial.println("ETH Connected");
    break;
  case SYSTEM_EVENT_ETH_GOT_IP:
    Serial.print("ETH MAC: ");
    Serial.print(ETH.macAddress());
    Serial.print(", IPv4: ");
    Serial.print(ETH.localIP());
    if (ETH.fullDuplex()) {
      Serial.print(", FULL_DUPLEX");
    }
    Serial.print(", ");
    Serial.print(ETH.linkSpeed());
    Serial.println("Mbps");
    eth_connected = true;
    break;
  case SYSTEM_EVENT_ETH_DISCONNECTED:
    Serial.println("ETH Disconnected");
    eth_connected = false;
    break;
  case SYSTEM_EVENT_ETH_STOP:
    Serial.println("ETH Stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}



void preTransmission()  // Put RS485 Transceiver in transmit mode
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1);
}

void postTransmission()  // Put RS485 Transceiver back in receive mode (default mode)
{
  delay(3);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}



void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  if (messageTemp == "RELAY1_TRIP")
  {
    digitalWrite(relay1, HIGH);
    delay(500);
    digitalWrite(relay1, LOW);
    client.publish("powerguard/status", "RELAY1_TRIP_CONFIRM");
    Serial.println("RELAY1_TRIP_CONFIRM");
  }
  if (messageTemp == "RELAY2_TRIP")
  {
    digitalWrite(relay2, HIGH);
    delay(500);
    digitalWrite(relay2, LOW);
    client.publish("powerguard/status", "RELAY2_TRIP_CONFIRM");
    Serial.println("RELAY2_TRIP_CONFIRM");
  }

  Serial.println();
}


void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("powerguard")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("powerguard/command");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      Serial.print("reconnect counter is now: ");
      Serial.println(reconnectCounter);
      // Wait 5 seconds before retrying
      //delay(5000);
      if (reconnectCounter > 9) {
        //reboot
        ESP.restart();
      }
      else
      {
        reconnectCounter++;
      }
      
    }
  }
  else
  {
    //reconnect success. 
    reconnectCounter = 0;
  }
}

pzemvalues getPzemReading(uint8_t address) {
//  uint32_t tempdouble = 0x00000000;
  uint8_t result;
  //voltage,current,power,energy,hz,pf
  pzemvalues v = {0,0,0,0,0,0,false};
  node.slaveid(address);
  result = node.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
  if (result == node.ku8MBSuccess)
  {
    v.voltage = node.getResponseBuffer(0x0000) / 10.0;  //get the 16bit value for the voltage, divide it by 10 and cast in the float variable

    v.current = ((node.getResponseBuffer(0x0002) << 16) + node.getResponseBuffer(0x0001)) / 1000.00;
    v.power = ((node.getResponseBuffer(0x0004) << 16) + node.getResponseBuffer(0x0003)) / 10;
    v.energy = (node.getResponseBuffer(0x0006) << 16) + node.getResponseBuffer(0x0005);
    v.hz = node.getResponseBuffer(0x0007) / 10.0;
    v.pf = node.getResponseBuffer(0x0008) / 100.00;
    
  }
  else
  {
    v.error = true;
    Serial.print("Modbus error: "); Serial.println(result,HEX);

  }
  return v;
}

void pzemrun(int i){
   Serial.println();
   Serial.print("PZEM RUN start:");Serial.println(i);
    

    pzemvalues v = getPzemReading(i);
     char voltString[10];
    char currentString[10];
    char powerString[10];
    char energyString[10];
    char hzString[10];
    char pfString[10];

    dtostrf(v.voltage, 9, 2, voltString);
    dtostrf(v.current, 9, 2, currentString);
    dtostrf(v.power, 9, 2, powerString);
    dtostrf(v.energy, 9, 2, energyString);
    dtostrf(v.hz, 9, 2, hzString);
    dtostrf(v.pf, 9, 2, pfString);

Serial.print("nummer: "); Serial.println(i);
Serial.print("Volt: "); Serial.println(voltString);
Serial.print("Amps: "); Serial.println(currentString);
Serial.print("hz: "); Serial.println(hzString);
Serial.print("pf: "); Serial.println(v.pf);


    switch (i)
    {
    case 1:
      client.publish("powerguard/voltage/1", voltString);
      client.publish("powerguard/current/1", currentString);
      client.publish("powerguard/power/1", powerString);
      client.publish("powerguard/energy/1", energyString);
      client.publish("powerguard/hz/1", hzString);
      client.publish("powerguard/pf/1", pfString);
      break;
    case 2:
      client.publish("powerguard/voltage/2", voltString);
      client.publish("powerguard/current/2", currentString);
      client.publish("powerguard/power/2", powerString);
      client.publish("powerguard/energy/2", energyString);
      client.publish("powerguard/hz/2", hzString);
      client.publish("powerguard/pf/2", pfString);

      break;
    case 3:
      client.publish("powerguard/voltage/3", voltString);
      client.publish("powerguard/current/3", currentString);
      client.publish("powerguard/power/3", powerString);
      client.publish("powerguard/energy/3", energyString);
      client.publish("powerguard/hz/3", hzString);
      client.publish("powerguard/pf/3", pfString);

      break;
    case 4:
      client.publish("powerguard/voltage/4", voltString);
      client.publish("powerguard/current/4", currentString);
      client.publish("powerguard/power/4", powerString);
      client.publish("powerguard/energy/4", energyString);
      client.publish("powerguard/hz/4", hzString);
      client.publish("powerguard/pf/4", pfString);

      break;

    default:
      break;
    }
    //send readings to server
  
    
  
Serial.println("PZEM RUN end");
}

void setup()
{
  Pzemserial.begin(9600, SERIAL_8N1, RXD2, TXD2);
 
  
  Serial.begin(115200);
  node.begin(pzemSlaveAddr, Pzemserial);  //Start the Modbusmaster

  pinMode(MAX485_RE_NEG, OUTPUT);  // Setting up the RS485 transceivers
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);  // Init in receive mode
  digitalWrite(MAX485_DE, 0);
//preTransmission
  node.preTransmission(preTransmission);  // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);

  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, LOW);
  pinMode(f1power, INPUT);
  pinMode(f2power, INPUT);
  pinMode(f3power, INPUT);
  

  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  while (!eth_connected) {
    //geven het netwerk even de rust om te configureren.
    Serial.println("Wachten op het netwerk..");
    delay(1000);
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Serial.println("Startup complete");

    while (!client.connected()) {
      reconnect();
    }
    
    

    nextrun = millis(); // eerste meting meteen doen
//}
}

void loop()
{
  digitalWrite(relay2, LOW);
  digitalWrite(relay1, LOW);
  uint8_t result;
  //if (!client.connected()) {
  //  delay(5000);
  //  reconnect();
  //}
  //client.loop();

 if(millis() >= nextrun){
   nextrun=millis()+1000;
   pzemrun(i);
   i++;
   if (i>3) {
    i=1;
   }
} 
  
  
  //preTransmission();  // Put RS485 Transceiver in transmit mode
  //Pzemserial.write(0x65);
  //delay(1000);
  //Serial.print('.');

  if (digitalRead(f1power)) {
    Serial.println("Fase 1 power OK");
  }
  else
  {
    Serial.println("Fase 1 power ERROR");
  }
  if (digitalRead(f2power)) {
    Serial.println("Fase 2 power OK");
  }
  else
  {
    Serial.println("Fase 2 power ERROR");
  }
  if (digitalRead(f3power)) {
    Serial.println("Fase 3 power OK");
  }
  else
  {
    Serial.println("Fase 3 power ERROR");
  }

  
  
  
  
  
  delay(1000);
}







/*
     if (Slimmemeter.available()) {
  
        c = Slimmemeter.read();
        // --- 7 bits instelling ---
        c &= ~(1 << 7);
        rebuf[i]=c;
        Serial.println(c);
        if (c=='\n'){
          rebuf[i]='\0';
          client.publish("P1Meter/data", rebuf);
          i=0;
        }
        else
        {
          i++;
        }
    }

*/
