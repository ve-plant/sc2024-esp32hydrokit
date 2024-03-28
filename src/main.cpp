#include <WiFi.h>         // include wifi lib
#include <PubSubClient.h> // lib used to handle mqtt
#include <Wire.h>         // default lib to communicate with sensors
#include <ThingSpeak.h>   // lib used to parse human readable values from sensors in wifi hydroponics kit
#include <sequencer4.h>   // imports a 4 function sequencer
#include <sequencer1.h>   // imports a 1 function sequencer
#include <Ezo_i2c_util.h> // brings in common print statements
#include <Ezo_i2c.h>      // used to communicate with sensors in wifi hydroponics kit
#include <iot_cmd.h>      // lib used to read commands from serial input

// ------ FUNCTION DECLARATIONS ------
void wifiConnect();
void wifiAutoReconnect();
void mqttReconnect();
void mqttAutoReconnect();
void mqttPublishMessage(String topic, String msg);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void mqttUpdateState();
void readTemperature();
void readEcAndPh();
void parseEcAndPhValues();
void printEcAndPhValues();
void printHelp();
void startReadingSensors();
bool processUserInput(const String &string_buffer);
void readEcKValue();

// ------ DEFINITION FOR CONFIG ------
typedef struct
{
    float temperature;
    float ec;
    float ph;
} configData_t;

configData_t config; // stores config

// ------ WIFI DATA ------
const char *wifiSSID = "sciencecamp08";
const char *wifiPassword = "camperfurt";
WiFiClient client;

unsigned long wifiPreviousMillis = 0;
unsigned long wifiInterval = 30000;

// ------ MQTT DATA ------
const char *mqttBroker = "192.168.20.1";
const char *mqttClientId = "esp32hykit";
const char *mqttUsername = "admin";
const char *mqttPassword = "root";
const int mqttPort = 1883;
WiFiClient mqttWifiClient;

PubSubClient mqttClient;
unsigned long mqttPreviousMillis = 0;
unsigned long mqttInterval = 5000;
unsigned long mqttStateDelay = 10000;
unsigned long mqttStateTimer = 0;

// ------ SENSOR INSTANCES ------
Ezo_board PH = Ezo_board(99, "PH");
Ezo_board EC = Ezo_board(100, "EC");
Ezo_board RTD = Ezo_board(102, "RTD");
Ezo_board sensorList[] = {
    PH,
    EC,
    RTD};
Ezo_board *defaultSensor = &sensorList[0];                                   // used to store the board were talking to
const uint8_t sensorListLength = sizeof(sensorList) / sizeof(sensorList[0]); // save length of device list for later use

// ------ GPIO PINS TO ENABLE SENSORS ------
const int EN_PH = 12;
const int EN_EC = 27;
const int EN_RTD = 15;
const int EN_AUX = 33;

// ------ GENERAL DEFINES FOR HYDROPONICS KIT ------
bool polling = true;                                            // Activate/Deactivate sensor readings
const unsigned long readTimeout = 1000;                         // Timeout for reading sensor values
unsigned int sensorReadInterval = 2000 - readTimeout * 2 - 300; // Interval in which we check the sensors

float helpMenuState = 0; // Hold the state the help menu is currently in

// Periodically read sensor values with delays
Sequencer4 readSensors(&readTemperature, readTimeout,
               &readEcAndPh, 300,
               &parseEcAndPhValues, readTimeout,
               &printEcAndPhValues, sensorReadInterval);

/**
 * This function will create a connection tho the given wifi,
 * defined in the const variables before
 */
void wifiConnect()
{
    // Connect to the network
    WiFi.begin(wifiSSID, wifiPassword);

    // print status to serial port
    Serial.print("Connecting to ");
    Serial.print(wifiSSID);
    Serial.println(" ...");

    // try the connection 30 seconds
    int i = 0;
    bool createAP = false;
    while (WiFi.status() != WL_CONNECTED)
    { // Wait for the Wi-Fi to connect
        delay(1000);
        Serial.print(++i);
        Serial.print(' ');

        if (i > 30)
        {
            createAP = true;
            break;
        }
    }
}

/**
 * This function will handle a auto-reconnect to the given wifi
 * with an delay of wifiInterval (Default: 30s) between the next try
 */
void wifiAutoReconnect()
{
    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiPreviousMillis >= wifiInterval))
    {
        Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        wifiPreviousMillis = currentMillis;
    }
}

/**
 * This function will handle connect and reconnect of the mqtt client.
 */
void mqttReconnect()
{
    if (!mqttClient.connected())
    {
        Serial.println("Try to connect MQTT");
        if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword))
        {
            Serial.println("Public MQTT broker connected!");
            size_t len;

            {
                // With this topic, other clients can request a state update from this device
                String key = String(mqttClientId) + "/cmd/state";
                len = key.length() + 1;
                char strBuffer[len];
                key.toCharArray(strBuffer, len);
                mqttClient.subscribe(strBuffer);
            }

            // TODO: Subscribe to more mqtt topics if needed

        }
        else
        {
            Serial.println("Public MQTT broker not connected! Error: " + mqttClient.state());
            Serial.println(" retrying in 5 seconds");
            mqttPreviousMillis = millis();
        }
    }
}

/**
 * This function will handle a auto-reconnect to the given mqtt broker
 * with an delay of mqttInterval (Default: 5s) between the next try
 */
void mqttAutoReconnect()
{
    // handle reconnect mqtt with a 5 second delay
    if (!mqttClient.connected() && mqttPreviousMillis + mqttInterval < millis())
    {
        mqttReconnect();
    }
}


/**
 * This function will handle all incoming messages by the mqtt broker
 */
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Incoming message [");
    Serial.print(topic);
    Serial.print("]: ");
    for (unsigned int i = 0; i < length; ++i)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    String topicStr = String(topic);

    if (topicStr.equals(String(mqttClientId) + "/cmd/state"))
    {
        // Immediatly send state update on request
        mqttUpdateState();
    }

    // TODO: Handle more mqtt incoming topics if needed
}

/**
 * This function will help you to send messages to the mqtt broker by topic and message text
 */
void mqttPublishMessage(String topic, String msg)
{
    size_t len = topic.length() + 1;
    char topicArray[len];
    topic.toCharArray(topicArray, len);

    size_t lenP = msg.length() + 1;
    char payloadArray[lenP];
    msg.toCharArray(payloadArray, lenP);

    mqttClient.publish(topicArray, payloadArray);
}

/**
 * This function will send all sensor states via mqtt
*/
void mqttUpdateState()
{
    config.temperature = RTD.get_last_received_reading();
    config.ec = EC.get_last_received_reading();
    config.ph = PH.get_last_received_reading();

    String msg = "{\"temp\":\"";
    msg += String(config.temperature, 2);
    msg += "\",\"ec\":\"";
    msg += String(config.ec, 2);
    msg += "\",\"ph\":\"";
    msg += String(config.ph, 2);
    msg += "\"}";
    mqttPublishMessage(String(mqttClientId) + "/state", msg);
}

void readTemperature()
{
    // send a read command. we use this command instead of RTD.send_cmd("R");
    // to let the library know to parse the reading
    RTD.send_read_cmd();
}

void readEcAndPh()
{
    receive_and_print_reading(RTD); // get the reading from the RTD circuit

    if ((RTD.get_error() == Ezo_board::SUCCESS) && (RTD.get_last_received_reading() > -1000.0))
    { // if the temperature reading has been received and it is valid
        PH.send_cmd_with_num("T,", RTD.get_last_received_reading());
        EC.send_cmd_with_num("T,", RTD.get_last_received_reading());
        ThingSpeak.setField(3, String(RTD.get_last_received_reading(), 2)); // assign temperature readings to the third column of thingspeak channel
    }
    else
    {                                     // if the temperature reading is invalid
        PH.send_cmd_with_num("T,", 25.0); // send default temp = 25 deg C to PH sensor
        EC.send_cmd_with_num("T,", 25.0);
        ThingSpeak.setField(3, String(25.0, 2)); // assign temperature readings to the third column of thingspeak channel
    }

    Serial.print(" ");
}

void parseEcAndPhValues()
{
    // send a read command. we use this command instead of PH.send_cmd("R");
    // to let the library know to parse the reading
    PH.send_read_cmd();
    EC.send_read_cmd();
}

void printEcAndPhValues()
{
    receive_and_print_reading(PH); // get the reading from the PH circuit
    if (PH.get_error() == Ezo_board::SUCCESS)
    {                                                                      // if the PH reading was successful (back in step 1)
        ThingSpeak.setField(1, String(PH.get_last_received_reading(), 2)); // assign PH readings to the first column of thingspeak channel
    }
    Serial.print("  ");
    receive_and_print_reading(EC); // get the reading from the EC circuit
    if (EC.get_error() == Ezo_board::SUCCESS)
    {                                                                      // if the EC reading was successful (back in step 1)
        ThingSpeak.setField(2, String(EC.get_last_received_reading(), 0)); // assign EC readings to the second column of thingspeak channel
    }

    Serial.println();
}

void startReadingSensors()
{
    polling = true;
}

void setup()
{
    Serial.begin(9600); // start the serial communication to the computer
    Wire.begin();       // start the I2C
    ThingSpeak.begin(client); // enable ThingSpeak connection
    readSensors.reset();

    // Set pins for sensors to output to enable/disable them
    pinMode(EN_PH, OUTPUT);
    pinMode(EN_EC, OUTPUT);
    pinMode(EN_RTD, OUTPUT);
    pinMode(EN_AUX, OUTPUT);

    digitalWrite(EN_PH, LOW);       // Activate PH sensor
    digitalWrite(EN_EC, LOW);       // Activate EC sensor
    digitalWrite(EN_RTD, HIGH);     // Activate temperature sensor
    digitalWrite(EN_AUX, HIGH);     // Deactivate aux

    // start wifi connection
    wifiConnect();

    // start mqtt connection
    mqttClient.setClient(mqttWifiClient);
    mqttClient.setServer(mqttBroker, mqttPort);
    mqttClient.setCallback(mqttCallback);
    Serial.println("Start MQTT_Connect");
    mqttReconnect();

    // Print local IP address and start web server
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP()); // show ip address when connected on serial monitor.
}

void loop()
{
    // handle auto reconnect wifi/mqtt if needed
    wifiAutoReconnect();
    mqttAutoReconnect();

    // handle mqtt loop
    mqttClient.loop();

    String cmd; // variable to hold commands we send to the kit

    if (receive_command(cmd))
    {                    // if we sent the kit a command it gets put into the cmd variable
        polling = false; // we stop polling
        if (!processUserInput(cmd))
        {                                                                      // then we evaluate the cmd for kit specific commands
            process_command(cmd, sensorList, sensorListLength, defaultSensor); // then if its not kit specific, pass the cmd to the IOT command processing function
        }
    }

    if (polling == true)
    { // if polling is turned on, run the sequencer
        readSensors.run();
    }

    // Every time the delay has passed, send update to mqtt server
    if ((millis() - mqttStateTimer) > mqttStateDelay)
    {
        mqttUpdateState();
        mqttStateTimer = millis();
    }
}

bool processUserInput(const String &string_buffer)
{ // function to process commands that manipulate global variables and are specifc to certain kits
    if (string_buffer == "HELP")
    {
        printHelp();
        return true;
    }
    else if (string_buffer.startsWith("DATALOG"))
    {
        startReadingSensors();
        return true;
    }
    else if (string_buffer.startsWith("POLL"))
    {
        polling = true;
        readSensors.reset();

        int16_t index = string_buffer.indexOf(','); // check if were passing a polling delay parameter
        if (index != -1)
        {                                                                   // if there is a polling delay
            float new_delay = string_buffer.substring(index + 1).toFloat(); // turn it into a float

            float mintime = readTimeout * 2 + 300;
            if (new_delay >= (mintime / 1000.0))
            {                                                       // make sure its greater than our minimum time
                readSensors.set_step4_time((new_delay * 1000.0) - mintime); // convert to milliseconds and remove the reading delay from our wait
            }
            else
            {
                Serial.println("delay too short"); // print an error if the polling time isnt valid
            }
        }
        return true;
    }
    return false; // return false if the command is not in the list, so we can scan the other list or pass it to the circuit
}

void readEcKValue()
{                       // function to query the value of the ec circuit
    char rx_buf[10];    // buffer to hold the string we receive from the circuit
    EC.send_cmd("k,?"); // query the k value
    delay(300);
    if (EC.receive_cmd(rx_buf, 10) == Ezo_board::SUCCESS)
    {                                                          // if the reading is successful
        helpMenuState = String(rx_buf).substring(3).toFloat(); // parse the reading into a float
    }
}

void printHelp()
{
    readEcKValue();
    Serial.println(F("Atlas Scientific I2C hydroponics kit                                       "));
    Serial.println(F("Commands:                                                                  "));
    Serial.println(F("datalog      Takes readings of all sensors every 15 sec send to thingspeak "));
    Serial.println(F("             Entering any commands stops datalog mode.                     "));
    Serial.println(F("poll         Takes readings continuously of all sensors                    "));
    Serial.println(F("                                                                           "));
    Serial.println(F("ph:cal,mid,7     calibrate to pH 7                                         "));
    Serial.println(F("ph:cal,low,4     calibrate to pH 4                                         "));
    Serial.println(F("ph:cal,high,10   calibrate to pH 10                                        "));
    Serial.println(F("ph:cal,clear     clear calibration                                         "));
    Serial.println(F("                                                                           "));
    Serial.println(F("ec:cal,dry           calibrate a dry EC probe                              "));
    Serial.println(F("ec:k,[n]             used to switch K values, standard probes values are 0.1, 1, and 10 "));
    Serial.println(F("ec:cal,clear         clear calibration                                     "));

    if (helpMenuState > 9)
    {
        Serial.println(F("For K10 probes, these are the recommended calibration values:            "));
        Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
        Serial.println(F("  ec:cal,high,150000   calibrate EC probe to 150,000us                   "));
    }
    else if (helpMenuState > .9)
    {
        Serial.println(F("For K1 probes, these are the recommended calibration values:             "));
        Serial.println(F("  ec:cal,low,12880     calibrate EC probe to 12,880us                    "));
        Serial.println(F("  ec:cal,high,80000    calibrate EC probe to 80,000us                    "));
    }
    else if (helpMenuState > .09)
    {
        Serial.println(F("For K0.1 probes, these are the recommended calibration values:           "));
        Serial.println(F("  ec:cal,low,84        calibrate EC probe to 84us                        "));
        Serial.println(F("  ec:cal,high,1413     calibrate EC probe to 1413us                      "));
    }

    Serial.println(F("                                                                           "));
    Serial.println(F("rtd:cal,t            calibrate the temp probe to any temp value            "));
    Serial.println(F("                     t= the temperature you have chosen                    "));
    Serial.println(F("rtd:cal,clear        clear calibration                                     "));
}
