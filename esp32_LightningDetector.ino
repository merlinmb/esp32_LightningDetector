#include "SPIFFS.h"
#include "connectionDetails.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_FONA.h"
#include <driver/adc.h>

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>

#include <SPI.h>
#include <AS3935SPI.h>

#include <NTPClient.h>
#include <TimeLib.h>

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(x)      Serial.print (x)
#define DEBUG_PRINTDEC(x,DEC) Serial.print (x, DEC)
#define DEBUG_PRINTLN(x)    Serial.println (x)
#define DEBUG_PRINTLNDEC(x,DEC) Serial.println (x, DEC)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x,DEC)
#define DEBUG_PRINTLN(x) 
#define DEBUG_PRINTLNDEC(x,DEC)
#endif

#define IRQpin 17
#define CSpin  5
#define SDA  21 
#define SCL  22
#define SCK 18
#define LED_BUILTIN  5

#define MAXNOISEFLOOR  7

#define CLEAR_AVERAGE_INTERVAL_MILLISECS	300000    //reset every 5 minutes of non-activity (i.e.: try group storm lightning counts together);
#define WIFI_TICKLED_INTERVAL_MILLISECS		15000    // every 15 seconds
long int _clear_Previous_Values_Interval_Millisecs = 1200000; //reset every 20 minutes of non-activity (i.e.: try group storm lightning counts together);

void setupAS3935();
void printAS3935Registers();
// Function prototype that provides SPI transfer and is passed to
// AS3935 to be used from within library, it is defined later in main sketch.
// That is up to user to deal with specific implementation of SPI
// Note that AS3935 library requires this function to have exactly this signature
// and it can not be member function of any C++ class, which happens
// to be almost any Arduino library
// Please make sure your implementation of choice does not deal with CS pin,
// library takes care about it on it's own
byte SPItransfer(byte sendByte);
// Iterrupt handler for AS3935 irqs
// and flag variable that indicates interrupt has been triggered
// Variables that get changed in interrupt routines need to be declared volatile
// otherwise compiler can optimize them away, assuming they never get changed
void AS3935Irq();
volatile int AS3935IrqTriggered;


#include "UpdateWebServer.h"
WebServer _httpServer(80);
bool _updatingFirmware = false;

AS3935 _AS3935(SPItransfer, CSpin, IRQpin);

WiFiClient _wifiClient;
PubSubClient _mqttClient(_wifiClient);

int calCap = 0;

//Lightning related vars
String _lastEvent = "";
String _lastEventTime = "";

int _noiseCount = 0;
int _disturberCount = 0;
int _lastStrokeDistance = 0;
long _lastLightningEnergy = 0;
long _runningTotal = 0L;
long _lightningCount = 0L;
long _lightningCountTotal = 0L;

String _lastMQTTMessage = "";
String _lastPublishedMQTTMessage = "";

unsigned long _runCurrent;
unsigned long _runClearPreviousValues;
unsigned long _runClearAverageValues;
unsigned long _runTickLED;

// Measure Signal Strength (RSSI) of Wi-Fi connection
long _rssi = 0;
// Measure Signal Strength (RSSI) of Wi-Fi connection
long _rssiQualityPercentage = 0;

struct configValues {

	int NoiseFloor = 3;
	int MinimumLightnings = 0;
	int SpikeRejection = 1;
	int WatchdogThreshold = 1;
	bool Indoors = false;
	bool EnableDisturbers = false;
	long int ResetInterval = 1200000; //every 20 minutes
};
configValues _currentConfigValues;
const bool _shouldSaveConfigWM = true;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
WiFiUDP ntpUDP;
//const byte offset = 2;
#define OFFSET 2
NTPClient timeClient(ntpUDP, "pool.ntp.org", OFFSET, 60000);
unsigned long _unixTime;
String _weatherDate = "";


void ICACHE_RAM_ATTR AS3935Irq();

unsigned long webUnixTime(Client& client)
{
	unsigned long time = 0;

	// Just choose any reasonably busy web server, the load is really low
	if (client.connect("g.cn", 80))
	{
		// Make an HTTP 1.1 request which is missing a Host: header
		// compliant servers are required to answer with an error that includes
		// a Date: header.
		client.print(F("GET / HTTP/1.1 \r\n\r\n"));

		char buf[5];			// temporary buffer for characters
		client.setTimeout(500);
		if (client.find((char*)"\r\nDate: ") // look for Date: header
			&& client.readBytes(buf, 5) == 5) // discard
		{
			unsigned day = client.parseInt();	   // day
			client.readBytes(buf, 1);	   // discard
			client.readBytes(buf, 3);	   // month
			int year = client.parseInt();	   // year
			byte hour = client.parseInt();   // hour
			byte minute = client.parseInt(); // minute
			byte second = client.parseInt(); // second

			int daysInPrevMonths;
			switch (buf[0])
			{
			case 'F': daysInPrevMonths = 31; break; // Feb
			case 'S': daysInPrevMonths = 243; break; // Sep
			case 'O': daysInPrevMonths = 273; break; // Oct
			case 'N': daysInPrevMonths = 304; break; // Nov
			case 'D': daysInPrevMonths = 334; break; // Dec
			default:
				if (buf[0] == 'J' && buf[1] == 'a')
					daysInPrevMonths = 0;		// Jan
				else if (buf[0] == 'A' && buf[1] == 'p')
					daysInPrevMonths = 90;		// Apr
				else switch (buf[2])
				{
				case 'r': daysInPrevMonths = 59; break; // Mar
				case 'y': daysInPrevMonths = 120; break; // May
				case 'n': daysInPrevMonths = 151; break; // Jun
				case 'l': daysInPrevMonths = 181; break; // Jul
				default: // add a default label here to avoid compiler warning
				case 'g': daysInPrevMonths = 212; break; // Aug
				}
			}

			// This code will not work after February 2100
			// because it does not account for 2100 not being a leap year and because
			// we use the day variable as accumulator, which would overflow in 2149
			day += (year - 1970) * 365;	// days from 1970 to the whole past year
			day += (year - 1969) >> 2;	// plus one day per leap year 
			day += daysInPrevMonths;	// plus days for previous months this year
			if (daysInPrevMonths >= 59	// if we are past February
				&& ((year & 3) == 0))	// and this is a leap year
				day += 1;			// add one day
									// Remove today, add hours, minutes and seconds this month
			time = (((day - 1ul) * 24 + hour) * 60 + minute) * 60 + second;
		}
	}
	//delay(10);
	client.flush();
	client.stop();

	return time;
}


void setupTimeClient() {
	DEBUG_PRINTLN("Fetching NTP time");
	timeClient.setTimeOffset(OFFSET * 3600);
	timeClient.begin();
}

void getTimeUpdate() {
	while (!timeClient.forceUpdate())
	{
		DEBUG_PRINTLN("Fetching Time Update");
		delay(350);
	}
	DEBUG_PRINTLN(timeClient.getFormattedTime());

	_unixTime = webUnixTime(_wifiClient);
	setTime(_unixTime);
}

void tickLED()
{
	//toggle state
	int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
	if (WiFi.status() == WL_CONNECTED)	//only switch LED if Wifi is connected
	{
		digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
	}
	else
	{
		// Turn the LED on (Note that LOW is the voltage level
		// but actually the LED is on; this is because 
		// it is acive low on the ESP-01)
		digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED off by making the voltage HIGH
	}
}

void mqttReconnect() {
	String __mqttClientID = String(MQTT_CLIENTNAME) + String(random(0xffff), HEX);
	DEBUG_PRINT("MQTT reconnecting as: ");
	DEBUG_PRINTLN(__mqttClientID);
	int __retryCount = 0;
	while (!_mqttClient.connected() && __retryCount < 3) {
		if (_mqttClient.connect(__mqttClientID.c_str())) {
			DEBUG_PRINTLN("connected");

			// Once connected, publish an announcement...
			DEBUG_PRINTLN("MQTT: Setting callback");
			_mqttClient.setCallback(mqttCallback);

			// Once connected, publish an announcement...
			String __outMessage = "tele/" + String(MQTT_CLIENTNAME) + "/alive";
			_mqttClient.publish(__outMessage.c_str(), "1");
			__outMessage = "tele/" + String(MQTT_CLIENTNAME) + "/ip";
			_mqttClient.publish(__outMessage.c_str(), IpAddress2String(WiFi.localIP()).c_str());

			// ... and resubscribe
			delay(200);
			String __topic = "cmnd/" + String(MQTT_CLIENTNAME) + "/Power";
			_mqttClient.subscribe(__topic.c_str());
		}
		else {
			DEBUG_PRINT("failed, rc=");
			DEBUG_PRINT(_mqttClient.state());
			DEBUG_PRINTLN(" try again in 2 seconds");
			// Wait a few seconds before retrying
			delay(2000);
			__retryCount++;
		}
	}
}

void mqttPublish(String name, String value)
{
	String __topic = ("stat/" + String(MQTT_CLIENTNAME) + "/" + name);
	DEBUG_PRINTLN("    " + __topic + " " + value);
	_mqttClient.publish(__topic.c_str(), value.c_str());

	_lastPublishedMQTTMessage = __topic + " " + value;
}

void mqttPublishStat(String name, String value)
{
	mqttPublishStat(name, value, false);
}

void mqttPublishStat(String name, String value, bool retain)
{
	DEBUG_PRINTLN("MQTT Sending ");
	String __ret = ("stat/" + String(MQTT_CLIENTNAME) + "/" + name);
	String __retVal = value;
	DEBUG_PRINTLN(__ret + " " + __retVal);
	_mqttClient.publish(__ret.c_str(), __retVal.c_str(), retain);
	DEBUG_PRINTLN("MQTT: Sent");
}

void transmitStat() {
	mqttPublish("NoiseCount", String(_noiseCount));
	mqttPublish("DisturberCount", String(_disturberCount));
	mqttPublish("LastEvent", _lastEvent);
	mqttPublish("LastEventTime", _lastEventTime);
	mqttPublish("EnableDisturbers", String(_currentConfigValues.EnableDisturbers));
	mqttPublish("CalibrationCap", String(calCap));
	mqttPublish("Indoors", String(_currentConfigValues.Indoors));
	mqttPublish("MinimumLightnings", String(_currentConfigValues.MinimumLightnings));
	mqttPublish("NoiseFloor", String(_currentConfigValues.NoiseFloor));
	mqttPublish("SpikeRejection", String(_currentConfigValues.SpikeRejection));
	mqttPublish("WatchdogThreshold", String(_currentConfigValues.WatchdogThreshold));
	mqttPublish("ResetInterval", String(_currentConfigValues.ResetInterval));
	//	publish("iftttkey", String(_iotKey));
	mqttPublish("RunningTotal", String(_runningTotal));
	mqttPublish("RunningCount", String(_lightningCount));
	mqttPublish("IPAddress", WiFi.localIP().toString());
}

String IpAddress2String(const IPAddress& ipAddress) {

	return String(ipAddress[0]) + String(".") + \
		String(ipAddress[1]) + String(".") + \
		String(ipAddress[2]) + String(".") + \
		String(ipAddress[3]);
}

void mqttTransmitInitStat() {
	mqttPublishStat("init", "{\"value1\":\"" + IpAddress2String(WiFi.localIP()) + "\",\"value2\":\"" + WiFi.macAddress() + "\",\"value3\":\"" + MQTT_CLIENTNAME + "\",\"signalQuality\":\"" + _rssiQualityPercentage + "\"}");
}


void transmitStrike()
{
	if (WiFi.status() == WL_CONNECTED) {

		DEBUG_PRINTLN("Send to MQTT server");

		if (!_mqttClient.connected()) { mqttReconnect(); }
		if (_mqttClient.connected()) {
			mqttPublish("LastlightningStrike", String(_lastStrokeDistance));
			mqttPublish("LastlightningEnergy", String(_lastLightningEnergy));
			mqttPublish("NoStrikesthisstorm", String(_lightningCountTotal));
			mqttPublish("LightningJSON", "{\"value1\":\"" + String(_lastStrokeDistance) + "\",\"value2\":\"" + String(_lastLightningEnergy) + "\",\"value3\":\"" + String(_lightningCountTotal) + "\"}");
		}
	}
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
	DEBUG_PRINT("Message arrived [");
	DEBUG_PRINT(topic);
	DEBUG_PRINT("] ");
	char message_buff[100];
	int i = 0;
	for (i = 0; i < length; i++) { message_buff[i] = payload[i]; }
	message_buff[i] = '\0';
	String __payloadString = String(message_buff);

	DEBUG_PRINTLN(__payloadString);

	String __incomingTopic = String(topic);

	_lastMQTTMessage = __incomingTopic + " " + __payloadString;

	if (__incomingTopic == "cmnd/" + String(MQTT_CLIENTNAME) + "/Power")
	{
		if (__payloadString == "2") {
			DEBUG_PRINTLN("Resetting ESP");
			ESP.restart();
		}
		if (__payloadString == "3") { //send test message
			DEBUG_PRINTLN("Building Payload");
			String __retValue = "{\"value1\":\"" + String(_lastStrokeDistance) + "\",\"value2\":\"" + String(_lastLightningEnergy) + "\",\"value3\":\"" + String(_lightningCountTotal) + "\"}";
			DEBUG_PRINTLN(__retValue);
			mqttPublish("LightningJSON", __retValue);
		}
	}
}

int raise_noise_floor() {
	//"Raise the noise floor by one step
	//MAXNOISEFLOOR is the maximum step that the noise_floor should be raised to.

	int __floor = _AS3935.getNoiseFloor();
	if (__floor < MAXNOISEFLOOR)
		__floor++;
	_AS3935.setNoiseFloor(__floor);
	return __floor;
}

void callback_clearPrevious() {
	_lastEvent = "";
	_lightningCount = 0L;
	_disturberCount = 0;
	_lastStrokeDistance = 0;
	_lastLightningEnergy = 0L;

	_runningTotal = 0L;
	_lightningCountTotal = 0L;
}

void setupSPI() {
	// first begin, then set parameters

	DEBUG_PRINTLN("SPI Begin");

	SPI.begin();

	DEBUG_PRINTLN("SPI Mode1");
	// NB! chip uses SPI MODE1
	SPI.setDataMode(SPI_MODE1);
	// NB! max SPI clock speed that chip supports is 2MHz,
	// but never use 500kHz, because that will cause interference
	// to lightning detection circuit
	DEBUG_PRINTLN("SPI Clock Div 64");
	SPI.setClockDivider(SPI_CLOCK_DIV64);   //Arduino: SPI_CLOCK_DIV16
											// and chip is MSB first
	DEBUG_PRINTLN("MSB First");
	SPI.setBitOrder(MSBFIRST);
	// reset all internal register values to defaults
}

//void loadCustomParamsSPIFFS() {
//	//read configuration from FS json
//	DEBUG_PRINTLN("Mounting FS...");
//
//	if (SPIFFS.begin()) {
//		DEBUG_PRINTLN("mounted file system");
//		if (SPIFFS.exists("/config.json")) {
//			//file exists, reading and loading
//			DEBUG_PRINTLN("reading config file");
//			File configFile = SPIFFS.open("/config.json", "r");
//			if (configFile) {
//				DEBUG_PRINT("opened config file: ");
//				size_t size = configFile.size();
//				DEBUG_PRINT(String(size)); DEBUG_PRINTLN(" bytes");
//				// Allocate a buffer to store contents of the file.
//				std::unique_ptr<char[]> buf(new char[size]);
//
//				configFile.readBytes(buf.get(), size);
//				DynamicJsonDocument jsonBuffer;
//				DeserializationError error = deserializeJson(jsonBuffer, buf);
//				JsonObject json = jsonBuffer.as<JsonObject>();
//				
//				if (!error) {
//					DEBUG_PRINTLN("\nparsed json");
//
//					_currentConfigValues.EnableDisturbers = json["EnableDisturbers"];
//					_currentConfigValues.Indoors = json["Indoors"];
//					_currentConfigValues.MinimumLightnings = json["MinimumLightnings"];
//					_currentConfigValues.NoiseFloor = json["NoiseFloor"];
//					_currentConfigValues.SpikeRejection = json["SpikeRejection"];
//					_currentConfigValues.WatchdogThreshold = json["WatchdogThreshold"];
//					_currentConfigValues.ResetInterval = json["ResetInterval"];
//
//				}
//				else {
//					DEBUG_PRINTLN("failed to load json config");
//				}
//			}
//		}
//	}
//	else {
//		DEBUG_PRINTLN("failed to mount FS");
//	}
//	//end read
//}
//
//void saveConfigValuesSPIFFS() {
//	if (_shouldSaveConfigWM) {
//		DEBUG_PRINTLN("Saving config to FS");
//
//		DynamicJsonDocument jsonBuffer;
//		JsonObject json = jsonBuffer.to<JsonObject>();
//
//		json["EnableDisturbers"] = _currentConfigValues.EnableDisturbers;
//		json["Indoors"] = _currentConfigValues.Indoors;
//		json["MinimumLightnings"] = _currentConfigValues.MinimumLightnings;
//		json["NoiseFloor"] = _currentConfigValues.NoiseFloor;
//		json["SpikeRejection"] = _currentConfigValues.SpikeRejection;
//		json["WatchdogThreshold"] = _currentConfigValues.WatchdogThreshold;
//		json["ResetInterval"] = _currentConfigValues.ResetInterval;
//
//
//		File configFile = SPIFFS.open("/config.json", "w");
//		if (!configFile) {
//			DEBUG_PRINTLN("failed to open config file for writing");
//		}
//		else {
//			serializeJson(json, Serial);
//			serializeJson(json, configFile);
//			configFile.close();
//			//end save
//			DEBUG_PRINTLN("Saving config completed");
//		}
//	}
//}

void writeConfigValuesToAS3935()
{
	DEBUG_PRINTLN("Setting Config values");
	_AS3935.setNoiseFloor(_currentConfigValues.NoiseFloor);
	_AS3935.setMinimumLightnings(_currentConfigValues.MinimumLightnings);
	_AS3935.setSpikeRejection(_currentConfigValues.SpikeRejection);
	_AS3935.setWatchdogThreshold(_currentConfigValues.WatchdogThreshold); //4 bits: default 0010



	if (_currentConfigValues.Indoors) {

		// first let's turn on disturber indication and print some register values from AS3935
		// tell AS3935 we are indoors, for outdoors use setOutdoors() function
		DEBUG_PRINTLN("setIndoors");
		_AS3935.setIndoors();
	}
	else {
		_AS3935.setOutdoors();
	}

	if (_currentConfigValues.EnableDisturbers) {
		// turn on indication of distrubers, once you have AS3935 all tuned, you can turn those off with disableDisturbers()
		DEBUG_PRINTLN("enableDisturbers");
		_AS3935.enableDisturbers();
	}
	else
	{
		_AS3935.disableDisturbers();
	}

	DEBUG_PRINTLN("outputCalibrationValues");
	outputCalibrationValues();
	DEBUG_PRINTLN("recalibrate");
	recalibrate();
}

void setupAS3935() {

	DEBUG_PRINTLN("AS3935.reset");
	_AS3935.reset();
	// and run calibration
	// if lightning detector can not tune tank circuit to required tolerance,
	// calibration function will return false

	if (!_AS3935.calibrate())
		DEBUG_PRINTLN("Tuning out of range, check your wiring, your sensor and make sure physics laws have not changed!");

	DEBUG_PRINTLN("outputCalibrationValues");
	outputCalibrationValues();
	DEBUG_PRINTLN("recalibrate");
	recalibrate();

	writeConfigValuesToAS3935();

	DEBUG_PRINTLN("printAS3935Registers");
	printAS3935Registers();
	AS3935IrqTriggered = 0;
	// Using interrupts means you do not have to check for pin being set continiously, chip does that for you and
	// notifies your code
	// demo is written and tested on ChipKit MAX32, irq pin is connected to max32 pin 2, that corresponds to interrupt 1
	// look up what pins can be used as interrupts on your specific board and how pins map to int numbers

	DEBUG_PRINTLN("Attaching to Interrupt");
	attachInterrupt(IRQpin, AS3935Irq, RISING);
	DEBUG_PRINTLN("Attach complete");
}

void printAS3935Registers() {
	int noiseFloor = _AS3935.getNoiseFloor();
	int spikeRejection = _AS3935.getSpikeRejection();
	int watchdogThreshold = _AS3935.getWatchdogThreshold();
	int minLightning = _AS3935.getMinimumLightnings();
	DEBUG_PRINT("Noise floor is: ");
	DEBUG_PRINTLNDEC(noiseFloor, DEC);
	DEBUG_PRINT("Spike rejection is: ");
	DEBUG_PRINTLNDEC(spikeRejection, DEC);
	DEBUG_PRINT("Watchdog threshold is: ");
	DEBUG_PRINTLNDEC(watchdogThreshold, DEC);
	DEBUG_PRINT("Minimum Lightning is: ");
	DEBUG_PRINTLNDEC(minLightning, DEC);
}

byte SPItransfer(byte sendByte) {
	// this is implementation of SPI transfer that gets passed to AS3935
	// you can (hopefully) wrap any SPI implementation in this
	return SPI.transfer(sendByte);
}

void AS3935Irq() {
	// this is irq handler for AS3935 interrupts, has to return void and take no arguments
	// always make code in interrupt handlers fast and short
	AS3935IrqTriggered = 1;
}

void recalibrate() {
	delay(50);
	DEBUG_PRINTLN();
	calCap = _AS3935.getBestTune();
	DEBUG_PRINT("antenna calibration picks value:\t ");
	DEBUG_PRINTLN(calCap);
	delay(50);
}

void outputCalibrationValues() {
	// output the frequencies that the different capacitor values set:
	delay(50);
	DEBUG_PRINTLN();
	for (byte i = 0; i <= 0x0F; i++) {
		int frequency = _AS3935.tuneAntenna(i);
		DEBUG_PRINT("tune antenna to capacitor ");
		DEBUG_PRINT(i);
		DEBUG_PRINT("\t gives frequency: ");
		DEBUG_PRINT(frequency);
		DEBUG_PRINT(" = ");
		long fullFreq = (long)frequency * 160;  // multiply with clock-divider, and 10 (because measurement is for 100ms)
		DEBUG_PRINTDEC(fullFreq, DEC);
		DEBUG_PRINTLN(" Hz");
		delay(10);
	}
}

void handleSendToRoot()
{
	_httpServer.sendHeader("Location", "/", true); //Redirect to our html web page 
	_httpServer.send(302, "text/plane", "");
}

void setupWebServer() {

	DEBUG_PRINTLN("Handling Web Request...");

	_httpServer.on("/", []() {

		String __infoStr = "";

		__infoStr += "<div align=left><H1><i>" + String(MQTT_CLIENTNAME) + "</i></H1>";
		__infoStr += "<br>Connected to: " + String(SSID) + " (" + _rssiQualityPercentage + "%)<br><hr>";

		__infoStr += "Last lightning Strike: " + String(_lastStrokeDistance) + "kms\n<br>";
		__infoStr += "Last lightning Energy: " + String(_lastLightningEnergy) + "\n<br>";

		__infoStr += "# Strikes this storm: " + String(_lightningCountTotal) + "\n<br>\n<br>";
		__infoStr += "Noise Count: " + String(_noiseCount) + "\n<br>";
		__infoStr += "Disturber Count: " + String(_disturberCount) + "\n<br>";
		__infoStr += "Last Event: " + _lastEvent + "\n<br>\n<br>";
		__infoStr += "Last Event Time:	" + _lastEventTime + "\n<br>\n<br>";

		__infoStr += "EnableDisturbers:	" + String(_currentConfigValues.EnableDisturbers) + "\n<br>";
		__infoStr += "Calibration Cap:	" + String(calCap) + "\n<br>";
		__infoStr += "Indoors:			" + String(_currentConfigValues.Indoors) + "\n<br>";
		__infoStr += "MinimumLightnings:	" + String(_currentConfigValues.MinimumLightnings) + "\n<br>";
		__infoStr += "NoiseFloor:        " + String(_currentConfigValues.NoiseFloor) + "\n<br>";
		__infoStr += "SpikeRejection:    " + String(_currentConfigValues.SpikeRejection) + "\n<br>";
		__infoStr += "WatchdogThreshold: " + String(_currentConfigValues.WatchdogThreshold) + "\n<br>";
		__infoStr += "ResetInterval:     " + String(_currentConfigValues.ResetInterval) + "\n<br>";
		__infoStr += "Running Total:         " + String(_runningTotal) + "\n<br>";
		__infoStr += "Running Count:         " + String(_lightningCount) + "\n<br><br><hr>";


		__infoStr += "<br>Last Message Received:  <br><i>" + _lastMQTTMessage;
		__infoStr += "<br>Last Message Published: <br><i>" + _lastPublishedMQTTMessage;

		__infoStr += "<hr><br>IP Address: " + IpAddress2String(WiFi.localIP());
		__infoStr += "<br>MAC Address: " + WiFi.macAddress();
		__infoStr += "</div>";

		String __retStr = loginIndex + __infoStr + loginIndex2;



		_httpServer.sendHeader("Connection", "close");
		_httpServer.send(200, "text/html", __retStr);

		});

	_httpServer.on("/serverIndex", HTTP_GET, []() {
		_httpServer.sendHeader("Connection", "close");
		_httpServer.send(200, "text/html", serverIndex);
		});

	_httpServer.on("/reset", []() {
		String _webClientReturnString = "Resetting NodeMCU";
		_httpServer.send(200, "text/plain", _webClientReturnString);
		ESP.restart();
		delay(1000);
		});
	_httpServer.on("/resetLightningSettings", []() {
		String _webClientReturnString = "Resetting AS3935 Settings";
		_httpServer.send(200, "text/plain", _webClientReturnString);

		if (SPIFFS.exists("/config.json")) {
			DEBUG_PRINTLN("Removing Configuration files from SPIFFS");
			SPIFFS.remove("/config.json");
		}

		//saveConfigValuesSPIFFS();
		//writeConfigValuesToAS3935();
		});


	_httpServer.on("/defaults", []() {

		String _webClientReturnString = "Resetting AS3935 to defaults";
		_httpServer.send(200, "text/plain", _webClientReturnString);
		configValues __defaultConfigValues;
		_currentConfigValues = __defaultConfigValues;

		//saveConfigValuesSPIFFS();
		//writeConfigValuesToAS3935();

		});

	_httpServer.on("/set", []()
		{
			String __retMessage = "";
			int __val = 0;
			bool __update = false;

			for (uint8_t i = 0; i < _httpServer.args(); i++) {
				__val = _httpServer.arg(i).toInt();

				if (_httpServer.argName(i) == "NoiseFloor") {
					if (_currentConfigValues.NoiseFloor != __val)
					{
						_currentConfigValues.NoiseFloor = __val;
						__update = true;
					}
				}
				else if (_httpServer.argName(i) == "MinimumLightnings") {
					if (_currentConfigValues.MinimumLightnings != __val)
					{
						_currentConfigValues.MinimumLightnings = __val;
						__update = true;
					}
				}
				else if (_httpServer.argName(i) == "SpikeRejection") {
					if (_currentConfigValues.SpikeRejection != __val)
					{
						_currentConfigValues.SpikeRejection = __val;
						__update = true;
					}
				}
				else if (_httpServer.argName(i) == "WatchdogThreshold") {
					if (_currentConfigValues.WatchdogThreshold != __val)
					{
						_currentConfigValues.WatchdogThreshold = __val;
						__update = true;
					}
				}
				else if (_httpServer.argName(i) == "ResetInterval") {
					if (_currentConfigValues.ResetInterval != __val)
					{
						_currentConfigValues.ResetInterval = __val;
						__update = true;
					}
				}

				__retMessage += " " + _httpServer.argName(i) + ": " + _httpServer.arg(i) + (__update?" set.":" not set.")+ "\n";
			}
			_httpServer.send(200, "text/plain", __retMessage);
			if (__update) {
				//saveConfigValuesSPIFFS();
				writeConfigValuesToAS3935();
			}
		});

	/*handling uploading firmware file */
	_httpServer.on("/update", HTTP_POST, []() {
		_httpServer.sendHeader("Connection", "close");
		_httpServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
		}, []() {
			HTTPUpload& upload = _httpServer.upload();
			if (upload.status == UPLOAD_FILE_START) {
				DEBUG_PRINT("Update: "); DEBUG_PRINTLN(upload.filename.c_str());
				if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_WRITE) {
				/* flashing firmware to ESP*/
				if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
					Update.printError(Serial);
				}
			}
			else if (upload.status == UPLOAD_FILE_END) {
				if (Update.end(true)) { //true to set the size to the current progress
					DEBUG_PRINTLN("Update Success:" + String(upload.totalSize) + "\nRebooting...\n");
				}
				else {
					Update.printError(Serial);
				}
			}
		});

	_httpServer.onNotFound(handleSendToRoot);

	_httpServer.begin();

	DEBUG_PRINTLN("Web Request Completed...");
}

void callback_clearAvg()
{
	_runningTotal = 0L;
	_lightningCount = 0L;
}

void setupOTA() {


	ArduinoOTA
		.onStart([]() {
		_updatingFirmware = true;
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		DEBUG_PRINTLN("Start updating " + type);
			})
		.onEnd([]() {
				DEBUG_PRINTLN("\nEnd");
			})
				.onProgress([](unsigned int progress, unsigned int total) {
				DEBUG_PRINT("Progress: \r"); DEBUG_PRINT((progress / (total / 100))); DEBUG_PRINTLN("%");
					})
				.onError([](ota_error_t error) {
						DEBUG_PRINT("Error[%u]: "); DEBUG_PRINTLN(error);
						if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
						else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
						else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
						else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
						else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
						delay(1000);
						ESP.restart();
					});

					ArduinoOTA.begin();
}

int WiFiConnectionCheck()
{

	if (WiFi.status() == WL_CONNECTED)
	{
		return true;
	}

	int connAttempts = 0;
	DEBUG_PRINT("********** Free Heap: "); 	DEBUG_PRINTLN(ESP.getFreeHeap());

	DEBUG_PRINTLN("\r\nConnecting to: " + String(SSID));
	//delay(1000); //pause to prevent 'brown out' - to much current required during startup
	WiFi.begin(SSID, WIFIPASSWORD);

	byte counter = 0;
	while (WiFi.status() != WL_CONNECTED)
	{
		DEBUG_PRINT(".");
		counter++;
		if (counter == 45) {  //45 seconds
			ESP.restart();
		}
		delay(500);
	}


	DEBUG_PRINTLN();
	DEBUG_PRINTLN("WiFi connected\r\n");
	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.localIP());
	DEBUG_PRINT("********** Free Heap: "); 	DEBUG_PRINTLN(ESP.getFreeHeap());

	_rssi = WiFi.RSSI();
	_rssiQualityPercentage = 2 * (_rssi + 100);
	DEBUG_PRINT("RSSI: ");	DEBUG_PRINTLNDEC(_rssi, 1);
	DEBUG_PRINT("WiFi Quality: ");	DEBUG_PRINTDEC(_rssiQualityPercentage, 1); DEBUG_PRINTLN("%");


	mqttTransmitInitStat();

	return 1;
}

void ResetPreviousValueTimer() {
	_runClearPreviousValues = millis(); //reset storm lightning counter timer
}

void checkIRQforInterrupt() {

	// here we go into loop checking if interrupt has been triggered, which kind of defeats
	// the whole purpose of interrupts, but in real life you could put your chip to sleep
	// and lower power consumption or do other nifty things
	if (AS3935IrqTriggered)
	{


		// reset the flag
		AS3935IrqTriggered = 0;
		_lastEvent = "";

		// wait 2 ms before reading register (according to datasheet?)
		delay(2);

		// first step is to find out what caused interrupt
		// as soon as we read interrupt cause register, irq pin goes low
		int irqSource = _AS3935.interruptSource();

		// returned value is bitmap field, bit 0 - noise level too high, bit 2 - disturber detected, and finally bit 3 - lightning!
		if (irqSource & 0b0001) {
			_lastEvent = "Noise";
			_noiseCount++;

			int __retNF = raise_noise_floor();
			DEBUG_PRINT("Noise level too high, adjusting noise floor to: ");
			DEBUG_PRINTLN(__retNF);

		}
		if (irqSource & 0b0100) {
			_lastEvent = "Disturber";
			_disturberCount++;

			DEBUG_PRINTLN("Disturber detected");

		}
		if (irqSource & 0b1000)
		{
			_lastEvent = "Lightning *";
			// need to find how far that lightning stroke, function returns approximate distance in kilometers,
			// where value 1 represents storm in detector's near victinity, and 63 - very distant, out of range stroke
			// everything in between is just distance in kilometers
			_lastStrokeDistance = _AS3935.lightningDistanceKm();
			_lastLightningEnergy = _AS3935.lightningEnergy();


			if (_lastStrokeDistance < 63 && _lastStrokeDistance >= 0)
			{
				_lastEventTime = _weatherDate + " " + timeClient.getFormattedTime();

				DEBUG_PRINT(_lastEventTime);
				DEBUG_PRINT(" Lightning detected ");
				DEBUG_PRINTDEC(_lastStrokeDistance, DEC);
				DEBUG_PRINTLN(" kilometers away.");

				long __currentAverage = 0L;
				if (_lightningCount > 0L)
					__currentAverage = (long)(_runningTotal / _lightningCount);

				_lightningCount++;
				_lightningCountTotal++;
				_runningTotal += _lastStrokeDistance;

				long __newAverage = (long)(_runningTotal / _lightningCount);

				if ((_lightningCountTotal == 1L) || (__currentAverage != __newAverage) || (_lightningCountTotal % 25 == 0)) //log every 25th strike anyway
				{
					//log the first strike
					//_iftttEvent.setValue(0, String(_lightningCountTotal));
					//_iftttEvent.setValue(1, String(_lastLightningEnergy));
					//_iftttEvent.setValue(2, String(_lastStrokeDistance));
					transmitStrike();
				}

				ResetPreviousValueTimer();
			}
		}
	}
}

void setup()
{
#ifdef DEBUG
	Serial.begin(115200);
#endif // DEBUG

	DEBUG_PRINTLN("Breathing LED setup");
	pinMode(LED_BUILTIN, OUTPUT);

	DEBUG_PRINTLN("   ");
	DEBUG_PRINTLN("Initialising");

	DEBUG_PRINT("Sketch size: ");
	DEBUG_PRINTLN(ESP.getSketchSize());
	DEBUG_PRINT("Free size: ");
	DEBUG_PRINTLN(ESP.getFreeSketchSpace());

	//loadCustomParamsSPIFFS();  //load the initial settings from EEPROM SPIFFS


	setupSPI();

	setupAS3935();

	_clear_Previous_Values_Interval_Millisecs = _currentConfigValues.ResetInterval;

	WiFiConnectionCheck();

	MDNS.begin(MQTT_CLIENTNAME);

	setupTimeClient();
	getTimeUpdate();

	_weatherDate = String(year(_unixTime)) + "/" + String(month(_unixTime)) + "/" + String(day(_unixTime));
	DEBUG_PRINTLN(_weatherDate);

	setupWebServer();

	randomSeed(micros());

	DEBUG_PRINT("Attempting MQTT connection to: ");
	DEBUG_PRINT(MQTT_SERVERADDRESS);
	DEBUG_PRINTLN(":1883");

	_mqttClient.setServer(MQTT_SERVERADDRESS, 1883);
	_mqttClient.setCallback(mqttCallback);

	//force first connection:
	mqttReconnect();
	transmitStat();
	mqttTransmitInitStat();

	DEBUG_PRINT(F("********** Free Heap: ")); 	DEBUG_PRINTLN(ESP.getFreeHeap());
	DEBUG_PRINTLN(F("ESP WiFi -> OTA Setup"));
	setupOTA();
	MDNS.addService("http", "tcp", 80);


}

void loop()
{
	if (!_mqttClient.connected()) { mqttReconnect(); }
	_mqttClient.loop();

	/* this function will handle incomming chunk of SW, flash and respond sender */
	ArduinoOTA.handle();
	//// Check if a client has connected
	_httpServer.handleClient();

	if (_runCurrent - _runTickLED >= WIFI_TICKLED_INTERVAL_MILLISECS)
	{
		_runTickLED = _runCurrent;
		tickLED();
		WiFiConnectionCheck();
	}

	if (!_updatingFirmware)
	{
		// re-runs program according to the value set for timebetweenruns
		_runCurrent = millis(); //sets the counter
		if (_runCurrent - _runClearPreviousValues >= _clear_Previous_Values_Interval_Millisecs)
		{
			_runClearPreviousValues = _runCurrent;
			callback_clearPrevious();
		}
		if (_runCurrent - _runClearAverageValues >= CLEAR_AVERAGE_INTERVAL_MILLISECS)
		{
			_runClearAverageValues = _runCurrent;
			callback_clearAvg();
		}


		checkIRQforInterrupt();
	}
}
