
#include "Adafruit_SSD1306.h"
#include "MQTTClientMbedOs.h"
#include "OPT3001.h"
#include "mb_pins.h"
#include "mbed.h"
#include "rtos.h"
#include "string.h"
#include <cstdio>

// OLED Sizable text enabled
#define GFX_SIZEABLE_TEXT
// OLED message duration:
#define OLED_MSG_DURATION 250
// Server keep alive interval
#define SERVER_KA_INTERVAL 30
// Set OLED width and heigth [pixel]:
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64
// Scaler to 3v3L
#define VOLTAGE_SCALER 3.3f
// Client yield timeout in miliseconds:
#define YIELD_TIMEOUT_MS 1000
// Maximum number of networks to scan for:
#define MAX_NETWORKS 15
// Small delay for network information printing:
#define PRINTF_DELAY_MS 10
// Address of OLED display:
#define I2C_ADDRESS 0x3c << 1
// Half of the potentiometer return value:
#define HALF_INTERVAL 0.5f
// I2C frequency:
#define I2C_FREQUENCY 400000
// Sub msg debug print:
#define DEBUG 0

// Global Variables
// Timeout for server
Timeout server;
// Ambient click
OPT3001 ambient(MB_MIKRO_SDA, MB_MIKRO_SCL);
// I2C OLED
I2C i2c(MB_OLED_SDA, MB_OLED_SCL);
// OLED
Adafruit_SSD1306_I2c myOled(i2c, PB_5, I2C_ADDRESS, OLED_HEIGHT_PX,
                            OLED_WIDTH_PX);
// Potentiometers:
AnalogIn pot1(MB_POT1);
AnalogIn pot2(MB_POT2);
// Left button on the motherboard:
InterruptIn sw1(MB_SW1);
// LEFT LED on the motherboard:
DigitalOut led1(MB_LED1);
// Right LED on the motherboard:
DigitalOut led2(MB_LED2);
// Pointer to a WiFi network object:
WiFiInterface *wifi;
// Creating TCP socket: - not in use
TCPSocket socket;
// Creating TLS socket:
TLSSocket tlsSocket;
// Creating MQTT client using the TCP socket;
MQTTClient client(&socket);
// Message handler:
MQTT::Message message;
// Display thread
Thread display_control;
// Sensors to be displayed
// // temperature, humidity, air_pressure, soil_moisture, light
double sensors[5] = {0, 0, 0, 0, 0};
const char *labels[2][5] = {{"Temp:", "VlazV:", "Prit:", "VlazZ:", "Osvet:"},
                            {"oC", "%", "bar", "%", "lx"}};
// Counter of arrived messages:
long long arrivedcount = 0; // !Maybe totally useles with keep alive! - TBD
// HiveMQ broker connectivity information:
const char *hostname = "broker.hivemq.com";
int port = 1883;
// System states indikator
int idle_state = 0;
int server_alive = 1;
// Topics
// subs
char *topic_sub_info = "PMK22/uC4/control";
char *topic_sub_telemetry = "PMK22/telemetry";
char *topic_sub_keep_alive_server = "PMK22/serverAlive";
char *topic_idle = "PMK22/idle";
// publishes
char *topic_keep_alive_node = "PMK22/nodeAlive";
char *topic_sensor = "PMK22/uC4/sensor";
// test topics
char *topic_sscanf = "PMK22/test/sscanf";

// Global Functions
// wifi functions
const char *sec2str(nsapi_security_t sec);
int scan_networks(WiFiInterface *wifi);
int connect();
// subscription and callbacks
void keep_alive_callback(MQTT::MessageData &md);
void telemetry_callback(MQTT::MessageData &md);
void info_callback(MQTT::MessageData &md);
void idle_callback(MQTT::MessageData &md);
void subscribe_to_topics();
// publish functions
void publish_ambient(int ambient);
void say_hi();
// Thread for display control
void display_function();
// interupts
void server_down();

int main() {
  // local variables
  int err_conn;
  int err_ambient;
  int als_value;

  // Initialize OLED:
  myOled.begin();
  display_control.start(display_function);

  // init wifi interface, open TCP port, connect to broker
  if ((err_conn = connect()) != 0) {
    printf("Error making connection to MQTT server: %d\n", err_conn);
    return -1;
  }

  // Subscribe to topics
  subscribe_to_topics();

  // set Timeout
  server.attach(server_down, SERVER_KA_INTERVAL);

  // Init ambient click
  if ((err_ambient = ambient.initialize()) != 0) {
    printf("Error communicating with Ambient sensor: %d\n", err_ambient);
    return -1;
  }

  // main loop
  while (true) {
    // Need to call yield API to maintain connection:
    client.yield(YIELD_TIMEOUT_MS);

    // Read ALS and publish
    if (!idle_state && server_alive) {

      als_value = ambient.readSensor();
      if (als_value < 0) {
        printf("Error reading ALS value\n");
      } else {
        publish_ambient(als_value);
      }
    }

    // State that I am alive
    say_hi();

    // delay
    wait_ms(500); // TBD
  }
}

const char *sec2str(nsapi_security_t sec) {
  switch (sec) {
  case NSAPI_SECURITY_NONE:
    return "None";
  case NSAPI_SECURITY_WEP:
    return "WEP";
  case NSAPI_SECURITY_WPA:
    return "WPA";
  case NSAPI_SECURITY_WPA2:
    return "WPA2";
  case NSAPI_SECURITY_WPA_WPA2:
    return "WPA/WPA2";
  case NSAPI_SECURITY_UNKNOWN:
  default:
    return "Unknown";
  }
}

int scan_networks(WiFiInterface *wifi) {
  printf("Scan:\n");

  // Scan only for the number of networks, first parameter is NULL:
  int count = wifi->scan(NULL, 0);
  // If there are no networks, count == 0, if there is an error, counter < 0:
  if (count <= 0) {
    printf("scan() failed with return value: %d\n", count);
    return 0;
  }

  // Limit number of network arbitrary to some reasonable number:
  count = count < MAX_NETWORKS ? count : MAX_NETWORKS;

  // Create a local pointer to an object, which is an array of WiFi APs:
  WiFiAccessPoint *ap = new WiFiAccessPoint[count];
  // Now scan again for 'count' networks and populate the array of APs:
  count = wifi->scan(ap, count);

  // This time, the number of entries to 'ap' is returned:
  if (count <= 0) {
    printf("scan() failed with return value: %d\n", count);
    return 0;
  }

  // Print out the parameters of each AP:
  for (int i = 0; i < count; i++) {
    printf("Network: %s secured: %s BSSID: %hhX:%hhX:%hhX:%hhx:%hhx:%hhx RSSI: "
           "%hhd Ch: %hhd\n",
           ap[i].get_ssid(), sec2str(ap[i].get_security()),
           ap[i].get_bssid()[0], ap[i].get_bssid()[1], ap[i].get_bssid()[2],
           ap[i].get_bssid()[3], ap[i].get_bssid()[4], ap[i].get_bssid()[5],
           ap[i].get_rssi(), ap[i].get_channel());
    thread_sleep_for(PRINTF_DELAY_MS);
  }
  printf("%d networks available.\n", count);

  // Since 'ap' is dynamically allocated pointer to the array of objects, it
  // needs to be deleted:
  delete[] ap;
  return count;
}

// Sub callbacks definitions

void keep_alive_callback(
    MQTT::MessageData &md) {
  MQTT::Message &message = md.message;
  ++arrivedcount;
  server.detach();
  server_alive = 1;
  server.attach(server_down, SERVER_KA_INTERVAL);
}

void info_callback(          // TBD
    MQTT::MessageData &md) { 
  MQTT::Message &message = md.message;
  if (DEBUG)
    printf("Message from the server: %.*s\r\n", message.payloadlen,
           (char *)message.payload);
  ++arrivedcount;
}

void telemetry_callback( 
    MQTT::MessageData &md) { 
  MQTT::Message &message = md.message;
  if (DEBUG)
    printf("Message from the server: %.*s\r\n", message.payloadlen,
           (char *)message.payload);

  ++arrivedcount;

  sscanf((char *)message.payload, "%lf,%lf,%lf,%lf,%lf", &sensors[0],
         &sensors[1], &sensors[2], &sensors[3], &sensors[4]);

  if (DEBUG) {
    printf("temperature=%lf\r\n", sensors[0]);
    printf("humidity=%lf\r\n", sensors[1]);
    printf("air_pressure=%lf\r\n", sensors[2]);
    printf("soil_moisture=%lf\r\n", sensors[3]);
    printf("light=%lf\r\n", sensors[4]);
  }
}

void idle_callback(
    MQTT::MessageData &md) {
  MQTT::Message &message = md.message;
  if (DEBUG)
    printf("Message from the server: %.*s\r\n", message.payloadlen,
           (char *)message.payload);

  ++arrivedcount;

  sscanf((char *)message.payload, "%d", &idle_state);

  if (DEBUG) {
    if (idle_state) {
      printf("Idle state is on! \r\n");
    } else {
      printf("Idle state is off! \r\n");
    }
  }
}

void say_hi() {
  char buf[100];
  sprintf(buf, "4\r\n");
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void *)buf;
  message.payloadlen = strlen(buf) + 1;
  client.publish(topic_keep_alive_node, message);
}

void publish_ambient(int ambient) {
  char buf[100];
  sprintf(buf, "%d", ambient);
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void *)buf;
  message.payloadlen = strlen(buf) + 1;
  client.publish(topic_sensor, message);
}

void subscribe_to_topics() {
  int rc;

  if ((rc = client.subscribe(topic_sub_keep_alive_server, MQTT::QOS0,
                             keep_alive_callback)) != 0)
    printf("rc from MQTT subscribe is %d\n", rc);

  if ((rc = client.subscribe(topic_sub_info, MQTT::QOS0, info_callback)) != 0)
    printf("rc from MQTT subscribe is %d\n", rc);

  if ((rc = client.subscribe(topic_sub_telemetry, MQTT::QOS0,
                             telemetry_callback)) != 0)
    printf("rc from MQTT subscribe is %d\n", rc);

  if ((rc = client.subscribe(topic_idle, MQTT::QOS0, idle_callback)) != 0)
    printf("rc from MQTT subscribe is %d\n", rc);
}

int connect() { // init wifi interface, open port, connect to MQTT
  // Create a default network interface:
  wifi = WiFiInterface::get_default_instance();
  if (!wifi) {
    printf("ERROR: No WiFiInterface found.\n");
    return -1;
  }

  // Scan for available networks and aquire information about Access Points:
  int count = scan_networks(wifi);
  if (count == 0) {
    printf("No WIFI APs found - can't continue further.\n");
    return -1;
  }

  // Connect to the network with the parameters specified in 'mbed_app.json':
  printf("\nConnecting to %s...\n", MBED_CONF_APP_WIFI_SSID);
  int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD,
                          NSAPI_SECURITY_WPA_WPA2);
  if (ret != 0) {
    printf("\nConnection error: %d\n", ret);
    return -1;
  }

  // Print out the information aquired:
  printf("Success\n\n");
  printf("MAC: %s\n", wifi->get_mac_address());
  printf("IP: %s\n", wifi->get_ip_address());
  printf("Netmask: %s\n", wifi->get_netmask());
  printf("Gateway: %s\n", wifi->get_gateway());
  printf("RSSI: %d\n\n", wifi->get_rssi());

  // Open TCP socket using WiFi network interface:
  socket.open(wifi);
  // Connect to the HiveMQ broker:
  socket.connect(hostname, port);

  // Fill connect data with default values:
  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  // Change only ID and protocol version:
  data.MQTTVersion = 3;
  data.clientID.cstring = "NUCLEO-L476RG-101";

  // Connect the MQTT
  int rc = 0;
  if ((rc = client.connect(data)) != 0) {
    printf("rc from MQTT connect is %d\n", rc);
    return -1;
  }
  return 0;
}

void display_function() {
  Thread::wait(1000);
  myOled.clearDisplay();
  myOled.setTextSize(2);
  myOled.clearDisplay();
  while (true) {
    if (!server_alive) {
      myOled.setTextSize(3);
      myOled.setTextCursor(0, 0);
      myOled.printf("SERVER\nDOWN!");
      myOled.display();
      thread_sleep_for(OLED_MSG_DURATION);
      myOled.clearDisplay();
    }

    // temperature, humidity, air_pressure, soil_moisture, light
    if (!idle_state && server_alive) {
      myOled.setTextSize(3);
      for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 20; j++) {
          myOled.setTextCursor(0, 0);
          myOled.printf("%s\n%.2f%s \n", labels[0][i], sensors[i],
                        labels[1][i]);
          myOled.display();
          thread_sleep_for(OLED_MSG_DURATION);
          myOled.clearDisplay();
          if (!server_alive || idle_state)
            break;
        }
        if (!server_alive || idle_state)
          break;
      }
    }
    if (server_alive && idle_state) {
      myOled.clearDisplay();
      myOled.display();
      thread_sleep_for(OLED_MSG_DURATION);
    }
  }
}

void server_down() { server_alive = 0; }