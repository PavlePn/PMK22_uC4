
#include "Adafruit_SSD1306.h"
#include "MQTTClientMbedOs.h"
#include "OPT3001.h"
#include "mb_pins.h"
#include "mbed.h"
#include "string.h"

// OLED Sizable text enabled
#define GFX_SIZEABLE_TEXT

// LED2 blinking rate:
#define BLINKING_RATE_MS 250
// Scaler to 3v3L
#define VOLTAGE_SCALER 3.3f
// Client yield timeout in miliseconds:
#define YIELD_TIMEOUT_MS 1000
// Maximum number of networks to scan for:
#define MAX_NETWORKS 15
// Small delay for network information printing:
#define PRINTF_DELAY_MS 10
// Set OLED width and heigth [pixel]:
#define OLED_WIDTH_PX 128
#define OLED_HEIGHT_PX 64
// Address of OLED display:
#define I2C_ADDRESS 0x3c << 1
// Half of the potentiometer return value:
#define HALF_INTERVAL 0.5f
// I2C frequency:
#define I2C_FREQUENCY 400000

// Global Variables
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

// Topics
// subs
char *topic_sub_info = "PMK22/uC4/info";
char *topic_sub_telemetry = "PMK22/uC4/telemetry";
char *topic_sub_keep_alive_server = "PMK22/serverAlive";
// publishes
char *topic_keep_alive_node = "PMK22/nodeAlive";
char *topic_sensor = "PMK22/uC4/ALS";

// Counter of arrived messages:
int arrivedcount = 0;

// Flag indicating that button is not pressed:
// int button_pressed = 0;

// HiveMQ broker connectivity information:
const char *hostname = "broker.hivemq.com";
int port = 1883;

// Global Functions
const char *sec2str(nsapi_security_t sec);
int scan_networks(WiFiInterface *wifi);
void messageArrived(MQTT::MessageData &md);
// void buttonFunction();
void sayHi();

int main() {
  // Turn off left led
  // led1 = 0;

  // Set the interrupt event:
  // sw1.fall(&buttonFunction);

  // Initialize OLED:
  myOled.begin();
  myOled.clearDisplay();

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
  if ((rc = client.connect(data)) != 0)
    printf("rc from MQTT connect is %d\n", rc);

  if ((rc = client.subscribe(topic_sub, MQTT::QOS0, messageArrived)) != 0)
    printf("rc from MQTT subscribe is %d\n", rc);

  // Init ambient click
  int err_ambient;
  while (1) {
    if ((err_ambient = ambient.initialize()) != 0) {
      printf("Error communicating with MAX44000: %d\n", err_ambient);
    } else {
      printf("Initialized CPT3001\n");
      break;
    }
    wait_ms(100);
  }

  while (true) {
    // Show that the loop is running by switching motherboard LED2:
    // led2 = !led2;

    // if (button_pressed == 1) { //Test send on button click
    //   button_pressed = 0;
    //   // QoS 0
    //   char buf[100];
    //   sprintf(buf, "V(POT1) = %1.2f\r\n", pot1 * VOLTAGE_SCALER);
    //   message.qos = MQTT::QOS0;
    //   message.retained = false;
    //   message.dup = false;
    //   message.payload = (void *)buf;
    //   message.payloadlen = strlen(buf) + 1;
    //   client.publish(topic, message);
    // }

    // Need to call yield API to maintain connection:
    client.yield(YIELD_TIMEOUT_MS);

    // text display tests - Constant display to be changed
    myOled.setTextSize(2);
    myOled.setTextColor(WHITE);
    myOled.setTextCursor(0, 0);
    myOled.printf("Hello,    World!");
    myOled.display();
    thread_sleep_for(BLINKING_RATE_MS);
    myOled.clearDisplay();

    // Read ALS
    int alsValue = ambient.readSensor();
    if (alsValue < 0) {
      printf("Error reading ALS value\n");
    } else {
      // printf("ALS = 0x%04X\n", alsValue);
    }

    // State that I am alive
    sayHi();

    //delay
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

void messageArrived(MQTT::MessageData &md) { //Change name, make functions for all subs
  MQTT::Message &message = md.message;
  // printf("Message arrived: qos %d, retained %d, dup %d, packetid %d\r\n",
  // message.qos, message.retained, message.dup, message.id);
  printf("Message from the browser: %.*s\r\n", message.payloadlen,
         (char *)message.payload);
  ++arrivedcount;
}

// void buttonFunction() { button_pressed = 1; }

void sayHi() {
  char buf[100];
  sprintf(buf, "4\r\n");
  message.qos = MQTT::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void *)buf;
  message.payloadlen = strlen(buf) + 1;
  client.publish(topic_keep_alive_node, message);
}