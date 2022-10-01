/*
  WiFi Web Server

 A simple web server that shows the value of the analog input pins.
 using a WiFi shield.

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the WiFi.begin() call accordingly.

 Circuit:
 * WiFi shield attached
 * Analog inputs attached to pins A0 through A5 (optional)

 created 13 July 2010
 by dlf (Metodo2 srl)
 modified 31 May 2012
 by Tom Igoe

 */

#include <SPI.h>
#include <WiFi.h>


char ssid[] = "Fibertel WiFi996 2.4GHz";      // your network SSID (name)
char pass[] = "00413322447";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)


uint16_t atu=15;
char hi[128];
char HTTPRequest[128];
uint16_t p=0;

int status = WL_IDLE_STATUS;

WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
}

void httpString(char **dest, char *input, const char *find) {
char *start;
char *o_input = input;
const char *o_find = find;
size_t length = 0;
size_t i = 0;
while (*input) {
    if (*input == '&' || input == o_input) {
        if (*input == '&') {
            input++;
            if (*input == 0) {
                return;
            }
        }
        while (*input == *find) {
            if (*input == 0 || *find == 0) {
                return;
            }
            input++;
            find++;
            if (*input == '=' && *find == 0) {
                input++;
                if (*input == 0) {
                    return;
                }
                start = input;
                while (*input != '&' && *input) {
                    input++;
                    length++;
                }
                *dest = (char*)malloc(length + 1);
                input = start;
                while (*input != '&' && *input) {
                    (*dest)[i] = *input;
                    input++;
                    i++;
                }
                (*dest)[i] = 0;
                return;
            }
        }
    }
    find = o_find;
    input++;
}
}

void loop() {
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          //client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          /*
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
            client.print("analog input ");
            client.print(analogChannel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");
          }
          */
          client.print("<form><div>");        
          sprintf(hi,"<label for=\"atu_field\">ATU </label><input type=\"text\" id=\"name\" name=\"atu\" required minlength=\"4\" maxlength=\"8\" size=\"10\" value=\"%d\">",atu);
          client.print(hi);
          //sprintf(hi,"<label for=\"HTTPRequest\">%s</label>",HTTPRequest);
          //client.print(hi);
          p=0;
/*
          client.print("<label for=\"example\">Let\'s submit some text</label>");
          client.print("<input id=\"example\" type=\"text\" name=\"text\" />");
*/          
          client.print("</div><div><input type=\"submit\" value=\"Send\"/>");
          client.print("</div></form>");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
          //HTTPRequest[p++]=c;
          //HTTPRequest[p]=0x00;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
