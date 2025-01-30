#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5CoreS3.h>
#include <map>



#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 2048
#define SERIAL_BAUD_RATE 115200
#define UDP_PORT 2222
#define NMEA_UDP_PORT 2002
#define DISPLAY_UPDATE_INTERVAL 1000  // Update display every 0.5 seconds
#define DEBUG_ENABLED false // Toggle debug output
#define ERROR_ENABLED true


// WiFi credentials
const char *ssid = "Sakura";
const char *password = "rosebuds";

// UDP server
WiFiUDP udp;
WiFiUDP nmea_udp;
char incomingPacket[MAX_BUFFER_SIZE];


// Global buffer
static uint8_t buffer[MAX_BUFFER_SIZE];
static size_t buffer_index = 0;
static size_t expected_size = 0;


// Global counters
volatile uint32_t input_channel_count = 0;
volatile uint32_t output_udp_count = 0;

// Last update timestamp
unsigned long last_display_update = 0;

QueueHandle_t udpQueue = xQueueCreate(10, sizeof(char[64]));  // Queue for NMEA messages


// Mutex for thread-safe register access
SemaphoreHandle_t register_mutex;


// Format size lookup table
size_t get_format_size(uint8_t format) {
    switch (format) {
        case 0x00: return 4;
        case 0x01: return 2;
        case 0x02: return 2;
        case 0x03: return 2;
        case 0x04: return 4;
        case 0x05: return 4;
        case 0x06: return 4;
        case 0x07: return 4;
        case 0x08: return 2;
        case 0x0A: return 4;
        default:   return 0; // Unknown format
    }
}

struct ChannelData {
    double last_value;
    unsigned long timestamp;  // Time of last update (milliseconds)
};

std::map<uint8_t, ChannelData> channel_register;




// Channel lookup table
const char *get_channel_name(uint8_t channel) {
    switch (channel) {
        case 0x00: return "Node Reset";
        case 0x0B: return "Rudder Angle";
        case 0x1C: return "Air Temperature (°F)";
        case 0x1D: return "Air Temperature (°C)";
        case 0x1E: return "Sea Temperature (°F)";
        case 0x1F: return "Sea Temperature (°C)";
        case 0x27: return "Head/Lift Trend";
        case 0x29: return "Off Course";
        case 0x32: return "Tacking Performance";
        case 0x33: return "Reaching Performance";
        case 0x34: return "Heel Angle";
        case 0x35: return "Optimum Wind Angle";
        case 0x36: return "Depth Sounder Receiver Gain";
        case 0x37: return "Depth Sounder Noise";
        case 0x3B: return "Linear 4";
        case 0x3C: return "Rate Motion";
        case 0x41: return "Boatspeed (Knots)";
        case 0x42: return "Boatspeed (Raw)";
        case 0x46: return "Autopilot Speed Fixed (Knots)";
        case 0x47: return "LatLon";
        case 0x49: return "Heading";
        case 0x4A: return "Heading (Raw)";
        case 0x4D: return "Apparent Wind Speed (Knots)";
        case 0x4E: return "Measured Wind Speed (Raw)";
        case 0x4F: return "Apparent Wind Speed (m/s)";
        case 0x51: return "Apparent Wind Angle";
        case 0x52: return "Measured Wind Angle (Raw)";
        case 0x53: return "Autopilot Target TWA";
        case 0x55: return "True Wind Speed (Knots)";
        case 0x56: return "True Wind Speed (m/s)";
        case 0x57: return "Measured Wind Speed (Knots)";
        case 0x59: return "True Wind Angle";
        case 0x5A: return "Measured Wind Angle Deg";
        case 0x64: return "Average Speed (Knots)";
        case 0x65: return "Average Speed (Raw)";
        case 0x69: return "Course";
        case 0x6D: return "True Wind Direction";
        case 0x6F: return "Next Leg Apparent Wind Angle";
        case 0x70: return "Next Leg Target Boat Speed";
        case 0x71: return "Next Leg Apparent Wind Speed";
        case 0x75: return "Timer";
        case 0x7D: return "Target Boatspeed";
        case 0x7F: return "Velocity Made Good (Knots)";
        case 0x81: return "Dead Reckoning Distance";
        case 0x82: return "Leeway";
        case 0x83: return "Tidal Drift";
        case 0x84: return "Tidal Set";
        case 0x85: return "Upwash";
        case 0x86: return "Barometric Pressure Trend";
        case 0x87: return "Barometric Pressure";
        case 0x8D: return "Battery Volts";
        case 0x9A: return "Heading on Next Tack";
        case 0x9B: return "Fore/Aft Trim";
        case 0x9C: return "Mast Angle";
        case 0x9D: return "Wind Angle to the Mast";
        case 0x9E: return "Pitch Rate (Motion)";
        case 0xA6: return "Autopilot Compass Target";
        case 0xAF: return "Autopilot Off Course";
        case 0xC1: return "Depth (Meters)";
        case 0xC2: return "Depth (Feet)";
        case 0xC3: return "Depth (Fathoms)";
        case 0xCD: return "Stored Log (NM)";
        case 0xCF: return "Trip Log (NM)";
        case 0xDC: return "Local Time";
        case 0xD3: return "Dead Reckoning Course";
        case 0xE0: return "Bearing Wpt. to Wpt. (True)";
        case 0xE1: return "Bearing Wpt. to Wpt. (Mag)";
        case 0xE2: return "Layline Distance";
        case 0xE3: return "Bearing to Waypoint (Rhumb True)";
        case 0xE4: return "Bearing to Waypoint (Rhumb Mag)";
        case 0xE5: return "Bearing to Waypoint (G.C. True)";
        case 0xE6: return "Bearing to Waypoint (G.C. Mag)";
        case 0xE7: return "Distance to Waypoint (Rhumb)";
        case 0xE8: return "Distance to Waypoint (G.C.)";
        case 0xE9: return "Course Over Ground (True)";
        case 0xEA: return "Course Over Ground (Mag)";
        case 0xEB: return "Speed Over Ground";
        case 0xEC: return "VMG to Waypoint (VMC)";
        case 0xED: return "Time to Waypoint";
        case 0xEE: return "Cross Track Error";
        case 0xEF: return "Remote 0";
        case 0xF0: return "Remote 1";
        case 0xF1: return "Remote 2";
        case 0xF2: return "Remote 3";
        case 0xF3: return "Remote 4";
        case 0xF4: return "Remote 5";
        case 0xF5: return "Remote 6";
        case 0xF6: return "Remote 7";
        case 0xF7: return "Remote 8";
        case 0xF8: return "Remote 9";
        case 0xFA: return "Next Waypoint Distance";
        case 0xFB: return "Time to Layline";
        default: return "Unknown";
    }
}

// FreeRTOS task functions
void task_udp_listener(void *pvParameters);
void update_display(void *pvParameters);


// Debug output function
void debug_output(const char *message) {
    if (DEBUG_ENABLED) {
        Serial.println(message);
    }
}



// Debug output function
void error_output(const char *message) {
    if (ERROR_ENABLED) {
        Serial.println(message);
    }
}

void udp_sender(const char *nmea_sentence) {
    if (xQueueSend(udpQueue, nmea_sentence, pdMS_TO_TICKS(10)) != pdPASS) {
        Serial.println("⚠️ UDP Queue Full! Dropping message.");
    }
}


void udp_task(void *pvParameters) {
    char message[64];  // Buffer to hold messages from queue
    while (true) {
        if (xQueueReceive(udpQueue, &message, portMAX_DELAY)) {
            // Send the UDP message
            nmea_udp.beginPacket("255.255.255.255", NMEA_UDP_PORT);
            nmea_udp.write((uint8_t*)message, strlen(message));
            nmea_udp.endPacket();
            output_udp_count++;
        }
    }
}
// LCD display function
void lcd_output(const char *message) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println(message);
}

// Calculate NMEA checksum (XOR of all characters between '$' and '*')
uint8_t calculate_nmea_checksum(const char *sentence) {
    uint8_t checksum = 0;
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}



double get_last_value(uint8_t channel) {
    if (channel_register.find(channel) != channel_register.end()) {
        return channel_register[channel].last_value;
    }
    return 0.0; // Default if channel not found
}


unsigned long get_last_update_time(uint8_t channel) {
    if (channel_register.find(channel) != channel_register.end()) {
        return channel_register[channel].timestamp;
    }
    return 0; // Default if channel not found
}


// Decoding function
double decode(uint8_t format, uint32_t data) {
    uint8_t divisorBits = (format >> 6) & 0x03;
    uint16_t divisor = (divisorBits == 0) ? 1 : (divisorBits == 1) ? 10 :
                       (divisorBits == 2) ? 100 : 1000;
    
    uint8_t format_id = format & 0x0F;
    double decoded_value = 0.0;

    switch (format_id) {
        case 0x00:  // Raw 32-bit unsigned integer
            decoded_value = (double)data / divisor;
            break;
        case 0x01:  // Signed 16-bit integer
            decoded_value = (double)(int16_t)(data & 0xFFFF) / divisor;
            break;
        case 0x02:  // Unsigned 10-bit integer
            decoded_value = (double)(data & 0x03FF) / divisor;
            break;
        case 0x03:  // Special format with segment code
        {
            uint8_t segment_code = (data >> 8) & 0xFF;
            uint8_t value = data & 0xFF;
            decoded_value = (double)value / divisor;
            if (segment_code == 0x28 || segment_code == 0xA0 || segment_code == 0x8C) {
                decoded_value = -decoded_value;
            }
            break;
        }
        case 0x04:  // 24-bit signed integer
            decoded_value = (double)(int32_t)(data & 0xFFFFFF) / divisor;
            break;
        case 0x06:
            decoded_value = 0;
            break;
        case 0x07:
            decoded_value = (double)(uint16_t)(data & 0x7FFF)/ divisor;
            break;
        case 0x08: //last 9 bits of the 2 bytes 
            decoded_value = (double)(int16_t)(data & 0x01FF) / divisor;
            break;
        case 0x0A:  // 16-bit signed integer (only upper 16 bits)
            decoded_value = (double)(int16_t)((data >> 16) & 0xFFFF) / divisor;
            break;
        default:
            decoded_value = 0.0;
            break;
    }
    return decoded_value;
}

// Function prototypes
//void debug_output(const char *message);
//uint8_t calculate_checksum(uint8_t *data, size_t length);
//void process_frame(uint8_t *body, size_t body_size);
//void process_stream();
//void read_serial_data();



// Checksum calculation
uint8_t calculate_checksum(uint8_t *data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (0x100 - (sum % 0x100)) & 0xFF;
}




// WiFi connection function
void connect_to_wifi() {
    WiFi.begin(ssid, password);
    debug_output("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        debug_output(".");
    }
    debug_output("WiFi connected!");
    char ip_msg[64];
    snprintf(ip_msg, sizeof(ip_msg), "IP Address: %s", WiFi.localIP().toString().c_str());
    debug_output(ip_msg);

    udp.begin(UDP_PORT);

    WiFi.setSleep(false);
    debug_output("Listening on UDP port 2222...");
}


// FreeRTOS Task: Listen for UDP packets
void task_udp_listener(void *pvParameters) {
    for (;;) {
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            if (buffer_index + packetSize > MAX_BUFFER_SIZE) {
                error_output("BUFFER FULL! Dropping data");
                buffer_index = 0;  // Reset buffer to prevent overflow
                continue;
            }

            int len = udp.read(incomingPacket, packetSize);
            if (len > 0) {
                memcpy(buffer + buffer_index, incomingPacket, len);
                buffer_index += len;
                process_stream();  // Process immediately
            }
        }
        vTaskDelay(1);
    }
}
// FreeRTOS Task: Update M5Stack LCD display
void update_display(void *pvParameters) {
    M5.Lcd.setTextSize(2);
    static uint32_t prev_input = 0, prev_output = 0;
    static String last_ip = "";
    static float last_input_rate = -1, last_output_rate = -1;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();  // Get current tick time

    for (;;) {  // FreeRTOS recommended loop structure
        unsigned long current_time = millis();
        float elapsed_sec = (current_time - last_display_update) / 1000.0;
        if (elapsed_sec < 0.001) elapsed_sec = 1;  // Prevent divide-by-zero

        // Compute rates
        float input_rate = (input_channel_count - prev_input) / elapsed_sec;
        float output_rate = (output_udp_count - prev_output) / elapsed_sec;

        // Update previous values
        prev_input = input_channel_count;
        prev_output = output_udp_count;

        // Get current IP Address
        String ipAddress = WiFi.localIP().toString();

        // **Erase previous text before writing new values**
        M5.Lcd.setTextColor(BLACK);  // Set color to black to "erase" old text

        // Overwrite previous IP if changed
        if (last_ip != ipAddress) {
            M5.Lcd.setCursor(10, 10);
            M5.Lcd.printf("IP: %s", last_ip.c_str());  
            last_ip = ipAddress;
        }

        // Overwrite previous Input Rate if changed
        if (last_input_rate != input_rate) {
            M5.Lcd.setCursor(10, 40);
            M5.Lcd.printf("Input:  %4.0f channels/s", last_input_rate);
            last_input_rate = input_rate;
        }

        // Overwrite previous Output Rate if changed
        if (last_output_rate != output_rate) {
            M5.Lcd.setCursor(10, 70);
            M5.Lcd.printf("Output: %4.0f NMEA/s", last_output_rate);
            last_output_rate = output_rate;
        }

        // **Draw new values in white**
        M5.Lcd.setTextColor(WHITE);

        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("IP: %s", ipAddress.c_str());

        M5.Lcd.setCursor(10, 40);
        M5.Lcd.printf("Input:  %4.0f channels/s", input_rate);

        M5.Lcd.setCursor(10, 70);
        M5.Lcd.printf("Output: %4.0f NMEA/s", output_rate);
        
        last_display_update = current_time;

        // Wait until next execution time (ensures stable intervals)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL));
    }
}


// Stream processing function
void process_stream() {
    while (buffer_index >= HEADER_SIZE) {
        uint8_t bodysize = buffer[2]; // Extract body size
        expected_size = HEADER_SIZE + bodysize + 1; // Compute expected size

        // ** Check if header checksum is valid**
        if (calculate_checksum(buffer, HEADER_SIZE - 1) != buffer[4]) {
            debug_output("Invalid header checksum! Dropping first byte for resync.");

            // **Drop only one byte and shift buffer left**
            memmove(buffer, buffer + 1, --buffer_index);
            expected_size = 0; // Reset expected size
            continue;  // Retry with the shifted buffer
        }

        // ** Check if we have enough data to process**
        if (buffer_index < expected_size) return; // Wait for more data

        // ** Validate body checksum**
        uint8_t body_checksum = buffer[HEADER_SIZE + bodysize];
        if (calculate_checksum(&buffer[HEADER_SIZE], bodysize) != body_checksum) {
            error_output("Invalid body checksum! Dropping only this frame.");

            // Move remaining data forward (drop bodysize + 1 bytes)
            memmove(buffer, buffer + HEADER_SIZE + bodysize + 1, buffer_index - (HEADER_SIZE + bodysize + 1));

            // Adjust buffer index
            buffer_index -= (HEADER_SIZE + bodysize + 1);

            // Continue processing remaining data
            continue;
        }
        
        uint8_t command = buffer[3];
        if (command == 0x01) {
            process_frame(&buffer[HEADER_SIZE], bodysize);
        } 
        else if (command == 0x03)  {
            process_frame_ascii(&buffer[HEADER_SIZE], bodysize);
        }
        else {
            char error_msg[50];
            snprintf(error_msg, sizeof(error_msg), "Unknown command type: 0x%02X", command);
            error_output(error_msg);
        }

        // ** Remove processed frame from buffer**
        memmove(buffer, buffer + expected_size, buffer_index - expected_size);
        buffer_index -= expected_size;
        expected_size = 0;
    }
}

void process_frame_ascii(uint8_t *body, size_t body_size) {
    // Ensure the string is null-terminated
    char ascii_str[body_size + 1];
    memcpy(ascii_str, body, body_size);
    ascii_str[body_size] = '\0';

    // Debug output
    debug_output("---- GPS Extraction Debug ----");
    debug_output(ascii_str);

    // Find latitude cardinal ('N' or 'S')
    char *lat_cardinal_ptr = strpbrk(ascii_str, "NS");
    if (lat_cardinal_ptr == NULL) {
        debug_output("Latitude cardinal (N/S) not found. Skipping.");
        return;
    }
    
    char lat_cardinal = *lat_cardinal_ptr;
    size_t lat_length = lat_cardinal_ptr - (ascii_str + 2);  // Compute latitude length dynamically

    // Extract latitude
    char lat_str[12] = {0};
    strncpy(lat_str, ascii_str + 2, lat_length);

    // Find longitude cardinal ('E' or 'W')
    char *lon_cardinal_ptr = strpbrk(lat_cardinal_ptr + 1, "EW");
    if (lon_cardinal_ptr == NULL) {
        debug_output("Longitude cardinal (E/W) not found. Skipping.");
        return;
    }

    char lon_cardinal = *lon_cardinal_ptr;
    size_t lon_length = lon_cardinal_ptr - (lat_cardinal_ptr + 1);

    // Extract longitude
    char lon_str[12] = {0};
    strncpy(lon_str, lat_cardinal_ptr + 1, lon_length);

    // Debug extracted parts
    char debug_lat[64];
    snprintf(debug_lat, sizeof(debug_lat), "Extracted Lat: [%s] %c", lat_str, lat_cardinal);
    debug_output(debug_lat);

    char debug_lon[64];
    snprintf(debug_lon, sizeof(debug_lon), "Extracted Lon: [%s] %c", lon_str, lon_cardinal);
    debug_output(debug_lon);

    // **✅ Format GLL Sentence Without Float Conversion**
    char gll_sentence[80];
    char full_nmea_sentence[90];

    // Get current time in hhmmss format
    unsigned long ms = millis() / 1000;
    int hh = (ms / 3600) % 24;
    int mm = (ms / 60) % 60;
    int ss = ms % 60;
    char utc_time[10];
    snprintf(utc_time, sizeof(utc_time), "%02d%02d%02d", hh, mm, ss);

    // Format GLL NMEA sentence
    snprintf(gll_sentence, sizeof(gll_sentence), "$IIGLL,%s,%c,%s,%c,,A*", lat_str, lat_cardinal,lon_str, lon_cardinal);

    // **✅ Calculate NMEA Checksum**
    uint8_t checksum = calculate_nmea_checksum(gll_sentence);
    snprintf(full_nmea_sentence, sizeof(full_nmea_sentence), "%s%02X\r\n", gll_sentence, checksum);

    // **✅ Send via UDP**
    udp_sender(full_nmea_sentence);
}

// Frame processing function
void process_frame(uint8_t *body, size_t body_size) {
    size_t index = 0;
    while (index < body_size) {
        if (index + 2 > body_size) break;

        uint8_t channel = body[index++];
        uint8_t format = body[index++];
        size_t data_size = get_format_size(format & 0x0F);

        if (index + data_size > body_size) {
            debug_output("Incomplete data for channel. Skipping.");
            break;
        }

        uint32_t data = 0;
        for (size_t i = 0; i < data_size; i++) {
            data = (data << 8) | body[index++];
        }

        double decoded_value = decode(format, data);

        // ** Correctly increment input counter**
        input_channel_count++;

        if (xSemaphoreTake(register_mutex, portMAX_DELAY) == pdTRUE) {
            channel_register[channel] = {decoded_value, millis()};
            xSemaphoreGive(register_mutex);
        }

        char msg[128];
        snprintf(msg, sizeof(msg), "Processed Channel: %-25s (0x%02X) | Format: 0x%02X | Data: 0x%08X | Decoded: %10.6f",
                 get_channel_name(channel), channel, format, data, decoded_value);
        debug_output(msg);

        // Construct and send NMEA messages
        char full_nmea_sentence[70];
        char nmea_sentence[64] = {0};  // Ensure it's initialized

        if (strcmp(get_channel_name(channel), "Boatspeed (Knots)") == 0) {
            double heading = get_last_value(0x49); // Get latest Heading (0x49 = Heading)
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIVHW,,,%.1f,M,%.1f,N,,*", heading, decoded_value);
        } 
        else if (strcmp(get_channel_name(channel), "Depth (Meters)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIDBT,,,%.2f,M,,*", decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "Rudder Angle") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIRSA,%.1f,A,,A*", decoded_value);
        }   
        else if (strcmp(get_channel_name(channel), "Battery Volts") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,U,%.2f,V,BATTV*", decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "True Wind Direction") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$WIMWD,,,%.1f,M,,N*", decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "Heading") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIHDG,%.1f,,,,*", decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "True Wind Speed (Knots)") == 0) {
            double twa = get_last_value(0x59); // Get latest True Wind Angle (TWA)

            // Convert TWA from ±180° to 0-360°
            if (twa < 0) {
                twa += 360.0;
            }
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,T,%.1f,N,A*", twa, decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "Apparent Wind Speed (Knots)") == 0) {
            double awa = get_last_value(0x51); // Get latest Apparent Wind Angle (0x51 = AWA)
            
            // Convert -180 to 180 range into 0 to 360
            if (awa < 0) {
                awa += 360.0;
            }

            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,R,%.1f,N,A*", awa, decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "Sea Temperature (°C)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMTW,%.1f,C*", decoded_value);
        }
        else if (strcmp(get_channel_name(channel), "Speed Over Ground") == 0) {
            double cog = get_last_value(0xEA); // Get latest Course Over Ground (Mag)
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIVTG,,%.1f,M,%.1f,N,,K*", cog, decoded_value);
        }
        // Only proceed if a sentence was set
        if (nmea_sentence[0] != '\0') {
            uint8_t checksum = calculate_nmea_checksum(nmea_sentence);
            snprintf(full_nmea_sentence, sizeof(full_nmea_sentence), "%s%02X\r\n", nmea_sentence, checksum);
            udp_sender(full_nmea_sentence);

        }
    }
}



// Setup function to start FreeRTOS tasks
void setup() {
    last_display_update = millis();
    Serial.begin(SERIAL_BAUD_RATE);
    M5.begin();
    M5.Lcd.setTextSize(2);
    debug_output("ESP32 FreeRTOS Decoder Initialized!");

    connect_to_wifi();

    register_mutex = xSemaphoreCreateMutex();
    if (register_mutex == NULL) {
        debug_output("Failed to create mutex!");
        while (1);
    }

    udpQueue = xQueueCreate(10, sizeof(char[64]));
    if (udpQueue == NULL) {
        Serial.println("⚠️ Failed to create UDP queue!");
        while (1);
    }


    udp.begin(UDP_PORT);
    nmea_udp.begin(NMEA_UDP_PORT);
    xTaskCreatePinnedToCore(task_udp_listener, "UDP Listener", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(update_display, "Update Display", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(udp_task, "UDP Sender", 4096, NULL, 1, NULL, 1);
}

void loop() {}