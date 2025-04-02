#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5CoreS3.h>
#include <map>
#include "secrets.h"



// ----------------------------------------------------------------------------------
// Configuration & Constants
// ----------------------------------------------------------------------------------
#define HEADER_SIZE 5                    // Number of bytes in the protocol header
#define MAX_BUFFER_SIZE 1024            // Maximum buffer size for incoming data
#define SERIAL_BAUD_RATE 115200         // Baud rate for standard serial debug output
#define BANDG_BAUD_RATE 28800           // Baud rate for the BandG data stream (Serial2)
#define NMEA_UDP_PORT 2002              // Port for UDP broadcast of NMEA sentences
#define DISPLAY_UPDATE_INTERVAL 1000    // Interval to update M5CoreS3 display (ms)
#define DEBUG_ENABLED false             // Toggle debug output
#define ERROR_ENABLED true              // Toggle error output

// WiFi credentials
//const char *ssid = "Sakura";
//const char *password = "rosebuds";

// ----------------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------------

// UDP server object
WiFiUDP nmea_udp;
char incomingPacket[MAX_BUFFER_SIZE];  // Temporary buffer for reading from Serial2

// Circular-like buffer for storing and processing BandG data
static uint8_t buffer[MAX_BUFFER_SIZE];
static size_t buffer_index = 0;        // Tracks current write position in 'buffer'
static size_t expected_size = 0;       // Expected frame size after parsing header

// Counters for debugging/performance measurement
volatile uint32_t input_channel_count = 0;    // How many channel updates have been processed
volatile uint32_t output_udp_count = 0;       // How many NMEA sentences have been sent via UDP

// Timestamp to control display updates
unsigned long last_display_update = 0;

// FreeRTOS queue for sending NMEA sentences from processing to the UDP sender task
QueueHandle_t udpQueue = xQueueCreate(10, sizeof(char[64]));

// Mutex for protecting access to the channel_register map
SemaphoreHandle_t register_mutex;


// ----------------------------------------------------------------------------------
// Data Structures & Helpers
// ----------------------------------------------------------------------------------

/**
 * @brief Returns the number of data bytes to read for a given 'format' nibble.
 * @param format The lower nibble of the format byte (or format & 0x0F).
 * @return The size in bytes to read for that format.
 */
size_t get_format_size(uint8_t format) {
    switch (format) {
        case 0x00: return 4;  // 32-bit raw
        case 0x01: return 2;  // 16-bit signed
        case 0x02: return 2;  // 10-bit data in 2 bytes
        case 0x03: return 2;  // special segment code + byte
        case 0x04: return 4;  // 24-bit signed
        case 0x05: return 4;  // (not fully supported)
        case 0x06: return 4;  // (not fully supported)
        case 0x07: return 4;  // (not fully supported)
        case 0x08: return 2;  // 9-bit data in 2 bytes
        case 0x0A: return 4;  // 16-bit signed in upper 16 bits of 4 bytes
        default:   return 0;  // Unknown or unimplemented format
    }
}

/**
 * @brief Holds the latest value and timestamp for each channel.
 */
struct ChannelData {
    double last_value;        // Last decoded numeric value from the channel
    unsigned long timestamp;  // Time (in ms) of last update
};

// Map of channel ID -> ChannelData
std::map<uint8_t, ChannelData> channel_register;


/**
 * @brief Returns a human-readable channel name.
 * @param channel The channel byte from BandG instrumentation.
 * @return A constant char pointer to the channel name.
 */
const char *get_channel_name(uint8_t channel) {
    switch (channel) {
        case 0x00: return "Node Reset";
        case 0x0B: return "Rudder Angle";
        case 0x0C: return "Linear 5";
        case 0x0D: return "Linear 6";
        case 0x0E: return "Linear 7";
        case 0x0F: return "Linear 8";
        case 0x10: return "Linear 9";
        case 0x11: return "Linear 10";
        case 0x12: return "Linear 11";
        case 0x13: return "Linear 12";
        case 0x14: return "Linear 13";
        case 0x15: return "Linear 14";
        case 0x16: return "Linear 15";
        case 0x17: return "Linear 16";
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
        case 0x38: return "Linear 1";
        case 0x39: return "Linear 2";
        case 0x3A: return "Linear 3";
        case 0x3B: return "Linear 4";
        case 0x3C: return "Rate Motion";
        case 0x41: return "Boatspeed (Knots)";
        case 0x42: return "Boatspeed (Raw)";
        case 0x44: return "Yaw rate";
        case 0x46: return "Autopilot Speed Fixed (Knots)";
        case 0x47: return "LatLon";
        case 0x49: return "Heading";
        case 0x4A: return "Heading (Raw)";
        case 0x4D: return "Apparent Wind Speed (Knots)";
        case 0x4E: return "Apparent Wind Speed (Raw)";
        case 0x4F: return "Apparent Wind Speed (m/s)";
        case 0x50: return "from NMEA";
        case 0x51: return "Apparent Wind Angle";
        case 0x52: return "Apparent Wind Angle (Raw)";
        case 0x53: return "Target TWA";
        case 0x55: return "True Wind Speed (Knots)";
        case 0x56: return "True Wind Speed (m/s)";
        case 0x57: return "Measured Wind Speed (Knots)";
        case 0x59: return "True Wind Angle";
        case 0x5A: return "Measured Wind Angle Deg";
        case 0x64: return "Average Speed (Knots)";
        case 0x65: return "Average Speed (Raw)";
        case 0x68: return "Request for Data";
        case 0x69: return "Course";
        case 0x6A: return "Act for Data";
        case 0x6D: return "True Wind Direction";
        case 0x6F: return "Next Leg Apparent Wind Angle";
        case 0x70: return "Next Leg Target Boat Speed";
        case 0x71: return "Next Leg Apparent Wind Speed";
        case 0x75: return "Timer";
        case 0x7C: return "Polar Performance";
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
        case 0xFA: return "Course to Sail";
        default: return "Unknown";
    }
}
// ----------------------------------------------------------------------------------
// FreeRTOS Task Function Declarations
// ----------------------------------------------------------------------------------

// Forward declaration for the display update task
void update_display(void *pvParameters);


/**
 * @brief Print a debug message if DEBUG_ENABLED is true.
 * @param message The message to print.
 */
void debug_output(const char *message) {
    if (DEBUG_ENABLED) {
        Serial.println(message);
    }
}

/**
 * @brief Print an error message if ERROR_ENABLED is true.
 * @param message The message to print.
 */
void error_output(const char *message) {
    if (ERROR_ENABLED) {
        Serial.println(message);
    }
}



// ----------------------------------------------------------------------------------
// UDP Sending & Task
// ----------------------------------------------------------------------------------

/**
 * @brief Send an NMEA sentence string to the UDP queue for asynchronous transmission.
 * @param nmea_sentence A null-terminated NMEA sentence.
 */
void udp_sender(const char *nmea_sentence) {
    if (xQueueSendToBack(udpQueue, nmea_sentence, pdMS_TO_TICKS(10)) != pdPASS) {
        error_output("UDP Queue Full! Dropping message.");
    }
}


/**
 * @brief Task that receives NMEA sentences from a queue and sends them via UDP.
 * @param pvParameters Unused.
 */
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

/**
 * @brief Print a message to the M5Stack LCD screen.
 * @param message The message to print.
 */
void lcd_output(const char *message) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println(message);
}



/**
 * @brief Calculate NMEA 0183 checksum for a sentence (excluding the '$' and trailing '*').
 * @param sentence Null-terminated NMEA sentence (with '$' start, '*' near the end).
 * @return The calculated checksum byte.
 */
uint8_t calculate_nmea_checksum(const char *sentence) {
    uint8_t checksum = 0;
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        checksum ^= sentence[i];
    }
    return checksum;
}



// ----------------------------------------------------------------------------------
// Channel Register Accessors
// ----------------------------------------------------------------------------------

/**
 * @brief Retrieve the last stored value for a channel.
 * @param channel The channel ID (0x00..0xFF).
 * @return The last_value or 0.0 if not found.
 */
double get_last_value(uint8_t channel) {
    if (channel_register.find(channel) != channel_register.end()) {
        return channel_register[channel].last_value;
    }
    return 0.0; // Default if channel not found
}


/**
 * @brief Retrieve the timestamp of the last update for a channel.
 * @param channel The channel ID (0x00..0xFF).
 * @return The timestamp in ms or 0 if not found.
 */
unsigned long get_last_update_time(uint8_t channel) {
    if (channel_register.find(channel) != channel_register.end()) {
        return channel_register[channel].timestamp;
    }
    return 0; // Default if channel not found
}


/**
 * @brief Decode raw data into a double value according to the BandG format byte.
 * @param format The format byte from the frame (0x..). Divisor is embedded in upper bits.
 * @param data   Raw data combined into a 32-bit integer from the frame.
 * @return Decoded floating-point value.
 */
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
                char debug_message[50]; // Adjust size as needed
                snprintf(debug_message, sizeof(debug_message), "Segment code: 0x%02X", segment_code);
                debug_output(debug_message);

            
            if (segment_code == 0xa8 || segment_code == 0xa0 || segment_code == 0x8c) {
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



/**
 * @brief Calculates the BandG-specific checksum of a data array (sum of bytes).
 * @param data Pointer to the data array.
 * @param length Number of bytes to include.
 * @return The resulting checksum (one byte).
 */
uint8_t calculate_checksum(uint8_t *data, size_t length) {
    uint16_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (0x100 - (sum % 0x100)) & 0xFF;
}






// ----------------------------------------------------------------------------------
// Wi-Fi Connection
// ----------------------------------------------------------------------------------

/**
 * @brief Connect to the configured Wi-Fi network. This function is used during setup.
 */
void connect_to_wifi() {
    WiFi.mode(WIFI_STA);  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        dots = (dots + 1) % 4;

        // Clear screen and print new status
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("Connecting to WiFi%s", 
            dots == 0 ? "" : (dots == 1 ? "." : (dots == 2 ? ".." : "...")));
    }
}





// ----------------------------------------------------------------------------------
// Main Serial Listener Task
// ----------------------------------------------------------------------------------

/**
 * @brief Task that continuously reads from Serial2 (BandG instrumentation).
 *        Data is buffered, and then `process_stream()` is called to parse frames.
 * @param pvParameters Unused.
 */
void task_serial_listener(void *pvParameters) {
    debug_output("Listening on Serial2 for incoming data...");

    for (;;) {
        int available_bytes = Serial2.available();  // Get number of available bytes
        if (available_bytes > 0) {
            int len = Serial2.readBytes(incomingPacket, available_bytes);  // Read all available data

            if (len > 0) {
                if (buffer_index + len > MAX_BUFFER_SIZE) {
                    error_output("⚠️ BUFFER FULL! Shifting data to make room...");
                    
                    // Shift buffer to make room for new data
                    memmove(buffer, buffer + len, MAX_BUFFER_SIZE - len);
                    buffer_index -= len;
                }

                memcpy(buffer + buffer_index, incomingPacket, len);
                buffer_index += len;

                if (buffer_index >= HEADER_SIZE) {
                    process_stream();  // Process collected data
                }
            }
        }

        vTaskDelay(0);
    }
}




// ----------------------------------------------------------------------------------
// M5Stack Display Task
// ----------------------------------------------------------------------------------

/**
 * @brief Task to periodically update the M5CoreS3 display with 
 *        network status and data throughput rates.
 * @param pvParameters Unused.
 */
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
            M5.Lcd.printf("%s", last_ip.c_str());  
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
        M5.Lcd.printf("%s", ipAddress.c_str());

        M5.Lcd.setCursor(10, 40);
        M5.Lcd.printf("Input:  %4.0f channels/s", input_rate);

        M5.Lcd.setCursor(10, 70);
        M5.Lcd.printf("Output: %4.0f NMEA/s", output_rate);
        
        last_display_update = current_time;

        // Wait until next execution time (ensures stable intervals)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_UPDATE_INTERVAL));
    }
}






// ----------------------------------------------------------------------------------
// BandG Stream Parsing
// ----------------------------------------------------------------------------------

/**
 * @brief Continuously check the buffer for complete frames. If a valid frame is found,
 *        decode it and remove it from the buffer.
 */
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





/**
 * @brief Process an ASCII data frame (e.g., latitude/longitude string).
 *        Here, we specifically build an NMEA GLL sentence and broadcast it via UDP.
 * @param body Pointer to the frame data (excluding header).
 * @param body_size Number of data bytes in this ASCII frame.
 */
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

    // ** Format GLL Sentence Without Float Conversion**
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

    // ** Calculate NMEA Checksum**
    uint8_t checksum = calculate_nmea_checksum(gll_sentence);
    snprintf(full_nmea_sentence, sizeof(full_nmea_sentence), "%s%02X\r\n", gll_sentence, checksum);

    // ** Send via UDP**
    udp_sender(full_nmea_sentence);
}



/**
 * @brief Process a numeric data frame: extract channel/format/data, decode, 
 *        store in channel_register, and build relevant NMEA sentences.
 * @param body Pointer to the numeric data bytes in the frame.
 * @param body_size How many bytes of data in the frame.
 */
void process_frame(uint8_t *body, size_t body_size) {
    size_t index = 0;
    while (index < body_size) {
        // Ensure there's enough space for channel + format
        if (index + 2 > body_size) break;

        // 1) Read the channel byte
        uint8_t channel = body[index++];
        // 2) Read the format byte
        uint8_t format = body[index++];
        // 3) Determine how many data bytes this format requires
        size_t data_size = get_format_size(format & 0x0F);

        // If there's not enough data left for this channel, skip
        if (index + data_size > body_size) {
            debug_output("Incomplete data for channel. Skipping.");
            break;
        }

        // 4) Read the data bytes and combine into a 32-bit
        uint32_t data = 0;
        for (size_t i = 0; i < data_size; i++) {
            data = (data << 8) | body[index++];
        }

        // 5) Decode the data
        double decoded_value = decode(format, data);

        // Increment the input channel counter
        input_channel_count++;

        // 6) Store the decoded value in the channel_register map (thread-safe)
        if (xSemaphoreTake(register_mutex, portMAX_DELAY) == pdTRUE) {
            if (channel_register.find(channel) == channel_register.end()) {
                // If new channel, initialize with timestamp=0
                channel_register[channel] = {decoded_value, 0};
            } else {
                channel_register[channel].last_value = decoded_value;
                channel_register[channel].timestamp = millis();
            }
            xSemaphoreGive(register_mutex);
        }

        // For debug: print out processed info
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Processed Channel: %-25s (0x%02X) | Format: 0x%02X | Data: 0x%08X | Decoded: %10.6f",
                 get_channel_name(channel), channel, format, data, decoded_value);
        debug_output(msg);

        // 7) Build optional NMEA sentences for certain known channels
        char full_nmea_sentence[70];
        char nmea_sentence[64] = {0};  // Initialized to empty string

        // -- Example channel: Boatspeed (Knots)
        if (strcmp(get_channel_name(channel), "Boatspeed (Knots)") == 0) {
            double heading = get_last_value(0x49); // Channel 0x49 is "Heading"
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIVHW,,,%.1f,M,%.1f,N,,*", heading, decoded_value);
        }
        // -- Depth (Meters)
        else if (strcmp(get_channel_name(channel), "Depth (Meters)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIDBT,,,%.2f,M,,*", decoded_value);
        }
        // -- Rudder Angle
        else if (strcmp(get_channel_name(channel), "Rudder Angle") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIRSA,%.1f,A,,A*", decoded_value);
        }
        // -- Battery Volts
        else if (strcmp(get_channel_name(channel), "Battery Volts") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,U,%.2f,V,BATTV*", decoded_value);
        }
        // -- True Wind Direction
        else if (strcmp(get_channel_name(channel), "True Wind Direction") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$WIMWD,,,%.1f,M,,N*", decoded_value);
        }
        // -- Heading
        else if (strcmp(get_channel_name(channel), "Heading") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIHDG,%.1f,,,,*", decoded_value);
        }
        // -- True Wind Speed (Knots)
        else if (strcmp(get_channel_name(channel), "True Wind Speed (Knots)") == 0) {
            double twa = get_last_value(0x59); // 0x59 is "True Wind Angle"
            // Convert TWA from ±180° to 0..360°
            if (twa < 0) {
                twa += 360.0;
            }
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,T,%.1f,N,A*", twa, decoded_value);
        }
        // -- True Wind Angle
        else if (strcmp(get_channel_name(channel), "True Wind Angle") == 0) {
            double tws = get_last_value(0x55); // 0x55 is "True Wind Speed (Knots)"
            double twa = decoded_value;
            if (twa < 0) {
                twa += 360.0;
            }
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,T,%.1f,N,A*", twa, tws);
        }
        // -- Apparent Wind Speed (Knots)
        else if (strcmp(get_channel_name(channel), "Apparent Wind Speed (Knots)") == 0) {
            double awa = get_last_value(0x51); // 0x51 is "Apparent Wind Angle"
            // Convert from ±180° to 0..360°
            if (awa < 0) {
                awa += 360.0;
            }
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,R,%.1f,N,A*", awa, decoded_value);
        }
        // -- Apparent Wind Angle
        else if (strcmp(get_channel_name(channel), "Apparent Wind Angle") == 0) {
            double aws = get_last_value(0x4D); // 0x4D is "Apparent Wind Speed (Knots)"
            double awa = decoded_value;
            if (awa < 0) {
                awa += 360.0;
            }
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMWV,%.1f,R,%.1f,N,A*", awa, aws);
        }
        // -- Sea Temperature (°C)
        else if (strcmp(get_channel_name(channel), "Sea Temperature (°C)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIMTW,%.1f,C*", decoded_value);
        }
        // -- Tidal Drift
        else if (strcmp(get_channel_name(channel), "Tidal Drift") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,N,%.2f,M,DRIFT*", decoded_value);
        }
        // -- Tidal Set
        else if (strcmp(get_channel_name(channel), "Tidal Set") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,A,%.0f,D,SET*", decoded_value);
        }
        // -- Apparent Wind Angle (Raw)
        else if (strcmp(get_channel_name(channel), "Apparent Wind Angle (Raw)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,A,%.2f,V,Wind_A_Raw*", decoded_value);
        }
        // -- Apparent Wind Speed (Raw)
        else if (strcmp(get_channel_name(channel), "Apparent Wind Speed (Raw)") == 0) {
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIXDR,N,%.2f,V,Wind_S_Raw*", decoded_value);
        }
        // -- Speed Over Ground
        else if (strcmp(get_channel_name(channel), "Speed Over Ground") == 0) {
            double cog = get_last_value(0xEA); // 0xEA is "Course Over Ground (Mag)"
            snprintf(nmea_sentence, sizeof(nmea_sentence), "$IIVTG,,,%.1f,M,%.1f,N,,,A*", cog, decoded_value);
        }

        // 8) If we built an NMEA sentence, finalize with checksum & send
        if (nmea_sentence[0] != '\0') {
            uint8_t checksum = calculate_nmea_checksum(nmea_sentence);
            snprintf(full_nmea_sentence, sizeof(full_nmea_sentence), "%s%02X\r\n", nmea_sentence, checksum);
            udp_sender(full_nmea_sentence);
        }
    }
}
// ----------------------------------------------------------------------------------
// Arduino Setup & Loop
// ----------------------------------------------------------------------------------

/**
 * @brief Arduino setup function - initializes M5, Serial, Wi-Fi, tasks, etc.
 */
void setup() {
    last_display_update = millis();

    // Initialize M5CoreS3 hardware
    M5.begin();

    // Prepare the LCD
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextColor(WHITE, BLACK);

    // Start the standard Serial for debugging
    Serial.begin(SERIAL_BAUD_RATE);

    // Setup Serial2 to read from BandG instrumentation
    // Arguments: baudRate, config, RX pin, TX pin, invert, bufferSize
    Serial2.begin(BANDG_BAUD_RATE, SERIAL_8O2, 18, 17, false, 1024);

    // Verify the actual baud rate set
    uint32_t actualBaud = Serial2.baudRate();
    char debug_msg[50];
    snprintf(debug_msg, sizeof(debug_msg), "Serial2 Config: %u baud, 8O2", actualBaud);
    debug_output(debug_msg);

    debug_output("ESP32 FreeRTOS Decoder Initialized!");

    // Show message on the screen while connecting to Wi-Fi
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.println("Connecting to WiFi...");

    // Attempt to connect to Wi-Fi (blocking) with a simple "dots" animation
    connect_to_wifi();

    // Once connected, print IP address
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("WiFi Connected!\nIP: %s", WiFi.localIP().toString().c_str());

    // Create mutex for channel_register
    register_mutex = xSemaphoreCreateMutex();
    if (register_mutex == NULL) {
        error_output("Failed to create mutex!");
        while (1);
    }

    // Create the UDP queue if not created globally
    udpQueue = xQueueCreate(10, sizeof(char[64]));
    if (udpQueue == NULL) {
        error_output("Failed to create UDP queue!");
        while (1);
    }

    // Begin listening for UDP on the specified port
    nmea_udp.begin(NMEA_UDP_PORT);

    // Clear the LCD for main usage
    M5.Lcd.fillScreen(BLACK);

    // Launch FreeRTOS tasks:
    // 1) Serial listener: read from BandG instrumentation

    // 2) UDP sending task
 
    // 3) Display update task
    xTaskCreatePinnedToCore(update_display, "Update Display", 8192, NULL, 1 , NULL, 0);
    xTaskCreatePinnedToCore(task_serial_listener, "Serial Listener", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(udp_task, "UDP Sender", 8192, NULL, 2, NULL, 1);
}

/**
 * @brief Arduino loop function - empty because we rely on FreeRTOS tasks instead.
 */
void loop() {
    // No code needed here because all functionality is handled in tasks
}