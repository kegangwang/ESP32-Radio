//***************************************************************************************************
//*  ESP32_Radio -- Webradio receiver for ESP32, VS1053 MP3 module and optional display.            *
//*                 By Ed Smallenburg.                                                              *
//***************************************************************************************************
// ESP32 libraries used:
//  - WiFiMulti
//  - nvs
//  - Adafruit_ST7735
//  - ArduinoOTA
//  - PubSubClientenc_dt_pin
//  - SD
//  - FS
//  - update
// A library for the VS1053 (for ESP32) is not available (or not easy to find).  Therefore
// a class for this module is derived from the maniacbug library and integrated in this sketch.
//
// See http://www.internet-radio.com for suitable stations.  Add the stations of your choice
// to the preferences in either Esp32_radio_init.ino sketch or through the webinterface.
//
// Brief description of the program:
// First a suitable WiFi network is found and a connection is made.
// Then a connection will be made to a shoutcast server.  The server starts with some
// info in the header in readable ascii, ending with a double CRLF, like:
//  icy-name:Classic Rock Florida - SHE Radio
//  icy-genre:Classic Rock 60s 70s 80s Oldies Miami South Florida
//  icy-url:http://www.ClassicRockFLorida.com
//  content-type:audio/mpeg
//  icy-pub:1
//  icy-metaint:32768          - Metadata after 32768 bytes of MP3-data
//  icy-br:128                 - in kb/sec (for Ogg this is like "icy-br=Quality 2"
//
// After de double CRLF is received, the server starts sending mp3- or Ogg-data.  For mp3, this
// data may contain metadata (non mp3) after every "metaint" mp3 bytes.
// The metadata is empty in most cases, but if any is available the content will be
// presented on the TFT.
// Pushing an input button causes the player to execute a programmable command.
//
// The display used is a Chinese 1.8 color TFT module 128 x 160 pixels.
// Now there is room for 26 characters per line and 16 lines.
// Software will work without installing the display.
// Other displays are also supported. See documentation.
// The SD card interface of the module may be used to play mp3-tracks on the SD card.
//
// For configuration of the WiFi network(s): see the global data section further on.
//
// The VSPI interface is used for VS1053, TFT and SD.
//
// Wiring. Note that this is just an example.  Pins (except 18,19 and 23 of the SPI interface)
// can be configured in the config page of the web interface.
// ESP32dev Signal  Wired to LCD        Wired to VS1053      SDCARD   Wired to the rest
// -------- ------  --------------      -------------------  ------   ---------------
// GPIO32           -                   pin 1 XDCS            -       -
// GPIO5            -                   pin 2 XCS             -       -
// GPIO4            -                   pin 4 DREQ            -       -
// GPIO2            pin 3 D/C or A0     -                     -       -
// GPIO22           -                   -                     CS      -
// GPIO16   RXD2    -                   -                     -       TX of NEXTION (if in use)
// GPIO17   TXD2    -                   -                     -       RX of NEXTION (if in use)
// GPIO18   SCK     pin 5 CLK or SCK    pin 5 SCK             CLK     -
// GPIO19   MISO    -                   pin 7 MISO            MISO    -
// GPIO23   MOSI    pin 4 DIN or SDA    pin 6 MOSI            MOSI    -
// GPIO15           pin 2 CS            -                     -       -
// GPI03    RXD0    -                   -                     -       Reserved serial input
// GPIO1    TXD0    -                   -                     -       Reserved serial output
// GPIO34   -       -                   -                     -       Optional pull-up resistor
// GPIO35   -       -                   -                     -       Infrared receiver VS1838B
// GPIO25   -       -                   -                     -       Rotary encoder CLK
// GPIO26   -       -                   -                     -       Rotary encoder DT
// GPIO27   -       -                   -                     -       Rotary encoder SW
// -------  ------  ---------------     -------------------  ------   ----------------
// GND      -       pin 8 GND           pin 8 GND                     Power supply GND
// VCC 5 V  -       pin 7 BL            -                             Power supply
// VCC 5 V  -       pin 6 VCC           pin 9 5V                      Power supply
// EN       -       pin 1 RST           pin 3 XRST                    -
//
// 26-04-2017, ES: First set-up, derived from ESP8266 version.
// 08-05-2017, ES: Handling of preferences.
// 20-05-2017, ES: Handling input buttons and MQTT.
// 22-05-2017, ES: Save preset, volume and tone settings.
// 23-05-2017, ES: No more calls of non-iram functions on interrupts.
// 24-05-2017, ES: Support for featherboard.
// 26-05-2017, ES: Correction playing from .m3u playlist. Allow single hidden SSID.
// 30-05-2017, ES: Add SD card support (FAT format), volume indicator.
// 26-06-2017, ES: Correction: start in AP-mode if no WiFi networks configured.
// 28-06-2017, ES: Added IR interface.
// 30-06-2017, ES: Improved functions for SD card play.
// 03-07-2017, ES: Webinterface control page shows current settings.
// 04-07-2017, ES: Correction MQTT subscription. Keep playing during long operations.
// 08-07-2017, ES: More space for streamtitle on TFT.
// 18-07-2017, ES: Time Of Day on TFT.
// 19-07-2017, ES: Minor corrections.
// 26-07-2017, ES: Flexible pin assignment. Add rotary encoder switch.
// 27-07-2017, ES: Removed tinyXML library.
// 18-08-2017, Es: Minor corrections
// 28-08-2017, ES: Preferences for pins used for SPI bus,
//                 Corrected bug in handling programmable pins,
//                 Introduced touch pins.
// 30-08-2017, ES: Limit number of retries for MQTT connection.
//                 Added MDNS responder.
// 11-11-2017, ES: Increased ringbuffer.  Measure bit rate.
// 13-11-2017, ES: Forward declarations.
// 16-11-2017, ES: Replaced ringbuffer by FreeRTOS queue, play function on second CPU,
//                 Included improved rotary switch routines supplied by fenyvesi,
//                 Better IR sensitivity.
// 30-11-2017, ES: Hide passwords in config page.
// 01-12-2017, ES: Better handling of playlist.
// 07-12-2017, ES: Faster handling of config screen.
// 08-12-2017, ES: More MQTT items to publish, added pin_shutdown.
// 13-12-2017, ES: Correction clear LCD.
// 15-12-2017, ES: Correction defaultprefs.h.
// 18-12-2017, ES: Stop playing during config.
// 02-01-2018, ES: Stop/resume is same command.
// 22-01-2018, ES: Read ADC (GPIO36) and display as a battery capacity percentage.
// 13-02-2018, ES: Stop timer during NVS write.
// 15-02-2018, ES: Correction writing wifi credentials in NVS.
// 03-03-2018, ES: Correction bug IR pinnumber.
// 05-03-2018, ES: Improved rotary encoder interface.
// 10-03-2018, ES: Minor corrections.
// 13-04-2018, ES: Guard against empty string send to TFT, thanks to Andreas Spiess.
// 16-04-2018, ES: ID3 tags handling while playing from SD.
// 25-04-2018, ES: Choice of several display boards.
// 30-04-2018, ES: Bugfix: crash when no IR is configured, no display without VS1063.
// 08-05-2018, ES: 1602 LCD display support (limited).
// 11-05-2018, ES: Bugfix: incidental crash in isr_enc_turn().
// 30-05-2018, ES: Bugfix: Assigned DRAM to global variables used in timer ISR.
// 31-05-2018, ES: Bugfix: Crashed if I2C is used, but pins not defined.
// 01-06-2018, ES: Run Playtask on CPU 0.
// 04-06-2018, ES: Made handling of playlistdata more tolerant (NDR).
// 09-06-2018, ES: Typo in defaultprefs.h.
// 10-06-2018, ES: Rotary encoder, interrupts on all 3 signals.
// 25-06-2018, ES: Timing of mp3loop.  Limit read from stream to free queue space.
// 16-07-2018, ES: Correction tftset().
// 25-07-2018, ES: Correction touch pins.
// 30-07-2018, ES: Added GPIO39 and inversed shutdown pin.  Thanks to fletsche.
// 31-07-2018, ES: Added TFT backlight control.
// 01-08-2018, ES: Debug info for IR.  Shutdown amplifier if volume is 0.
// 02-08-2018, ES: Added support for ILI9341 display.
// 03-08-2018, ES: Added playlistposition for MQTT.
// 06-08-2018, ES: Correction negative time offset, OTA through remote host.
// 16-08-2018, ES: Added Nextion support.
// 18-09-2018, ES: "uppreset" and "downpreset" for MP3 player.
// 04-10-2018, ES: Fixed compile error OLED 64x128 display.
// 09-10-2018, ES: Bug fix xSemaphoreTake.
// 05-01-2019, ES: Fine tune datarate.
// 05-01-2019, ES: Basic http authentication. (just one user)
// 

// Define the version number, also used for webserver as Last-Modified header and to
// check version for update.  The format must be exactly as specified by the HTTP standard!
#define VERSION     "Tue, 05 Jan 2019 19:48:00 GMT"
// ESP32-Radio can be updated (OTA) to the latest version from a remote server.
// The download uses the following server and files:
#define UPDATEHOST  "smallenburg.nl"                    // Host for software updates
#define BINFILE     "/Arduino/Esp32_radio.ino.bin"      // Binary file name for update software
#define TFTFILE     "/Arduino/ESP32-Radio.tft"          // Binary file name for update NEXTION image
//
// Define (just one) type of display.  See documentation.
#define BLUETFT                      // Works also for RED TFT 128x160
//#define OLED                         // 64x128 I2C OLED
//#define DUMMYTFT                     // Dummy display
//#define LCD1602I2C                   // LCD 1602 display with I2C backpack
//#define ILI9341                      // ILI9341 240*320
//#define NEXTION                      // Nextion display. Uses UART 2 (pin 16 and 17)
//
#include <nvs.h>
#include <PubSubClient.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_partition.h>
#include <driver/adc.h>
#include <Update.h>
#include <base64.h>
// Number of entries in the queue
#define QSIZ 400
// Debug buffer size
#define DEBUG_BUFFER_SIZE 150
// Access point name if connection to WiFi network fails.  Also the hostname for WiFi and OTA.
// Not that the password of an AP must be at least as long as 8 characters.
// Also used for other naming.
#define NAME "ESP32Radio"
// Maximum number of MQTT reconnects before give-up
#define MAXMQTTCONNECTS 5
// Adjust size of buffer to the longest expected string for nvsgetstr
#define NVSBUFSIZE 150
// Position (column) of time in topline relative to end
#define TIMEPOS -52
// SPI speed for SD card
#define SDSPEED 1000000
// Size of metaline buffer
#define METASIZ 1024
// Max. number of NVS keys in table
#define MAXKEYS 200
// Time-out [sec] for blanking TFT display (BL pin)
#define BL_TIME 45
//
// Subscription topics for MQTT.  The topic will be pefixed by "PREFIX/", where PREFIX is replaced
// by the the mqttprefix in the preferences.  The next definition will yield the topic
// "ESP32Radio/command" if mqttprefix is "ESP32Radio".
#define MQTT_SUBTOPIC     "command"           // Command to receive from MQTT
//
#define otaclient mp3client                   // OTA uses mp3client for connection to host

//**************************************************************************************************
// Forward declaration and prototypes of various functions.                                        *
//**************************************************************************************************
void        displaytime ( const char* str, uint16_t color = 0xFFFF ) ;
void        showstreamtitle ( const char* ml, bool full = false ) ;
void        handlebyte_ch ( uint8_t b ) ;
void        handleFSf ( const String& pagename ) ;
void        handleCmd()  ;
char*       dbgprint( const char* format, ... ) ;
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
void        chomp ( String &str ) ;
String      httpheader ( String contentstype ) ;
bool        nvssearch ( const char* key ) ;
void        mp3loop() ;
void        tftlog ( const char *str ) ;
void        playtask ( void * parameter ) ;       // Task to play the stream
void        spftask ( void * parameter ) ;        // Task for special functions
void        gettime() ;
void        reservepin ( int8_t rpinnr ) ;


//**************************************************************************************************
// Several structs.                                                                                *
//**************************************************************************************************
//

struct scrseg_struct                                  // For screen segments
{
  bool     update_req ;                               // Request update of screen
  uint16_t color ;                                    // Textcolor
  uint16_t y ;                                        // Begin of segment row
  uint16_t height ;                                   // Height of segment
  String   str ;                                      // String to be displayed
} ;

enum qdata_type { QDATA, QSTARTSONG, QSTOPSONG } ;    // datatyp in qdata_struct
struct qdata_struct
{
  int datatyp ;                                       // Identifier
  __attribute__((aligned(4))) uint8_t buf[32] ;       // Buffer for chunk
} ;

struct ini_struct
{
  String         mqttbroker ;                         // The name of the MQTT broker server
  String         mqttprefix ;                         // Prefix to use for topics
  uint16_t       mqttport ;                           // Port, default 1883
  String         mqttuser ;                           // User for MQTT authentication
  String         mqttpasswd ;                         // Password for MQTT authentication
  uint8_t        reqvol ;                             // Requested volume
  uint8_t        rtone[4] ;                           // Requested bass/treble settings
  int8_t         newpreset ;                          // Requested preset
  String         clk_server ;                         // Server to be used for time of day clock
  int8_t         clk_offset ;                         // Offset in hours with respect to UTC
  int8_t         clk_dst ;                            // Number of hours shift during DST
  int8_t         ir_pin ;                             // GPIO connected to output of IR decoder
  int8_t         enc_clk_pin ;                        // GPIO connected to CLK of rotary encoder
  int8_t         enc_dt_pin ;                         // GPIO connected to DT of rotary encoder
  int8_t         enc_sw_pin ;                         // GPIO connected to SW of rotary encoder
  int8_t         tft_cs_pin ;                         // GPIO connected to CS of TFT screen
  int8_t         tft_dc_pin ;                         // GPIO connected to D/C or A0 of TFT screen
  int8_t         tft_scl_pin ;                        // GPIO connected to SCL of i2c TFT screen
  int8_t         tft_sda_pin ;                        // GPIO connected to SDA of I2C TFT screen
  int8_t         tft_bl_pin ;                         // GPIO to activate BL of display
  int8_t         tft_blx_pin ;                        // GPIO to activate BL of display (inversed logic)
  int8_t         sd_cs_pin ;                          // GPIO connected to CS of SD card
  int8_t         vs_cs_pin ;                          // GPIO connected to CS of VS1053
  int8_t         vs_dcs_pin ;                         // GPIO connected to DCS of VS1053
  int8_t         vs_dreq_pin ;                        // GPIO connected to DREQ of VS1053
  int8_t         vs_shutdown_pin ;                    // GPIO to shut down the amplifier
  int8_t         vs_shutdownx_pin ;                   // GPIO to shut down the amplifier (inversed logic)
  int8_t         spi_sck_pin ;                        // GPIO connected to SPI SCK pin
  int8_t         spi_miso_pin ;                       // GPIO connected to SPI MISO pin
  int8_t         spi_mosi_pin ;                       // GPIO connected to SPI MOSI pin
  uint16_t       bat0 ;                               // ADC value for 0 percent battery charge
  uint16_t       bat100 ;                             // ADC value for 100 percent battery charge
} ;

struct WifiInfo_t                                     // For list with WiFi info
{
  uint8_t inx ;                                       // Index as in "wifi_00"
  char * ssid ;                                       // SSID for an entry
  char * passphrase ;                                 // Passphrase for an entry
} ;

struct nvs_entry
{
  uint8_t  Ns ;                                       // Namespace ID
  uint8_t  Type ;                                     // Type of value
  uint8_t  Span ;                                     // Number of entries used for this item
  uint8_t  Rvs ;                                      // Reserved, should be 0xFF
  uint32_t CRC ;                                      // CRC
  char     Key[16] ;                                  // Key in Ascii
  uint64_t Data ;                                     // Data in entry
} ;

struct nvs_page                                       // For nvs entries
{ // 1 page is 4096 bytes
  uint32_t  State ;
  uint32_t  Seqnr ;
  uint32_t  Unused[5] ;
  uint32_t  CRC ;
  uint8_t   Bitmap[32] ;
  nvs_entry Entry[126] ;
} ;

struct keyname_t                                      // For keys in NVS
{
  char      Key[16] ;                                 // Mac length is 15 plus delimeter
} ;


//**************************************************************************************************
// Global data section.                                                                            *
enum display_t { T_UNDEFINED, T_BLUETFT, T_OLED,         // Various types of display
                 T_DUMMYTFT, T_LCD1602I2C, T_ILI9341,
                 T_NEXTION } ;

enum datamode_t { INIT = 1, HEADER = 2, DATA = 4,        // State for datastream
                  METADATA = 8, PLAYLISTINIT = 16,
                  PLAYLISTHEADER = 32, PLAYLISTDATA = 64,
                  STOPREQD = 128, STOPPED = 256
                } ;


// Global variables
extern int               DEBUG ;                            // Debug on/off
extern int               numSsid ;                              // Number of available WiFi networks
extern WiFiMulti         wifiMulti ;                            // Possible WiFi networks
extern ini_struct        ini_block ;                            // Holds configurable data
extern WiFiServer        cmdserver ;                     // Instance of embedded webserver, port 80
extern WiFiClient        mp3client ;                            // An instance of the mp3 client, also used for OTA
extern WiFiClient        cmdclient ;                            // An instance of the client for commands
extern WiFiClient        wmqttclient ;                          // An instance for mqtt
extern PubSubClient      mqttclient ;           // Client for MQTT subscriber
extern HardwareSerial*   nxtserial ;                     // Serial port for NEXTION (if defined)
extern TaskHandle_t      maintask ;                             // Taskhandle for main task
extern TaskHandle_t      xplaytask ;                            // Task handle for playtask
extern TaskHandle_t      xspftask ;                             // Task handle for special functions
extern SemaphoreHandle_t SPIsem ;                        // For exclusive SPI usage
extern hw_timer_t*       timer ;                         // For timer
extern char              timetxt[9] ;                           // Converted timeinfo
extern char              cmd[130] ;                             // Command from MQTT or Serial
extern uint8_t           tmpbuff[6000] ;                        // Input buffer for mp3 or data stream 
extern QueueHandle_t     dataqueue ;                            // Queue for mp3 datastream
extern QueueHandle_t     spfqueue ;                             // Queue for special functions
extern qdata_struct      outchunk ;                             // Data to queue
extern qdata_struct      inchunk ;                              // Data from queue
extern uint8_t*          outqp ;                 // Pointer to buffer in outchunk
extern uint32_t          totalcount ;                       // Counter mp3 data
extern datamode_t        datamode ;                             // State of datastream
extern int               metacount ;                            // Number of bytes in metadata
extern int               datacount ;                            // Counter databytes before metadata
extern char              metalinebf[METASIZ + 1] ;              // Buffer for metaline/ID3 tags
extern int16_t           metalinebfx ;                          // Index for metalinebf
extern String            icystreamtitle ;                       // Streamtitle from metadata
extern String            icyname ;                              // Icecast station name
extern String            ipaddress ;                            // Own IP-address
extern int               bitrate ;                              // Bitrate in kb/sec
extern int               mbitrate ;                             // Measured bitrate
extern int               metaint ;                          // Number of databytes between metadata
extern int8_t            currentpreset ;                   // Preset station playing
extern String            host ;                                 // The URL to connect to or file to play
extern String            playlist ;                             // The URL of the specified playlist
extern bool              hostreq ;                      // Request for new host
extern bool              reqtone ;                      // New tone setting requested
extern bool              muteflag ;                     // Mute output
extern bool              resetreq ;                     // Request to reset the ESP32
extern bool              updatereq ;                    // Request to update software from remote host
extern bool              NetworkFound ;                 // True if WiFi network connected
extern bool              mqtt_on ;                      // MQTT in use
extern String            networks ;                             // Found networks in the surrounding
extern uint16_t          mqttcount ;                        // Counter MAXMQTTCONNECTS
extern int8_t            playingstat ;                      // 1 if radio is playing (for MQTT)
extern int16_t           playlist_num ;                     // Nonzero for selection from playlist
extern File              mp3file ;                              // File containing mp3 on SD card
extern uint32_t          mp3filelength ;                        // File length
extern bool              localfile ;                    // Play from local mp3-file or not
extern bool              chunked ;                      // Station provides chunked transfer
extern int               chunkcount ;                       // Counter for chunked transfer
extern String            http_getcmd ;                          // Contents of last GET command
extern String            http_rqfile ;                          // Requested file
extern bool              http_reponse_flag ;            // Response required
extern uint16_t          ir_value ;                         // IR code
extern uint32_t          ir_0 ;                           // Average duration of an IR short pulse
extern uint32_t          ir_1 ;                          // Average duration of an IR long pulse
extern struct tm         timeinfo ;                             // Will be filled by NTP server
extern bool              time_req ;                     // Set time requested
extern bool              SD_okay ;                      // True if SD card in place and readable
extern String            SD_nodelist ;                          // Nodes of mp3-files on SD
extern int               SD_nodecount ;                     // Number of nodes in SD_nodelist
extern String            SD_currentnode ;                  // Node ID of song playing ("0" if random)
extern uint16_t          adcval ;                               // ADC value (battery voltage)
extern uint32_t          clength ;                              // Content length found in http header
extern uint32_t          max_mp3loop_time ;                 // To check max handling time in mp3loop (msec)
extern int16_t           scanios ;                              // TEST*TEST*TEST
extern int16_t           scaniocount ;                          // TEST*TEST*TEST
extern uint16_t          bltimer ;                          // Backlight time-out counter
extern display_t         displaytype  ;            // Display type
extern std::vector<WifiInfo_t> wifilist ;                       // List with wifi_xx info
// nvs stuff
extern nvs_page                nvsbuf ;                         // Space for 1 page of NVS info
extern const esp_partition_t*  nvs ;                            // Pointer to partition struct
extern esp_err_t               nvserr ;                         // Error code from nvs functions
extern uint32_t                nvshandle ;                  // Handle for nvs access
extern uint8_t                 namespace_ID ;                   // Namespace ID found
extern char                    nvskeys[MAXKEYS][16] ;           // Space for NVS keys
extern std::vector<keyname_t> keynames ;                        // Keynames in NVS
// Rotary encoder stuff
#define sv DRAM_ATTR static volatile
extern sv uint16_t       clickcount ;                       // Incremented per encoder click
extern sv int16_t        rotationcount ;                    // Current position of rotary switch
extern sv uint16_t       enc_inactivity ;                   // Time inactive
extern sv bool           singleclick  ;                  // True if single click detected
extern sv bool           doubleclick  ;                  // True if double click detected
extern sv bool           tripleclick  ;                  // True if triple click detected
extern sv bool           longclick  ;                    // True if longclick detected
enum enc_menu_t { VOLUME, PRESET, TRACK } ;              // State for rotary encoder menu
extern enc_menu_t        enc_menu_mode ;               // Default is VOLUME mode

//
struct progpin_struct                                    // For programmable input pins
{
  int8_t         gpio ;                                  // Pin number
  bool           reserved ;                              // Reserved for connected devices
  bool           avail ;                                 // Pin is available for a command
  String         command ;                               // Command to execute when activated
  // Example: "uppreset=1"
  bool           cur ;                                   // Current state, true = HIGH, false = LOW
} ;

extern progpin_struct   progpin[] =                             // Input pins and programmed function
{
  {  0, false, false,  "", false },
  //{  1, true,  false,  "", false },                    // Reserved for TX Serial output
  {  2, false, false,  "", false },
  //{  3, true,  false,  "", false },                    // Reserved for RX Serial input
  {  4, false, false,  "", false },
  {  5, false, false,  "", false },
  //{  6, true,  false,  "", false },                    // Reserved for FLASH SCK
  //{  7, true,  false,  "", false },                    // Reserved for FLASH D0
  //{  8, true,  false,  "", false },                    // Reserved for FLASH D1
  //{  9, true,  false,  "", false },                    // Reserved for FLASH D2
  //{ 10, true,  false,  "", false },                    // Reserved for FLASH D3
  //{ 11, true,  false,  "", false },                    // Reserved for FLASH CMD
  { 12, false, false,  "", false },
  { 13, false, false,  "", false },
  { 14, false, false,  "", false },
  { 15, false, false,  "", false },
  { 16, false, false,  "", false },                      // May be UART 2 RX for Nextion
  { 17, false, false,  "", false },                      // May be UART 2 TX for Nextion
  { 18, false, false,  "", false },                      // Default for SPI CLK
  { 19, false, false,  "", false },                      // Default for SPI MISO
  //{ 20, true,  false,  "", false },                    // Not exposed on DEV board
  { 21, false, false,  "", false },                      // Also Wire SDA
  { 22, false, false,  "", false },                      // Also Wire SCL
  { 23, false, false,  "", false },                      // Default for SPI MOSI
  //{ 24, true,  false,  "", false },                    // Not exposed on DEV board
  { 25, false, false,  "", false },
  { 26, false, false,  "", false },
  { 27, false, false,  "", false },
  //{ 28, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 29, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 30, true,  false,  "", false },                    // Not exposed on DEV board
  //{ 31, true,  false,  "", false },                    // Not exposed on DEV board
  { 32, false, false,  "", false },
  { 33, false, false,  "", false },
  { 34, false, false,  "", false },                      // Note, no internal pull-up
  { 35, false, false,  "", false },                      // Note, no internal pull-up
  //{ 36, true,  false,  "", false },                    // Reserved for ADC battery level
  { 39, false,  false,  "", false },                     // Note, no internal pull-up
  { -1, false, false,  "", false }                       // End of list
} ;

struct touchpin_struct                                   // For programmable input pins
{
  int8_t         gpio ;                                  // Pin number GPIO
  bool           reserved ;                              // Reserved for connected devices
  bool           avail ;                                 // Pin is available for a command
  String         command ;                               // Command to execute when activated
  // Example: "uppreset=1"
  bool           cur ;                                   // Current state, true = HIGH, false = LOW
  int16_t        count ;                                 // Counter number of times low level
} ;
extern touchpin_struct   touchpin[] =                           // Touch pins and programmed function
{
  {   4, false, false, "", false, 0 },                   // TOUCH0
  {   0, true,  false, "", false, 0 },                   // TOUCH1, reserved for BOOT button
  {   2, false, false, "", false, 0 },                   // TOUCH2
  {  15, false, false, "", false, 0 },                   // TOUCH3
  {  13, false, false, "", false, 0 },                   // TOUCH4
  {  12, false, false, "", false, 0 },                   // TOUCH5
  {  14, false, false, "", false, 0 },                   // TOUCH6
  {  27, false, false, "", false, 0 },                   // TOUCH7
  {  33, false, false, "", false, 0 },                   // TOUCH8
  {  32, false, false, "", false, 0 },                   // TOUCH9
  {  -1, false, false, "", false, 0 }                    // End of list
  // End of table
} ;



//**************************************************************************************************
// End of global data section.                                                                     *
//**************************************************************************************************
