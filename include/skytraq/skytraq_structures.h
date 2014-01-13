// Skytraq Binary Data Structures
#ifndef SKYTRAQSTRUCTURES_H
#define SKYTRAQSTRUCTURES_H

#include "stdint.h"

namespace skytraq {

#define MAX_NOUT_SIZE      (5000)   // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
#define MAXCHAN		66  // Maximum number of signal channels + 1
#define MAX_SAT     33  // maximum number of prns+1

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

//! Header prepended to Skytraq binary messages
#define HEADER_LENGTH 4 //!< (includes "sync1 sync2 payload_length")
#define FOOTER_LENGTH 3 
#define SKYTRAQ_SYNC_BYTE_1 0xA0
#define SKYTRAQ_SYNC_BYTE_2 0xA1
#define SKYTRAQ_END_BYTE_1 0x0D
#define SKYTRAQ_END_BYTE_2 0x0A

//! I/O Message Payload Lengths
#define SYSTEM_RESTART_PAYLOAD_LENGTH 15 //!< [bytes]
#define QUERY_SOFTWARE_VERSION_PAYLOAD_LENGTH 2 //!< [bytes]
#define QUERY_SOFTWARE_CRC_VERSION_PAYLOAD_LENGTH 2 //!< [bytes]
#define CONFIGURE_SERIAL_PORT_PAYLOAD_LENGTH 4 //!< [bytes]
#define CONFIGURE_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH 3 //!< [bytes]
#define QUERY_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH 1 //!< [bytes]
#define CONFIGURE_WAAS_PAYLOAD_LENGTH 3 //!< [bytes]
#define QUERY_WAAS_PAYLOAD_LENGTH 1 //!< [bytes]
#define CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH 3 //!< [bytes]
#define QUERY_NAVIGATION_MODE_PAYLOAD_LENGTH 1 //!< [bytes]

#define CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH 3 //!< [bytes]
#define CONFIGURE_BINARY_OUTPUT_RATE_PAYLOAD_LENGTH 8 //!< [bytes]
#define GET_ALMANAC_PAYLOAD_LENGTH 2 //!< [bytes]
#define GET_EPHEMERIS_PAYLOAD_LENGTH 2 //!< [bytes]
#define SOFTWARE_VERSION_PAYLOAD_LENGTH 14 //!< [bytes]
#define SOFTWARE_CRC_PAYLOAD_LENGTH 4 //!< [bytes]
#define ACK_PAYLOAD_LENGTH 2; //!< [bytes]
#define NACK_PAYLOAD_LENGTH 2; //!< [bytes]
#define POSITION_UPDATE_RATE_PAYLOAD_LENGTH 2 //!< [bytes]
#define ALMANAC_PAYLOAD_LENGTH 28; //!< [bytes]
#define EPHEMERIS_PAYLOAD_LENGTH 87; //!< [bytes]
#define MEASUREMENT_TIME_PAYLOAD_LENGTH 10; //!< [bytes]
#define RECEIVER_NAV_STATUS_PAYLOAD_LENGTH 81 //!< [bytes]
#define SUBFRAME_BUFFER_DATA_PAYLOAD_LENGTH 33 //!< [bytes]


// Payload is transmitted in big endian
// Receiver sends an ACK or NACK to every input message

/*!
 * ------------------------------------------
 * Header Footer Structures
 * ------------------------------------------
 */
PACK(
    struct SkytraqHeader 
    {
        uint8_t sync1;   //!< start of packet first byte (0xB5)
        uint8_t sync2;   //!< start of packet second byte (0x62)
        uint16_t payload_length; //!< length of the payload data
    }
);

PACK(
    struct SkytraqFooter 
    {
        uint8_t checksum;
        uint8_t end1;
        uint8_t end2;
    }
);

/*!
 * ------------------------------------------
 * Common Enums
 * ------------------------------------------
 */
enum DisableEnable {
    DISABLE = 0,
    ENABLE = 1,
};
enum Attributes {
    UPDATE_TO_SRAM = 0,
    UPDATE_TO_SRAM_AND_FLASH = 01,
};

/*!
 * ------------------------------------------
 * Input System Messages
 * ------------------------------------------
 */

//! (0x01) System Restart
enum StartMode {
    NO_MODE_CHANGE = 0x00,
    HOT_START = 0x01,
    WARM_START = 0x02,
    COLD_START = 0x03,
    TEST_MODE = 0x04,
};
PACK(
    struct SystemRestart
    {
        SkytraqHeader header;
        uint8_t message_id;
        StartMode start_mode;
        uint16_t utc_year;      //!< >=1980
        uint8_t utc_month;      //!< 1-12
        uint8_t utc_day;        //!< 1-31
        uint8_t utc_hour;       //!< 0-23
        uint8_t utc_minute;     //!< 0-59
        uint8_t utc_second;     //!< 0-59
        int16_t latitude;       //!< -9000 to 9000 (>0 North Hem, <0 South Hem) [1/100 deg]
        int16_t longitude;      //!< -18000 to 18000 (>0 East Hem, <0 West Hem) [1/100 deg]
        int16_t altitude;       //!< -1000 to 18300 [m]
        SkytraqFooter footer;
    }
);

//! (0x02) Query Software Version
enum SoftwareType {
    RESERVED = 0x00,
    SYSTEM_CODE = 0x01,
};
PACK(
    struct QuerySoftwareVersion
    {
        SkytraqHeader header;
        uint8_t message_id;
        SoftwareType software_type;
        SkytraqFooter footer;
    }
);

//! (0x03) Query Software CRC
PACK(
    struct QuerySoftwareCrcVersion
    {
        SkytraqHeader header;
        uint8_t message_id;
        SoftwareType software_type;
        SkytraqFooter footer;
    }
);

//! (0x04) Set to Factory Defaults
PACK(
    struct RestoreFactoryDefaults
    {
        SkytraqHeader header;
        uint8_t message_id;
        uint8_t type;           //!< = 0x01
        SkytraqFooter footer;
    }
);

//! (0x05) Configure Serial Port
PACK(

    struct ConfigureSerialPort
    {
        SkytraqHeader header;
        uint8_t message_id;
        uint8_t com_port;
        uint8_t baudrate;
            //!< 4800, 9600, 19200, 38400, 57600, 115200
        Attributes attributes;
        SkytraqFooter footer;
    }
);

//! (0x09) Configure Output Message Format
enum OutputType {
    NO_OUTPUT = 0,
    NMEA_OUTPUT = 01,
    BINARY_OUTPUT = 02,
};
PACK(
    struct ConfigureOutputFormat
    {
        SkytraqHeader header;
        uint8_t message_id;     //!< Message ID
        OutputType type;
        Attributes attributes;
        SkytraqFooter footer;
    }
);

//! (0xE) Configure System Position Update Rate
PACK(
    struct ConfigurePositionUpdateRate
    {
        SkytraqHeader header;
        uint8_t message_id;
        uint8_t update_rate;
        Attributes attributes;
        SkytraqFooter footer;
    }
);

//! (0x10) Query Position Update Rate
PACK(
    struct QueryPositionUpdateRate
    {
        SkytraqHeader header;
        uint8_t message_id;
        SkytraqFooter footer;
    }
);

//! (0x12) Configure Binary Measurement Output Rate
enum BinaryOutputRate {
    ONE_HZ = 0,
    TWO_HZ = 1,
    FOUR_HZ = 2,
    FIVE_HZ = 3,
    TEN_HZ = 4,
    TWENTY_HZ = 5,
};
PACK(
    struct ConfigureBinaryOutputRate
    {
        SkytraqHeader header;
        uint8_t message_id;                     //!< Message ID
        BinaryOutputRate binary_output_rate;
        DisableEnable measurement_time;         //!< (0xDC)
        DisableEnable raw_measurements;         //!< (0xDD)
        DisableEnable sv_channel_status;        //!< (0xDE)
        DisableEnable receiver_state;           //!< (0xDF)
        DisableEnable subframe;                 //!< (0xEO)
        Attributes attributes;
        SkytraqFooter footer;
    }
);

/*!
 * ------------------------------------------
 * Input GPS Messages
 * ------------------------------------------
 */

//! (0x11) Get Almanac
PACK(
    struct GetAlmanac
    {
        SkytraqHeader header;
        uint8_t message_id;     //!< Message ID
        uint8_t prn;            //!< (0 = all SVs),(1-32 = specific SV)
        SkytraqFooter footer;
    }
);

//! (0x30) Get Ephemeris
PACK(
    struct GetEphemeris
    {
        SkytraqHeader header;
        uint8_t message_id;     //!< Message ID
        uint8_t prn;            //!< (0 = all SVs),(1-32 = specific SV)
        SkytraqFooter footer;
    }
);

//! (0x37) Configure WAAS
PACK(
    struct ConfigureWAAS
    {
        SkytraqHeader header;
        uint8_t message_id;
        DisableEnable enable_waas;
        Attributes attributes;
        SkytraqFooter footer;
    }
);

//! (0x38) Query WAAS Status
PACK(
    struct QueryWAAS
    {
        SkytraqHeader header;
        uint8_t message_id;
        SkytraqFooter footer;
    }
);

//! (0x3c) Configure Navigation Mode
enum NavMode {
    CAR = 0x00,
    PEDESTRIAN = 0x01,
};
PACK(
    struct ConfigureNavigationMode
    {
        SkytraqHeader header;
        uint8_t message_id;
        NavMode nav_mode;
        Attributes attributes;
        SkytraqFooter footer;
    }
);

//! (0x3d) Query Navigation Mode
PACK(
    struct QueryNavigationMode
    {
        SkytraqHeader header;
        uint8_t message_id;
        SkytraqFooter footer;
    }
);

/*!
 * ------------------------------------------
 * Output System Messages
 * ------------------------------------------
 */

 //! (0x80) Software Version
PACK(
    struct SoftwareVersion
    {
        SkytraqHeader header;
        uint8_t message_id;     //!< Message ID
        uint32_t kernel_version;
        uint32_t odm_version;
        uint32_t revision;
        SkytraqFooter footer;
    }
);

//! (0X81) Software CRC
PACK(
    struct SoftwareCRC
    {
        SkytraqHeader header;
        uint8_t message_id;     //!< Message ID
        uint8_t software_type;
        uint16_t crc_value;
        SkytraqFooter footer;
    }
);

//! (0x83) ACK
PACK(
    struct Ack
    {
        SkytraqHeader header;
        uint8_t message_id;
        SkytraqFooter footer;
    }
);

//! (0x84) NACK
PACK(
    struct Nack
    {
        SkytraqHeader header;
        uint8_t message_id;
        SkytraqFooter footer;
    }
);

//! (0x86) Position Update Rate
PACK(
    struct PositionUpdateRate
    {
        SkytraqHeader header;
        uint8_t message_id;
        uint8_t update_rate; //!< [Hz]
        SkytraqFooter footer;
    }
);

/*!
 * ------------------------------------------
 * Output GPS Messages
 * ------------------------------------------
 */

//! (0xB3) GPS WAAS Status
PACK(
    struct WaasStatus
    {
        SkytraqHeader header;
        uint8_t message_id;
        DisableEnable waas_status;
        SkytraqFooter footer;
    }
);

//! (0xB5) Navigation Mode
PACK(
    struct NavigationMode
    {
        SkytraqHeader header;
        uint8_t message_id;
        NavMode nav_mode;
        SkytraqFooter footer;
    }
);

//! (0x87) Almanac (polled message response)
PACK(
    struct Word
    {
        uint8_t byte[3];            //!< Bit 1-24 (MSB->LSB)
    }
);
PACK(
    struct Almanac
    {
        SkytraqHeader header;
        uint8_t message_id;         //!< Message ID
        uint8_t prn;
        Word word3;
        Word word4;
        Word word5;
        Word word6;
        Word word7;
        Word word8;
        Word word9;
        Word word10;
        int16_t issue_week;
        SkytraqFooter footer;
    }
);

//! (0xB1) Ephemeris (polled message response)
PACK(
    struct Subframe
    {
        Word word[9];       //!< (TLM Word is not included)
    }
);
PACK(
    struct Ephemeris
    {
        SkytraqHeader header;
        uint8_t message_id;         //!< Message ID
        uint16_t svid;
        uint8_t reserved;
        Subframe subframe1;
        Subframe subframe2;
        Subframe subframe3;
        SkytraqFooter footer;
    }
);

//! (0xDC) Measurement time info (periodic message)
    // Contains time information on when the raw GPS 
    // measurement is taken.
PACK(
    struct MeasurementTime
    {
        SkytraqHeader header;
        uint8_t message_id;         //!< Message ID
        uint8_t issue_of_data;      //!< (0-255)
        uint16_t week_number;       //!< (0-65535)
        uint32_t time_of_week;      //!< (0-604799999) [ms]
        uint16_t measurement_period;//!< (1-1000) [ms]
        SkytraqFooter footer;
    }
);

//! (0xDD) Raw Measurements (periodic message)
    // payload length = 3+23*N
PACK(
    struct ChannelMeasurements
    {
        uint8_t prn;
        uint8_t cno;                        //!< [dBHz]
        double pseudorange;                 //!< [m]
        double accumulated_carrier;         //!< L1 [cycles] 
            // (polarity convention: approaching SV = decreasing accumulated cariier)
        float doppler_frequency;            //!< [Hz]
        uint8_t channel_indicator;          //!< (Bit 0 = LSB)
            // Bit 0 ON = pseudorange available
            // Bit 1 ON = Doppler available
            // Bit 2 ON = Carrier phase available
            // Bit 3 ON = Cycle slip possible
            // Bit 4 ON = Coherent integration time >= 10 ms
    }
);
PACK(
    struct RawMeasurements
    {
        SkytraqHeader header;
        uint8_t message_id;                 //!< Message ID
        uint8_t issue_of_data;            //!< (0-255)
        uint8_t number_of_measurements;
        ChannelMeasurements channel_measurements[MAXCHAN];
        SkytraqFooter footer;
    }
);

//! (0xDE) SV and Channel status info (peiodic message)
PACK(
    struct ChannelInfo
    {
        uint8_t channel_id;                 //!< ID # (0-11)
        uint8_t prn;
        uint8_t sv_status_indicator;        //!< bit 0 ON: Almanac received (LSB)
                                            //!< bit 1 ON: Ephemeris received
                                            //!< bit 2 ON: SV is healthy
        uint8_t ura;                        //!< User Range Accuracy
        int8_t cno;                         //!< Carrier/Noise Ratio (dBHz)
        int16_t elevation;                  //!< SV elevation (deg)
        int16_t azimuth;                    //!< SV Azimuth (deg)
        uint8_t channel_status_indicator;   //!< bit 0 ON: Pull in stage is done (LSB)
                                            //!< bit 1 ON: Bit synch is done
                                            //!< bit 2 ON: Frame sync is done
                                            //!< bit 3 ON: Ephemeris received
                                            //!< bit 4 ON: Used in normal fix mode
                                            //!< bit 5 ON: Used in differential fix mode
    }
);

PACK(
    struct ChannelStatus
    {
        SkytraqHeader header;
        uint8_t message_id;                 //!< Message ID
        uint8_t issue_of_data;              //!< (0-255)
        uint8_t number_of_svs;              
        ChannelInfo channel_info[MAXCHAN];  
        SkytraqFooter footer;
    }
);


//! (0xDF) Receiver Navigation Statues (periodic message)
enum NavState
{
    NO_FIX = 0x00,
    FIX_PREDICTION = 0x01,
    FIX_2D = 0x02,
    FIX_3D = 0x03,
    FIX_DIFFERENTIAL = 0x04,
};

PACK(
    struct ReceiverNavStatus
    {
        SkytraqHeader header;
        uint8_t message_id;                 //!< Message ID
        uint8_t issue_of_data;              //!< (0-255)
        NavState nav_state;
        uint16_t week_number;               //!< [weeks]
        double time_of_week;                //!< [sec]
        double ecef_x;                      //!< [m]
        double ecef_y;                      //!< [m]
        double ecef_z;                      //!< [m]
        float ecef_vel_x;                   //!< [m/s]
        float ecef_vel_y;                   //!< [m/s]
        float ecef_vel_z;                   //!< [m/s]
        double receiver_clock_bias;         //!< [m]
        float receiver_clock_drift;         //!< [m/s]
        float gdop;
        float pdop;
        float hdop;
        float vdop;
        float tdop;
        SkytraqFooter footer;
    }
);

//! (0xE0) Subframe Buffer Data (periodic message)
PACK(
    struct SubframeBufferData
    {
        SkytraqHeader header;
        uint8_t message_id;                 //!< Message ID
        uint8_t prn;
        uint8_t subframe_id;                //!< (1-5)
        Word word1;
        Word word2;
        Word word3;
        Word word4;
        Word word5;
        Word word6;
        Word word7;
        Word word8;
        Word word9;
        Word word10;
        SkytraqFooter footer;
    }
);

//! Skytraq Protocol Class/Message ID's
enum Message_ID
{
    //! Input System Messages
    SYSTEM_RESTART = 1,         //!< (0x01) Force system restart
    QUERY_SOFTWARE_VERSION = 2, //!< (0x02) Query revision information about software
    QUERY_SOFTWARE_CRC = 3,     //!< (0x03) Query software CRC
    SET_FACTORY_DEFAULTS = 4,   //!< (0X04) Set system to factory default values
    CFG_SERIAL_PORT = 5,        //!< (0x05) Configure serial port
    CFG_NMEA = 8,               //!< (0x08) Configure NMEA output message
    CFG_OUTPUT_FORMAT = 9,      //!< (0x09) Configure Output Message Format
    CFG_POWER_MODE = 12,        //!< (0x0C) Set the system power mode
    CFG_POS_UPDATE_RATE = 14,   //!< (0x0E) Configure the position update rate
    QUERY_POS_UPDATE_RATE = 16, //!< (0X10) Query the position update rate
    CFG_OUTPUT_RATE = 18,       //!< (0x12) Configure Binary Measurement Output Rates
    //! Input GPS Messages
    GET_ALMANAC = 17,           //!< (0x11) Retrieve almanac data from receiver
    GET_EPHEMERIS = 48,         //!< (0x30) Retrieve ephemeris data from receiver
    SET_EPHEMERIS = 49,         //!< (0x31) Set ephemeris data to receiver
    CFG_WAAS = 55,              //!< (0x37) Configure the enable/disable of WAAS
    QUERY_WAAS = 56,            //!< (0x38) Query WAAS status
    CFG_NAV_MODE = 60,          //!< (0x3C) Configure Navigation mode of GPS
    QUERY_NAV_MODE = 61,        //!< (0x3D) Query navigation mode
    //! Output System Messages
    SOFTWARE_VERSION = 128,     //!< (0x80) Software version of the receiver
    SOFTWARE_CRC = 129,         //!< (0x81) Software CRC of the receiver
    ACK = 131,                  //!< (0x83) ACK to successful input message
    NACK = 132,                 //!< (0x84) NACK to unsuccessful input message
    POS_UPDATE_RATE = 134,      //!< (0X86) Position update rate of GPS system
    //! Output GPS Messages
    GPS_WAAS_STATUS = 179,      //!< (0xB3) GPS WAAS status
    GPS_NAV_MODE = 180,         //!< (0xB5) GPS Navigation mode
    GPS_ALMANAC = 134,          //!< (0x87) GPS Almanac data
    GPS_EPHEMERIS = 177,        //!< (0xB1) GPS Ephemeris data
    MEAS_TIME = 220,            //!< (0xDC) Measurement time information
    RAW_MEAS = 221,             //!< (0xDD) Raw Channel measurements
    SV_CH_STATUS = 222,         //!< (0xDE) SV and channel status information
    RCV_STATE = 223,            //!< (0xDF) GPS receiver navigation state
    SUBFRAME = 224,             //!< (0xE0) Subframe buffer data
};

} // end namespace
#endif
