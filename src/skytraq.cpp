#include "skytraq/skytraq.h"
#include <iostream>

using namespace std;
using namespace skytraq;

/////////////////////////////////////////////////////
// includes for default time callback
//#define WIN32_LEAN_AND_MEAN
#define PI 3.14159265
//#include "boost/date_time/posix_time/posix_time.hpp"
////////////////////////////////////////////////////

inline void printHex(char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}

inline void printHex(unsigned char *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", data[i]);
    }
    printf("\n");
}

/*!
 * Default callback method for time stamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
inline double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

//! Logging Default Callbacks
inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cout << "Skytraq Debug: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cout << "Skytraq Info: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string &msg) {
    std::cout << "Skytraq Warning: " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cout << "Skytraq Error: " << msg << std::endl;
}


//! System Output Message Default Callbacks
inline void DefaultSoftwareVersionCallback(skytraq::SoftwareVersion& software_version, double& timestamp){
    std::cout << "Software Version message received. " << std::endl;
}

inline void DefaultSoftwareCrcCallback(skytraq::SoftwareCRC& software_crc, double& timestamp){
    std::cout << "Software CRC message received. " << std::endl;
}

inline void DefaultAckCallback(skytraq::Ack& ack, double& timestamp){
    std::cout << "Ack message received. " << std::endl;
}

inline void DefaultNackCallback(skytraq::Nack& nack, double& timestamp){
    std::cout << "Nack message received. " << std::endl;
}

inline void DefaultPosUpdateRateCallback(skytraq::PositionUpdateRate& pos_update_rate, double& timestamp){
    std::cout << "Position Update Rate message received. " << std::endl;
}


//! GPS Output Message Default Callbacks
inline void DefaultWaasStatusCallback(skytraq::WaasStatus& waas_status, double& timestamp){
    std::cout << "WAAS Status message received. " << std::endl;
}

inline void DefaultNavigationModeCallback(skytraq::NavigationMode& nav_mode, double& timestamp){
    std::cout << "Navigation Mode message received. " << std::endl;
}

inline void DefaultAlmanacCallback(skytraq::Almanac& almanac, double& timestamp){
    std::cout << "Almanac message received. " << std::endl;
}

inline void DefaultEphemerisCallback(skytraq::Ephemeris& ephemeris, double& timestamp){
    std::cout << "Ephemeris message received. " << std::endl;
}

inline void DefaultMeasurementTimeCallback(skytraq::MeasurementTime& measurement_time, double& timestamp){
    std::cout << "Measurement Time message received. " << std::endl;
}

inline void DefaultRawMeasurementCallback(skytraq::RawMeasurements& raw_measurements, double& timestamp){
    std::cout << "Raw Measurements message received. " << std::endl;
}

inline void DefaultChannelStatusCallback(skytraq::ChannelStatus& channel_status, double& timestamp){
    std::cout << "Channel Status message received. " << std::endl;
}

inline void DefaultNavStatusCallback(skytraq::ReceiverNavStatus& nav_status, double& timestamp){
    std::cout << "Nav Status message received. " << std::endl;
}

inline void DefaultSubframeBufferDataCallback(skytraq::SubframeBufferData& subframe_buffer_data, double& timestamp){
    std::cout << "Subframe Buffer Data message received. " << std::endl;
}


Skytraq::Skytraq() {
    serial_port_ = NULL;
    reading_status_ = false;
    time_handler_ = DefaultGetTime;
    //! Logging Callbacks
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    //! System Output Message Callbacks
    software_version_callback_ = DefaultSoftwareVersionCallback;
    software_crc_callback_ = DefaultSoftwareCrcCallback;
    ack_callback_ = DefaultAckCallback;
    nack_callback_ = DefaultNackCallback;
    pos_update_rate_callback_ = DefaultPosUpdateRateCallback;
    //! GPS Output Message Callbacks
    waas_status_callback_ = DefaultWaasStatusCallback;
    nav_mode_callback_ = DefaultNavigationModeCallback;
    almanac_callback_ = DefaultAlmanacCallback;
    ephemeris_callback_ = DefaultEphemerisCallback;
    measurement_time_callback_ = DefaultMeasurementTimeCallback;
    raw_measurement_callback_ = DefaultRawMeasurementCallback;
    channel_status_callback_ = DefaultChannelStatusCallback;
    receiver_nav_status_callback_ = DefaultNavStatusCallback;
    subframe_buffer_data_callback_ = DefaultSubframeBufferDataCallback;
    reading_acknowledgement_ = false;
    bytes_remaining_ = false;
    header_length_ = 0;
    msgID = 0;
    data_read_ = NULL;
    buffer_index_ = 0;
    read_timestamp_ = 0;
    parse_timestamp_ = 0;
    is_connected_ = false;
}

Skytraq::~Skytraq() {
    Disconnect();
}

bool Skytraq::Connect(std::string port, int baudrate) {
    //serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
    try {
        serial::Timeout my_timeout(100, 1000, 0, 1000, 0);
    
        serial_port_ = new serial::Serial(port, baudrate, my_timeout);


		if (!serial_port_->isOpen()) {
			std::stringstream output;
			output << "Serial port: " << port << " failed to open.";
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
			return false;
		} else {
			std::stringstream output;
			output << "Serial port: " << port << " opened successfully.";
			log_info_(output.str());
		}

		//std::cout << "Flushing port" << std::endl;
		serial_port_->flush();

		// look for GPS by sending ping and waiting for response
        if (false) {
//		if (!Ping()) {
			std::stringstream output;
			output << "Skytraq GPS not found on port: " << port << std::endl;
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
			return false;
		}

        // start reading
        cout << "Before entering StartReading().";
        StartReading();
        is_connected_ = true;

    } catch (std::exception e) {
           std::stringstream output;
           output << "Failed to open port " << port << "  Err: " << e.what();
           log_error_(output.str());
           serial_port_ = NULL;
           is_connected_ = false;
           return false;
    }
    return true;
}

bool Skytraq::Ping(int num_attempts) {
    try {
        while ((num_attempts--) > 0) {
            log_info_("Searching for Skytraq receiver...");
            // request version information
            QuerySoftwareVersion();

            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

            unsigned char result[5000];
            size_t bytes_read;
            bytes_read = serial_port_->read(result, 5000);

            std::cout << "bytes read: " << (int)bytes_read << std::endl;
            printHex(result,bytes_read);

            if (bytes_read < (HEADER_LENGTH+SOFTWARE_VERSION_PAYLOAD_LENGTH+FOOTER_LENGTH)) {
                stringstream output;
                output << "Only read " << bytes_read
                        << " bytes in response to ping.";
                log_debug_(output.str());
                continue;
            }

            // search through result for version message
            size_t ack_message_length = HEADER_LENGTH+ACK_PAYLOAD_LENGTH+FOOTER_LENGTH;
            for (int ii = 0; ii < (bytes_read-ack_message_length); ii++) {
                //std::cout << hex << (unsigned int)result[ii] << std::endl;
                if (result[ii] == SKYTRAQ_SYNC_BYTE_1) {
                    // NOTE: FIX != to ==
                    if (result[ii + 1] == SKYTRAQ_SYNC_BYTE_2)
                        continue;
                    if (result[ii + HEADER_LENGTH] == SOFTWARE_CRC)
                        continue;
                    if (result[ii+ack_message_length-2] == SKYTRAQ_END_BYTE_1)
                        continue;
                    if (result[ii+ack_message_length-1] == SKYTRAQ_END_BYTE_2)
                        continue;
                    log_info_("Skytraq receiver found.");
                    // TODO: add version parsing and output
                    return true;
                }
            }
            stringstream output;
            output << "Read " << bytes_read
               << " bytes, but version message not found.";
            log_debug_(output.str());
        }
    } catch (exception &e) {
        std::stringstream output1;
        output1 << "Error pinging receiver: " << e.what();
        log_error_(output1.str());
        return false;
    }
    return false;
}

void Skytraq::Disconnect() {
	try {
		if (reading_status_) {
			StopReading();
			// TODO: wait here for reading to stop
		}
		if (serial_port_ != NULL) {
			if (serial_port_->isOpen())
				serial_port_->close();
			delete serial_port_;
			serial_port_ = NULL;
		}
	} catch (std::exception &e) {
		std::stringstream output;
				output << "Error disconnecting from Skytraq: " << e.what();
				log_error_(output.str());
	}
}

void Skytraq::StartReading() {
	try {
        cout << "In StartReading()." << endl;
		// create thread to read from sensor
		reading_status_ = true;
		read_thread_ptr_ = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&Skytraq::ReadSerialPort, this)));
        cout << "End of StartReading()" << endl;
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error starting Skytraq read thread: " << e.what();
		log_error_(output.str());
	}
}

void Skytraq::StopReading() {
    reading_status_ = false;
}

void Skytraq::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        // read data
        try {
            len = serial_port_->read(buffer, MAX_NOUT_SIZE);
            // timestamp the read
            read_timestamp_ = time_handler_();
            // add data to the buffer to be parsed
            BufferIncomingData(buffer, len);
        } catch (exception &e) {
            stringstream output;
            output << "Error reading serial port: " << e.what();
            log_info_(output.str());
            Disconnect();
            return;
        }
    }
    return;
}

// Send Message
bool Skytraq::SendMessage(uint8_t* msg_ptr, size_t length)
{
    try {
        stringstream output1;
        uint8_t message_id;
        //printHex((char*) msg_ptr, length);
        size_t bytes_written;

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
            bytes_written=serial_port_->write(msg_ptr, length);
        } else {
            log_error_("Unable to send message. Serial port not open.");
            return false;
        }
        // check that full message was sent to serial port
        if (bytes_written != length) {
            log_error_("Full message was not sent over serial port.");
            output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
            log_error_(output1.str());
            return false;
        }
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SendMessage(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////
// System Input Message Methods
/////////////////////////////////////////////////////////////////////////////
bool Skytraq::RestartReceiver(skytraq::StartMode start_mode, uint16_t utc_year,
                            uint8_t utc_month, uint8_t utc_day, uint8_t utc_hour, 
                            uint8_t utc_minute, uint8_t utc_second, int16_t latitude,
                            int16_t longitude, int16_t altitude)
{
    bool send_result;
    try {
        SystemRestart restart_msg;
        restart_msg.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        restart_msg.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        restart_msg.header.payload_length = SYSTEM_RESTART_PAYLOAD_LENGTH;
        restart_msg.message_id = SYSTEM_RESTART;
        restart_msg.start_mode = start_mode;
        restart_msg.utc_year = utc_year;        //!< >=1980
        restart_msg.utc_month = utc_month;      //!< 1-12
        restart_msg.utc_day = utc_day;          //!< 1-31
        restart_msg.utc_hour = utc_hour;        //!< 0-23
        restart_msg.utc_minute = utc_minute;    //!< 0-59
        restart_msg.utc_second = utc_second;    //!< 0-59
        restart_msg.latitude = latitude;        //!< -9000 to 9000 (>0 North Hem, <0 South Hem) [1/100 deg]
        restart_msg.longitude = longitude;      //!< -18000 to 18000 (>0 East Hem, <0 West Hem) [1/100 deg]
        restart_msg.altitude = altitude;        //!< -1000 to 18300 [m]
        restart_msg.footer.end1 = SKYTRAQ_END_BYTE_1;
        restart_msg.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&restart_msg;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, SYSTEM_RESTART_PAYLOAD_LENGTH,
                          &restart_msg.footer.checksum);
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+SYSTEM_RESTART_PAYLOAD_LENGTH+FOOTER_LENGTH);

    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::RestartReceiver(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

void Skytraq::HotRestartReceiver(uint16_t utc_year, uint8_t utc_month, uint8_t utc_day, 
                                uint8_t utc_hour, uint8_t utc_minute, uint8_t utc_second,
                                int16_t latitude, int16_t longitude, int16_t altitude) 
{
    try {
        RestartReceiver(HOT_START, utc_year, utc_month, utc_day, utc_hour, utc_minute, 
                        utc_second, latitude, longitude, altitude);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::HotRestartReceiver(): " << e.what();
        log_error_(output.str());
        return;
    }
    return;
}
void Skytraq::WarmRestartReceiver()
{
    try {
        RestartReceiver(WARM_START);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::WarmRestartReceiver(): " << e.what();
        log_error_(output.str());
        return;
    }
    return;
}
void Skytraq::ColdRestartReceiver()
{
    try {
        RestartReceiver(COLD_START);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ColdRestartReceiver(): " << e.what();
        log_error_(output.str());
        return;
    }
    return;
}

bool Skytraq::QuerySoftwareVersion() {
    bool send_result;
    try {
        skytraq::QuerySoftwareVersion query_software_version;
        query_software_version.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        query_software_version.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        query_software_version.header.payload_length = QUERY_SOFTWARE_VERSION_PAYLOAD_LENGTH;
        query_software_version.message_id = QUERY_SOFTWARE_VERSION;
        query_software_version.software_type = SYSTEM_CODE;
        query_software_version.footer.end1 = SKYTRAQ_END_BYTE_1;
        query_software_version.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&query_software_version;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, QUERY_SOFTWARE_VERSION_PAYLOAD_LENGTH,
                          &query_software_version.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+SYSTEM_RESTART_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::QuerySoftwareVersion(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::QuerySoftwareCrcVersion() {
    bool send_result;
    try {
        skytraq::QuerySoftwareCrcVersion query_software_crc_version;
        query_software_crc_version.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        query_software_crc_version.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        query_software_crc_version.header.payload_length = QUERY_SOFTWARE_CRC_VERSION_PAYLOAD_LENGTH;
        query_software_crc_version.message_id = QUERY_SOFTWARE_CRC;
        query_software_crc_version.software_type = SYSTEM_CODE;
        query_software_crc_version.footer.end1 = SKYTRAQ_END_BYTE_1;
        query_software_crc_version.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&query_software_crc_version;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, QUERY_SOFTWARE_CRC_VERSION_PAYLOAD_LENGTH,
                          &query_software_crc_version.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+SYSTEM_RESTART_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::QuerySoftwareCrcVersion(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::RestoreFactoryDefaults() 
{
    bool send_result;
    try {
        skytraq::RestoreFactoryDefaults restore_factory_defaults;
        restore_factory_defaults.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        restore_factory_defaults.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        restore_factory_defaults.header.payload_length = RESTORE_FACTORY_DEFAULTS_PAYLOAD_LENGTH;
        restore_factory_defaults.message_id = SET_FACTORY_DEFAULTS;
        restore_factory_defaults.type = 0x01;
        restore_factory_defaults.footer.end1 = SKYTRAQ_END_BYTE_1;
        restore_factory_defaults.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&restore_factory_defaults;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, RESTORE_FACTORY_DEFAULTS_PAYLOAD_LENGTH,
                          &restore_factory_defaults.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+RESTORE_FACTORY_DEFAULTS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::RestoreFactoryDefaults(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::ConfigureSerialPort(uint8_t com_port, uint8_t baudrate) 
{
    // NOTE: Probably need to reconnect to serial port after changing baudrate
    bool send_result;
    try {
        skytraq::ConfigureSerialPort configure_serial_port;
        configure_serial_port.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_serial_port.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_serial_port.header.payload_length = CONFIGURE_SERIAL_PORT_PAYLOAD_LENGTH;
        configure_serial_port.message_id = SET_FACTORY_DEFAULTS;
        configure_serial_port.com_port = com_port;
        if(baudrate == 4800) {
            configure_serial_port.baudrate = 0;
        } else if(baudrate == 9600) {
            configure_serial_port.baudrate = 1;
        } else if(baudrate == 19200) {
            configure_serial_port.baudrate = 2;
        } else if(baudrate == 38400) {
            configure_serial_port.baudrate = 3;
        } else if(baudrate == 57600) {
            configure_serial_port.baudrate = 4;
        } else if(baudrate == 115200) {
            configure_serial_port.baudrate = 5;
        } else { // else baudrate not valid option, default to 115200
            configure_serial_port.baudrate = 5;
            log_info_("Invalid baudrate given. Defaulting to 115200.");
        }
        configure_serial_port.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_serial_port.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_serial_port.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_serial_port;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_SERIAL_PORT_PAYLOAD_LENGTH,
                          &configure_serial_port.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_SERIAL_PORT_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ConfigureSerialPort(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}


bool Skytraq::ConfigureNmeaIntervals(uint8_t gga_interval, uint8_t gsa_interval, 
                                    uint8_t gsv_interval, uint8_t gll_interval, 
                                    uint8_t rcm_interval, uint8_t vtg_interval, 
                                    uint8_t zda_interval) 
{
    bool send_result;
    try {
        skytraq::ConfigureNmeaIntervals configure_nmea_intervals;
        configure_nmea_intervals.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_nmea_intervals.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_nmea_intervals.header.payload_length = CONFIGURE_NMEA_INTERVALS_PAYLOAD_LENGTH;
        configure_nmea_intervals.message_id = CFG_NMEA;
        configure_nmea_intervals.gga_interval = gga_interval;   //!< [sec]
        configure_nmea_intervals.gsa_interval = gsa_interval;   //!< [sec]
        configure_nmea_intervals.gsv_interval = gsv_interval;   //!< [sec]
        configure_nmea_intervals.gll_interval = gll_interval;   //!< [sec]
        configure_nmea_intervals.rcm_interval = rcm_interval;   //!< [sec]
        configure_nmea_intervals.vtg_interval = vtg_interval;   //!< [sec]
        configure_nmea_intervals.zda_interval = zda_interval;   //!< [sec]
        configure_nmea_intervals.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_nmea_intervals.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_nmea_intervals.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_nmea_intervals;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_NMEA_INTERVALS_PAYLOAD_LENGTH,
                          &configure_nmea_intervals.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_NMEA_INTERVALS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ConfigureNmeaIntervals(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::ConfigureOutputFormat(OutputType output_type) 
{
    bool send_result;
    try {
        skytraq::ConfigureOutputFormat configure_output_format;
        configure_output_format.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_output_format.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_output_format.header.payload_length = CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH;
        configure_output_format.message_id = CFG_OUTPUT_FORMAT;
        configure_output_format.type = output_type;
        configure_output_format.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_output_format.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_output_format.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_output_format;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH,
                          &configure_output_format.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ConfigureOutputFormat(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::SetOutputFormatToBinary()
{
    bool send_result;
    try {
        send_result = ConfigureOutputFormat(BINARY_OUTPUT);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetOutputFormatToBinary(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::SetOutputFormatToNmea()
{
    bool send_result;
    try {
        send_result = ConfigureOutputFormat(NMEA_OUTPUT);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetOutputFormatToNmea(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::DisableAllOutput()
{
    bool send_result;
    try {
        send_result = ConfigureOutputFormat(NO_OUTPUT);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::DisableAllOutput(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::EnablePowerSaveMode() 
{
    bool send_result;
    try {
        skytraq::ConfigurePowerSaveMode configure_power_save_mode;
        configure_power_save_mode.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_power_save_mode.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_power_save_mode.header.payload_length = CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH;
        configure_power_save_mode.message_id = CFG_POWER_MODE;
        configure_power_save_mode.power_save_mode = skytraq::ENABLE;
        configure_power_save_mode.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_power_save_mode.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_power_save_mode.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_power_save_mode;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH,
                          &configure_power_save_mode.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::EnablePowerSaveMode(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::DisablePowerSaveMode() 
{
    bool send_result;
    try {
        skytraq::ConfigurePowerSaveMode configure_power_save_mode;
        configure_power_save_mode.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_power_save_mode.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_power_save_mode.header.payload_length = CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH;
        configure_power_save_mode.message_id = CFG_POWER_MODE;
        configure_power_save_mode.power_save_mode = skytraq::DISABLE;
        configure_power_save_mode.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_power_save_mode.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_power_save_mode.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_power_save_mode;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH,
                          &configure_power_save_mode.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_POWER_SAVE_MODE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::DisablePowerSaveMode(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::ConfigurePositionUpdateRate(uint8_t update_rate) 
{
    bool send_result;
    try {
        skytraq::ConfigurePositionUpdateRate configure_position_update_rate;
        configure_position_update_rate.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_position_update_rate.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_position_update_rate.header.payload_length = CONFIGURE_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH;
        configure_position_update_rate.message_id = CFG_POS_UPDATE_RATE;
        configure_position_update_rate.update_rate = update_rate;
        configure_position_update_rate.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_position_update_rate.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_position_update_rate.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_position_update_rate;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH,
                          &configure_position_update_rate.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ConfigurePositionUpdateRate(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::QueryPositionUpdateRate() 
{
    bool send_result;
    try {
        skytraq::QueryPositionUpdateRate query_position_update_rate;
        query_position_update_rate.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        query_position_update_rate.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        query_position_update_rate.header.payload_length = QUERY_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH;
        query_position_update_rate.message_id = QUERY_POS_UPDATE_RATE;
        query_position_update_rate.footer.end1 = SKYTRAQ_END_BYTE_1;
        query_position_update_rate.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&query_position_update_rate;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, QUERY_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH,
                          &query_position_update_rate.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+QUERY_SYSTEM_POSITION_RATE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::QueryPositionUpdateRate(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

// (CFG-MSG) Set message output rate for specified message
bool Skytraq::ConfigureMessagesOutputRate(skytraq::BinaryOutputRate rate, 
                                        skytraq::DisableEnable meas_time_message, 
                                        skytraq::DisableEnable raw_meas_message, 
                                        skytraq::DisableEnable channel_status_message, 
                                        skytraq::DisableEnable receiver_state_message, 
                                        skytraq::DisableEnable subframe_buffer_message) {
    bool send_result;
    try {
        skytraq::ConfigureBinaryOutputRate message;
        message.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        message.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        message.header.payload_length = CONFIGURE_BINARY_OUTPUT_RATE_PAYLOAD_LENGTH;
        message.message_id = CFG_OUTPUT_RATE;
        message.binary_output_rate = rate;
        message.measurement_time = meas_time_message;               //!< (0xDC)
        message.raw_measurements = raw_meas_message;                //!< (0xDD)
        message.sv_channel_status = channel_status_message;         //!< (0xDE)
        message.receiver_state = receiver_state_message;            //!< (0xDF)
        message.subframe = subframe_buffer_message;                 //!< (0xEO)
        message.attributes = UPDATE_TO_SRAM_AND_FLASH;

        unsigned char* msg_ptr = (unsigned char*) &message;
        calculateCheckSum(msg_ptr + HEADER_LENGTH, 
                          CONFIGURE_BINARY_OUTPUT_RATE_PAYLOAD_LENGTH, 
                          &message.footer.checksum);

        message.footer.end1 = SKYTRAQ_END_BYTE_1;
        message.footer.end2 = SKYTRAQ_END_BYTE_2;

        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_BINARY_OUTPUT_RATE_PAYLOAD_LENGTH
                            +FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::ConfigureMessagesOutputRate(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

//////////////////////////////////////////////////////////////////////////////
//! GPS Input Message Methods
/////////////////////////////////////////////////////////////////////////////

bool Skytraq::PollAlmanac(uint8_t prn) 
{
    bool send_result;
    try {
        if((prn<0)||(prn>32)) {
            log_error_("Error in PollAlmanac(): Input prn outside of acceptable range.");
            return false;
        }
        skytraq::GetAlmanac get_almanac;
        get_almanac.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        get_almanac.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        get_almanac.header.payload_length = GET_ALMANAC_PAYLOAD_LENGTH;
        get_almanac.message_id = GET_ALMANAC;
        get_almanac.prn = prn;
        get_almanac.footer.end1 = SKYTRAQ_END_BYTE_1;
        get_almanac.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&get_almanac;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, GET_ALMANAC_PAYLOAD_LENGTH,
                          &get_almanac.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+GET_ALMANAC_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::PollAlmanac(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::PollEphemeris(uint8_t prn)
{
    bool send_result;
    try {
        if((prn<0)||(prn>32)) {
            log_error_("Error in PollEphemeris(): Input prn outside of acceptable range.");
            return false;
        }
        skytraq::GetEphemeris get_ephemeris;
        get_ephemeris.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        get_ephemeris.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        get_ephemeris.header.payload_length = GET_EPHEMERIS_PAYLOAD_LENGTH;
        get_ephemeris.message_id = skytraq::GET_EPHEMERIS;
        get_ephemeris.prn = prn;
        get_ephemeris.footer.end1 = SKYTRAQ_END_BYTE_1;
        get_ephemeris.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&get_ephemeris;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, GET_EPHEMERIS_PAYLOAD_LENGTH,
                          &get_ephemeris.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+GET_EPHEMERIS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::PollEphemeris(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::SetEphemeris(skytraq::SetEphemeris set_ephemeris)
{
    bool send_result;
    try {
        uint8_t* msg_ptr = (uint8_t*)&set_ephemeris;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, EPHEMERIS_PAYLOAD_LENGTH, &set_ephemeris.footer.checksum);
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+EPHEMERIS_PAYLOAD_LENGTH+FOOTER_LENGTH);

    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetEphemeris(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}
bool Skytraq::SetEphemeris(uint16_t svid, skytraq::Subframe subframe1, 
                            skytraq::Subframe subframe2, skytraq::Subframe subframe3)
{
    bool send_result;
    try {
        if((svid<0)||(svid>32)) {
            log_error_("Error in SetEphemeris(): Input prn outside of acceptable range.");
            return false;
        }
        skytraq::SetEphemeris set_ephemeris;
        set_ephemeris.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        set_ephemeris.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        set_ephemeris.header.payload_length = EPHEMERIS_PAYLOAD_LENGTH;
        set_ephemeris.message_id = skytraq::SET_EPHEMERIS;
        set_ephemeris.svid = svid;
        set_ephemeris.subframe1 = subframe1;
        set_ephemeris.subframe2 = subframe2;
        set_ephemeris.subframe3 = subframe3;
        set_ephemeris.footer.end1 = SKYTRAQ_END_BYTE_1;
        set_ephemeris.footer.end2 = SKYTRAQ_END_BYTE_2;

        send_result = SetEphemeris(set_ephemeris);

    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetEphemeris(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

// Send AID-EPH to Receiver
bool Skytraq::SetEphemerides(skytraq::Ephemerides ephemerides)
{
    try {
        bool sent_ephem = false;
        for (uint8_t prn_index = 1; prn_index <= 32; prn_index++) {
            if (ephemerides.ephemeris[prn_index].header.payload_length == GET_EPHEMERIS_PAYLOAD_LENGTH) {
                sent_ephem = SetEphemeris(ephemerides.ephemeris[prn_index]);
                if(sent_ephem == true) { // sent ephemeris for this prn

                } else { // else didn't send ephemeris for this prn

                }
            } else { // not a full ephemeris message
                
            }
        }
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetEphemerides(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return true;
}

bool Skytraq::EnableWAAS()
{
    bool send_result;
    try {
        skytraq::ConfigureWAAS config_waas;
        config_waas.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        config_waas.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        config_waas.header.payload_length = CONFIGURE_WAAS_PAYLOAD_LENGTH;
        config_waas.message_id = CFG_WAAS;
        config_waas.enable_waas = ENABLE;
        config_waas.attributes = UPDATE_TO_SRAM_AND_FLASH;
        config_waas.footer.end1 = SKYTRAQ_END_BYTE_1;
        config_waas.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&config_waas;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_WAAS_PAYLOAD_LENGTH,
                          &config_waas.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_WAAS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::EnableWAAS(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::DisableWAAS()
{
    bool send_result;
    try {
        skytraq::ConfigureWAAS config_waas;
        config_waas.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        config_waas.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        config_waas.header.payload_length = CONFIGURE_WAAS_PAYLOAD_LENGTH;
        config_waas.message_id = CFG_WAAS;
        config_waas.enable_waas = DISABLE;
        config_waas.attributes = UPDATE_TO_SRAM_AND_FLASH;
        config_waas.footer.end1 = SKYTRAQ_END_BYTE_1;
        config_waas.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&config_waas;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_WAAS_PAYLOAD_LENGTH,
                          &config_waas.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_WAAS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::DisableWAAS(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::QueryWAASConfiguration()
{
    bool send_result;
    try {
        skytraq::QueryWAAS query_waas;
        query_waas.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        query_waas.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        query_waas.header.payload_length = QUERY_WAAS_PAYLOAD_LENGTH;
        query_waas.message_id = QUERY_WAAS;
        query_waas.footer.end1 = SKYTRAQ_END_BYTE_1;
        query_waas.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&query_waas;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, QUERY_WAAS_PAYLOAD_LENGTH,
                          &query_waas.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+QUERY_WAAS_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::QueryWAASConfiguration(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::SetCarNavigationMode()
{
    bool send_result;
    try {
        skytraq::ConfigureNavigationMode configure_nav_mode;
        configure_nav_mode.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_nav_mode.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_nav_mode.header.payload_length = CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH;
        configure_nav_mode.message_id = CFG_NAV_MODE;
        configure_nav_mode.nav_mode = CAR;
        configure_nav_mode.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_nav_mode.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_nav_mode.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_nav_mode;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH,
                          &configure_nav_mode.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetCarNavigationMode(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::SetPedestrianNavigationMode()
{
    bool send_result;
    try {
        skytraq::ConfigureNavigationMode configure_nav_mode;
        configure_nav_mode.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        configure_nav_mode.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        configure_nav_mode.header.payload_length = CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH;
        configure_nav_mode.message_id = CFG_NAV_MODE;
        configure_nav_mode.nav_mode = PEDESTRIAN;
        configure_nav_mode.attributes = UPDATE_TO_SRAM_AND_FLASH;
        configure_nav_mode.footer.end1 = SKYTRAQ_END_BYTE_1;
        configure_nav_mode.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&configure_nav_mode;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH,
                          &configure_nav_mode.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+CONFIGURE_NAVIGATION_MODE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::SetCarNavigationMode(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}

bool Skytraq::QueryNavigationMode()
{
    bool send_result;
    try {
        skytraq::QueryNavigationMode query_nav_mode;
        query_nav_mode.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
        query_nav_mode.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
        query_nav_mode.header.payload_length = QUERY_NAVIGATION_MODE_PAYLOAD_LENGTH;
        query_nav_mode.message_id = QUERY_WAAS;
        query_nav_mode.footer.end1 = SKYTRAQ_END_BYTE_1;
        query_nav_mode.footer.end2 = SKYTRAQ_END_BYTE_2;

        unsigned char* msg_ptr = (unsigned char*)&query_nav_mode;
        calculateCheckSum(msg_ptr+HEADER_LENGTH, QUERY_NAVIGATION_MODE_PAYLOAD_LENGTH,
                          &query_nav_mode.footer.checksum);
        
        send_result = SendMessage(msg_ptr,HEADER_LENGTH+QUERY_NAVIGATION_MODE_PAYLOAD_LENGTH+FOOTER_LENGTH);
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in Skytraq::QueryNavigationMode(): " << e.what();
        log_error_(output.str());
        return false;
    }
    return send_result;
}


//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
void Skytraq::BufferIncomingData(uint8_t *msg, size_t length) {
//    cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
    // add incoming data to buffer
//    printHex(reinterpret_cast<char*>(msg),length);
	try {

		for (unsigned int i = 0; i < length; i++) {
			//cout << i << ": " << hex << (int)msg[i] << dec << endl;
			// make sure buffer_index_ is not larger than buffer
			if (buffer_index_ >= MAX_NOUT_SIZE) {
				buffer_index_ = 0;
				log_warning_(
						"Overflowed receiver buffer. See Skytraq::BufferIncomingData()");

			}
//            cout << "buffer_index_ = " << buffer_index_ << endl;

			if (buffer_index_ == 0) {	// looking for beginning of message
				if (msg[i] == SKYTRAQ_SYNC_BYTE_1) {	// beginning of msg found - add to buffer
//                    cout << "got first bit" << endl;
//                    std::cout << hex << (int)msg[i] << dec << std::endl;
					data_buffer_[buffer_index_++] = msg[i];
				}	// end if (msg[i]
			} 
            else if (buffer_index_ == 1) {	// verify 2nd character of header
				if (msg[i] == SKYTRAQ_SYNC_BYTE_2) {	// 2nd byte ok - add to buffer
//                    cout << " got second synch bit" << endl;
//                    std::cout << hex << (int)msg[i] << dec << std::endl;
					data_buffer_[buffer_index_++] = msg[i];
				} else {
					// start looking for new message again
					buffer_index_ = 0;
					//readingACK=false;
				} // end if (msg[i]==UBX_SYNC_BYTE_2)
				// end else if (buffer_index_==1)
			} 
            else if (buffer_index_ == 4) {
				// msg[i] defines message ID
				data_buffer_[buffer_index_++] = msg[i];
				//printHex(reinterpret_cast < char * > (data_buffer_),4);
				msgID = data_buffer_[buffer_index_ - 1];
//                cout << "msgID = " << msgID << endl;
			} 
            else if ((msg[i-1]==SKYTRAQ_END_BYTE_1)&&(msg[i]==SKYTRAQ_END_BYTE_2)) {
                data_buffer_[buffer_index_] = msg[i];
//                std::cout << hex << (int)msg[i] << dec << std::endl;
//                cout << " msgID = " << msgID << std::endl;
				ParseLog(data_buffer_, msgID);
				// reset counters
				buffer_index_ = 0;
//                cout << "Message Done." << std::endl;
			}
			else {	// add data to buffer
				data_buffer_[buffer_index_++] = msg[i];
//                std::cout << hex << (int)msg[i] << dec << std::endl;
			}
		}	// end for
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error buffering incoming Skytraq data: " << e.what();
		log_error_(output.str());
        buffer_index_ = 0;
	}
}

void Skytraq::ParseLog(uint8_t *log, size_t logID) {
	try {
		uint16_t payload_length;
//        cout << "In ParseLog()." << endl;
        payload_length = (((uint16_t) *(log+2)) << 8) + ((uint16_t) *(log+3));
        cout << "payload length: " << (double)payload_length << endl;
		switch (logID) {

        //! Output System Messages
		case SOFTWARE_VERSION: // Receiver outputs if accurate internally stored pos and time aren't available
            skytraq::SoftwareVersion cur_software_version;
            memcpy(&cur_software_version, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(software_version_callback_)
                software_version_callback_(cur_software_version,read_timestamp_);
			break;
        case SOFTWARE_CRC:
            skytraq::SoftwareCRC cur_software_crc;
            memcpy(&cur_software_crc, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(software_crc_callback_)
                software_crc_callback_(cur_software_crc,read_timestamp_);
            break;
        case ACK:
            skytraq::Ack cur_ack;
            memcpy(&cur_ack, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(ack_callback_)
                ack_callback_(cur_ack,read_timestamp_);
            break;
        case NACK:
            skytraq::Nack cur_nack;
            memcpy(&cur_nack, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(nack_callback_)
                nack_callback_(cur_nack,read_timestamp_);
            break;
        case POS_UPDATE_RATE:
            skytraq::PositionUpdateRate cur_pos_update_rate;
            memcpy(&cur_pos_update_rate, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(pos_update_rate_callback_)
                pos_update_rate_callback_(cur_pos_update_rate,read_timestamp_);
            break;

        //! Output GPS Messages
        case GPS_WAAS_STATUS:
            skytraq::WaasStatus cur_waas_status;
            memcpy(&cur_waas_status, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(waas_status_callback_)
                waas_status_callback_(cur_waas_status,read_timestamp_);
            break;
        case GPS_NAV_MODE:
            skytraq::NavigationMode cur_nav_mode;
            memcpy(&cur_nav_mode, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(nav_mode_callback_)
                nav_mode_callback_(cur_nav_mode,read_timestamp_);
            break;
        case GPS_ALMANAC:
            skytraq::Almanac cur_almanac;
            memcpy(&cur_almanac, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(almanac_callback_)
                almanac_callback_(cur_almanac,read_timestamp_);
            break;
        case GPS_EPHEMERIS:
            skytraq::Ephemeris cur_ephemeris;
            memcpy(&cur_ephemeris, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(ephemeris_callback_)
                ephemeris_callback_(cur_ephemeris,read_timestamp_);
            break;
        case MEAS_TIME:
            skytraq::MeasurementTime cur_measurement_time;
            memcpy(&cur_measurement_time, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(measurement_time_callback_)
                measurement_time_callback_(cur_measurement_time,read_timestamp_);
            break;
        case RAW_MEAS:
            skytraq::RawMeasurements cur_raw_measurements;
            // Copy header and payload
            memcpy(&cur_raw_measurements, log, payload_length+HEADER_LENGTH);
            //Copy Footer
            memcpy(&cur_raw_measurements.footer, log+payload_length+HEADER_LENGTH, FOOTER_LENGTH);
            if(raw_measurement_callback_)
                raw_measurement_callback_(cur_raw_measurements,read_timestamp_);
            break;
        case SV_CH_STATUS:
            skytraq::ChannelStatus cur_channel_status;
            // Copy header and payload
            memcpy(&cur_channel_status, log, payload_length+HEADER_LENGTH);
            //Copy Footer
            memcpy(&cur_channel_status.footer, log+payload_length+HEADER_LENGTH, FOOTER_LENGTH);
            if(channel_status_callback_)
                channel_status_callback_(cur_channel_status,read_timestamp_);
            break;
        case RCV_STATE:
            skytraq::ReceiverNavStatus cur_reciever_status;
            memcpy(&cur_reciever_status, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(receiver_nav_status_callback_)
                receiver_nav_status_callback_(cur_reciever_status,read_timestamp_);
            break;
        case SUBFRAME:
            skytraq::SubframeBufferData cur_subframe_buffer_data;
            memcpy(&cur_subframe_buffer_data, log, payload_length+HEADER_LENGTH+FOOTER_LENGTH);
            if(subframe_buffer_data_callback_)
                subframe_buffer_data_callback_(cur_subframe_buffer_data,read_timestamp_);
            break;
        
		} // end switch (logID)
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error Skytraq::ParseLog(): " << e.what();
		log_error_(output.str());
	}
} // end ParseLog()

void Skytraq::calculateCheckSum(uint8_t* in, size_t length, uint8_t* cs) 
{
	try {
		uint8_t sum = 0;
		for (uint8_t i = 0; i < length; i++) {
			sum = sum ^ in[i];
		}
        cs[0] = sum;
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Skytraq::calculateCheckSum(): " << e.what();
		log_error_(output.str());
	}
}
