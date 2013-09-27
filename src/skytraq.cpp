#include "skytraq/.h"
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

/*!
 * Default callback method for time stamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

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

inline void DefaultSoftwareVersionCallback(SoftwareVersion& software_version, double& timestamp){

}

inline void DefaultSoftwareCrcCallback(SoftwareCRC& software_crc, double& timestamp){

}

inline void DefaultAckCallback(Ack& ack, double& timestamp){

}

inline void DefaultNackCallback(Nack& nack, double& timestamp){

}

inline void DefaultEphemerisCallback(Ephemeris& ephemeris, double& timestamp){

}

inline void DefaultRawMeasurementCallback(RawMeasurements& raw_measurements, double& timestamp){

}

inline void DefaultNavStatusCallback(ReceiverNavStatus& nav_status, double& timestamp){

}

inline void DefaultSubframeBufferDataCallback(SubframeBufferData& subframe_buffer_data, double& timestamp){

}

Skytraq::Skytraq() {
    serial_port_ = NULL;
    reading_status_ = false;
    time_handler_ = DefaultGetTime;
    log_debug_ = DefaultDebugMsgCallback;
    log_info_ = DefaultInfoMsgCallback;
    log_warning_ = DefaultWarningMsgCallback;
    log_error_ = DefaultErrorMsgCallback;
    software_version_callback_ = DefaultSoftwareVersionCallback;
    software_crc_callback_ = DefaultSoftwareCrcCallback;
    ack_callback_ = DefaultAckCallback;
    nack_callback_ = DefaultNackCallback;
    almanac_callback_ = DefaultAlmanacCallback;
    ephemeris_callback_ = DefaultEphemerisCallback;
    raw_measurement_callback_ = DefaultRawMeasurementCallback;
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
    serial::Timeout my_timeout(100, 1000, 0, 1000, 0);
    try {
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
		if (!Ping()) {
			std::stringstream output;
			output << "Skytraq GPS not found on port: " << port << std::endl;
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
			return false;
		}


    } catch (std::exception e) {
           std::stringstream output;
           output << "Failed to open port " << port << "  Err: " << e.what();
           log_error_(output.str());
           serial_port_ = NULL;
           is_connected_ = false;
           return false;
       }

    // start reading
    StartReading();
    is_connected_ = true;
    return true;

}

bool Skytraq::Ping(int num_attempts) {
    try {
        while ((num_attempts--) > 0) {
            log_info_("Searching for Skytraq receiver...");
            // request version information

            // ask for version
            PollMessage(MSG_CLASS_MON, MSG_ID_MON_VER);

            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

            unsigned char result[5000];
            size_t bytes_read;
            bytes_read = serial_port_->read(result, 5000);

            //std::cout << "bytes read: " << (int)bytes_read << std::endl;
            //std::cout << dec << result << std::endl;

            if (bytes_read < 8) {
                stringstream output;
                output << "Only read " << bytes_read
                        << " bytes in response to ping.";
                log_debug_(output.str());
                continue;
            }

            uint16_t length;
            // search through result for version message
            for (int ii = 0; ii < (bytes_read - 8); ii++) {
                //std::cout << hex << (unsigned int)result[ii] << std::endl;
                if (result[ii] == UBX_SYNC_BYTE_1) {
                    if (result[ii + 1] != UBX_SYNC_BYTE_2)
                        continue;
                    if (result[ii + 2] != MSG_CLASS_MON)
                        continue;
                    if (result[ii + 3] != MSG_ID_MON_VER)
                        continue;
                    //std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
                    //std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
                    length = (result[ii + 4]) + (result[ii + 5] << 8);
                    if (length < 40) {
                        log_debug_("Incomplete version message received");
                        //    //return false;
                        continue;
                    }

                    string sw_version;
                    string hw_version;
                    string rom_version;
                    sw_version.append((char*) (result + 6));
                    hw_version.append((char*) (result + 36));
                    //rom_version.append((char*)(result+46));
                    log_info_("Skytraq receiver found.");
                    log_info_("Software Version: " + sw_version);
                    log_info_("Hardware Version: " + hw_version);
                    //log_info_("ROM Version: " + rom_version);
                    return true;
                }
            }
            stringstream output;
            output << "Read " << bytes_read
                    << " bytes, but version message not found.";
            log_debug_(output.str());

        }
    } catch (exception &e) {
        std::stringstream output;
        output << "Error pinging receiver: " << e.what();
        log_error_(output.str());
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
		// create thread to read from sensor
		reading_status_ = true;
		read_thread_ptr_ = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&Skytraq::ReadSerialPort, this)));
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
        } catch (exception &e) {
            stringstream output;
            output << "Error reading serial port: " << e.what();
            log_info_(output.str());
            Disconnect();
            return;
        }
        // timestamp the read
        read_timestamp_ = time_handler_();
        // add data to the buffer to be parsed
        BufferIncomingData(buffer, len);
    }

}

//////////////////////////////////////////////////////////////////////////////
// AIDING DATA POLL MESSAGES
/////////////////////////////////////////////////////////////////////////////

// Poll Message used to request for all SV
bool Skytraq::PollMessage(uint8_t class_id, uint8_t msg_id) {
	try {
		uint8_t message[8];

		message[0]=UBX_SYNC_BYTE_1;        // sync 1
		message[1]=UBX_SYNC_BYTE_2;        // sync 2
		message[2]=class_id;
		message[3]=msg_id;
		message[4]=0;           // length 1
		message[5]=0;           // length 2
		message[6]=0;           // checksum 1
		message[7]=0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;

		calculateCheckSum(msg_ptr + 2, 4, msg_ptr + 6);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  size_t bytes_written = serial_port_->write(message, 8);
          return bytes_written == 8;
        } else {
            log_error_("Unable to send poll message. Serial port not open.");
            return false;
        }

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending Skytraq poll message: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// Poll Message used to request for one SV
bool Skytraq::PollMessageIndSV(uint8_t class_id, uint8_t msg_id, uint8_t svid) {
    try {
		uint8_t message[9];

		message[0] = UBX_SYNC_BYTE_1;        // sync 1
		message[1] = UBX_SYNC_BYTE_2;        // sync 2
		message[2] = class_id;
		message[3] = msg_id;
		message[4] = 1;           // length 1
		message[5] = 0;           // length 2
		message[6] = svid;        // Payload
		message[7] = 0;           // checksum 1
		message[8] = 0;           // checksum 2

		uint8_t* msg_ptr = (uint8_t*) &message;
		calculateCheckSum(msg_ptr + 2, 5, msg_ptr + 7);

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		    size_t bytes_written = serial_port_->write(msg_ptr, 9);
            return bytes_written == 9;
        } else {
            log_error_("Unable to send poll ind. sv. message. Serial port not open.");
            return false;
        }

    } catch (std::exception &e) {
		std::stringstream output;
		output << "Error polling individual svs: " << e.what();
		log_error_(output.str());
		return 0;
	}
}

// (AID-EPH) Polls for Ephemeris data
bool Skytraq::PollEphem(int8_t svid) {

    if (svid < -1) {
        log_error_("Error in PollEphem: Invalid input 'svid'");
        return 0;
    } else if (svid == -1) { // Requests Ephemerides for all SVs
        log_debug_("Polling for all Ephemerides..");
        return PollMessage(MSG_CLASS_AID, MSG_ID_AID_EPH);
    } else if (svid > 0) { // Requests Ephemeris for a single SV
        stringstream output;
        output << "Polling for SV# " << (int) svid << " Ephemeris..";
        log_debug_(output.str());
        return PollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_EPH, (uint8_t) svid);
    } else {
        log_error_("Error in PollEphem: Invalid input 'svid'");
        return 0;
    }
}

// (AID-ALM) Polls for Almanac Data
bool Skytraq::PollAlmanac(int8_t svid) {

    if (svid < -1) {
        log_error_("Error in PollAlmanac: Invalid input 'svid'");
        return 0;
    } else if (svid == -1) { // Requests Almanac Data for all SVs
        log_debug_("Polling for all Almanac Data..");
        return PollMessage(MSG_CLASS_AID, MSG_ID_AID_ALM);
    } else if (svid > 0) { // Requests Almanac Data for a single SV
        stringstream output;
        output << "Polling for SV# " << (int) svid << " Almanac Data..";
        log_debug_(output.str());
        return PollMessageIndSV(MSG_CLASS_AID, MSG_ID_AID_ALM, (uint8_t) svid);
    } else {
        log_error_("Error in PollAlmanac: Invalid input 'svid'");
        return 0;
    }
}

// (CFG-MSG) Set message output rate for specified message
bool Skytraq::ConfigureMessageRate(uint8_t class_id, uint8_t msg_id,
        uint8_t rate) {
	try {
		CfgMsgRate message;
		message.header.sync1 = UBX_SYNC_BYTE_1;
		message.header.sync2 = UBX_SYNC_BYTE_2;
		message.header.message_class = MSG_CLASS_CFG;
		message.header.message_id = MSG_ID_CFG_MSG;
		message.header.payload_length = 3;

		message.message_class = class_id;
		message.message_id = msg_id;
		message.rate = rate;

		unsigned char* msg_ptr = (unsigned char*) &message;
		calculateCheckSum(msg_ptr + 2, 7, message.checksum);

		return serial_port_->write(msg_ptr, sizeof(message)) == sizeof(message);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring Skytraq message rate: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Set Port Configuration
void Skytraq::SetOutputFormatToBinary() {
	try {
		ConfigureOutputFormat message;
		//std::cout << sizeof(message) << std::endl;
		message.header.sync1 = SKYTRAQ_SYNC_BYTE_1;
		message.header.sync2 = SKYTRAQ_SYNC_BYTE_2;
		message.header.payload_length = CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH;
        message.message_id = CFG_OUTPUT_FORMAT;
        message.type = BINARY_OUTPUT;
        message.attributes = UPDATE_TO_SRAM_AND_FLASHs;

        unsigned char* msg_ptr = (unsigned char*) &message;
        calculateCheckSum(msg_ptr + HEADER_LENGTH, CONFIGURE_OUTPUT_FORMAT_PAYLOAD_LENGTH,
                        message.footer.checksum);

        message.footer.end1 = SKYTRAQ_END_BYTE_1;
        message.footer.end2 = SKYTRAQ_END_BYTE_2;

		log_info_("ConfigureOutputFormat Message Sent.");

		//printHex((char*) &message, sizeof(message));

		serial_port_->write(msg_ptr, sizeof(message));
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring Skytraq port: " << e.what();
		log_error_(output.str());
	}
}

// Poll Port Configuration
void Skytraq::PollPortConfiguration(uint8_t port_identifier)
{ // Port identifier = 3 for USB (default value if left blank)
  //                 = 1 or 2 for UART
	try {
		uint8_t message[9];
		message[0]=UBX_SYNC_BYTE_1;
		message[1]=UBX_SYNC_BYTE_2;
		message[2]=MSG_CLASS_CFG;
		message[3]=MSG_ID_CFG_PRT;
		message[4]=1;
		message[5]=0;
		message[6]=port_identifier;         //Port identifier for USB Port (3)
		message[7]=0;                       // Checksum A
		message[8]=0;                       // Checksum B

		unsigned char* msg_ptr = (unsigned char*)&message;
		calculateCheckSum(msg_ptr+2,5,msg_ptr+7);

		serial_port_->write(msg_ptr, sizeof(message));
		log_info_("Polling for Port Protocol Configuration.");
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error polling Skytraq port configuration: " << e.what();
		log_error_(output.str());
	}
}

//////////////////////////////////////////////////////////////
// Functions to  Aiding Data to Receiver
//////////////////////////////////////////////////////////////
// Send Message
bool Skytraq::SendMessage(uint8_t* msg_ptr, size_t length)
{
	try {
		stringstream output1;
		//std::cout << length << std::endl;
		//std::cout << "Message Pointer" << endl;
		//printHex((char*) msg_ptr, length);
        size_t bytes_written;

        if ((serial_port_!=NULL)&&(serial_port_->isOpen())) {
		  bytes_written=serial_port_->write(msg_ptr, length);
        } else {
            log_error_("Unable to send message. Serial port not open.");
            return false;
        }
		// check that full message was sent to serial port
		if (bytes_written == length) {
			return true;
		}
		else {
			log_error_("Full message was not sent over serial port.");
			output1 << "Attempted to send " << length << "bytes. " << bytes_written << " bytes sent.";
			log_error_(output1.str());
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending Skytraq message: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-INI to Receiver
bool Skytraq::SendAidIni(AidIni ini)
{  
    //stringstream output;
	try {
		unsigned char* msg_ptr = (unsigned char*)&ini;
		calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_INI + 4,
							ini.checksum);

		// Check that provided ini message is correct size before sending
		if (sizeof(ini) == FULL_LENGTH_AID_INI)
		{
			bool sent_ini = SendMessage(msg_ptr, FULL_LENGTH_AID_INI);
			//output << "Sending AID-INI to receiver.";
			//log_debug_(output.str());
			return true;
		}
		else
		{
			//output << "Provided AID-INI message not of correct length.";
			//log_error_(output.str());
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending aid ini data to Skytraq: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-EPH to Receiver
bool Skytraq::SendAidEphem(Ephemerides ephems)
{
	try {
		bool sent_ephem [32];

		for (uint8_t prn_index = 1; prn_index <= 32; prn_index++) {
			//stringstream output;
			if (ephems.ephemsv[prn_index].header.payload_length == PAYLOAD_LENGTH_AID_EPH_WITH_DATA) {
				//output << "Sending AID-EPH for PRN # "
						//<< (int) ephems.ephemsv[prn_index].svprn << " ..";
				uint8_t* msg_ptr = (uint8_t*) &ephems.ephemsv[prn_index];
				calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_EPH_WITH_DATA + 4,
									ephems.ephemsv[prn_index].checksum);
				sent_ephem[prn_index] = SendMessage(msg_ptr, FULL_LENGTH_AID_EPH_WITH_DATA);

			} else { // not a full ephemeris message
				//output << "No AID-EPH data for PRN # " << (int) prn_index << " ..";
			}
			//log_debug_(output.str());
		}
		return true;
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending ephemeris data to Skytraq: " << e.what();
		log_error_(output.str());
		return false;
	}
}

// Send AID-ALM to Receiver
bool Skytraq::SendAidAlm(Almanac almanac) {

	try {
		bool sent_alm [32];

		for (uint8_t prn_index = 1; prn_index <= 32; prn_index++) {
			//stringstream output;

			if (almanac.almsv[prn_index].header.payload_length == PAYLOAD_LENGTH_AID_ALM_WITH_DATA) {
				//output << "Sending AID-ALM for PRN # "
						//<< (int) almanac.almsv[prn_index].svprn << " ..";
				uint8_t* msg_ptr = (uint8_t*) &almanac.almsv[prn_index];
				calculateCheckSum(msg_ptr + 2, PAYLOAD_LENGTH_AID_ALM_WITH_DATA + 4,
									almanac.almsv[prn_index].checksum);
				sent_alm[prn_index] = SendMessage(msg_ptr, FULL_LENGTH_AID_ALM_WITH_DATA);
			}
			else {
				//output << "No AID-ALM data for PRN # " << (int) prn_index << " ..";
			}
			//log_debug_(output.str());
		}
		return true;

	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error sending almanac data to Skytraq: " << e.what();
		log_error_(output.str());
		return false;
	}
}

//////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////
void Skytraq::BufferIncomingData(uint8_t *msg, size_t length) {
    //MOOSTrace("Inside BufferIncomingData\n");
    //cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
    // add incoming data to buffer

    //printHex(reinterpret_cast<char*>(msg),length);
	try {

		for (unsigned int i = 0; i < length; i++) {
			//cout << i << ": " << hex << (int)msg[i] << dec << endl;
			// make sure buffer_index_ is not larger than buffer
			if (buffer_index_ >= MAX_NOUT_SIZE) {
				buffer_index_ = 0;
				log_warning_(
						"Overflowed receiver buffer. See Skytraq.cpp BufferIncomingData");

			}
			//cout << "buffer_index_ = " << buffer_index_ << endl;

			if (buffer_index_ == 0) {	// looking for beginning of message
				if (msg[i] == UBX_SYNC_BYTE_1) {	// beginning of msg found - add to buffer
										//cout << "got first bit" << endl;
					data_buffer_[buffer_index_++] = msg[i];
					bytes_remaining_ = 0;
				}	// end if (msg[i]
			} // end if (buffer_index_==0)
			else if (buffer_index_ == 1) {	// verify 2nd character of header
				if (msg[i] == UBX_SYNC_BYTE_2) {	// 2nd byte ok - add to buffer
										//cout << " got second synch bit" << endl;
					data_buffer_[buffer_index_++] = msg[i];
				} else {
					// start looking for new message again
					buffer_index_ = 0;
					bytes_remaining_ = 0;
					//readingACK=false;
				} // end if (msg[i]==UBX_SYNC_BYTE_2)
			}	// end else if (buffer_index_==1)
			else if (buffer_index_ == 2) {	//look for ack

				if (msg[i] == MSG_CLASS_ACK)   // ACK or NAK message class
						{
					// Get message id from payload
					char* class_id = reinterpret_cast<char*>(msg[i + 4]);
					char* msg_id = reinterpret_cast<char*>(msg[i + 5]);

					// Add function which takes class_id and msg_id and returns name of corresponding message

					if (msg[i + 1] == MSG_ID_ACK_ACK) // ACK Message
							{
						//std::cout << "Receiver Acknowledged Message " << std::endl;
						//printf("0x%.2X ", (unsigned)class_id);
						//std::cout << " ";
						//printf("0x%.2X ", (unsigned)msg_id);
						//std::cout << endl;

					}

					else if (msg[i + 1] == MSG_ID_ACK_NAK)    // NAK Message
							{
						//std::cout << "Receiver Did Not Acknowledged Message " << std::endl;
						//printf("0x%.2X ", (unsigned)class_id);
						//std::cout << " ";
						//printf("0x%.2X ", (unsigned)msg_id);
						//std::cout << endl;
					}

					buffer_index_ = 0;
					bytes_remaining_ = 0;
					//readingACK = false;			//? Why is readingACK = false in if & else statement? - CC
				} else {
					data_buffer_[buffer_index_++] = msg[i];
					//readingACK = false;
				}
			} else if (buffer_index_ == 3) {
				// msg[i] and msg[i-1] define message ID
				data_buffer_[buffer_index_++] = msg[i];
				// length of header is in byte 4

				//printHex(reinterpret_cast < char * > (data_buffer_),4);

				msgID = ((data_buffer_[buffer_index_ - 2]) << 8)
						+ data_buffer_[buffer_index_ - 1];
				//cout << "msgID = " << msgID << endl;
			} else if (buffer_index_ == 5) {
				// add byte to buffer
				data_buffer_[buffer_index_++] = msg[i];
				// length of message (payload + 2 byte check sum )
				bytes_remaining_ = ((data_buffer_[buffer_index_ - 1]) << 8)
						+ data_buffer_[buffer_index_ - 2] + 2;

				//cout << "bytes_remaining_ = " << bytes_remaining_ << endl;

				///cout << msgID << endl;
			} else if (buffer_index_ == 6) {	// set number of bytes
				data_buffer_[buffer_index_++] = msg[i];
				bytes_remaining_--;
			} else if (bytes_remaining_ == 1) {	// add last byte and parse
				data_buffer_[buffer_index_++] = msg[i];
				//std::cout << hex << (int)msg[i] << dec << std::endl;
				//cout << " msgID = " << msgID << std::endl;
				ParseLog(data_buffer_, msgID);
				// reset counters
				buffer_index_ = 0;
				bytes_remaining_ = 0;
				//cout << "Message Done." << std::endl;
			}  // end else if (bytes_remaining_==1)
			else {	// add data to buffer
				data_buffer_[buffer_index_++] = msg[i];
				bytes_remaining_--;
			}
		}	// end for
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error buffering incoming Skytraq data: " << e.what();
		log_error_(output.str());
	}
}

void Skytraq::ParseLog(uint8_t *log, size_t logID) {
	try {
		uint16_t payload_length;
		uint8_t num_of_svs;
		uint8_t num_of_channels;

		switch (logID) {

		case AID_REQ: // Receiver outputs if accurate internally stored pos and time aren't available
			log_info_("AID-REQ message received by computer.");
			break;

		} // end switch (logID)
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error parsing Skytraq log: " << e.what();
		log_error_(output.str());
	}
} // end ParseLog()

void Skytraq::calculateCheckSum(uint8_t* in, size_t length, uint8_t* out) {

	try {
		uint8_t a = 0;
		uint8_t b = 0;

		for (uint8_t i = 0; i < length; i++) {

			a = a + in[i];
			b = b + a;

		}

		out[0] = (a & 0xFF);
		out[1] = (b & 0xFF);
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error calculating Skytraq checksum: " << e.what();
		log_error_(output.str());
	}
}
