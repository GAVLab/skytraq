/*!
 * \file skytraq/skytraq.h
 * \author  Chris Collins <cnc0003@tigermail.auburn.edu>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 IS4S / Auburn University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for skytraq GPS receivers using
 * the skytraq binary protocol.
 */

#ifndef SKYTRAQ_H
#define SKYTRAQ_H

#include "skytraq_structures.h"
#include <serial/serial.h>
#include <fstream>

#include <boost/function.hpp>
#include <boost/thread.hpp>
//#include <boost/bind.hpp>

namespace skytraq {

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarningMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;

//! System Output Message Callbacks
typedef boost::function<void(SoftwareVersion&, double&)> SoftwareVersionCallback;
typedef boost::function<void(SoftwareCRC&, double&)> SoftwareCRCCallback;
typedef boost::function<void(Ack&, double&)> AckCallback;
typedef boost::function<void(Nack&, double&)> NackCallback;
typedef boost::function<void(PositionUpdateRate&, double&)> PositionUpdateRateCallback;
//! GPS Output Message Callbacks
typedef boost::function<void(WaasStatus&, double&)> WaasStatusCallback;
typedef boost::function<void(NavigationMode&, double&)> NavigationModeCallback;
typedef boost::function<void(Almanac&, double&)> AlmanacCallback;
typedef boost::function<void(Ephemeris&, double&)> EphemerisCallback;
typedef boost::function<void(MeasurementTime&, double&)> MeasurementTimeCallback;
typedef boost::function<void(RawMeasurements&, double&)> RawMeasurementsCallback;
typedef boost::function<void(ChannelStatus&, double&)> ChannelStatusCallback;
typedef boost::function<void(ReceiverNavStatus&, double&)> ReceiverNavStatusCallback;
typedef boost::function<void(SubframeBufferData&, double&)> SubframeBufferDataCallback;


class Skytraq
{
public:
	Skytraq();
	~Skytraq();

	/*!
	 * Connects to the Skytraq receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 */
	bool Connect(std::string port, int baudrate=9600);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();

    //! Indicates if a connection to the receiver has been established.
    bool IsConnected() {return is_connected_;}

    /*!
     * Pings the GPS to determine if it is properly connected
     *
     * This method sends a ping to the GPS and waits for a response.
     *
     * @param num_attempts The number of times to ping the device
     * before giving up
     * @param timeout The time in milliseconds to wait for each reponse
     *
     * @return True if the GPS was found, false if it was not.
     */
    bool Ping(int num_attempts=5);

    //////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    DebugMsgCallback log_debug_;
    InfoMsgCallback log_info_;
    WarningMsgCallback log_warning_;
    ErrorMsgCallback log_error_;

    //////////////////////////////////////////////////////
    // Data Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping

    void set_time_handler(GetTimeCallback time_handler) {
         this->time_handler_ = time_handler;
    }
    
    //! System Input Message Methods
    bool SendMessage(uint8_t *msg_ptr, size_t length);
    bool RestartReceiver(skytraq::StartMode start_mode, uint16_t utc_year=0,
                        uint8_t utc_month=0, uint8_t utc_day=0, uint8_t utc_hour=0,
                        uint8_t utc_minute=0, uint8_t utc_second=0, int16_t latitude=0,
                        int16_t longitude=0, int16_t altitude=0);
    void HotRestartReceiver(uint16_t utc_year=0, uint8_t utc_month=0, uint8_t utc_day=0,
                            uint8_t utc_hour=0, uint8_t utc_minute=0, uint8_t utc_second=0,
                            int16_t latitude=0, int16_t longitude=0, int16_t altitude=0);
    void WarmRestartReceiver();
    void ColdRestartReceiver();
    bool QuerySoftwareVersion();
    bool QuerySoftwareCrcVersion();
    bool RestoreFactoryDefaults();
    bool ConfigureSerialPort(uint8_t com_port, uint8_t baudrate=115200);
    bool ConfigureNmeaIntervals(uint8_t gga_interval=0, uint8_t gsa_interval=0, 
                                    uint8_t gsv_interval=0, uint8_t gll_interval=0, 
                                    uint8_t rcm_interval=0, uint8_t vtg_interval=0, 
                                    uint8_t zda_interval=0);
    bool ConfigureOutputFormat(OutputType output_type);
    bool SetOutputFormatToBinary();
    bool SetOutputFormatToNmea();
    bool DisableAllOutput();
    bool EnablePowerSaveMode();
    bool DisablePowerSaveMode();
    bool ConfigurePositionUpdateRate(uint8_t update_rate);
    bool QueryPositionUpdateRate();
    bool ConfigureMessagesOutputRate(skytraq::BinaryOutputRate rate,  
                                skytraq::DisableEnable meas_time_message, 
                                skytraq::DisableEnable raw_meas_message, 
                                skytraq::DisableEnable channel_status_message, 
                                skytraq::DisableEnable receiver_state_message, 
                                skytraq::DisableEnable subframe_buffer_message);
    
    //! GPS Input Message Methods
    bool PollAlmanac(uint8_t prn);
    bool PollEphemeris(uint8_t prn);
    bool SetEphemeris(uint16_t svid, skytraq::Subframe subframe1, 
                        skytraq::Subframe subframe2, skytraq::Subframe subframe3);
    bool SetEphemerides(skytraq::Ephemerides ephemerides);
    bool EnableWAAS();
    bool DisableWAAS();
    bool QueryWAASConfiguration();
    bool SetCarNavigationMode();
    bool SetPedestrianNavigationMode();
    bool QueryNavigationMode();

    //////////////////////////////////////////////////////
    // Set Callback Methods
    //////////////////////////////////////////////////////
    //! System Output Message Callbacks
    void set_software_version_callback_(SoftwareVersionCallback callback){software_version_callback_=callback;}
    void set_software_crc_callback_(SoftwareCRCCallback callback){software_crc_callback_=callback;}
    void set_ack_callback_(AckCallback callback){ack_callback_=callback;}
    void set_nack_callback_(NackCallback callback){nack_callback_=callback;}
    void set_pos_update_rate_callback_(PositionUpdateRateCallback callback){pos_update_rate_callback_=callback;}
    //! GPS Output Message Callbacks
    void set_waas_status_callback_(WaasStatusCallback callback){waas_status_callback_=callback;}
    void set_nav_mode_callback_(NavigationModeCallback callback){nav_mode_callback_=callback;}
    void set_almanac_callback_(AlmanacCallback callback){almanac_callback_=callback;}
    void set_ephemeris_callback_(EphemerisCallback callback){ephemeris_callback_=callback;}
    void set_measurement_time_callback_(MeasurementTimeCallback callback){measurement_time_callback_=callback;}
    void set_raw_measurement_callback_(RawMeasurementsCallback callback){raw_measurement_callback_=callback;}
    void set_channel_status_callback_(ChannelStatusCallback callback){channel_status_callback_=callback;}
    void set_receiver_nav_status_callback_(ReceiverNavStatusCallback callback){receiver_nav_status_callback_=callback;}
    void set_subframe_buffer_data_callback_(SubframeBufferDataCallback callback){subframe_buffer_data_callback_=callback;}

    void calculateCheckSum(uint8_t* in, size_t length, uint8_t* cs);
private:

	/*!
	 * Starts a thread to continuously read from the serial port.
	 *
	 * Starts a thread that runs 'ReadSerialPort' which constatly reads
	 * from the serial port.  When valid data is received, parse and then
	 *  the data callback functions are called.
	 *
	 * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
	 */
	void StartReading();

	/*!
	 * Starts the thread that reads from the serial port
	 *
	 * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
	 */
	void StopReading();

	/*!
	 * Method run in a seperate thread that continuously reads from the
	 * serial port.  When a complete packet is received, the parse
	 * method is called to process the data
	 *
	 * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
	 */
	void ReadSerialPort();

	bool WaitForAck(int timeout); //!< waits for an ack from receiver (timeout in seconds)

    void BufferIncomingData(uint8_t* msg, size_t length);
	//! Function to parse logs into a usable structure
    void ParseLog(uint8_t* log, size_t logID);

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.


    //////////////////////////////////////////////////////
    // Data Callbacks
    //////////////////////////////////////////////////////
    //! System Output Message Callbacks
    SoftwareVersionCallback software_version_callback_;
    SoftwareCRCCallback software_crc_callback_;
    AckCallback ack_callback_;
    NackCallback nack_callback_;
    PositionUpdateRateCallback pos_update_rate_callback_;
    //! GPS Output Message Callbacks
    WaasStatusCallback waas_status_callback_;
    NavigationModeCallback nav_mode_callback_;
    AlmanacCallback almanac_callback_;
    EphemerisCallback ephemeris_callback_;
    MeasurementTimeCallback measurement_time_callback_;
    RawMeasurementsCallback raw_measurement_callback_;
    ChannelStatusCallback channel_status_callback_;
    ReceiverNavStatusCallback receiver_nav_status_callback_;
    SubframeBufferDataCallback subframe_buffer_data_callback_;

	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began
	unsigned short msgID;
	
    bool is_connected_; //!< indicates if a connection to the receiver has been established

};
}

#endif
