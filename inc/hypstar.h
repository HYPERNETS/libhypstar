/**
 * \brief	Hypstar instrument Linux driver with Python support
 *	Tested on 64bit Debian-based system with Python 3.9
 *
 *	\usage	1. Instantiate Hypstar *hs = Hypstar::getInstance(port);
 *			2. Switch baudrate hs->setBaudRate(B_6000000)
 *			3. Call functions
 *			4. Destroy if needed.
 *
 *	\author Joel Kuusk, Kaspars Laizans (Tartu Observatory)
 */

#ifndef LIBHYPSTAR_H
#define LIBHYPSTAR_H

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <stdio.h>
#include <libhypstar_linuxserial.h>
#include <string>
#include <hypstar_typedefs.hpp>
#include <vector>
#include <stdarg.h>
#include <iostream>

#define LOG(level, stream, format, ...) printLogStatic(level, #level, stream, format, ##__VA_ARGS__)
/**
 * Main driver class. Handles communications with the instrument
 * with input parameter sanity checks and some level of error recovery.
 *
 */
class Hypstar
{
	struct s_hypstar_instance {
		std::string port;
		Hypstar *instance;
	};

	public:
		/**
		 * \brief	To avoid multiple access, Singleton-style instantiation is done
		 * New instances are initialized to default baud rate and are reset to it in destructor
		 * On initialization driver sets instrument timestamp to UTC time. Use setTime() to set to local time or whatever
		 * \param portname name of the port (e.g. '/dev/ttyUSB0').
		 * \param log_prefix can optionally be used to prefix logging output, e.g. with "#" which is often used
		 * as a default comment character. Each instance can have different loglevel and log_prefix.
		 */
		static Hypstar* getInstance(std::string portname, e_loglevel* loglevel = NULL, const char* logprefix = NULL);

		/**
		 * Destructor. Also resets baud rate to default 115k
		 */
		~Hypstar();

		/* To prevent accidental copying */
		Hypstar(Hypstar const&) = delete;
		void operator = (Hypstar const&) = delete;

		/**
		* \brief	Fills hw_info struct with values. Called on boot.
		 * \return status of execution: True if successful, false if not.
		 */
		bool getHardWareInfo(void);

		/**
		 * \brief	Sets baud rate on the instrument and the used serial port.
		 * \param baud - valid baud rate, can be one of @e_baud
		 * \return status of execution: True if successful, false if not.
		 */
		bool setBaudRate(e_baudrate baudRate);

		/**
		 * \brief	Fills cal_coefs struct with values.
		 * \return status of execution: True if successful, false if not.
		 */
		bool getCalibrationCoefficientsBasic(void);

		/**
		 * \brief	Fills cal_coefs_ext struct with values.
		 * \return status of execution: True if successful, false if not.
		 */
		bool getCalibrationCoefficientsExtended(void);

		/**
		 * \brief	Shorthand for successive requests for basic and extended coefficients
		 * \return status of execution: True if successful, false if not.
		 */
		bool getCalibrationCoefficientsAll(void);

		/**
		 * \brief	 Get instrument internal time
		 * \return UNIX timestamp in milliseconds.
		 */
		uint64_t getTime(void);

		/**
		 * \brief	 Set instrument internal time
		 * \param tm UNIX timestamp in seconds
		 * \return status of execution: True if successful, false if not.
		 */
		bool setTime(uint64_t time_s);

		/**
		 * \brief	 Turn on/off Validation Module light source
		 * \param enable true to turn on, false to turn off
		 * \return status of execution: True if successful, false if not.
		 */
		bool enableVM(bool enable);

		/**
		 * \brief	 Fill the system message log structure with log data
		 * \param pTarget pointer to memory region allocated for log
		 * \param index 0-indexed reverse-order (latest=0) log index to return
		 * \return status of execution: True if successful, false if not.
		 */
		bool getSystemLogEntry(struct s_log_item *pTarget, unsigned char index);

		/**
		 * \brief	 Fill the environment log structure with log data
		 * \param envlog pointer to memory region allocated for log
		 * \param index 0-indexed reverse-order (latest=0) log index to return
		 * \return status of execution: True if successful, false if not.
		 */
		bool getEnvironmentLogEntry(struct s_environment_log_entry *pTarget, unsigned char index);

		/**
		 * \brief	 Recalculates raw accelerometer data into acceleration in Gs
		 * \param reading single signed short raw value
		 * \return float value in Gs
		 */

		float convertRawAccelerometerReadingToGs(int16_t reading);
		/**
		 * \brief	 Recalculates raw accelerometer data into acceleration in Gs
		 * \param readings_ADU pointer to array of 3 signed shorts to be converted
		 * \param out array of 3 floats where results are to be stored
		 */
		void convertRawAccelerometerDataToGs(int16_t *readings_ADU, float *out);

		/**
		 * \brief	 Recalculates raw accelerometer data into acceleration in Gs from environmental log entry
		 * \param log pointer to environmental log entry from which to extract acceleration data
		 * \param out array of 3 floats where results are to be stored
		 */
		void convertRawAccelerometerDataToGsFromEnvLog(struct s_environment_log_entry *log, float *out);

		/**
		 * \brief	 Convert accelerometer readings from gravities (g's) to metric (m*s^2)
		 * \param gs pointer to array of 3 floats with readings in g`s
		 * \param ms array of 3 floats where results are to be stored
		 */
		void convertAccelerationFromGsToMs(float *gs, float *ms);

		/**
		 * \brief	 Recalculates raw accelerometer data into acceleration in m*s^2 from environmental log entry
		 * \param log pointer to environmental log entry from which to extract acceleration data
		 * \param out array of 3 floats where results are to be stored
		 */
		void convertRawAccelerometerDataToMsFromEnvLog(struct s_environment_log_entry *log, float *out);

		/**
		 * \brief	 Capture camera image and store it on the instrument for further retrieval.
		 * \param cap_img image capture parameters
		 * \param timeout_s timeout in seconds for how long to wait for image capture done response
		 * \return size of captured image as a number of data packets to be transferred from the instrument
		 */
		unsigned short captureImage(struct s_capture_image_request captureRequestParameters, float timeout_s);

		/**
		 * \brief	 Shorthand version for image capture, specifically JPEG-compressed
		 * \param res JPEG image resolution, must be one of the standard @e_jpg_resolution
		 * \param flags image capture flags
		 * \param timeout timeout in seconds for how long to wait for image capture done response
		 * \return size of the image in packets
		 */
		unsigned short captureJpegImage(enum e_jpg_resolution resolution, struct s_capture_image_request_flags flags, float timeout_s);

		/**
		 * \brief	 Download image from the instrument
		 * \param img_dataset pointer to allocated image memory location
		 * \return size of the image dataset (including type information, headers and crc32) in bytes
		 */
		int getImage(struct s_img_data_holder *pImageDatasetTarget);

		/**
		 * \brief	 Shorthand for capture and download of a single JPEG image with 5MP resolution
		 * \param flip bool image vertical flip if true
		 * \param mirror bool image horizontal mirror if true
		 * \param autoFocus bool try to auto-focus the camera. If not, focuses to infinity
		 * \param img_dataset pointer to allocated image memory location
		 * \return size of the image dataset (including type information, headers and crc32) in bytes
		 */
		int acquireJpegImage(bool flip, bool mirror, bool autoFocus, struct s_img_data_holder *pImageDatasetTarget);

		/**
		 * \brief	 Capture spectra series (multiple spectra) to the instrument internal memory for subsequent retrieval.
		 * \param spec which radiometer to use: VNIR, SWIR or BOTH
		 * \param mux which optical channel to use: radiance, irradiance or dark
		 * \param vnir_inttime_ms VNIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param swir_inttime_ms SWIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param scan_count number of exposures to perform
		 * \param series_time_s maximum series duration in seconds, instrument will not exceed this parameter when capturing
		 * \return number of captured spectra
		 */
		unsigned short captureSpectra(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
				unsigned short swirIntegrationTime_ms, unsigned short scanCount, unsigned short seriesMaxDuration_s, bool reuse_last_AIT_value = false);

		/**
		 * \brief	 Get instrument internal memory slots of last capture series for data acquisition
		 * \param slots pointer to preallocated array to be filled with memory slot addresses
		 * \param n_captures number of captures to obtain, should be equal to return value of captureSpec call
		 * \return number of slots assigned in array, should be equal to n_captures
		 */
		unsigned short getLastSpectraCaptureMemorySlots(unsigned short *pMemorySlotIdTarget, unsigned short numberOfCaptures);

		/**
		 * \brief 	Download spectra data from the instrument.
		 * \param slots pointer to list of memory slots to download from
		 * \param n_captures number of spectra to download
		 * \param spec_data pointer to allocated memory for spectra
		 * \return number of spectra downloaded
		 */
		unsigned short getSpectraFromMemorySlots(unsigned short *pMemorySlotIds, unsigned short numberOfCaptures, struct s_spectrum_dataset *pSpectraDataTarget);

		/**
		 * \brief	Download single spectrum data from the instrument.
		 * \param slot memory slot identifier to download
		 * \param spec_data pointer to preallocated memory to fill
		 * \return number of spectra downloaded
		 */
		unsigned short getSingleSpectrumFromMemorySlot(unsigned short memorySlotId, struct s_spectrum_dataset *pSpectraDataTarget);

		/**
		 * \brief 	Shorthand for "capture spectra" -> "get memory slots" -> "download spectra".
		 *  		Performs all the necessary actions without the need for manual intervention
		 * \param spec which radiometer to use: VNIR, SWIR or BOTH
		 * \param mux which optical channel to use: radiance, irradiance or dark
		 * \param vnir_inttime_ms VNIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param swir_inttime_ms SWIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param scan_count number of exposures to perform
		 * \param series_time_s maximum series duration in seconds, instrument will not exceed this parameter when capturing
		 * \return vector of Spectrum elements with capture results
		 */
		std::vector<Spectrum> acquireSpectraVector(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
				unsigned short swirIntegrationTime_ms, unsigned short scanCount, unsigned short seriesMaxDuration_s);

		/**
		 * \brief 	Shorthand for "capture spectra" -> "get memory slots" -> "download spectra".
		 *  		Performs all the necessary actions without the need for manual intervention
		 * \param spec which radiometer to use: VNIR, SWIR or BOTH
		 * \param mux which optical channel to use: radiance, irradiance or dark
		 * \param vnir_inttime_ms VNIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param swir_inttime_ms SWIR exposure time. 0 for autoexposure, value in range [1..65635] for fixed exposure time in ms.
		 * \param scan_count number of exposures to perform
		 * \param series_time_s maximum series duration in seconds, instrument will not exceed this parameter when capturing
		 * \param target Target memory location pointer, must be preallocated
		 * \return number of captures performed
		 */
		unsigned short acquireSpectra(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
				unsigned short swirIntegrationTime_ms, unsigned short scanCount, unsigned short seriesMaxDuration_s, s_spectrum_dataset *pSpectraTarget, bool reuse_last_AIT_value = false);

		/**
		 * \brief	Sets verbosity of this driver.
		 * \param	loglevel verbosity level
		 */
		void setLoglevel(e_loglevel loglevel);

		/**
		 * \brief	Sets prefix for all logs output by this driver.
		 * \param	logprefix the prefix added to the beginning of all the output log lines
		 */
		void setLogprefix(const char* logprefix);

		/**
		 * \brief	sets SWIR module thermal control setpoint
		 * \param float target temperature in 'C, must be in range [-15..40]
		 * \return status of execution: True if successful, false if not.
		 */
		bool setTECSetpoint(float setpoint_C);

		/**
		 * \brief	shuts down SWIR module thermal controller
		 * \return status of execution: True if successful, false if not.
		 */
		bool shutdown_TEC(void);

		/**
		 * \brief	reboot the instrument
		 * \return status of execution: True if successful (BOOTED packet got in response), false if not.
		 */
		bool reboot(void);

		/**
		 * \brief	listens on given serial port for instrument BOOTED message at default baudrate
		 * \param portname serial port name (e.g. /dev/ttyUSB0
		 * \param timeout_s for how long should we wait
		 * \return status of execution: True if there was a response, false if there were none.
		 */
		static bool waitForInstrumentToBoot(std::string portname, float timeout_s, e_loglevel loglevel = INFO);

		/**
		 * \brief	Fills firmware_info structure.
		 * \return status of execution: True if successful, false if not.
		 */
		bool getFirmwareInfo(void);


		/********************* UNSAFE! THESE CAN BRICK YOUR INSTRUMENT IF YOU TRY REAL HARD! **********************/

		/**
		 * \brief	Enters unsafe mode, in which instrument accepts firmware and calibration coefficient updates
		 * To exit this mode instrument needs to be rebooted
		 * \return status of execution: True if successful, false if not.
		 */
		bool enterFlashWriteMode(void);

		/**
		 * \brief	Sends extended calibration coefficients to the instrument.
		 * Must be called in flash write mode.
		 * \param pNewExternalCalibrationCoeficients pointer to structure with new coefficients to be saved.
		 * \return status of execution: True if successful, false if not.
		 */
		bool sendCalibrationCoefficients(s_extended_calibration_coefficients *pNewExternalCalibrationCoeficients);

		/**
		 * \brief	Sends extended calibration coefficients to the VM.
		 * \param pNewExternalCalibrationCoeficients pointer to structure with new coefficients to be saved.
		 * \return status of execution: True if successful, false if not.
		 */
		bool sendVMCalibrationCoefficients(s_vm_calibration_coefficients *pNewExternalCalibrationCoeficients);
		/**
		 * \brief	Saves previously sent extended calibration coefficients to the instrument flash.
		 * Must be called in flash write mode.
		 * \return status of execution: True if successful, false if not.
		 */
		bool saveCalibrationCoefficients(void);

		/**
		 * \brief	Sends new firmware data to the instrument.
		 * Must be called in flash write mode.
		 * \param filePath path to the binary file to send to the instrument
		 * \return status of execution: True if successful, false if not.
		 */
		bool sendNewFirmwareData(std::string filePath);

		/**
		 * \brief	Sends new VM firmware data to the instrument.
		 * \param filePath path to the binary file to send to the instrument
		 * \return status of execution: True if successful, false if not.
		 */
		bool sendNewVMFirmwareData(std::string filePath);

		/**
		 * \brief	Checks for dataset checksum and aves previously sent new firmware binary image to the instrument flash.
		 * Must be called in flash write mode.
		 * \return status of execution: True if successful, false if not.
		 */
		bool saveNewFirmwareData(void);

		/**
		 * \brief	Switches default firmware slot to boot into and reboots
		 * Must be called in flash write mode.
		 * \return status of execution: True if successful, false if not.
		 */
		bool switchFirmwareSlot(void);

		/**
		 * Intentionally undocumented :P
		 */
		bool sendDebugRequest(unsigned char *pPayload, int payloadLength, char *pResponseBuffer);

		/* Dumps a crash report into /tmp/hypstar/ dir */
		bool dumpFaultInfo(void);

		/**
		 * \brief	Sets up validation module for measurement and captures a series of spectra
		 * \param entrance sets default output power for given entrance
		 * \param source selects light source: VISible, SWIR 1350nm or SWIR 1500nm
		 * \param integration_time sets measurement integration time
		 * \param current overrides default output power
		 * \param pSpectraTarget is target array pointer where captured spectra are to be saved
		 * \param count: number of spectra to be captured
		 * \return status of execution: True if successful, false if not.
		 */
		bool measureVM(e_entrance entrance, e_vm_light_source source, unsigned short integration_time, float current, s_spectrum_dataset *pSpectraTarget, uint16_t count = 100);
		/********************* END OF UNSAFE **********************/

		/* General information about the instrument */
		struct s_booted hw_info;
		struct s_firwmare_info firmware_info;

		/* Hardware availability on this particular unit*/
		struct s_available_hardware available_hardware;
		/* Basic calibration coefficients, required for running (pixel to wavelength mapping, non-linearity compensation, etc) */
		struct s_calibration_coefficients_unpacked calibration_coefficients_basic;
		/* Advanced calibration coefficients, necessary for uncertainty evaluation */
		struct s_extended_calibration_coefficients extended_calibration_coefficients;

		static std::vector<s_hypstar_instance> instance_holder;
		// public for resting
		static int readPacket(LibHypstar::linuxserial *pSerial, unsigned char * buf, float timeout_s, unsigned char * pExpected = NULL);
		static LibHypstar::linuxserial* getSerialPort(std::string portname, int baudrate);
	private:
		unsigned char rxbuf[RX_BUFFER_PLUS_CRC32_SIZE];
		static bool is_log_message_waiting;
		Hypstar(LibHypstar::linuxserial *serial, e_loglevel loglevel, const char* logprefix);

		bool sendCmd(unsigned char cmd, unsigned char * pParameters, unsigned short paramLength);
		bool sendCmd(unsigned char cmd);
		bool sendAndWaitForAcknowledge(unsigned char cmd, unsigned char * pPacketParams, unsigned short packetParamLength, const char * pCommandNameString);
		bool waitForDone(unsigned char cmd, const char * pCommandNameString, float timeout_s);
		bool sendAndWaitForDone(unsigned char cmd, unsigned char* pPacketParams, unsigned short paramLength, const char* pCommandNameString, float timeout_s = 1);
		bool sendAndWaitForAckAndDone(unsigned char cmd, unsigned char * pPacketParams, unsigned short paramLength, const char * pCommandNameString, float timeout_s);
		int readData(unsigned char *pRxBuf, float timeout_s = READTIMEOUT);
		int exchange(unsigned char cmd, unsigned char * pPacketParams, unsigned short paramLength, const char * pCommandNameString, int retry_count = 5, float timeout_s = 0.5);
		int getPacketedData(char cmd, unsigned char * pPacketParams, unsigned short paramLength, unsigned char * pTargetMemory, const char * pCommandNameString);
		bool sendPacketedData(const char commandId, unsigned char * pDataSet, int datasetLength, const char *pCommandIdtring);
		void outputStream(FILE *stream, const char * type, const char* fmt, ...);
		int findInstrumentBaudrate(int expectedBaudrate);
		void logBinPacket(const char * direction, unsigned char * pPacket, int packetLength);
		static void logBinPacketStatic(const char * direction, unsigned char * pPacket, int packetLength);
		void logBytesRead(int rx_count, const char * expectedCommand, const char * pCommandNameString);
		void outputLog(e_loglevel level, const char* level_string, FILE *stream, const char* fmt, ...);
		static int checkPacketLength(unsigned char * pBuf, int lengthInPacketHeader, int packetLengthReceived);
		static bool checkPacketCRC(unsigned char *pBuf, unsigned short length);
		static void printLog(const char* prefix_string, const char* level_string, FILE *stream, const char* fmt, va_list args);
		static void printLogStatic(e_loglevel level_target, const char* level_string, FILE *stream, const char* fmt,  ...);
		static void signal_handler(int signal);
		bool waitForInitDone(void);

		LibHypstar::linuxserial *hnport; //serial port object
		s_outgoing_packet lastOutgoingPacket;
		e_loglevel _loglevel;
		static e_loglevel _loglevel_static;
		char* _log_prefix;
		static char* _log_prefix_static;
		unsigned short lastCaptureLongestIntegrationTime_ms;
};

#define REQUEST(x) exchange(x, NULL, 0, #x)
#define EXCHANGE(x, y, z) exchange(x, y, z, #x)
#define GET_PACKETED_DATA(x, y, z, q) getPacketedData(x, y, z, q, #x)
#define SEND_AND_WAIT_FOR_ACK(x, y, z) sendAndWaitForAcknowledge(x, y, z, #x)
#define SEND_AND_WAIT_FOR_DONE(x, y, z) sendAndWaitForDone(x, y, z, #x)
#define WAIT_FOR_DONE(x, y) waitForDone(x, #x, y)
#define SEND_AND_WAIT_FOR_ACK_AND_DONE(x, y, z, q) sendAndWaitForAckAndDone(x, y, z, #x, q)
#define SEND_PACKETED_DATA(x, y, z) sendPacketedData(x, y, z, #x)


// Wrapper for interfacing with C or Python via ctypes
extern "C"
{
	struct hs_object_holder;
	typedef struct hs_object_holder hypstar_t;

	struct s_libhypstar_version {
		int major;
		int minor;
		int revision;
		char hash[16];
	};

	hypstar_t *hypstar_init(const char *port, e_loglevel* loglevel = NULL, const char* logprefix = NULL);
	void hypstar_close(hypstar_t *hs);
	bool hypstar_wait_for_instrument(const char *port, float timeout_s, e_loglevel loglevel);

	bool hypstar_set_baudrate(hypstar_t *hs, e_baudrate new_baudrate);
	uint64_t hypstar_get_time(hypstar_t *hs);
	bool hypstar_set_time(hypstar_t *hs, uint64_t time);
	void hypstar_set_loglevel(hypstar_t *hs, e_loglevel loglevel);
	void hypstar_set_logprefix(hypstar_t *hs, const char* logprefix);
	bool hypstar_get_hw_info(hypstar_t *hs, s_booted *target);
	bool hypstar_get_env_log(hypstar_t *hs, unsigned char index, s_environment_log_entry *target);
	bool hypstar_get_calibration_coefficients_basic(hypstar_t *hs, s_calibration_coefficients_unpacked *coef_target);
	bool hypstar_get_calibration_coefficients_extended(hypstar_t *hs, s_extended_calibration_coefficients *ext_cal_coef_target);
	bool hypstar_get_calibration_coefficients_all(hypstar_t *hs, s_calibration_coefficients_unpacked *coef_target, s_extended_calibration_coefficients *ext_cal_coef_target);
	unsigned short hypstar_capture_spectra(hypstar_t *hs, enum e_radiometer spec, enum e_entrance mux,
			unsigned short vnir_inttime_ms, unsigned short swir_inttime_ms, unsigned short scan_count, unsigned short series_time_s, bool reuse_last_AIT_value);
	unsigned short hypstar_get_last_capture_memory_slots(hypstar_t *hs, unsigned short *target, unsigned short number_of_captures);
	unsigned short hypstar_download_spectra(hypstar_t *hs, unsigned short *memory_slots, unsigned short number_of_captures, s_spectrum_dataset *target);
	unsigned short hypstar_acquire_spectra(hypstar_t *hs, enum e_radiometer spec, enum e_entrance mux,
			unsigned short vnir_inttime_ms,	unsigned short swir_inttime_ms,	unsigned short scan_count, unsigned short series_time_s, s_spectrum_dataset *target, bool reuse_last_AIT_value);
	unsigned short hypstar_capture_JPEG_image(hypstar_t *hs, bool flip, bool mirror, bool auto_focus);
	int hypstar_download_JPEG_image(hypstar_t *hs, s_img_data_holder *target);
	bool hypstar_set_TEC_target_temperature(hypstar_t *hs, float target_temp_deg_C);
	bool hypstar_shutdown_TEC(hypstar_t *hs);
	bool hypstar_reboot(hypstar_t *hs);
	bool hypstar_enter_flash_write_mode(hypstar_t *hs);
	bool hypstar_send_calibration_coefficients(hypstar_t *hs, s_extended_calibration_coefficients *pNewExternalCalibrationCoeficients);
	bool hypstar_save_calibration_coefficients(hypstar_t *hs);
	bool hypstar_test_callback(hypstar_t *hs, void(*cb_function)(s_automatic_integration_time_adjustment_status *), int paramA, int paramB);
	bool hypstar_VM_enable(hypstar_t *hs, uint8_t enable);
	bool hypstar_VM_set_current(hypstar_t *hs, float current);
	bool hypstar_VM_measure(hypstar_t *hs, e_entrance entrance, e_vm_light_source source, unsigned short integration_time, float current, s_spectrum_dataset *pSpectraTarget, uint16_t scan_count);
	struct s_libhypstar_version getLibHypstarVersion(void);
}

#endif // include guard
