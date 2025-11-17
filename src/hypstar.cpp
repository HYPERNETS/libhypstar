#include "../inc/hypstar.h"

#include "crc32.h"
#include <string.h>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <iostream>

#define LOG_DEBUG(format, ...) outputLog(DEBUG, "DEBUG", stdout, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) outputLog(INFO, "INFO",  stdout, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) outputLog(ERROR, "ERROR", stderr, format, ##__VA_ARGS__)
#define LOG_TRACE(format, ...) outputLog(TRACE, "TRACE", stdout, format, ##__VA_ARGS__)

std::vector<Hypstar::s_hypstar_instance> Hypstar::instance_holder;

// set default log level
e_loglevel Hypstar::_loglevel_static = INFO;
bool Hypstar::is_log_message_waiting = false;
char* Hypstar::_log_prefix_static = NULL;
//unsigned char Hypstar::rxbuf[RX_BUFFER_PLUS_CRC32_SIZE];

#include <signal.h>

// static member function
void Hypstar::signal_handler(int signal) {
	LOG(INFO, stdout, "Got signal %d\n", signal);
	delete Hypstar::instance_holder[0].instance;
	exit(signal);
}

Hypstar::Hypstar(LibHypstar::linuxserial *serial, e_loglevel loglevel, const char* logprefix)
{
	// sigkill should kill it in a dirty way
	signal(SIGINT, Hypstar::signal_handler);
	signal(SIGTERM, Hypstar::signal_handler);
	hnport = serial;
	setLoglevel(loglevel);
	_log_prefix = NULL;

	if (logprefix == NULL)
		setLogprefix("");
	else
		setLogprefix(logprefix);

	CalculateCrcTable_CRC32();

	// wait 2.5 s until LED source boots up and reports S/N and FW
	usleep(2.5e6);

	// clean buffer in case we got some garbage, e.g. due to reboot caused by voltage drop
	hnport->emptyInputBuf();

	try {
		getHardWareInfo();
	}
	catch (eBadInstrumentState &e) {
		// we are in firmware upgrade mode, regular commands will fail now
		return;
	}
	catch (eHypstar &e)
	{
		LOG_INFO("Did not get response from instrument, will try different baud rates\n");
		int response = findInstrumentBaudrate(DEFAULT_BAUD_RATE);
		if(!response)
		{
			throw eHypstar();
		}
		LOG_INFO("Got baud rate %d\n", response);
	}

	// remap to proper bools for python
	available_hardware.vnir_module = hw_info.vnir_module_available;
	available_hardware.swir_module = hw_info.swir_module_available;
	available_hardware.optical_multiplexer = hw_info.optical_multiplexer_available;
	available_hardware.accelerometer = hw_info.accelerometer_available;
	available_hardware.camera = hw_info.camera_available;
	available_hardware.humidity_sensor = hw_info.humidity_sensor_available;
	available_hardware.pressure_sensor = hw_info.pressure_sensor_available;
	available_hardware.swir_tec_module = hw_info.swir_tec_module_available;
	available_hardware.validation_module = hw_info.vm_available;

	setTime(time(NULL));
	// log S/N and FW at all log levels
	outputLog((enum e_loglevel)0, "INFO",  stdout,
			"Instrument S/N: %d, FW revision: %d.%d.%d, MCU revision: %d, PSU revision: %d, VM S/N: %d, VM FW: %d.%d.%d\n",
			hw_info.instrument_serial_number, hw_info.firmware_version_major, hw_info.firmware_version_minor, hw_info.firmware_version_revision,
			hw_info.mcu_hardware_version, hw_info.psu_hardware_version,
			hw_info.vm_serial_number, hw_info.vm_firmware_version_major, hw_info.vm_firmware_version_minor, hw_info.vm_firmware_version_revision);
}

//destructor
Hypstar::~Hypstar()
{
	LOG_INFO("Called destructor!\n");

	// try reading out data from last command we might have interrupted
	try {
		readData(rxbuf, 0.5f);
	} catch (LibHypstar::eSerialReadTimeout &e) {
		// just ignore if there's no response, we wouldn't expect one normally anyway
	}
	setBaudRate(B_115200);
//	 destructor has to find and remove own entry from the instance_holder vector
	for (uint i = 0; i < Hypstar::instance_holder.size(); i++)
	{
		// if found, return pointer to that
		if (Hypstar::instance_holder[i].instance == this)
		{
			LOG_DEBUG("Found driver instance %p, index %d. Deleting...\n", static_cast<void*>(this), i);
			Hypstar::instance_holder.erase(Hypstar::instance_holder.begin()+i);
		}
	}
	delete hnport;
	free(_log_prefix);
}

// static member function
Hypstar* Hypstar::getInstance(std::string portname, e_loglevel* loglevel, const char *logprefix)
{
	if (loglevel != NULL)
		_loglevel_static = *loglevel;

	if (logprefix != NULL)
	{
		_log_prefix_static = (char*)realloc(_log_prefix_static, strlen(logprefix) + 1);

		if (_log_prefix_static)
			strcpy(_log_prefix_static, logprefix);
		else
		{
			LOG(ERROR, stderr, "Failed to reallocate memory for Hypstar::_log_prefix_static\n");
			return NULL;
		}
	}

	// log driver version at all log levels
	printLogStatic((enum e_loglevel)0, "INFO", stdout, "LibHypstar driver v%d.%d.%d (commit #%s)\n", DVER_MAJOR, DVER_MINOR, DVER_REVISION, DVER_HASH);

	Hypstar* h;

	// look through instance_holder for instance with the same portname
	for (s_hypstar_instance i : Hypstar::instance_holder)
	{
		// if found, return pointer to that
		if (portname.compare(i.port) == 0)
		{
			if (loglevel != NULL)
				i.instance->setLoglevel(*loglevel);

			if (logprefix != NULL)
				i.instance->setLogprefix(logprefix);

			LOG(DEBUG, stdout, "Returning existing driver instance %p\n", i.instance);
			return i.instance;
		}
	}

	LibHypstar::linuxserial *s = getSerialPort(portname, DEFAULT_BAUD_RATE);
	// otherwise instantiate and append to instance_holder
	try
	{
		h = new Hypstar(s, _loglevel_static, logprefix);
	}
	catch (eHypstar &e)
	{
		LOG(ERROR, stderr, "Could not establish communications with instrument\n");
		delete s;
		return NULL;
	}

	LOG(DEBUG, stdout, "Created driver instance %p\n", static_cast<void*>(h));
	s_hypstar_instance new_hs = {
			.port = portname,
			.instance = h
	};
	Hypstar::instance_holder.push_back(new_hs);
	return h;
	// destructor has to find and remove own entry from the vector
}

bool Hypstar::reboot(void)
{
	int timeout_s = 20;
	sendCmd(REBOOT);
	try
	{
		readData(rxbuf, timeout_s);
	}
	catch (eHypstar &e)
	{
		LOG(ERROR, stderr, "Did not receive BOOTED packet from the instrument during %.2fs\n", timeout_s);
		return false;
	}
	return true;
}

// static member function
bool Hypstar::waitForInstrumentToBoot(std::string portname, float timeout_s, e_loglevel loglevel)
{
	_loglevel_static = loglevel;

	LibHypstar::linuxserial *s = getSerialPort(portname, DEFAULT_BAUD_RATE);
	unsigned char buf[RX_BUFFER_PLUS_CRC32_SIZE];
	CalculateCrcTable_CRC32();
	try {
		int expected_size = PACKET_DECORATORS_TOTAL_SIZE + sizeof(struct s_booted);
		unsigned char expected_packet = 0xCB;
		int len = readPacket(s, buf, timeout_s, &expected_packet);
		logBinPacketStatic("<<", buf, len);

		if (((len == expected_size) || (len == expected_size -4)) && (checkPacketCRC(buf, len))) {
			LOG(INFO, stdout, "Got packet with length %d, expected length: %d, CRC matches, probably instrument\n", len, expected_size);
			delete s;
			return true;
		}
		else
		{
			LOG(INFO, stdout, "Got packet with length %d, expected length: %d, not sure if instrument\n", len, expected_size);
			delete s;
			return false;
		}
	}
	catch (LibHypstar::eSerialReadTimeout &e)
	{
		LOG(ERROR, stderr, "Did not receive BOOTED packet from the instrument during %.2fs\n", timeout_s);
		delete s;
		return false;
	}
	catch (ePacketLengthMismatch &e)
	{
		LOG(ERROR, stderr, "Bad BOOTED packet length! (length_in_header = %d, received %d)\n", e.lengthInPacket, e.packetLengthReceived);
		logBinPacketStatic("<<", buf, e.packetLengthReceived);
	}
	catch (eHypstar &e)
	{
		LOG(ERROR, stderr, "Caught unhandled exception while waiting for the instrument to boot (%s)\n", e.what());
		delete s;
		return false;
	}

	// should never reach here, just in case
	LOG(ERROR, stderr, "Something unexpected happened while waiting for the instrument to boot\n");
	delete s;
	return false;
}

bool Hypstar::getHardWareInfo(void)
{
	exchange(BOOTED, NULL, 0, "BOOTED", 1, 0.01, true);
	memcpy(&hw_info, (rxbuf + 3), sizeof(struct s_booted));
	if (hw_info.firmware_version_minor < 15) {
		hw_info.vnir_pixel_count = 0;
		hw_info.swir_pixel_count = 0;
	} else if (hw_info.firmware_version_minor < 18) {
		memcpy(&(hw_info.vnir_pixel_count), &rxbuf[ 3 + sizeof(s_booted)-11], 11);
		hw_info.vm_firmware_version_major = 0;
		hw_info.vm_firmware_version_minor = 0;
		hw_info.vm_firmware_version_revision = 0;
		hw_info.vm_serial_number = 0;
	}

	LOG_DEBUG("memory slots %hu, vnir=%d (%d), swir=%d (%d), mux=%d, cam=%d, accel=%d, rh=%d, pressure=%d, swir_tec=%d SD=%d, PM1=%d, PM2=%d VNIR pix=%d, SWIR pix=%d, 1MB device=%d, IA=%d, VM=%d, init=%d\n",
			hw_info.memory_slot_count, hw_info.vnir_module_available, hw_info.vnir_pixel_count, hw_info.swir_module_available, hw_info.swir_pixel_count, hw_info.optical_multiplexer_available, hw_info.camera_available,
			hw_info.accelerometer_available, hw_info.humidity_sensor_available, hw_info.pressure_sensor_available, hw_info.swir_tec_module_available, hw_info.sd_card_available, hw_info.power_monitor_1_available, hw_info.power_monitor_2_available,
			hw_info.vnir_pixel_count, hw_info.swir_pixel_count, hw_info.is_1MB_device, hw_info.isolated_adc, hw_info.vm_available, hw_info.hw_init_done);

	return true;
}

bool Hypstar::getCalibrationCoefficientsBasic(void)
{
	struct s_calibration_coefficients_raw *coefs_raw;
		char tmp[15];
		int rx_count;
		int i;

		// get standard set of calibration coefficients
		rx_count = REQUEST(GET_CAL_COEF);

		if ((rx_count - 4) != (int)sizeof(struct s_calibration_coefficients_raw))
		{
			LOG_ERROR("Received calibration coefficients data packet (%d) does not match the size of packet structure (%zu)\n", (rx_count - 4), sizeof(struct s_calibration_coefficients_unpacked));
			return false;
		}

		coefs_raw = (struct s_calibration_coefficients_raw *)(rxbuf + 3);

		// vnir wl coefs
		for (i = 0; i < 6; i++)
		{
			memcpy(tmp, coefs_raw->vnir_wavelength_coefficientss_raw + i * 14, 14);
			tmp[14] = 0;
			calibration_coefficients_basic.vnir_wavelength_coefficients[i] = atof(tmp);

			LOG_DEBUG("VNIR wl coef %d: \"%s\" = %+.7e\n", i, tmp, calibration_coefficients_basic.vnir_wavelength_coefficients[i]);
		}

		// vnir lin coefs
		for (i = 0; i < 8; i++)
		{
			memcpy(tmp, coefs_raw->vnir_linerity_coefficients_raw + i * 14, 14);
			tmp[14] = 0;
			calibration_coefficients_basic.vnir_linearity_coefficients[i] = atof(tmp);

			LOG_DEBUG("VNIR lin coef %d: \"%s\" = %+.7e\n", i, tmp, calibration_coefficients_basic.vnir_linearity_coefficients[i]);
		}

		// swir wl coefs
		for (i = 0; i < 5; i++)
		{
			memcpy(tmp, coefs_raw->swir_wavelength_coefficients_raw + i * 14, 14);
			tmp[14] = 0;
			calibration_coefficients_basic.swir_wavelength_coefs[i] = atof(tmp);

			// remove CR
			for (int j = 0; j < 14; j++)
				if (tmp[j] == 0x0D)
				{
					tmp[j] = 0;
					break;
				}

			LOG_DEBUG("SWIR wl coef %d: \"%s\" = %+.7e\n", i, tmp, calibration_coefficients_basic.swir_wavelength_coefs[i]);
		}

		// accelerometer cal coefs
		for (i = 0; i < 3; i++)
		{
			calibration_coefficients_basic.accelerometer_horizontal_reference[i] = coefs_raw->accelerometer_horizontal_reference[i];
			LOG_DEBUG("Accelerometer cal coef %d: %hu\n", i, calibration_coefficients_basic.accelerometer_horizontal_reference[i]);
		}
		return true;
}

bool Hypstar::getCalibrationCoefficientsExtended(void)
{
	unsigned char *p_cal_data;

	// get extended set of calibration coefficients
	p_cal_data = (unsigned char*)(&extended_calibration_coefficients);

	try
	{
		GET_PACKETED_DATA(GET_CAL_COEF, NULL, 0, p_cal_data);
	}

	// uninitialised flash is filled with 0xFF, including the CRC32 bytes.
	// in this case, skip the CRC32 check
	catch (eBadRxDatasetCRC &e)
	{
//		if (ext_cal_coefs.crc32 == 0xFFFFFFFF)
		{
			LOG_ERROR("Extended calibration coefficients not available (at least CRC32 isn't)\n");
		}
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Caught unhandled eHypstar exception (%s), failed to get calibration coefficients\n", e.what());
		return false;
	}

	return true;
}

bool Hypstar::getCalibrationCoefficientsAll(void)
{
	bool retval = false;
	retval = getCalibrationCoefficientsBasic();
	if (!retval) {
		return retval;
	}
	return getCalibrationCoefficientsExtended();
}

uint64_t Hypstar::getTime(void)
{
	uint64_t *tm;
	REQUEST(GET_SYSTIME);
	tm = (uint64_t *)(rxbuf + 3);
	return (*tm);
}


bool Hypstar::setTime(uint64_t time_s)
{
	LOG_DEBUG("Setting system time to %" PRIu64 "\n", time_s);
	// @TODO: repeated instantiation sends done response right away, at least with higher default baud rate
	EXCHANGE(SET_SYSTIME, (unsigned char *)&time_s, (unsigned short)sizeof(time_s));
	return true;
}

bool Hypstar::enableVM(bool enable)
{
	if (!hw_info.vm_available) {
		return false;
	}
	exchange(VM_ON, (unsigned char *) &enable, (unsigned short)sizeof(uint8_t), "VM_ON", 2, 5, true);
	return true;
}

bool Hypstar::getVmStatus(struct VM_Status_t *pTarget)
{
	try
	{
		LOG_DEBUG("VM status requested\n");
		exchange(VM_GET_STATUS, NULL, 0, "VM_GET_STATUS", 2, 0.2);
		memcpy(pTarget, (rxbuf + 3), sizeof(struct VM_Status_t));
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to get VM status\n");
		return false;
	}
	return true;
}

bool Hypstar::measureVM(e_entrance entrance, e_vm_light_source source, unsigned short integration_time, float current, s_spectrum_dataset *pSpectraTarget, uint16_t count) {
	if (!hw_info.vm_available) {
		return false;
	}
//	enableVM(true);
//	usleep(500000);
	// request measurement
	VM_Status_t *vm_status;
	s_vm_measurement_request_packet request = {
			.entrance = entrance,
			.light_source = source,
			.integration_time = integration_time,
			.current = current,
	};

	EXCHANGE(VM_MEASURE, (unsigned char *) &request, sizeof(s_vm_measurement_request_packet));
	int counter = 1;
	while(true)
	{
		try
		{
			readData(rxbuf, 20);
		}
		catch (LibHypstar::eSerialReadTimeout &e){
			LOG_ERROR("Serial timeout exception?\n");
			return false;
		}
//			catch (ePacketLengthMismatch &e)
//			{
//				LOG_ERROR("Bad packet length!\n");
//			}
//			catch (LibHypstar::eSerialSelectInterrupted &e) {
//				LOG_ERROR("Serial select interrupted\n");
//			}
//			catch (eBadResponse &e) {
//				// probably capture timeout, rethrow
//				throw e;
//			}
		catch (eHypstar &e)
		{
			if ((rxbuf[0] == NAK) && (rxbuf[3] == VM_MEASURE) && (rxbuf[4] == STATUS_TIMEOUT))
			{
				// capture VM command timed out on reaching setpoint
				LOG_ERROR("VM stabilisation timed out\n");
				return false;
			}
			else
				LOG_ERROR("Something else?\n");
		}

		if ((rxbuf[0] == DONE) && (rxbuf[3] == VM_MEASURE))
		{
			break;
		}

		else if (rxbuf[0] != VM_STATUS) {
			LOG_ERROR("Got unexpected packet %02x %02x %02x %02x\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
			hnport->emptyInputBuf();
			continue;
		}
		if (rxbuf[0] == VM_STATUS) {
			vm_status = (VM_Status_t *) &rxbuf[3];
			LOG_INFO("%d: VM setpoint: %2.2f, VM temperature: %2.2f, VM sink temp: %2.2f, current setting: %2.2f, voltage: %2.4f V\n", 
					counter, vm_status->temp_setpoint, vm_status->temp_current, 
					vm_status->temp_sink, vm_status->led_current, vm_status->led_voltage);
			counter++;
		}
	}
	e_radiometer radiometer = source == VM_LIGHT_VIS ? VNIR : SWIR;
	unsigned short vit = source == VM_LIGHT_VIS ? integration_time : 0;
	unsigned short sit = source == VM_LIGHT_VIS ? 0 : integration_time;

	acquireSpectra(radiometer, entrance, vit, sit, count, 0, pSpectraTarget, false);
//	enableVM(false);
	return true;
}

bool Hypstar::getSystemLogEntry(struct s_log_item *pTarget, unsigned char index)
{
	// do raw requests to avoid overwriting global rxBuffer
	unsigned char logRxBuf[200];

	try
	{
		LOG_DEBUG("SysLog index requested: %d\n", index);
		sendCmd(GET_LOG);
		readData(logRxBuf, 1);
		// @TODO: calculate timeout based on actual baudrate and expected data sizes + a couple of USB ticks at least
//		EXCHANGE(GET_LOG, &index, 1);
		memcpy(pTarget, (logRxBuf + 3), sizeof(struct s_log_item));
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to get SysLog\n");
		return false;
	}
	return true;
}

bool Hypstar::getEnvironmentLogEntry(struct s_environment_log_entry *pTarget, unsigned char index)
{
	try
	{
		LOG_DEBUG("Env index requested: %d\n", index);
		// @TODO: calculate timeout based on actual baudrate and expected data sizes + a couple of USB ticks at least
		EXCHANGE(GET_ENV, &index, 1);
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to get envlog\n");
		return false;
	}

	// energy meter info has changed new hardware version
	/* 2 power monitors x4 channels each
	 * FW < 0.15.x returns info only on channels enabled
	 * FW > 0.15.0 returns all channels, even if they are not measured or disabled on particular hardware
	 * HW > v4 has mix of 4 and 3 channel ones due to chip shortage (YAY CORONAA!)
	 * 3 channel ones should return 0 on ones with least interesting data (MUX and camera info, which can be deduced from general 12V and 3V3 data)
	 * HW > v4 also has changed output mappings to optimize layout:
	 * PM1 Ch1: 5V VNIR; Ch2: 3V3_MCU; Ch3: 3V3_common; Ch4: 3V3_camera
	 * PM2 Ch1: 12V SWIR; Ch2: 12V VM; Ch3: 12V_total; Ch4: 12V_MUX
	 *
	 * HW < v4 has:
	 * PM1 Ch1: 3v3_common; Ch2: 3V3_MCU; Ch3: 3V3_camera; Ch4: empty
	 * PM2 Ch1: 12V SWIR; Ch2: 12V MUX; Ch3: 5V VNIR; Ch4: 12V_total
	 */

	if ((hw_info.psu_hardware_version > 3) || (hw_info.firmware_version_minor >= 17)) {
		// map directly, since full-length (including 0 values) is provided by the instrument
		// also all the new hardware should have newer firmware anyway
		memcpy(pTarget, (rxbuf + 3), sizeof(struct s_environment_log_entry));
	}
	else
	{
		// Beginning of packet hasn't changed
		uint8_t offset = 38;
		memcpy(pTarget, (rxbuf + 3), offset);
		offset = offset +3;
		float *pFloats = (float*)&rxbuf[offset];
		int idx = 0;

		pTarget->energy_common_3v3 = pFloats[idx++];
		pTarget->energy_mcu_3v3 = pFloats[idx++];
		pTarget->energy_camera_3v3 = pFloats[idx++];
		if (hw_info.firmware_version_minor > 14)
		{
			pTarget->energy_validation_module_12v = pFloats[idx++];
		}
		pTarget->voltage_common_3v3 = pFloats[idx++];
		pTarget->voltage_mcu_3v3 = pFloats[idx++];
		pTarget->voltage_camera_3v3 = pFloats[idx++];
		if (hw_info.firmware_version_minor > 14)
		{
			pTarget->voltage_validation_module_12v = pFloats[idx++];
		}
		pTarget->current_common_3v3 = pFloats[idx++];
		pTarget->current_mcu_3v3 = pFloats[idx++];
		pTarget->current_camera_3v3 = pFloats[idx++];
		if (hw_info.firmware_version_minor > 14)
		{
			pTarget->current_validation_module_12v = pFloats[idx++];
		}
		pTarget->energy_swir_module_12v = pFloats[idx++];
		pTarget->energy_multiplexer_12v = pFloats[idx++];
		pTarget->energy_vnir_module_5v = pFloats[idx++];
		pTarget->energy_input_12v = pFloats[idx++];
		pTarget->voltage_swir_module_12v = pFloats[idx++];
		pTarget->voltage_multiplexer_12v = pFloats[idx++];
		pTarget->voltage_vnir_module_5v = pFloats[idx++];
		pTarget->voltage_input_12v = pFloats[idx++];
		pTarget->current_swir_module_12v = pFloats[idx++];
		pTarget->current_multiplexer_12v = pFloats[idx++];
		pTarget->current_vnir_module_5v = pFloats[idx++];
		pTarget->current_input_12v = pFloats[idx++];

		if (hw_info.firmware_version_minor < 15)
		{
			pTarget->energy_validation_module_12v = 0.0f;
			pTarget->voltage_validation_module_12v = 0.0f;
			pTarget->current_validation_module_12v = 0.0f;
		}
	}

	return true;
}

float Hypstar::convertRawAccelerometerReadingToGs(int16_t reading)
{
	// full scale is +-2g in signed short
	return reading * (2.0f / 32768.0f);
}

void Hypstar::convertRawAccelerometerDataToGs(int16_t *readings_ADU, float *out)
{
	for (int i = 0; i < 3; i++) {
		out[i] = convertRawAccelerometerReadingToGs(readings_ADU[i]);
	}
}

void Hypstar::convertRawAccelerometerDataToGsFromEnvLog(struct s_environment_log_entry *log, float *out)
{
	for (int i = 0; i < 3; i++) {
		out[i] = convertRawAccelerometerReadingToGs(log->accelerometer_readings_XYZ[i]);
	}
}

void Hypstar::convertAccelerationFromGsToMs(float *gs, float *ms) {
	for (int i = 0; i < 3; i++) {
		ms[i] = gs[i] * (9.8);
	}
}

void Hypstar::convertRawAccelerometerDataToMsFromEnvLog(struct s_environment_log_entry *log, float *out)
{
	convertRawAccelerometerDataToGsFromEnvLog(log, out);
	convertAccelerationFromGsToMs(out, out);
}

bool Hypstar::setBaudRate(e_baudrate baudRate)
{
	// @TODO: check for BR validity
	try
	{
		if (!SEND_AND_WAIT_FOR_ACK(SET_BAUD, (unsigned char *)&baudRate, (unsigned short)sizeof(baudRate)))
		{
			LOG_ERROR("Did not receive ACK when requestig new baud rate\n");
			return false;
		}

		/* switch terminal baudrate and wait for DONE packet from the instrument */
		hnport->setBaud((int)baudRate);
		readData(rxbuf, 1.0); // DONE

		if ((rxbuf[0] != DONE) || (rxbuf[3] != SET_BAUD))
		{
			LOG_ERROR("Did not receive DONE after switching baud rate\n");
			return false;
		}
		LOG_DEBUG("Successfully switched baudrate to %d\n", baudRate);
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to switch baud rate\n");
		return false;
	}

	return true;
}

int Hypstar::acquireJpegImage(bool flip, bool mirror, bool autoFocus, struct s_img_data_holder *pImageDatasetTarget)
{
	if (!hw_info.camera_available) {
		return 0;
	}
	s_capture_image_request_flags flags = {
			.scale = 0,
			.flip_v = flip,
			.mirror_h = mirror,
			.auto_focus = autoFocus
	};
	int img_size = captureJpegImage(JPG_5MP, flags, 30.0);
	if (!img_size) {
		return 0;
	}
	getImage(pImageDatasetTarget);

	return pImageDatasetTarget->image_size;
}

unsigned short Hypstar::captureImage(struct s_capture_image_request captureRequestParameters, float timeout_s)
{
	if (!hw_info.camera_available) {
		return 0;
	}
	// clear not used flags
	captureRequestParameters.flags.na = 0;

	try
	{
		if (!SEND_AND_WAIT_FOR_ACK_AND_DONE(CAPTURE_MM_IMG, (unsigned char*)&captureRequestParameters, (unsigned short)sizeof(struct s_capture_image_request), timeout_s))
		{
			LOG_ERROR("Failed to capture image\n");
			return 0;
		}
		return *((unsigned short*)(rxbuf + 4));
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to capture image\n");
	}

	return 0;
}


unsigned short Hypstar::captureJpegImage(enum e_jpg_resolution resolution, struct s_capture_image_request_flags flags, float timeout_s)
{
	if (!hw_info.camera_available) {
		return 0;
	}
	struct s_capture_image_request cap_img;

	switch(resolution)
	{
	case QQVGA:
		cap_img.resolution_h = 160;
		cap_img.resolution_v = 120;
		break;
	case QCIF:
		cap_img.resolution_h = 176;
		cap_img.resolution_v = 144;
		break;
	case QVGA:
		cap_img.resolution_h = 320;
		cap_img.resolution_v = 240;
		break;
	case WQVGA:
		cap_img.resolution_h = 400;
		cap_img.resolution_v = 240;
		break;
	case CIF:
		cap_img.resolution_h = 352;
		cap_img.resolution_v = 288;
		break;
	case VGA:
		cap_img.resolution_h = 640;
		cap_img.resolution_v = 480;
		break;
	case WVGA:
		cap_img.resolution_h = 800;
		cap_img.resolution_v = 600;
		break;
	case XGA:
		cap_img.resolution_h = 1024;
		cap_img.resolution_v = 768;
		break;
	case JPG_720p:
		cap_img.resolution_h = 1280;
		cap_img.resolution_v = 720;
		break;
	case SXGA:
		cap_img.resolution_h = 2080;
		cap_img.resolution_v = 960;
		break;
	case UXGA:
		cap_img.resolution_h = 1600;
		cap_img.resolution_v = 1200;
		break;
	case JPG_1080p:
		cap_img.resolution_h = 1920;
		cap_img.resolution_v = 1080;
		break;
	case WUXGA:
		cap_img.resolution_h = 1920;
		cap_img.resolution_v = 1200;
		break;
	case QXGA:
		cap_img.resolution_h = 2048;
		cap_img.resolution_v = 1536;
		break;
	case JPG_5MP:
	default:
		cap_img.resolution_h = 2592;
		cap_img.resolution_v = 1944;
		break;
	}

	cap_img.flags = flags;
	cap_img.flags.na = 0;
	cap_img.flags.auto_focus = false;
	cap_img.format = JPEG;

	return captureImage(cap_img, timeout_s);
}

int Hypstar::getImage(struct s_img_data_holder *pImageDatasetTarget)
{
	if (!hw_info.camera_available) {
		return 0;
	}

	int total_length = 0;

	try
	{
		memset(pImageDatasetTarget, 0, sizeof(struct s_img_data_holder));

		total_length = GET_PACKETED_DATA(GET_MM_IMG, NULL, 0, (unsigned char*)pImageDatasetTarget);
		pImageDatasetTarget->image_size = total_length;
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to get image\n");
	}

	return total_length;
}

bool Hypstar::waitForInitDone(void)
{
	uint8_t itr = 10;
	while (itr-- && !(hw_info.hw_init_done)) {
		LOG_INFO("Required hardware still initializing, retry in 1s\n");
		sleep(1);
		getHardWareInfo();
	}
	if (itr) {
		return true;
	}
	return false;
}

unsigned short Hypstar::captureSpectra(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
		unsigned short swirIntegrationTime_ms, unsigned short scanCount, unsigned short seriesMaxDuration_s, bool reuse_last_AIT_value)
{
	// check hardware
	// MUX should be the slowest
	if (!hw_info.optical_multiplexer_available)
	{
		if (!hw_info.hw_init_done)
		{
			bool r = waitForInitDone();
			if (!r && !hw_info.optical_multiplexer_available)
			{
				return 0;
			}
		} else {
			return 0;
		}
	}
	if ((spectrumType & VNIR) && !(hw_info.vnir_module_available))
	{
		return 0;
	}

	if ((spectrumType & SWIR) && !(hw_info.swir_module_available))
	{
		return 0;
	}

	struct s_capture_spectra_request_packet capture_spec_packet;
	unsigned short n_captures = 0;
	float timeout_s = READTIMEOUT;

	// zero-initialize all parameter flags
	capture_spec_packet.capture_spectra_parameters = s_spectrum_optical_configuration();

	if (spectrumType == VNIR || spectrumType == BOTH)
		capture_spec_packet.capture_spectra_parameters.vnir = 1;

	if (spectrumType == SWIR || spectrumType == BOTH)
		capture_spec_packet.capture_spectra_parameters.swir = 1;

	if (entranceType == RADIANCE)
		capture_spec_packet.capture_spectra_parameters.radiance = 1;

	if (entranceType == IRRADIANCE)
		capture_spec_packet.capture_spectra_parameters.irradiance = 1;

	if (reuse_last_AIT_value)
		capture_spec_packet.capture_spectra_parameters.start_from_last_AIT = 1;

	capture_spec_packet.vnir_integration_time_ms = vnirIntegrationTime_ms;
	capture_spec_packet.swir_integration_time_ms = swirIntegrationTime_ms;
	capture_spec_packet.scan_count = scanCount;
	capture_spec_packet.maximum_total_series_time_s = seriesMaxDuration_s;

	LOG_DEBUG("vnir=%d, swir=%d, L=%d, E=%d, v_it=%hu, s_it=%hu, scan_count=%hu, series_time=%hu\n",
			capture_spec_packet.capture_spectra_parameters.vnir, capture_spec_packet.capture_spectra_parameters.swir,
			capture_spec_packet.capture_spectra_parameters.radiance, capture_spec_packet.capture_spectra_parameters.irradiance,
			capture_spec_packet.vnir_integration_time_ms, capture_spec_packet.swir_integration_time_ms,
			capture_spec_packet.scan_count, capture_spec_packet.maximum_total_series_time_s);

	if (!SEND_AND_WAIT_FOR_ACK(CAPTURE_SPEC, (unsigned char *)&capture_spec_packet, (unsigned short)sizeof(struct s_capture_spectra_request_packet)))
	{
		LOG_ERROR("Failed to capture spectra\n");
		return 0;
	}

	try
	{
		// fixed integration time
		if (((capture_spec_packet.capture_spectra_parameters.vnir == 0) || (vnirIntegrationTime_ms != 0)) &&
				((capture_spec_packet.capture_spectra_parameters.swir == 0) || (swirIntegrationTime_ms != 0)))
		{
			LOG_DEBUG("Capture fixed IT\n");
			unsigned short max_inttime = (vnirIntegrationTime_ms > swirIntegrationTime_ms ? vnirIntegrationTime_ms : swirIntegrationTime_ms);
			lastCaptureLongestIntegrationTime_ms = max_inttime;

			if ((scanCount != 0) && (seriesMaxDuration_s == 0))
				// expected scan_count * inttime * 1.2 + 0.2 seconds
				timeout_s = CAPTURE_TIMEOUT_MULT * scanCount * (max_inttime / 1000.0 + CAPTURE_TIMEOUT_ADD_EACH) + CAPTURE_TIMEOUT_ADD;
			else if ((scanCount == 0) && (seriesMaxDuration_s != 0))
				// expected series_time * 1.2 + 0.2 seconds
				timeout_s = CAPTURE_TIMEOUT_MULT * seriesMaxDuration_s + CAPTURE_TIMEOUT_ADD;
			else if ((scanCount != 0) && (seriesMaxDuration_s != 0))
			{
				timeout_s = (((max_inttime / 1000.0 + CAPTURE_TIMEOUT_ADD_EACH) * scanCount) > seriesMaxDuration_s ?
								((max_inttime / 1000.0 + CAPTURE_TIMEOUT_ADD_EACH) * scanCount) : seriesMaxDuration_s);
				timeout_s = timeout_s * CAPTURE_TIMEOUT_MULT + CAPTURE_TIMEOUT_ADD;
			}
			else
			{
				// should receive error if both are 0
				timeout_s = CAPTURE_TIMEOUT_ADD;
			}

			LOG_DEBUG("Waiting for done fixed IT\n");
			if (!WAIT_FOR_DONE(CAPTURE_SPEC, timeout_s))
			{
				return 0;
			}
		}
		else // Automatic integration time: vnir_inttime_ms == 0 || swir_inttime_ms == 0
		{
			// DARK auto integration time should reuse last used exposure, otherwise it would adjust itself to infinity and beyond
			// @TODO: AIT capture timeout calculations are actually incorrect. I
			/* It takes about 1.5ms per VNIR spectrum to transfer data from radiometer to the instrument
			 * It takes about 4.5ms per SWIR spectrum to transfer data from radiometer to the instrument
			 * We also have to take into account ratios of integration times. Due to VNIR radiometer being more sensitive, it usually has much shorter integration times.
			 * Instrument then calculates maximum duration for slowest radiometer T = (n_captures * IT), then uses that for caluculating number of captures
			 * it can do with other radiometer n_captures_faster = (T / IT_faster).
			 * It can happen that SWIR has 2048ms while VIS has 1ms, which would mean that we do n*2048ms + n*2048*1ms number of acquisitions.
			 * Thus for single capture with above parameters we get timeout value of 2048ms + 2048 * 1.5ms or 5 seconds instead of expected 2-3s.
			 */
			if (entranceType == DARK)
			{
				timeout_s = scanCount * lastCaptureLongestIntegrationTime_ms * 1e-3 * CAPTURE_TIMEOUT_MULT + CAPTURE_TIMEOUT_ADD;
				/* If driver has been (re)instantiated between calls, but instrument has been working,
				 * We don't know what is the last capture integration time, so we default to maximum.
				 */
				if (!lastCaptureLongestIntegrationTime_ms)
				{
					timeout_s = 66;
				}
				WAIT_FOR_DONE(CAPTURE_SPEC, timeout_s);
			}
			else
			{
				lastCaptureLongestIntegrationTime_ms = 0;
				struct s_automatic_integration_time_adjustment_status status;
				int k = 1;
				float next_timeout;

				// use long timeout so that MUX has enough time for switching to the requested entrance
				timeout_s = 10.0;

				while(true)
				{
					try
					{
						readData(rxbuf, timeout_s);
					}
					catch (ePacketLengthMismatch &e)
					{
						LOG_ERROR("Bad packet length! (length_in_header = %d, received %d)\n", e.lengthInPacket, e.packetLengthReceived);
					}
					catch (LibHypstar::eSerialReadTimeout &e){
						LOG_ERROR("Serial timeout exception\n");

						// bail out, otherwise we'll reprocess the same package indefinitely
						return 0;
					}
					catch (LibHypstar::eSerialSelectInterrupted &e) {
						LOG_ERROR("Serial select interrupted\n");
						return 0;
					}
					catch (eBadResponse &e) {
						// probably capture timeout, rethrow
						throw e;
					}
					catch (eHypstar &e)
					{
						LOG_ERROR("Something else (%s)?\n", e.what());
					}

					if ((rxbuf[0] == DONE) && (rxbuf[3] == CAPTURE_SPEC))
					{
						break;
					}
					else if (rxbuf[0] != AUTOINT_STATUS) {
						LOG_ERROR("Got unexpected packet %02x %02x %02x %02x\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3]);
						hnport->emptyInputBuf();
						continue;
					}

					if (rxbuf[0] == AUTOINT_STATUS)
					{
						memcpy(&status, (rxbuf + 3), sizeof(struct s_automatic_integration_time_adjustment_status));
						next_timeout = status.next_integration_time_ms * 1e-3 * CAPTURE_TIMEOUT_MULT * scanCount + CAPTURE_TIMEOUT_ADD;

						// if vnir and swir packets are mixed and integration times
						// are different it is possible that the module with shorter
						// integration time reports next_inttime_ms that is too short
						// for the other module
						if (next_timeout > timeout_s)
						{
							timeout_s = next_timeout;
						}

						std::stringstream dbg_out;
						dbg_out << "Autoadjust inttime step " << k++ << ": spec=";

						if (status.spectrum_config.vnir && status.spectrum_config.swir)
							dbg_out << "both";
						else if (status.spectrum_config.vnir)
							dbg_out << "VNIR";
						else if (status.spectrum_config.swir)
							dbg_out << "SWIR";
						else
							dbg_out << "none";

						dbg_out << ", entrance=";
						if (status.spectrum_config.radiance)
							dbg_out << "L";
						else if (status.spectrum_config.irradiance)
							dbg_out << "E";
						else
							dbg_out << "dark";

						dbg_out << ", this_inttime_ms=" << status.current_integration_time_ms << ", peak_adc=" << status.peak_adc_value << ", next_inttime_ms=" << status.next_integration_time_ms << "\n",

						dbg_out << "vnir=" << status.spectrum_config.vnir << ", swir=" << status.spectrum_config.swir \
							<< ", radiance=" << status.spectrum_config.radiance << ", irradiance=" \
							<< status.spectrum_config.irradiance << ", slot=" << status.memory_slot_id;
						LOG_DEBUG("%s\n", dbg_out.str().c_str());

						// save last capture integration time, to provide correct read timeout, since DARK automatic integration time uses that
						lastCaptureLongestIntegrationTime_ms = status.next_integration_time_ms > lastCaptureLongestIntegrationTime_ms ? status.next_integration_time_ms : lastCaptureLongestIntegrationTime_ms;
					}
				}
			}
		} // inttime_ms == 0, automatic integration time is set now

		// rxbuf[0] == DONE
		n_captures = *((unsigned short*)(rxbuf + 4));

		LOG_DEBUG("Captured %d spectra\n", n_captures);
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to capture spectrum\n");
		return 0;
	}

	return n_captures;
}

unsigned short Hypstar::getLastSpectraCaptureMemorySlots(unsigned short *pMemorySlotIdTarget, unsigned short numberOfCaptures)
{
	unsigned short slot_count = 0;
	int i;
	slot_count = (GET_PACKETED_DATA(GET_SLOTS, NULL, 0, (unsigned char*)pMemorySlotIdTarget) / sizeof(unsigned short));

	if (slot_count != numberOfCaptures)
	{
		LOG_ERROR("Memory slot count (%d) does not match number of captured spectra (%d).\n", slot_count, numberOfCaptures);
		return 0;
	}

	std::stringstream dbg_out;
	dbg_out << "Captured " << numberOfCaptures << " spectra in slots";

	for (i = 0; i < numberOfCaptures; i++)
	{
		dbg_out << " " << pMemorySlotIdTarget[i];
	}

	LOG_DEBUG("%s\n", dbg_out.str().c_str());
	return slot_count;
}

unsigned short Hypstar::getSingleSpectrumFromMemorySlot(unsigned short memorySlotId, s_spectrum_dataset * pSpectraDataTarget)
{
	unsigned short spectrum_length = 0;
	memset(pSpectraDataTarget->spectrum_body, 0, MAX_SPEC_LENGTH * sizeof(unsigned short));

	int retries = 0;
	for (; retries < CMD_RETRY; retries++)
	{
		try
		{
			LOG_DEBUG("Downloading spectrum from slot %d\n", memorySlotId);
			spectrum_length = GET_PACKETED_DATA(GET_SPEC, (unsigned char*)&memorySlotId, (unsigned short)sizeof(memorySlotId), (unsigned char*)pSpectraDataTarget);
		}
		catch (eBadRxDatasetCRC &e)
		{
			LOG_ERROR("Bad spectrum CRC, retrying\n");
			continue;
		}
		LOG_DEBUG("Slot=%d, spectrum total_length=%d, crc_slot pointer = %p, target slot pointer = %p, crc32_in position = 0x%.8X\n",
				memorySlotId, spectrum_length, pSpectraDataTarget, (void*)((long)pSpectraDataTarget+spectrum_length-4),
				*((uint32_t*) ((long)pSpectraDataTarget+spectrum_length-4) ));
		break;
	}

	if (retries == CMD_RETRY)
	{
		LOG_ERROR("Failed to download spectrum from slot %d in %d retries\n", memorySlotId, retries);
		throw eHypstar();
//		return 0;
	}

	// copy over CRC32 to the correct position for SWIR dataset and remove CRC32 from spectral data body
	if (pSpectraDataTarget->spectrum_header.spectrum_config.swir)
	{
		memcpy(&pSpectraDataTarget->crc32_spaceholder, (uint32_t*) ((long)pSpectraDataTarget+spectrum_length-4), sizeof(typeof(pSpectraDataTarget->crc32_spaceholder)));
		memset(&pSpectraDataTarget->spectrum_body[256], 0, 4);
	}

	return spectrum_length;
}

unsigned short Hypstar::getSpectraFromMemorySlots(unsigned short *pMemorySlotIds, unsigned short numberOfCaptures, struct s_spectrum_dataset *pSpectraDataTarget)
{
	unsigned char *p_spec_data;
	unsigned short n = 0, n_success = 0;

	try
	{
		for (n = 0; n < numberOfCaptures; n++)
		{
			p_spec_data = (unsigned char*)(pSpectraDataTarget + n_success);

			if (getSingleSpectrumFromMemorySlot(pMemorySlotIds[n], (s_spectrum_dataset *)p_spec_data))
				n_success++;
			else
			{
				LOG_ERROR("Skipping spectrum in slot %d\n", pMemorySlotIds[n]);
				continue;
			}
		}
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Caught unhandled eHypstar exception (%s), failed to get spectrum\n", e.what());
	}

	return n_success;
}

unsigned short Hypstar::acquireSpectra(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
		unsigned short swirIntegrationTime_ms, unsigned short scanCount, unsigned short seriesMaxDuration_s, s_spectrum_dataset *pSpectraTarget, bool reuse_last_AIT_value)
{
	unsigned short cnt = captureSpectra(spectrumType, entranceType, vnirIntegrationTime_ms, swirIntegrationTime_ms, scanCount, seriesMaxDuration_s, reuse_last_AIT_value);
	unsigned short slots[cnt];
	getLastSpectraCaptureMemorySlots(slots, cnt);
	getSpectraFromMemorySlots(slots, cnt, pSpectraTarget);
	return cnt;
}

std::vector<Spectrum> Hypstar::acquireSpectraVector(enum e_radiometer spectrumType, enum e_entrance entranceType, unsigned short vnirIntegrationTime_ms,
		unsigned short swirIntegrationTime_ms,	unsigned short scanCount, unsigned short seriesMaxDuration_s)
{
	unsigned short cnt = captureSpectra(spectrumType, entranceType, vnirIntegrationTime_ms, swirIntegrationTime_ms, scanCount, seriesMaxDuration_s);
	unsigned short slots[cnt];
	getLastSpectraCaptureMemorySlots(slots, cnt);
	s_spectrum_dataset specs[cnt];
	getSpectraFromMemorySlots(slots, cnt, specs);

	std::vector<Spectrum> ret_val;
	for (int i = 0; i < cnt; i++) {
		ret_val.push_back(Spectrum(&specs[i]));
	}
	return ret_val;
}

bool Hypstar::setTECSetpoint(float setpoint_C)
{
	if (!hw_info.swir_tec_module_available)
	{
		return false;
	}

	if ((setpoint_C != TEC_OFF) && ((setpoint_C < MIN_TEC_SETPOINT) || (setpoint_C > MAX_TEC_SETPOINT)))
	{
		LOG_ERROR("TEC setpoint (%.1f) is outside the allowed range [%.1f ... %.1f]\n\n",
				setpoint_C, MIN_TEC_SETPOINT, MAX_TEC_SETPOINT);

		return false;
	}

	try
	{
		return SEND_AND_WAIT_FOR_ACK_AND_DONE(SET_SWIR_TEMP, (unsigned char *)&setpoint_C, (unsigned short)sizeof(setpoint_C), 120);
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("Failed to stabilize SWIR temperature\n");
		return false;
	}

	return true;
}

bool Hypstar::shutdown_TEC(void)
{
	return setTECSetpoint(-100);
}


/* UNSAFE FLASH FUNCTIONS */
bool Hypstar::enterFlashWriteMode(void)
{
	// dummy length, another protection method
	int fw_len = 100000;
	return SEND_AND_WAIT_FOR_DONE(ENTER_FLASH_WRITE_MODE, (unsigned char *)&fw_len, (unsigned short)sizeof(fw_len));
}

bool Hypstar::sendCalibrationCoefficients(s_extended_calibration_coefficients *pNewExternalCalibrationCoeficients)
{
	LOG_DEBUG("Starting calibration coefficient upload, ptr: %p\n", pNewExternalCalibrationCoeficients);
	LOG_DEBUG("Cal date in send coefs: %d-%d-%d\n", pNewExternalCalibrationCoeficients->calibration_year, pNewExternalCalibrationCoeficients->calibration_month, pNewExternalCalibrationCoeficients->calibration_day);
	// update dataset crc32
	pNewExternalCalibrationCoeficients->crc32 = 0;

	int total_length = sizeof(s_extended_calibration_coefficients);
	int crc32_buflen = ((total_length - 4) % 4) ?
			(total_length - 4) + 4 - ((total_length - 4) % 4) :
			(total_length - 4);

	int calc_crc32 = Compute_CRC32_BE(crc32_buflen, (unsigned char*)pNewExternalCalibrationCoeficients);
	pNewExternalCalibrationCoeficients->crc32 = calc_crc32;
	LOG_DEBUG("Calibration coefficient crc32: 0x%08X\n", pNewExternalCalibrationCoeficients->crc32);

	// According to "protection from user" logic, we should already be in FLASH_WRITE mode
	return SEND_PACKETED_DATA(SET_CAL_COEF, (unsigned char *) pNewExternalCalibrationCoeficients, sizeof(s_extended_calibration_coefficients));
}

bool Hypstar::sendVMCalibrationCoefficients(s_vm_calibration_coefficients *pNewExternalCalibrationCoeficients)
{
	LOG_DEBUG("Starting VM calibration coefficient upload, ptr: %p\n", pNewExternalCalibrationCoeficients);
	LOG_DEBUG("Cal date in send coefs: %d-%d-%d\n", pNewExternalCalibrationCoeficients->calibration_year, pNewExternalCalibrationCoeficients->calibration_month, pNewExternalCalibrationCoeficients->calibration_day);
	// update dataset crc32
	pNewExternalCalibrationCoeficients->crc32 = 0;

	int total_length = sizeof(s_vm_calibration_coefficients);
	int crc32_buflen = ((total_length - 4) % 4) ?
			(total_length - 4) + 4 - ((total_length - 4) % 4) :
			(total_length - 4);

	int calc_crc32 = Compute_CRC32_BE(crc32_buflen, (unsigned char*)pNewExternalCalibrationCoeficients);
	pNewExternalCalibrationCoeficients->crc32 = calc_crc32;
	LOG_DEBUG("Calibration coefficient crc32: 0x%08X\n", pNewExternalCalibrationCoeficients->crc32);

	return SEND_PACKETED_DATA(VM_CAL_DATA, (unsigned char *) pNewExternalCalibrationCoeficients, sizeof(s_vm_calibration_coefficients));
}

bool Hypstar::saveCalibrationCoefficients(void)
{
	return SEND_AND_WAIT_FOR_ACK_AND_DONE(SAVE_CAL_COEF, 0, 0, 5);
}

bool Hypstar::getFirmwareInfo(void)
{
	bool r = REQUEST(GET_FW_VER);
	if (r)
	{
		memcpy(&firmware_info, (rxbuf + 3), sizeof(struct s_firwmare_info));
		return true;
	}
	else
	{
		return false;
	}
	return true;
}

bool Hypstar::sendNewVMFirmwareData(std::string filePath) {
	std::ifstream binFile(filePath.c_str(), std::ios::in | std::ios::binary);
	if (!binFile) {
		LOG_ERROR("No firmware file in path given\n");
		return false;
	}
	std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(binFile), {});

	binFile.close();

	int crc32_buflen = ((buffer.size()) % 4) ?
			(buffer.size()) + 4 - ((buffer.size() ) % 4) :
			(buffer.size());

	uint32_t calc_crc32 = Compute_CRC32_BE(crc32_buflen, (unsigned char*)buffer.data());
	std::vector<unsigned char> crcvec(((unsigned char*)&calc_crc32), ((unsigned char*)&calc_crc32)+4);
	buffer.insert(buffer.end(), crcvec.begin(), crcvec.end());
	LOG_DEBUG("Firmware buffer length with CRC: %lu, CRC: %08X, in buf: %08X\n", buffer.size(), calc_crc32, *(uint32_t*)&buffer[buffer.size()-4]);

	unsigned int size = buffer.size();
	// notify the instrument about length of our new firmware
	SEND_AND_WAIT_FOR_ACK(VM_START_FW_UPLOAD, (unsigned char *)&size, 4);
	return SEND_PACKETED_DATA(VM_FIRMWARE_DATA, (unsigned char *) buffer.data(), size);
}

bool Hypstar::sendNewFirmwareData(std::string filePath) {
	std::ifstream binFile(filePath.c_str(), std::ios::in | std::ios::binary);
	if (!binFile) {
		LOG_ERROR("No firmware file in path given\n");
		return false;
	}
	std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(binFile), {});

	binFile.close();

	int crc32_buflen = ((buffer.size()) % 4) ?
			(buffer.size()) + 4 - ((buffer.size() ) % 4) :
			(buffer.size());

	uint32_t calc_crc32 = Compute_CRC32_BE(crc32_buflen, (unsigned char*)buffer.data());
	std::vector<unsigned char> crcvec(((unsigned char*)&calc_crc32), ((unsigned char*)&calc_crc32)+4);
	buffer.insert(buffer.end(), crcvec.begin(), crcvec.end());
	LOG_DEBUG("Firmware buffer length with CRC: %lu, CRC: %08X, in buf: %08X\n", buffer.size(), calc_crc32, *(uint32_t*)&buffer[buffer.size()-4]);

	unsigned int size = buffer.size();
	// notify the instrument about length of our new firmware
	SEND_AND_WAIT_FOR_DONE(ENTER_FLASH_WRITE_MODE, (unsigned char *)&size, 4);
	return SEND_PACKETED_DATA(FW_DATA, (unsigned char *) buffer.data(), size);
}

bool Hypstar::saveNewFirmwareData(void) {
	return exchange(SAVE_NEW_FW, 0, 0, "SAVE_NEW_FW", 1, 30, true);
}

bool Hypstar::switchFirmwareSlot(void) {
	memset(&firmware_info, 0, sizeof(s_firwmare_info));
	return exchange(BOOT_NEW_FW, 0, 0, "BOOT_NEW_FW", 1, 30, true);
}

bool Hypstar::sendDebugRequest(unsigned char *pPayload, int payloadLength, char *pResponseBuffer) {
	if (pPayload[0] == 0x68)	{
		sendCmd(DEBUG_COMMAND, pPayload, payloadLength);
		return true;
	}
	int responseLength =  exchange(DEBUG_COMMAND, pPayload, payloadLength, "DEBUG", 1, 0.1, true);
	if (pResponseBuffer && responseLength) {
		memcpy(pResponseBuffer, rxbuf, responseLength);
	}
	return true;
}

bool Hypstar::dumpFaultInfo(void) {
	char timestr[40];
	std::string path = "/tmp/hypstar/";
	struct tm *timenow;
	time_t now = time(NULL);
	timenow = gmtime(&now);

	struct stat st = {0};
	if (stat(path.c_str(), &st) == -1) {
		mkdir(path.c_str(), 0777);
	}
	strftime(timestr, sizeof(timestr), "HS_dump_%Y-%m-%d_%H-%M-%S.bin", timenow);
	std::string fn = std::string(timestr);
	unsigned char buf[4096];

	unsigned short total_length = GET_PACKETED_DATA(DEBUG_DUMP, NULL, 0, (unsigned char*)buf);
	std::ofstream outfile((path + fn).c_str(), std::ofstream::binary);
	outfile.write((const char *)buf, total_length);
	outfile.flush();
	outfile.close();
	LOG_ERROR("Dumped fault into %s\n", (path + fn).c_str());
	LOG_ERROR("Please send this file to support team for debugging\n");
	return true;
}

/* Serial interface */
int Hypstar::findInstrumentBaudrate(int expectedBbaudrate)
{
	for (auto br : {B_115200, B_460800, B_921600, B_3000000, B_6000000, B_8000000})
	{
		LOG_INFO("Trying baud rate %d\n", br);
		hnport->setBaud(br);
		try
		{
			while (true)
			{
				hnport->serialRead();
			}
		}
		catch (LibHypstar::eSerialReadTimeout &e){}
		try
		{
			getHardWareInfo();
			return br;
		}
		catch (LibHypstar::eSerialReadTimeout &e){}
		catch (eBadTxCRC &e)
		{
			// if we managed to unpack error, this is it
			return br;
		}
		catch (eBadRx &e) {
		}
		catch (eHypstar &e){
		}
	}
	return 0;
}

// static member function
LibHypstar::linuxserial* Hypstar::getSerialPort(std::string portname, int baudrate)
{
	LibHypstar::linuxserial *s;
	try
	{
		LOG(INFO, stdout, "Creating serial port (baud=%d, portname=%s)\n", DEFAULT_BAUD_RATE, portname.c_str());
		s = new LibHypstar::linuxserial(DEFAULT_BAUD_RATE, portname.c_str());
	}
	catch (LibHypstar::eSerialOpenFailed &e)
	{
		LOG(ERROR, stderr, "%s port open failed\n\n", portname.c_str());
		throw eHypstar();
	}

	LOG(INFO, stdout, "Got serial port\n");

	// clear serial buffer
	try
	{
		while (true)
			s->serialRead();
	}
	catch (LibHypstar::eSerialReadTimeout &e){}
	return s;
}

/* Communications protocol low level */
bool Hypstar::sendCmd(unsigned char cmd, unsigned char* pPacketParams, unsigned short paramLength)
{
	// packet length 2 octets little endian cmd(1) + length(2) + payload(len) + crc(1)
	unsigned short txlen = paramLength + PACKET_DECORATORS_TOTAL_SIZE;

	// round up to multiple of 32bit words for crc calculation (w/o crc byte itself, thus -1)
	unsigned short crclen;
	crclen = ((txlen - 1) % 4) ? ((txlen - 1) + 4 - ((txlen - 1) % 4)) : (txlen - 1);

	unsigned short buflen = crclen > txlen ? crclen : txlen;
	unsigned char crcbuf[buflen];
	unsigned int crc; //CRC of command + parameters

	// clear buffer
	memset(crcbuf, 0, buflen);

	crcbuf[0] = cmd;
	// packet length 2 octets little endian cmd(1) + length(2) + payload(len) + crc(1)
	crcbuf[1] = txlen & 0xFF;
	crcbuf[2] = (txlen >> 8) & 0xFF;

	// copy optional parameters
	if (paramLength)
	{
		memcpy(crcbuf + 3, pPacketParams, paramLength);
	}

	//calculate CRC
	crc = Compute_CRC32_BE(crclen, crcbuf);
	crcbuf[txlen - 1] = crc & 0xFF;

	LOG_DEBUG("sendCmd, len=%d, txlen=%d, crclen=%d, buflen=%d, crc=0x%.8X\n", paramLength, txlen, crclen, buflen, crc);
	logBinPacket(">>", crcbuf, txlen);

	lastOutgoingPacket.length = txlen;
	memcpy(lastOutgoingPacket.data, crcbuf, txlen);
	try
	{
		hnport->serialWrite(crcbuf, txlen);
	}
	catch(LibHypstar::eSerialError &e)
	{
		LOG_ERROR("%s: could not send command to spectrometer\n", __PRETTY_FUNCTION__);
		return false;
	}

	return true;
}


bool Hypstar::sendCmd(unsigned char cmd)
{
	return sendCmd(cmd, NULL, 0);
}

// static member function
int Hypstar::checkPacketLength(unsigned char * pBuf, int lengthInPacketHeader, int packetLengthReceived)
{
	if (packetLengthReceived < 3)
	{
		LOG(DEBUG, stdout, "received only %d bytes\n", packetLengthReceived);
		throw ePacketReceivedTooShort(packetLengthReceived);
	}

	if (packetLengthReceived != lengthInPacketHeader)
	{
		LOG(DEBUG, stdout, "received only %d bytes instead of %d\n", packetLengthReceived, lengthInPacketHeader);
		throw ePacketLengthMismatch(lengthInPacketHeader, packetLengthReceived, pBuf);
	}
	return packetLengthReceived;
}

// static member function
int Hypstar::readPacket(LibHypstar::linuxserial *pSerial, unsigned char * pBuf, float timeout_s, unsigned char * pExpected)
{
	uint16_t count = 0, length = 0;

	count += pSerial->serialRead(pBuf, 1, timeout_s);

	if (pExpected != NULL) {
		// we expect a specific packet and ignore all other garbage
		while (pBuf[0] != pExpected[0]) {
			LOG(DEBUG, stdout, "Expecting 0x%.2X, got 0x%.2X instead, discarding.\n", pExpected[0], pBuf[0]);
			pSerial->serialRead(pBuf, 1, timeout_s);
		}
	} else {
		// due to possible FTDI/serial bug, at higher baud rates with long cabling packet is prepended with 0xFF or 0xFE or 0x00
		// since we don't have any commands starting with 0xFy, we check for that and skip it
		while (((pBuf[0] & 0xF0) == 0xF0) || (pBuf[0] == 0x00)) {
			LOG(DEBUG, stdout, "Discarding garbage 0x%.2X at the start of package.\n", pBuf[0]);
			pSerial->serialRead(pBuf, 1, timeout_s);
		}
	}

	// read response code and packet length
	while (count < 3) {
		count += pSerial->serialRead(pBuf + count, 3 - count, timeout_s);
	}

	length = *((unsigned short*)(pBuf + 1));

	// at higher baud rates instrument responds with error on partial packets mis-decoded
	// this error is also misinterpreted at wrong baud rates. Length check used as a sanity-check here
	// & 0x7FFF because the highest bit is log message flag
	if ((length & 0x7FFF) > RX_BUFFER_PLUS_CRC32_SIZE) {
		LOG(DEBUG, stdout, "Length in packet (%d) is larger than buffer size (%d)\n", length & 0x7FFF, RX_BUFFER_PLUS_CRC32_SIZE);
		throw ePacketLengthMismatch(length & 0x7FFF, count, pBuf);
	}

	if (length & 0x8000) {
		is_log_message_waiting = true;
		// clear bit, we don't plan to have THAT large packets
		LOG(TRACE, stdout, "Log message waiting, changing RxBuf[2] %.2X->%.2X\n", pBuf[2], pBuf[2] & ~(0x80));
		length = length & ~(0x8000);
		pBuf[2] = pBuf[2] & ~(0x80);
	}

	try {
		while ((count < length) && (count < RX_BUFFER_PLUS_CRC32_SIZE)) {
			count += pSerial->serialRead(pBuf + count, length - count, timeout_s);
		}
	}
	catch (LibHypstar::eSerialReadTimeout &e){}

	int retval = checkPacketLength(pBuf, length, count);

	return retval;
}

// static member function
bool Hypstar::checkPacketCRC(unsigned char *pBuf, unsigned short length)
{
	unsigned short crc_buflen;
	unsigned char calc_crc, rx_crc;

	rx_crc = pBuf[length - 1];

	crc_buflen = ((length - 1) % 4) ? (length - 1) + 4 - ((length - 1) % 4) : (length - 1);

	if (crc_buflen > RX_BUFFER_PLUS_CRC32_SIZE)
	{
		LOG(ERROR, stderr, "Data buffer (len=%d) is too small for CRC calculation (required len=%d)\n", RX_BUFFER_PLUS_CRC32_SIZE, crc_buflen);
		throw eHypstar();
	}

	// pad with zeros if necessary (zero the crc byte too, hence -/+ 1)
	if (crc_buflen != (length - 1) && (crc_buflen <= RX_BUFFER_PLUS_CRC32_SIZE))
		memset(pBuf + length - 1, 0, crc_buflen - (length - 1));

	//calculate CRC
	calc_crc = Compute_CRC32_BE(crc_buflen, pBuf);
	if ((calc_crc & 0xFF) != rx_crc)
	{
		throw eBadRxPacketCRC(calc_crc, rx_crc);
	}

	return true;
}

int Hypstar::readData(unsigned char *pRxBuf, float timeout_s)
{
	int count = 0;
	unsigned int i;
	bool good_id = false;
	unsigned short length = 0, cmd_len = 0;
	unsigned char errcode;
	std::stringstream error_ss;

	LOG_DEBUG("readData timeout_sec = %.3f\n", timeout_s);
	auto t1 = std::chrono::high_resolution_clock::now();

	try {
		count = readPacket(hnport, pRxBuf, timeout_s);
	}
	catch (ePacketLengthMismatch &e) {
		LOG_DEBUG("Got garbage (length in header 0x%04X), wrong baud rate?\n", e.lengthInPacket);
		logBinPacket("<<", e.pBuf, e.packetLengthReceived);

		// try reading out extra, maybe we are getting mid-packet or something
		count = readData(pRxBuf, timeout_s);
		if (count) {
			LOG_DEBUG("Got extra %d bytes", count);
			logBinPacket("<<", pRxBuf, count);
		}
		throw eBadResponse();
	}

	auto t2 = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

	LOG_DEBUG("Got %d bytes in %d ms\n", count, duration);
	logBinPacket("<<", pRxBuf, count);

	for (i = 0; i < (sizeof(packet_identifiers) / sizeof(packet_identifiers[0])); i++)
	{
		if(pRxBuf[0] == packet_identifiers[i])
		{
			good_id = true;
			break;
		}
	}

	if (!good_id)
	{
		LOG_ERROR("Unknown packet identifier 0x%.2X\n", pRxBuf[0]);
		throw eBadID();
	}

	checkPacketCRC(pRxBuf, count);
	// check response code
	if (pRxBuf[0] == NAK)
	{
		// check for CRC error 0xE0 first
		// in the case of CRC error the packet received and returned by the instrument
		// is faulty and decoding it is meaningless
		if (pRxBuf[count - 2] == BAD_CRC)
		{
			LOG_ERROR("Spectrometer responded with error 0x%.2X - bad crc\n", BAD_CRC);
			char out[count*3 +3];
			for (int i = 0; i < count; i++)
			{
				sprintf(&out[i*3], "%.2X ", pRxBuf[i]);
			}
			LOG_ERROR("%s %s\n", "<<", out);
			throw eBadTxCRC();
		}

		// check for too_short error 0xE4 next
		// in the case of too_short error the packet received and returned by the instrument
		// is shorter than declared in the header and decoding it is meaningless
		if (pRxBuf[count - 2] == TOO_SHORT)
		{
			LOG_ERROR("Spectrometer responded with error 0x%.2X - too short\n", TOO_SHORT);
			throw eBadTxCRC();
		}

		// probably temporary fix to bug in fw that sends NOT_IMPLEMENTED back without
		// the original command
		// throw eBadTxCRC because the NOT_IMPLEMENTED error is occasionally
		// returned to command that is actually implemented (comms error with matching crc?)
		if (pRxBuf[count - 2] == NOT_IMPLEMENTED)
		{
			LOG_ERROR("Spectrometer responded with error 0x%.2X - not implemented\n", NOT_IMPLEMENTED);
			throw eBadTxCRC();
		}

		if (pRxBuf[count - 2] == CAM_ERROR_TIMEOUT)
		{
			LOG_ERROR("Spectrometer responded with error 0x%.2X - camera capture timeout (too dark?)\n", CAM_ERROR_TIMEOUT);
			throw eBadResponse();
		}

		if (pRxBuf[count - 2] == STATUS_TIMEOUT)
		{
			LOG_ERROR("Spectrometer responded with error 0x%.2X - timeout\n", STATUS_TIMEOUT);
			throw eBadResponse();
		}

		// Response packet: response code(1), packet_length(2), cmd_code(1), cmd_packet_length(2), rest_of_cmd_packet(...), error_code(1)
		cmd_len = *((unsigned short*)(pRxBuf + 4));


		unsigned short errcode2, parm2, n_errors;
		// AIT command does not send whole packet body, just the CMD identifier
		if (pRxBuf[3] == CAPTURE_SPEC)
		{
			errcode = BAD_PARM;
			errcode2 = pRxBuf[9];
			unsigned short vis_val, swir_val;
			vis_val = *(unsigned short*) &pRxBuf[10];
			swir_val = *(unsigned short*) &pRxBuf[12];
			parm2 = vis_val != 0 ? vis_val : swir_val;
			n_errors = 1;
		}

		// sanity check
		else if ((3 + cmd_len) > (count - 1))
		{
			LOG_ERROR("Command length (%hu) in error packet is too long compared to received data length (%hu)\n", cmd_len, count);
			throw eBadRx();
		}
		else
		{
			errcode = pRxBuf[3 + cmd_len];
			n_errors = (count - PACKET_DECORATORS_TOTAL_SIZE - lastOutgoingPacket.length -1) / 2;
		}

		error_ss << "Spectrometer responded with error " << std::hex <<  (unsigned short)errcode << std::dec << " - ";

		// Response packet: response code(1), packet_length(2), outgoing_command_packet(x), general_error_code, (n x {error_code(1), parm_no(1)}, crc(1)
		LOG_DEBUG("Len: %d, last packet: %d, Error count: %d\n", length, lastOutgoingPacket.length, n_errors);

		switch(errcode)
		{
		case BAD_CRC:
			LOG_ERROR("bad crc\n");
			throw eBadTxCRC();
			break;
		case BAD_LENGTH:
			LOG_ERROR("bad length\n");
			break;
		case BAD_PARM:
			error_ss << "bad parameters:\n";
			for (unsigned short j = 0; j < n_errors; j++)
			{
				if (pRxBuf[3] != CAPTURE_SPEC)
				{
					errcode2 = pRxBuf[3 + cmd_len + 1 + 2 * j];
					parm2 = pRxBuf[3 + cmd_len + 1 + 2 * j + 1];
				}

				switch(errcode2)
				{
				case PARM_OUT_OF_RANGE:
					error_ss << "Parameter " << parm2 << " error - out of range\n";
					break;
				case HW_NA:
					error_ss << "Parameter " << parm2 << " error - hardware not available\n";
					break;
				case WRONG_SPEC:
					error_ss << "Parameter " << parm2 << " error - wrong spectrometer selection, probably none active\n";
					break;
				case WRONG_OPTICS:
					error_ss << "Parameter " << parm2 << " error - wrong MUX setting, probably radiance and irradiance simultaneously\n";
					break;
				case NO_LIMIT:
					error_ss << "Parameter " << parm2 << " error - no limiting parameter has been provided (scan_count, series_time or DARK AUTO with no previous capture)\n";
					break;
				case INT_TOO_LONG:
					error_ss << "Parameter error - integration time too long: (" << parm2 << " ms) \n";
					break;
				case SEQ_TOO_LONG:
					error_ss << "Parameter " << parm2 << " error - series time too long\n";
					break;
				case BAD_IMG_TYPE:
					error_ss << "Parameter " << parm2 << " error - bad image type code\n";
					break;
				case BAD_RESOLUTION:
					error_ss << "Parameter " << parm2 << " error - bad image resolution\n";
					break;
				default:
					error_ss << "Parameter " << parm2 << " error -  - unknown error "  << errcode2 << "\n";
					break;
				}
			}
			break;
		case TOO_SHORT:
			LOG_ERROR("too short\n");
			break;
		case NOT_IMPLEMENTED:
			LOG_ERROR("command 0x%.2X not implemented\n", pRxBuf[3]);
			break;
		case BAD_STATE:
			LOG_ERROR("instrument is in firmware update mode\n");
			throw eBadInstrumentState();
			break;
		case MISSING_PARMS:
			LOG_ERROR("command too short to fill all the parameters\n");
			break;
		default:
			LOG_ERROR("unknown error code\n");
			break;
		}

		for (int i = 0; i < count; i++)
			error_ss << " " << std::hex << std::setfill('0') << std::uppercase << std::setw(2) << (unsigned short) pRxBuf[i];

		LOG_ERROR("%s\n", error_ss.str().c_str());

		throw eBadResponse();
	}

	return count;
}

int Hypstar::exchange(unsigned char cmd, unsigned char* pPacketParams, unsigned short paramLength, const char* pCommandNameString, int retry_count, float timeout_s, bool readSyslog)
{
	int receivedByteCount = 0;
	bool resend = false;
	for (int i = 0; i < retry_count; i++)
	{
		try
		{
			if (resend)
			{
				LOG_DEBUG("%s: sendCmd(RESEND)\n", __PRETTY_FUNCTION__);
				hnport->emptyInputBuf();
				sendCmd(RESEND);
				resend = false;
			}
			else
			{
				LOG_DEBUG("%s: sendCmd(%s)\n", __PRETTY_FUNCTION__, pCommandNameString);
				sendCmd(cmd, pPacketParams, paramLength);
			}

			try
			{
				receivedByteCount = readData(rxbuf, timeout_s);

				while (readSyslog && is_log_message_waiting)
				{
					is_log_message_waiting = false;
					s_log_item l;
					if (getSystemLogEntry(&l, 0))
					{
						if (l.log_type == LOG_ERROR)
						{
							LOG_ERROR("SYSLOG ERROR [%" PRId64 "]: %.*s\n", l.timestamp, l.body_length, l.body.message);
							const char hf[10] = "Hardfault";
							if (memcmp(l.body.message, hf, 9) == 0) {
								dumpFaultInfo();
							}
						}
						else
						{
							LOG_DEBUG("SYSLOG DEBUG [%" PRId64 "]: %.*s\n", l.timestamp, l.body_length, l.body.message);
						}
					}
				}
			}
			catch (eBadTxCRC &e)
			{
				// try again if instrument reported Tx crc error
				LOG_DEBUG("Instrument reported TX CRC error\n");
				char out[(paramLength+10)*3 +3];
				sprintf(out, "%.2X ", cmd);
				int l = paramLength + 2 + 2;
				sprintf(&out[3], "%.2X ", l & 0xFF);
				sprintf(&out[6], "%.2X ", (l >> 8) & 0xFF);
				for (int i = 0; i < paramLength; i++)
				{
					sprintf(&out[9+i*3], "%.2X ", pPacketParams[i]);
				}
				LOG_DEBUG("%s %s\n", "Was >>", out);
				continue;
			}
			catch (ePacketLengthMismatch &e) {
				LOG_DEBUG("Got %d bytes instead of %d\n", e.packetLengthReceived, e.lengthInPacket);
				logBinPacket("<<", e.pBuf, e.packetLengthReceived);
				continue;
			}
			catch (ePacketReceivedTooShort &e) {
				LOG_DEBUG("Too short! Got %d bytes\n", e.length);
				continue;
			}
			catch (eBadRx &e)
			{
				LOG_DEBUG("Got garbage from instrument, requesting repeat\n");
				resend = true;
				continue;
			}
			catch (LibHypstar::eSerialReadTimeout &e)
			{
				LOG_DEBUG("Timed out, rerequesting\n");
				continue;
			}
		}
		catch (eBadInstrumentState &e) {
			throw e;
		}
		catch (eHypstar &e)
		{
			LOG_ERROR("Failed to get %s packet\n", pCommandNameString);
			throw e;
		}

		break;
	}
	if (!receivedByteCount) {
		LOG_ERROR("No response to %s command\n", pCommandNameString);
		throw eHypstar();
	}
	return receivedByteCount;
}

int Hypstar::getPacketedData(char cmd, unsigned char * pPacketParams, unsigned short paramLength, unsigned char * pTargetMemory, const char * pCommandNameString)
{
	unsigned short packet_count = 0;
	unsigned short data_len = 0;
	int total_length = 0;
	unsigned char *dataset_tail = pTargetMemory;

	unsigned char param_holder[paramLength +sizeof(unsigned short)];
	unsigned short *packet_id = (unsigned short*)&param_holder[paramLength];
	*packet_id = 0;
	int packet_param_len = sizeof(param_holder);

	if (pPacketParams) {
		memcpy(param_holder, pPacketParams, paramLength);
	}

	do
	{
		LOG_DEBUG("packet=%hu/%hu, data_len=%hu\n", *packet_id + 1, packet_count, data_len);
		exchange(cmd, param_holder, packet_param_len, pCommandNameString, true);
		data_len = *((unsigned short*)(rxbuf + 1)) - 1 - 2 - 2 - 2 - 1;
		packet_count = *((unsigned short*)(rxbuf + 5));
		memcpy(dataset_tail, rxbuf + 7, data_len);
		dataset_tail += data_len;
		total_length += data_len;
	} while (++(*packet_id) < packet_count);

	// check crc of dataset
	// crc32 was last 4 bytes of payload
	unsigned int rx_crc32 = *(unsigned int*)(dataset_tail - 4);
	// clear provided CRC32, we might need to append 0x00 to match 32-bit boundaries
	// this is to avoid copying whole dataset around
	*(unsigned int*)(dataset_tail - 4) = 0;
	unsigned int crc32_buflen = ((total_length - 4) % 4) ?
			(total_length - 4) + 4 - ((total_length - 4) % 4) :
			(total_length - 4);
	unsigned int calc_crc32 = Compute_CRC32_BE(crc32_buflen, (unsigned char*)pTargetMemory);

	// put back CRC32
	*(unsigned int*)(dataset_tail - 4) = rx_crc32;

	LOG_DEBUG("Dataset total length=%d, crc32_buflen=%d, calc_crc32=0x%.8X, rx_crc32=0x%.8X\n",
					total_length, crc32_buflen, calc_crc32, rx_crc32);

		/* @TODO: Outstanding bug in firmware, where CRC32 of GET_SLOTS dataset is not appended. Will get fixed in further FW release */
	if (((cmd & 0xFF) != GET_SLOTS) && ((cmd & 0xFF) != DEBUG_DUMP)) {
		if (calc_crc32 != rx_crc32) {
			LOG_ERROR("Dataset CRC32 mismatch!\n");
			// application should decide whether to do re-request
			throw eBadRxDatasetCRC(calc_crc32, rx_crc32);
		}
	}
	LOG_DEBUG("Dataset CRC32 matches\n");

	return total_length;
}

bool Hypstar::sendPacketedData(const char commandId, unsigned char * pDataSet, int datasetLength, const char *pCommandIdString)
{
	unsigned short totalPacketCount = ceil((float)datasetLength/DATA_PACKET_BODY_SIZE_MAX);
	unsigned char *pDatasetHead = pDataSet;

	unsigned char currentPacket[PACKET_BODY_SIZE_MAX];
	unsigned short *pPacketNumber = (unsigned short*)&currentPacket[0];
	*(unsigned short*) &currentPacket[2] = totalPacketCount;
	*pPacketNumber = 0;
	unsigned short packetLength = datasetLength >= DATA_PACKET_BODY_SIZE_MAX ? DATA_PACKET_BODY_SIZE_MAX : datasetLength;
	long datasetEndAddress = (long)pDataSet + (long)datasetLength;
	do
	{
		memcpy(&currentPacket[4], pDatasetHead, packetLength);
		long chunk = (long)pDatasetHead-(long)pDataSet;
		LOG_DEBUG("Sending packet %d/%d (bytes [%lu..%lu]/%lu) \n", *pPacketNumber+1, totalPacketCount, chunk, chunk+packetLength,  datasetLength);
		// @TODO: should unify this in firmware (FW returns DONE, while CAL_COEFS returns ACK on last packet
		// +DATA_PACKET_ID_LEN_SIZE packetLength here to include space for packet number/total number without affecting tracking of location in dataset
		if (((commandId & 0xFF) == FW_DATA) && ((totalPacketCount-1) == *pPacketNumber))
		{
			sendAndWaitForDone(commandId, currentPacket, packetLength+DATA_PACKET_ID_LEN_SIZE, pCommandIdString, 5);
		}
		else
		{
			sendAndWaitForAcknowledge(commandId, currentPacket, packetLength+DATA_PACKET_ID_LEN_SIZE, pCommandIdString);
		}

		// Increment dataset pointer
		pDatasetHead = pDatasetHead + packetLength;
		// Check and adjust length of next packet
		if (((long)pDatasetHead + DATA_PACKET_BODY_SIZE_MAX) > datasetEndAddress)
		{
			LOG_TRACE("Start: %p, Head: %p, next head: %p, end: %p\n", pDataSet, pDatasetHead, (char*)(pDatasetHead + DATA_PACKET_BODY_SIZE_MAX), (char*)datasetEndAddress);
			packetLength = (long)datasetEndAddress - (long)pDatasetHead;
		}

		LOG_DEBUG("Instrument accepted packet %d/%d\n", ++*pPacketNumber, totalPacketCount);
	} while (*pPacketNumber < totalPacketCount);
	return false;
}

bool Hypstar::sendAndWaitForAcknowledge(unsigned char cmd, unsigned char* pPacketParams, unsigned short packetParamLength, const char* pCommandNameString)
{
	unsigned short receivedByteCount = 0;

	try
	{
		receivedByteCount = exchange(cmd, pPacketParams, packetParamLength, pCommandNameString, 1, 2, false);

		if ((rxbuf[0] != ACK))
		{
			logBytesRead(receivedByteCount, "ACK", pCommandNameString);
			return false;
		}
		LOG_DEBUG("Got ACK for %s\n", pCommandNameString);
	}
	catch (eHypstar &e)
	{
		LOG_ERROR("sendAckCmd failed with exception (%s), cmd = %.2X\n", e.what(), cmd);
		return false;
	}

	return true;
}

bool Hypstar::waitForDone(unsigned char cmd, const char* cmd_str, float timeout_s) {
	unsigned short receivedByteCount;
	int retryCount;

	// DONE
	for (retryCount = 0; retryCount < CMD_RETRY; retryCount++)
	{
		try
		{
			receivedByteCount = readData(rxbuf, timeout_s);
		}
		catch (LibHypstar::eSerialReadTimeout &e)
		{
			LOG_ERROR("Timeout while waiting for done!\n");
			return false;
		}

		if (rxbuf[0] == DONE)
		{
			if (rxbuf[3] == cmd)
			{
				LOG_DEBUG("Got DONE for %s\n", cmd_str);
				return true;
			}
			else
			{
				LOG_ERROR("Got unexpected DONE for %s\n", cmd_str);
			}
		}
	}

	if (retryCount == CMD_RETRY)
	{
		logBytesRead(receivedByteCount, "DONE", cmd_str);
		return false;
	}
	LOG_ERROR("NO DONE for %s\n", cmd_str);
	return false;
}

bool Hypstar::sendAndWaitForAckAndDone(unsigned char cmd, unsigned char* pPacketParams, unsigned short paramLength, const char* pCommandNameString, float timeout_s)
{
	if (!sendAndWaitForAcknowledge(cmd, pPacketParams, paramLength, pCommandNameString))
		return false;

	return waitForDone(cmd, pCommandNameString, timeout_s);
}

bool Hypstar::sendAndWaitForDone(unsigned char cmd, unsigned char* pPacketParams, unsigned short paramLength, const char* pCommandNameString, float timeout_s)
{
	sendCmd(cmd, pPacketParams, paramLength);
	return waitForDone(cmd, pCommandNameString, timeout_s);
}

/* Logging */
// static member function
void Hypstar::printLog(const char* prefix_string, const char* level_string, FILE *stream, const char* fmt, va_list args)
{
	time_t now = time(NULL);
	auto tm = localtime(&now);
	char timebuf[22];
	strftime(timebuf, 20, "%Y-%m-%dT%H:%M:%S", tm);
	fprintf(stream, "%s[%s]\t[%s] ", prefix_string, level_string, timebuf);
	vfprintf(stream, fmt, args);
	fflush(stream);
}

// static member function
void Hypstar::printLogStatic(e_loglevel level_target, const char* level_string, FILE *stream, const char* fmt,  ...)
{
	if (_loglevel_static >= level_target)
	{
		va_list args;
		va_start(args, fmt);
		if (_log_prefix_static != NULL)
			printLog(_log_prefix_static, level_string, stream, fmt, args);
		else
			printLog("", level_string, stream, fmt, args);
		va_end(args);
	}
}

void Hypstar::outputLog(e_loglevel level, const char* level_string, FILE *stream, const char* fmt, ...)
{
	if (_loglevel >= level)
	{
		va_list args;
		va_start(args, fmt);
		printLog(_log_prefix, level_string, stream, fmt, args);
		va_end(args);
	}
}

void Hypstar::logBinPacket(const char* direction, unsigned char* packet, int len)
{
	char out[len*3 +3];
	out[0] = 0;
	for (int i = 0; i < len; i++)
	{
		sprintf(&out[i*3], "%.2X ", packet[i]);
	}
	LOG_TRACE("%s %s\n", direction, out);
}

// static member function
void Hypstar::logBinPacketStatic(const char* direction, unsigned char* packet, int len)
{
	char out[len*3 +3];
	out[0] = 0;

	for (int i = 0; i < len; i++)
		sprintf(&out[i*3], "%.2X ", packet[i]);

	LOG(TRACE, stdout, "%s %s\n", direction, out);
}

void Hypstar::logBytesRead(int rx_count, const char * expectedCommand, const char * cmd)
{
	char out[rx_count*3 +16];
	for (int i = 0; i < rx_count; i++)
	{
		sprintf(&out[i*3], "%.2X ", rxbuf[i]);
	}
	LOG_ERROR("Did not receive %s for command %.2X in %d attempts\n", expectedCommand, cmd, CMD_RETRY);
	LOG_ERROR("%d bytes read: %s\n", rx_count, out);
}

void Hypstar::setLoglevel(e_loglevel loglevel)
{
	_loglevel = loglevel;
}

void Hypstar::setLogprefix(const char* logprefix)
{
	_log_prefix = (char*)realloc(_log_prefix, strlen(logprefix) + 1);

	if (_log_prefix)
		strcpy(_log_prefix, logprefix);
	else
	{
		LOG(ERROR, stderr, "Failed to reallocate memory for Hypstar::_log_prefix\n");
	}
}

/* C wrapper functions */
struct hs_object_holder
{
	void *hs_instance;
};

static std::vector<hypstar_t*> object_holder_instances;

hypstar_t* hypstar_init(const char *port, e_loglevel* loglevel, const char* logprefix)
{
	hypstar_t *hs_wrapper;
	Hypstar *obj;
	try
	{
		obj = Hypstar::getInstance(port, loglevel, logprefix);
		if (!obj)
		{
			delete static_cast<Hypstar *> (obj);
			return NULL;
		}

	}
	catch (eHypstar &e)
	{
		return NULL;
	}

	/* Try getting instance. Duplicate port checks are done within class itself
	 * If returned instance is already in some wrapper, return that wrapper
	*/
	for (hypstar_t *i : object_holder_instances)
	{
		if (i->hs_instance == obj)
			return i;
	}
	// otherwise instantiate new wrapper and append to object_holder_instances
	hs_wrapper = (typeof(hs_wrapper)) malloc(sizeof(*hs_wrapper));
	hs_wrapper->hs_instance = obj;
	object_holder_instances.push_back(hs_wrapper);
	return hs_wrapper;
}

void hypstar_close(hypstar_t *hs)
{
	if (hs == NULL)
	{
		return;
	}

	// find and delete existing wrapper from vector
	for (uint i = 0; i < object_holder_instances.size(); i++)
	{
		if (object_holder_instances[i]->hs_instance == hs->hs_instance)
		{
			object_holder_instances.erase(object_holder_instances.begin()+i);
			delete static_cast<Hypstar *> (hs->hs_instance);
			delete hs;
		}
	}
}

void hypstar_set_loglevel(hypstar_t *hs, e_loglevel loglevel)
{
	if (hs == NULL)
		return;

	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	instance->setLoglevel(loglevel);
}

void hypstar_set_logprefix(hypstar_t *hs, const char* logprefix)
{
	if (hs == NULL)
		return;

	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	instance->setLogprefix(logprefix);
}

uint64_t hypstar_get_time(hypstar_t *hs)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->getTime();
}

bool hypstar_set_time(hypstar_t *hs, uint64_t time)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	instance->setTime(time);
	return true;
}

bool hypstar_get_hw_info(hypstar_t *hs, s_booted *target)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	bool response = instance->getHardWareInfo();
	if (!response)
	{
		return response;
	}
	memcpy(target, &instance->hw_info, sizeof(instance->hw_info));
	return response;
}

bool hypstar_get_env_log(hypstar_t *hs, unsigned char index, s_environment_log_entry *target)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	bool response = instance->getEnvironmentLogEntry(target, index);

	return response;
}

bool hypstar_get_calibration_coefficients_basic(hypstar_t *hs, s_calibration_coefficients_unpacked *coef_target)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	bool response = instance->getCalibrationCoefficientsBasic();
	if (!response)
	{
		return response;
	}
	memcpy(coef_target, &instance->calibration_coefficients_basic, sizeof(s_calibration_coefficients_unpacked));

	return response;
}

bool hypstar_get_calibration_coefficients_extended(hypstar_t *hs, s_extended_calibration_coefficients *ext_cal_coef_target)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	bool response = instance->getCalibrationCoefficientsExtended();
	if (!response)
	{
		return response;
	}
	memcpy(ext_cal_coef_target, &instance->extended_calibration_coefficients, sizeof(s_extended_calibration_coefficients));

	return response;
}

bool hypstar_get_calibration_coefficients_all(hypstar_t *hs, s_calibration_coefficients_unpacked *coef_target, s_extended_calibration_coefficients *ext_cal_coef_target)
{
	if (hs == NULL)
	{
		return false;
	}
	bool response = hypstar_get_calibration_coefficients_basic(hs, coef_target);
	if (!response)
	{
		return response;
	}
	return hypstar_get_calibration_coefficients_extended(hs, ext_cal_coef_target);
}

unsigned short hypstar_capture_spectra(hypstar_t *hs, enum e_radiometer spec, enum e_entrance mux,
		unsigned short vnir_inttime_ms, unsigned short swir_inttime_ms, unsigned short scan_count, unsigned short series_time_s, bool reuse_last_AIT_value)
{
	if (hs == NULL)
	{
		return false;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->captureSpectra(spec, mux, vnir_inttime_ms, swir_inttime_ms, scan_count, series_time_s, reuse_last_AIT_value);
}

unsigned short hypstar_get_last_capture_memory_slots(hypstar_t *hs, unsigned short *target, unsigned short number_of_captures)
{
	if ((hs == NULL) || (target == NULL))
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->getLastSpectraCaptureMemorySlots(target, number_of_captures);
}

unsigned short hypstar_download_spectra(hypstar_t *hs, unsigned short *memory_slots, unsigned short number_of_captures, s_spectrum_dataset *target)
{
	if ((hs == NULL) || (target == NULL))
	{
		return 0;
	}

	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->getSpectraFromMemorySlots(memory_slots, number_of_captures, target);
}

unsigned short hypstar_acquire_spectra(hypstar_t *hs, enum e_radiometer spec, enum e_entrance mux,
		unsigned short vnir_inttime_ms,	unsigned short swir_inttime_ms,	unsigned short scan_count, unsigned short series_time_s, s_spectrum_dataset *target, bool reuse_last_AIT_value)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->acquireSpectra(spec, mux, vnir_inttime_ms, swir_inttime_ms, scan_count, series_time_s, target, reuse_last_AIT_value);
}

bool hypstar_set_baudrate(hypstar_t *hs, e_baudrate new_baudrate)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->setBaudRate(new_baudrate);
}

unsigned short hypstar_capture_JPEG_image(hypstar_t *hs, bool flip, bool mirror, bool auto_focus)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	s_capture_image_request_flags flags = {
			.scale = 0,
			.flip_v = flip,
			.mirror_h = mirror,
			.auto_focus = auto_focus,
			.na = 0
	};
	return instance->captureJpegImage(JPG_5MP, flags, 10.0);
}

int hypstar_download_JPEG_image(hypstar_t *hs, s_img_data_holder *target)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->getImage(target);
}

bool hypstar_set_TEC_target_temperature(hypstar_t *hs, float target_temp_deg_C) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->setTECSetpoint(target_temp_deg_C);
}

bool hypstar_shutdown_TEC(hypstar_t *hs) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->shutdown_TEC();
}

bool hypstar_reboot(hypstar_t *hs) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->reboot();
}

bool hypstar_enter_flash_write_mode(hypstar_t *hs) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->enterFlashWriteMode();
}

bool hypstar_send_calibration_coefficients(hypstar_t *hs, s_extended_calibration_coefficients *pNewExternalCalibrationCoeficients)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->sendCalibrationCoefficients(pNewExternalCalibrationCoeficients);
}

bool hypstar_VM_enable(hypstar_t *hs, uint8_t enable)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->enableVM((bool) enable);
}

bool hypstar_VM_set_current(hypstar_t *hs, float current) {
	return true;
}

bool hypstar_save_calibration_coefficients(hypstar_t *hs)
{
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->saveCalibrationCoefficients();
}

bool hypstar_wait_for_instrument(const char *port, float timeout_s, e_loglevel loglevel)
{
	try
	{
		return Hypstar::waitForInstrumentToBoot(port, timeout_s, loglevel);
	}
	catch (eHypstar &e)
	{
		return NULL;
	}
}
// @TODO: acceleration to Gs and gravity vector offset
// @TODO: automatic IT callbacks on adjust
// @TODO: automatic IT callback on done

bool hypstar_test_callback(hypstar_t *hs, void(*cb_function)(s_automatic_integration_time_adjustment_status*), int paramA, int paramB)
{
	s_spectrum_optical_configuration spectrum_config = {
			.na = 0,
			.start_from_last_AIT = 0,
			.irradiance = 0,
			.radiance = 1,
			.semolator_fake_saturation = 0,
			.swir = 0,
			.vnir = 1,
	};
	s_automatic_integration_time_adjustment_status s = {
			.spectrum_config = spectrum_config,
			.current_integration_time_ms = 64,
			.peak_adc_value = 12345,
			.next_integration_time_ms = 128,
			.memory_slot_id = 15
	};
	(*cb_function)(&s);
	return true;
}

struct s_libhypstar_version getLibHypstarVersion(void)
{
	struct s_libhypstar_version libver;
	libver.major = DVER_MAJOR;
	libver.minor = DVER_MINOR;
	libver.revision = DVER_REVISION;
	strncpy(libver.hash, DVER_HASH, sizeof(libver.hash));

	return libver;
}

bool hypstar_VM_measure(hypstar_t *hs, e_entrance entrance, e_vm_light_source source, unsigned short integration_time, float current, s_spectrum_dataset *pSpectraTarget, uint16_t scan_count) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->measureVM(entrance, source, integration_time, current, pSpectraTarget, scan_count);
}

bool hypstar_VM_get_status(hypstar_t *hs, struct VM_Status_t *pTarget) {
	if (hs == NULL)
	{
		return 0;
	}
	Hypstar *instance = static_cast<Hypstar *>(hs->hs_instance);
	return instance->getVmStatus(pTarget);
}
