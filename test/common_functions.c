using namespace std;
#include <iomanip>
#include <ctime>
#include <fstream>
#include <chrono>

void test_and_print_single_spec(unsigned short slot, s_spectrum_dataset *s, int inttime_vis, int inttime_swir, e_radiometer rad, e_entrance entr)
{
	int pix_cnt_exp = 2048;
	int inttime_exp = inttime_vis;

	if (s->spectrum_header.spectrum_config.swir) {
		pix_cnt_exp = 256;
		inttime_exp = inttime_swir;
	}

	Spectrum sp = Spectrum(s);
	assert(s->spectrum_header.integration_time_ms == sp.integration_time);
	assert(s->spectrum_header.pixel_count == sp.pixel_count);
	if (s->spectrum_header.spectrum_config.swir) {
		assert (*(uint32_t*)&s->spectrum_body[256] == 0);
	}
	assert(s->crc32_spaceholder);

	char timebuff[100];
	time_t t = s->spectrum_header.timestamp_ms / 1000;
	auto tm = *localtime(&t);
	strftime(timebuff, sizeof(timebuff), "%Y-%m-%d %H:%M:%S", &tm);

	printf("\nSlot: %d\n", slot);
	printf("Timestmap: %ld (%s)\n", (long)s->spectrum_header.timestamp_ms, timebuff);
	printf("Radiance: %d, irradiance: %d\n", s->spectrum_header.spectrum_config.radiance, s->spectrum_header.spectrum_config.irradiance);
	printf("Entrances: VIS: %d, SWIR: %d\n", s->spectrum_header.spectrum_config.vnir, s->spectrum_header.spectrum_config.swir);
	printf("Integration time: %d ms\n", s->spectrum_header.integration_time_ms);
	printf("Pixel count: %d\n", s->spectrum_header.pixel_count);
	printf("Acceleration:\n");
	printf("\tX: %i +- %i\n",  s->spectrum_header.acceleration_statistics.X.mean_acceleration,  s->spectrum_header.acceleration_statistics.X.standard_deviation);
	printf("\tY: %i +- %i\n",  s->spectrum_header.acceleration_statistics.Y.mean_acceleration,  s->spectrum_header.acceleration_statistics.Y.standard_deviation);
	printf("\tZ: %i +- %i\n",  s->spectrum_header.acceleration_statistics.Z.mean_acceleration,  s->spectrum_header.acceleration_statistics.Z.standard_deviation);
	printf("Sensor temp: %.2f 'C\n", s->spectrum_header.sensor_temperature);
	printf("CRC32: 0x%08X\n", s->crc32_spaceholder);

	if (s->spectrum_header.spectrum_config.swir) {
		printf("Pixel 150: %d\n", s->spectrum_body[150]);
	} else {
		printf("Pixel 1500: %d\n", s->spectrum_body[1500]);
	}

	if (entr != DARK) {
		assert(s->spectrum_header.spectrum_config.radiance == (entr == RADIANCE));
		assert(s->spectrum_header.spectrum_config.irradiance == (entr == IRRADIANCE));
	}
	else
	{
		assert(!s->spectrum_header.spectrum_config.radiance);
		assert(!s->spectrum_header.spectrum_config.irradiance);
	}
	if (inttime_exp)
	{
		assert(s->spectrum_header.integration_time_ms == inttime_exp);
	}
}

int save_spec(Hypstar *hs, e_radiometer rad, e_entrance entr, int inttime_vis, int inttime_swir, int cap_count) {
	int count = hs->captureSpectra(rad, entr, inttime_vis, inttime_swir, cap_count, 0);
		if (count == 0)
			return 0;
		if (rad == BOTH)
		{
			if (inttime_vis && inttime_swir) {
				cap_count = cap_count * 2;
			}
		}
	//	assert(count == cap_count);
		unsigned short slots[count];
		hs->getLastSpectraCaptureMemorySlots(slots, count);

		int spec_size = sizeof(s_spectrum_dataset) * count;
		// for some reason instantiating array segfaults
		s_spectrum_dataset *specs = (s_spectrum_dataset *) malloc(spec_size);
		memset(specs, 0, spec_size);
	//	s_spectrum_dataset specs[count];

		s_spectrum_dataset *s = &specs[0];

		printf("Got slots: ");
		for (int i = 0; i < count; i++) {
			printf("%d ", slots[i]);
		}
		printf("\n");

		int counter = 0;
		// test getting single
//		hs->getSingleSpectrumFromMemorySlot(slots[counter], s);
		hs->getSpectraFromMemorySlots(&slots[counter], count, &specs[counter]);
//		test_and_print_single_spec(slots[counter], s, inttime_vis, inttime_swir, rad, entr);
		struct tm *timenow;
		time_t now = time(NULL);
		timenow = gmtime(&now);
		char filename[40];
		strftime(filename, sizeof(filename), "data/spec_%Y-%m-%d_%H-%M-%S.spe", timenow);

		std::ofstream outfile2(filename, std::ofstream::binary);
		while (counter--) {
			outfile2.write((const char *)&specs[counter], sizeof(s_spectrum_dataset));
		}
		outfile2.close();
		free(specs);

		return count;
}

int test_spec(Hypstar *hs, e_radiometer rad, e_entrance entr, int inttime_vis, int inttime_swir, int cap_count) {
	int count = hs->captureSpectra(rad, entr, inttime_vis, inttime_swir, cap_count, 0);
	if (count == 0)
		return 0;
	if (rad == BOTH)
	{
		if (inttime_vis && inttime_swir) {
			cap_count = cap_count * 2;
		}
	}
//	assert(count == cap_count);
	unsigned short slots[count];
	hs->getLastSpectraCaptureMemorySlots(slots, count);

	int spec_size = sizeof(s_spectrum_dataset) * count;
	// for some reason instantiating array segfaults
	s_spectrum_dataset *specs = (s_spectrum_dataset *) malloc(spec_size);
	memset(specs, 0, spec_size);
//	s_spectrum_dataset specs[count];

	s_spectrum_dataset *s = &specs[0];

	printf("Got slots: ");
	for (int i = 0; i < count; i++) {
		printf("%d ", slots[i]);
	}
	printf("\n");

	int counter = 0;
	// test getting single
	printf("Slot single: %d\n", slots[counter]);
	hs->getSingleSpectrumFromMemorySlot(slots[counter], s);
	test_and_print_single_spec(slots[counter], s, inttime_vis, inttime_swir, rad, entr);

	// test getting many
	if (++counter < count)
	{
		hs->getSpectraFromMemorySlots(&slots[counter], count-1, &specs[counter]);
		do {
			s = &specs[counter];
			test_and_print_single_spec(slots[counter], s, inttime_vis, inttime_swir, rad, entr);
		} while (++counter < count);
	}
	free(specs);
	return count;
}

void printEnv(s_environment_log_entry *item, Hypstar *pHs) {
	s_environment_log_entry log = *item;
	time_t t = log.timestamp / 1000;
	auto tm = *localtime(&t);

	cout << "Timestamp: " << log.timestamp << " (" << put_time(&tm, "%Y-%m-%d %H:%M:%S") << ")\n";
	cout << setw(30) << left << "Temp RH ('C):" << log.humidity_sensor_temperature / 100.0 << "\n";
	cout << setw(30) << left << "Temp Pressure ('C):" << log.pressure_sensor_temperature / 100.0 << "\n";
	cout << setw(30) << left << "Temp ambient ('C):" << log.internal_ambient_temperature << "\n";
	cout << setw(30) << left << "Temp SWIR body ('C):" << log.swir_body_temperature << "\n";
	cout << setw(30) << left << "Temp SWIR sink ('C):" << log.swir_heatsink_temperature << "\n";
	cout << setw(30) << left << "RH (%):" << log.humidity_sensor_humidity / 10.0 << "\n";
	cout << setw(30) << left << "Pressure (mbar)\t" << log.pressure_sensor_pressure / 100.0 << "\n";

	cout << setw(30) << left << "E common (mWh):" << log.energy_common_3v3 << "\n";
	cout << setw(30) << left << "E e_cam_3v3 (mWh):" << log.energy_camera_3v3 << "\n";
	cout << setw(30) << left << "E e_mcu_3v3 (mWh):" << log.energy_mcu_3v3 << "\n";
	cout << setw(30) << left << "E e_swir_12v (mWh):" << log.energy_swir_module_12v << "\n";
	cout << setw(30) << left << "E e_mux_12v (mWh):" << log.energy_multiplexer_12v << "\n";
	cout << setw(30) << left << "E e_vnir_5v (mWh):" << log.energy_vnir_module_5v << "\n";
	cout << setw(30) << left << "E e_input_12v (mWh):" << log.energy_input_12v << "\n";

	cout << setw(30) << left << "i_common_3v3:" << log.current_common_3v3 << "\n";
	cout << setw(30) << left << "i_mcu_3v3:" << log.current_mcu_3v3 << "\n";
	cout << setw(30) << left << "i_cam_3v3:" << log.current_camera_3v3 << "\n";
	cout << setw(30) << left << "i_swir_12v:" << log.current_swir_module_12v << "\n";
	cout << setw(30) << left << "i_mux_12v:" << log.current_multiplexer_12v << "\n";
	cout << setw(30) << left << "i_vnir_5v:" << log.current_vnir_module_5v << "\n";
	cout << setw(30) << left << "i_input_12v:" << log.current_input_12v << "\n";

	cout << setw(30) << left << "u_common_3v3:" << log.voltage_common_3v3 << "\n";
	cout << setw(30) << left << "u_cam_3v3:" << log.voltage_camera_3v3 << "\n";
	cout << setw(30) << left << "u_mcu_3v3:" << log.voltage_mcu_3v3 << "\n";
	cout << setw(30) << left << "u_swir_12v:" << log.voltage_swir_module_12v << "\n";
	cout << setw(30) << left << "u_mux_12v:" << log.voltage_multiplexer_12v << "\n";
	cout << setw(30) << left << "u_vnir_5v:" << log.voltage_vnir_module_5v << "\n";
	cout << setw(30) << left << "u_input_12v:" << log.voltage_input_12v << "\n";

	float accel_gs[3] = {0.0f, 0.0f, 0.0f};
	pHs->convertRawAccelerometerDataToGsFromEnvLog(item, accel_gs);
	float accel_ms[3] = {0.0f, 0.0f, 0.0f};
	pHs->convertRawAccelerometerDataToMsFromEnvLog(item, accel_ms);

	cout << "Acceleration:" << endl;
	printf("\tX: %+06d \t( %9.6f g \t / %11.6f ms2)\n", log.accelerometer_readings_XYZ[0], accel_gs[0], accel_ms[0]);
	printf("\tY: %+06d \t( %9.6f g \t / %11.6f ms2)\n", log.accelerometer_readings_XYZ[1], accel_gs[1], accel_ms[1]);
	printf("\tZ: %+06d \t( %9.6f g \t / %11.6f ms2)\n", log.accelerometer_readings_XYZ[2], accel_gs[2], accel_ms[2]);
	fflush(stdout);
//	cout << "\tX:" << setw(10) << std::setprecision(6) << log.accelerometer_readings_XYZ[0] << "( " << accel_gs[0] << " g\t/ " << accel_ms[0] <<" ms2)" << endl;
//	cout << "\tY:" << setw(10) << std::setprecision(6) << log.accelerometer_readings_XYZ[1] << "( " << accel_gs[1] << " g\t/ " << accel_ms[1] <<" ms2)" << endl;
//	cout << "\tZ:" << setw(10) << std::setprecision(6) << log.accelerometer_readings_XYZ[2] << "( " << accel_gs[2] << " g\t/ " << accel_ms[2] <<" ms2)" << endl;
}
