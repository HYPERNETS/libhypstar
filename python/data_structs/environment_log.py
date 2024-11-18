from _ctypes import Structure
from ctypes import c_uint64, c_int16, c_uint16, c_int32, c_float
from datetime import datetime, timezone


class PowerBusInfo:
	voltage = 0.0
	current = 0.0
	total_energy = 0.0

	def __init__(self):
		self.voltage = 0.0
		self.current = 0.0
		self.total_energy = 0.0

	def __str__(self):
		return 'Voltage: \t{0: .2f} V,\t\tcurrent: \t{1: .2f} A,\t\ttotal energy consumed:\t{2: .0f} mWh'.format(self.voltage, self.current, self.total_energy)

	def parse(self, name, logentry):
		exec('self.voltage = logentry.voltage_' + name)
		exec('self.current = logentry.current_' + name)
		exec('self.total_energy = logentry.energy_' + name)
		return self


class AccelerationInfo:
	x_raw = 0
	y_raw = 0
	z_raw = 0
	x_g = 0
	y_g = 0
	z_g = 0

	def __init__(self):
		self.x_raw = 0
		self.y_raw = 0
		self.z_raw = 0
		self.x_g = 0
		self.y_g = 0
		self.z_g = 0

	def __str__(self):
		return 'Acceleration: X: {}, Y: {}, Z: {}'.format(self.x_raw, self.y_raw, self.z_raw)

	def parse(self, logentry):
		self.x_raw = logentry.accelerometer_readings_XYZ[0]
		self.y_raw = logentry.accelerometer_readings_XYZ[1]
		self.z_raw = logentry.accelerometer_readings_XYZ[2]
		return self


class EnvironmentLogEntry(Structure):
	_pack_ = 1
	_fields_ = [
		('timestamp', c_uint64),
		('humidity_sensor_temperature', c_int16),
		('humidity_sensor_humidity', c_uint16),
		('pressure_sensor_pressure', c_int32),
		('pressure_sensor_temperature', c_int32),
		('accelerometer_readings_XYZ', c_int16 * 3),
		('internal_ambient_temperature', c_float),
		('swir_body_temperature', c_float),
		('swir_heatsink_temperature', c_float),
		('energy_vnir_module_5v', c_float),
		('energy_mcu_3v3', c_float),
		('energy_common_3v3', c_float),
		('energy_camera_3v3', c_float),
		('voltage_vnir_module_5v', c_float),
		('voltage_mcu_3v3', c_float),
		('voltage_common_3v3', c_float),
		('voltage_camera_3v3', c_float),
		('current_vnir_module_5v', c_float),
		('current_mcu_3v3', c_float),
		('current_common_3v3', c_float),
		('current_camera_3v3', c_float),
		('energy_swir_module_12v', c_float),
		('energy_validation_module_12v', c_float),
		('energy_input_12v', c_float),
		('energy_multiplexer_12v', c_float),
		('voltage_swir_module_12v', c_float),
		('voltage_validation_module_12v', c_float),
		('voltage_input_12v', c_float),
		('voltage_multiplexer_12v', c_float),
		('current_swir_module_12v', c_float),
		('current_validation_module_12v', c_float),
		('current_input_12v', c_float),
		('current_multiplexer_12v', c_float),
	]

	def parse(self):
		self.humidity_sensor_temp = self.humidity_sensor_temperature / 100
		self.humidity = self.humidity_sensor_humidity/10
		self.pressure_sensor_temp = self.pressure_sensor_temperature / 100
		self.pressure = self.pressure_sensor_pressure / 100
		self.internal_ambient_temp = self.internal_ambient_temperature
		# self.swir_body_temperature = self.swir_body_temperature if self.swir_body_temperature > -40 else 'N/A'
		# self.swir_heatsink_temperature = self.swir_heatsink_temperature if self.swir_heatsink_temperature > -40 else 'N/A'
		self.input_12V = PowerBusInfo().parse('input_12v', self)
		self.optical_multiplexer_12V = PowerBusInfo().parse('multiplexer_12v', self)
		self.swir_12V = PowerBusInfo().parse('swir_module_12v', self)
		self.validation_module_12V = PowerBusInfo().parse('validation_module_12v', self)
		self.vnir_5V = PowerBusInfo().parse('vnir_module_5v', self)
		self.common_3V3 = PowerBusInfo().parse('common_3v3', self)
		self.digital_electronics_3V3 = PowerBusInfo().parse('mcu_3v3', self)
		self.camera_3V3 = PowerBusInfo().parse('camera_3v3', self)
		self.accelerometer_data = AccelerationInfo().parse(self)

	def __str__(self):
		return 'TS: {} ({})\n' \
			'Temperatures:\n ' \
			'\tInternal: {:.2f}, Humidity sensor: {}, Pressure sensor: {}\n' \
			'\tSWIR body: {:.2f}, SWIR heatsink: {:.2f}\n' \
			'RH: {}%, internal pressure: {:.1f} mBar\n' \
			'Power buses:\n' \
			'\t12V input: \t\t\t\t{}\n' \
			'\t12V Multiplexer: \t\t{}\n' \
			'\t12V SWIR: \t\t\t\t{}\n' \
			'\t12V Validation module: \t{}\n' \
			'\t5V VIS-NIR: \t\t\t{}\n' \
			'\t3.3V common: \t\t\t{}\n' \
			'\t3.3V digital: \t\t\t{}\n' \
			'\t3.3V camera: \t\t\t{}\n' \
			'{}'.format(self.timestamp, datetime.fromtimestamp(int(self.timestamp / 1000), timezone.utc),
						self.internal_ambient_temp, self.humidity_sensor_temp, self.pressure_sensor_temp,
						self.swir_body_temperature, self.swir_heatsink_temperature,
						self.humidity, self.pressure,
						self.input_12V, self.optical_multiplexer_12V, self.swir_12V, self.validation_module_12V,
						self.vnir_5V, self.common_3V3, self.digital_electronics_3V3, self.camera_3V3,
						self.accelerometer_data)


	# Joel already has a bunch of processing scripts using default packet layout
	def get_csv_line(self):
		return "Env:\t{}\t{:.2f}\t{:.1f}\t{:.2f}\t{:.1f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t" \
			"{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}".format(
					datetime.fromtimestamp(int(self.timestamp / 1000), timezone.utc).strftime('%Y-%m-%d %H:%M:%S'),
					self.humidity_sensor_temp, self.humidity,
					self.pressure_sensor_temp, self.pressure,
					self.internal_ambient_temp, self.swir_body_temperature, self.swir_heatsink_temperature,
					self.common_3V3.total_energy, self.digital_electronics_3V3.total_energy, self.camera_3V3.total_energy,
					self.common_3V3.voltage, self.digital_electronics_3V3.voltage, self.camera_3V3.voltage,
					self.common_3V3.current, self.digital_electronics_3V3.current, self.camera_3V3.current,
					self.swir_12V.total_energy, self.optical_multiplexer_12V.total_energy, self.vnir_5V.total_energy, self.input_12V.total_energy,
					self.swir_12V.voltage, self.optical_multiplexer_12V.voltage, self.vnir_5V.voltage, self.input_12V.voltage,
					self.swir_12V.current, self.optical_multiplexer_12V.current, self.vnir_5V.current, self.input_12V.current
				)


def get_csv_header():
	return "#Env:\ttimestamp\trh_temp\trh_RH\tpressure_temp\tpressure\tambient_t\tswir_body_t\tswir_sink_t\t" \
		"e_common_3v3\te_mcu_3v3\te_cam_3v3\tu_common_3v3\tu_mcu_3v3\tu_cam_3v3\ti_common_3v3\ti_mcu_3v3\ti_cam_3v3\t" \
		"e_swir_12v\te_mux_12v\te_vnir_5v\te_input_12v\tu_swir_12v\tu_mux_12v\tu_vnir_5v\tu_input_12v\ti_swir_12v\ti_mux_12v\ti_vnir_5v\ti_input_12v"
