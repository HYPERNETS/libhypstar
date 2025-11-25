from _ctypes import Structure
from ctypes import c_float, c_uint8


class VMStatus(Structure):
	_pack_ = 1
	_fields_ = [
		('temp_setpoint', c_float),
		('temp_current', c_float),
		('temp_sink', c_float),
		('led_current', c_float),
		('led_voltage', c_float),
		('tec_voltage', c_float),
		('tec_current', c_float),
	]


	def __str__(self):
		return 'VM temperature setpoint: {:.2f}\'C, ' \
			'LED temperature: {:.2f} \'C, ' \
			'sink temperature: {:.2f} \'C ' \
			'LED current: {:.2f} A, ' \
			'LED voltage: {:.4f} V, ' \
			'TEC current: {:.2f} A, ' \
			'TEC voltage: {:.2f} V' \
			.format(self.temp_setpoint, self.temp_current, self.temp_sink,
						self.led_current, self.led_voltage, self.tec_voltage, self.tec_current)
