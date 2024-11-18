from _ctypes import Structure
from ctypes import c_uint8, c_uint32, c_uint16, c_int
from enum import IntEnum


class HypstarSupportedBaudRates(IntEnum):
	B_115200 = 115200
	B_460800 = 460800
	B_921600 = 921600
	B_3000000 = 3000000
	B_6000000 = 6000000
	B_8000000 = 8000000

	def __init__(self, value):
		self._as_parameter = int(value)

	# needed for CTypes passing as argument
	@classmethod
	def from_param(cls, obj):
		return int(obj)



class BootedPacketStruct(Structure):
	_pack_ = 1
	_fields_ = [
		("firmware_version_major", c_uint8),
		("firmware_version_minor", c_uint8),
		("firmware_version_revision", c_uint8),
		("instrument_serial_number", c_uint32),
		("mcu_hardware_version", c_uint8),
		("psu_hardware_version", c_uint8),
		("vis_serial_number", c_uint16),
		("swir_serial_number", c_uint32),
		("memory_slot_count", c_uint16),
		("vnir_module_available", c_uint16, 1),
		("swir_module_available", c_uint16, 1),
		("optical_multiplexer_available", c_uint16, 1),
		("camera_available", c_uint16, 1),
		("accelerometer_available", c_uint16, 1),
		("humidity_sensor_available", c_uint16, 1),
		("pressure_sensor_available", c_uint16, 1),
		("swir_tec_module_available", c_uint16, 1),
		("sd_card_available", c_uint16, 1),
		("power_monitor_1_available", c_uint16, 1),
		("power_monitor_2_available", c_uint16, 1),
		("is_1MB_device", c_uint16, 1),
		("isolated_adc", c_uint16, 1),
		("vm_available", c_uint16, 1),
		("dummy2", c_uint16, 1),
		("dummy3", c_uint16, 1),
		("vm_firmware_version_major", c_uint8),
		("vm_firmware_version_minor", c_uint8),
		("vm_firmware_version_revision", c_uint8),
		("vm_serial_number", c_uint32),
		("vnir_pixel_count", c_uint16),
		("swir_pixel_count", c_uint16),
	]

	def __str__(self):
		return "FW version: {}.{}.{}, \n" \
		   "instrument S/N: {},\n" \
		   "MCU HW V: {}, PSU HW V: {}\n" \
		   "VIS SPEC S/N: {}\n" \
		   "SWIR_SPEC S/N: {}\n" \
		   "Memory slots available: {} \n" \
		   "Available hardware:\n" \
		   "VIS:\t\t{}\n" \
		   "SIWR:\t\t{}\n" \
		   "MUX:\t\t{}\n" \
		   "CAM:\t\t{}\n" \
		   "Accelerometer:\t{}\n" \
		   "Humidity sensor:\t{}\n" \
		   "Pressure sensor:\t{}\n" \
		   "SWIR TEC:\t{}\n" \
		   "SD Card:\t\t{}\n" \
		   "Power monitor 1:\t{}\n" \
		   "Power monitor 2:\t{}\n" \
		   "Is 1MB flash dev:\t{}\n" \
		   "Isolated ADC:\t{}\n" \
		   "VM:\t\t{}\n" \
		   "VM FW version: {}.{}.{}\n" \
		   "VM S/N: {}\n" \
		   "VNIR pixel count: {}\n" \
		   "SWIR pixel count: {}".format(
			   self.firmware_version_major, self.firmware_version_minor, self.firmware_version_revision, self.instrument_serial_number,
				self.mcu_hardware_version, self.psu_hardware_version,
				self.vis_serial_number, self.swir_serial_number, self.memory_slot_count,
				self.vnir_module_available, self.swir_module_available, self.optical_multiplexer_available, self.camera_available,
				self.accelerometer_available, self.humidity_sensor_available, self.pressure_sensor_available,
				self.swir_tec_module_available, self.sd_card_available, self.power_monitor_1_available, self.power_monitor_2_available,
				self.is_1MB_device, self.isolated_adc, self.vm_available,
				self.vm_firmware_version_major,self.vm_firmware_version_minor, self.vm_firmware_version_revision, self.vm_serial_number,
				self.vnir_pixel_count, self.swir_pixel_count)
