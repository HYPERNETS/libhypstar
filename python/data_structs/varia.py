from _ctypes import Structure
from ctypes import c_uint16
from enum import IntEnum

from .spectrum_raw import OpticalConfiguration


class HypstarAutoITStatus(Structure):
	_pack_ = 1
	_fields_ = [
		("spectrum_config", OpticalConfiguration),	# LSB of flags byte, should be 0
		("current_integration_time_ms", c_uint16),
		("peak_adc_value", c_uint16),
		("next_integration_time_ms", c_uint16),
		("memory_slot_id", c_uint16)
	]

class ValidationModuleLightType(IntEnum):
	LIGHT_NONE = 0
	LIGHT_VIS = 1
	LIGHT_SWIR_1300nm = 2
	LIGHT_SWIR_1550nm = 4

	def __init__(self, value):
		self._as_parameter = int(value)

	# needed for CTypes passing as argument
	@classmethod
	def from_param(cls, obj):
		return int(obj)
