import io
import unittest

import time
from PIL import Image

from .data_structs.varia import ValidationModuleLightType
from .data_structs.hardware_info import HypstarSupportedBaudRates
from .data_structs.spectrum_raw import RadiometerType, RadiometerEntranceType
from .data_structs.spectrum import Radiometer, EntranceType
from .hypstar_wrapper import Hypstar, HypstarLogLevel, wait_for_instrument
from matplotlib.pyplot import sca

# to run:
# python3 -m unittest hypernets.hypstar.libhypstar.python.ctypes_test.CtypeTests.test_VM_capture_one

serial_port = '/dev/ttyUSB0'


class CtypeTestsNoInit(unittest.TestCase):

	def test_wait_for_instrument_on_port_fail(self):
		assert wait_for_instrument(serial_port, 15) == False

	def test_wait_for_instrument_on_port_succeed(self):
		assert wait_for_instrument(serial_port, 15) == True


class CtypeTests(unittest.TestCase):

	def setUp(cls):
		try:
			cls.radiometer = Hypstar(serial_port)
		except IOError as e:
			assert str(e) == "Could not retrieve instrument instance!"
			cls.skipTest(cls, reason="No instrument")

	def test_hw_info(self):
		self.radiometer.set_log_level(HypstarLogLevel.DEBUG)
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		print(self.radiometer.hw_info)

	def test_reboot(self):
		assert self.radiometer.reboot() == 1

	def test_double_instantiation_should_return_same_handle(self):
		self.radiometer2 = Hypstar(serial_port)
		assert self.radiometer.handle == self.radiometer2.handle

	def test_baud_rate_changing_valid(self):
		radiometer = Hypstar(serial_port)
		radiometer.set_baud_rate(HypstarSupportedBaudRates.B_3000000)
		radiometer.get_hw_info()
		del radiometer
		# test that baud rate got reset after reinit
		radiometer = Hypstar(serial_port)
		radiometer.get_hw_info()

	def test_baud_rate_invalid_should_throw(self):
		try:
			self.radiometer.set_baud_rate(123456)
		except ValueError as e:
			assert str(e) == "Bad baud rate provided"

	def test_env_log(self):
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		log = self.radiometer.get_env_log()
		print(log)

	def test_env_right_after_boot(self):
		self.radiometer.reboot()
		log = self.radiometer.get_env_log()
		print(log)

	def test_time(self):
		self.radiometer.set_log_level(HypstarLogLevel.DEBUG)
		ts_in = 1234578790
		self.radiometer.set_time_s(ts_in)
		ts_out = self.radiometer.get_time_ms()
		print('\n\rExecution time: {} ms'.format(int(ts_out - ts_in*1000)))

	def test_calibration_coefficients(self):
		# self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		coefficients = self.radiometer.get_calibration_coeficients_basic()
		print(coefficients)
		extended_coefficients = self.radiometer.get_calibration_coeficients_extended()
		basic, extended_coefficients = self.radiometer.get_calibration_coeficients_all()

	def test_start_SWIR_TEC(self):
		self.radiometer.set_SWIR_module_temperature(-5.23)

	def test_shutdown_SWIR_TEC(self):
		self.radiometer.shutdown_SWIR_module_thermal_control()

	def test_capture_spectrum_fixed_it_vis(self):
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.DARK, 100, 0, 1, 0)
		assert count == 1
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 100, 0, 2, 0)
		assert count == 2
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 100, 0, 1, 0)
		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		print(slots[0])
		assert len(slots) == count
		spectra = self.radiometer.download_spectra(slots)
		print(spectra[0])
		for i in range(spectra[0].spectrum_header.pixel_count):
			print("{}: {}".format(i, spectra[0].spec[i]))
		assert (spectra[0].spectrum_header.spectrum_config.vnir == 1)
		assert (spectra[0].spectrum_header.spectrum_config.swir == 0)
		assert (spectra[0].spectrum_header.spectrum_config.radiance == 0)
		assert (spectra[0].spectrum_header.spectrum_config.irradiance == 1)
		assert (spectra[0].spectrum_header.pixel_count == 2048)
		assert (spectra[0].spectrum_header.integration_time_ms == 100)

	def test_capture_spectrum_fixed_it_swir(self):
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.DARK, 0, 100, 1, 0)
		assert count == 1
		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.RADIANCE, 0, 100, 2, 0)
		assert count == 2
		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.IRRADIANCE, 0, 100, 0, 1)
		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		assert len(slots) == count
		spectra = self.radiometer.download_spectra(slots)
		print(spectra[0])
		for i in range(spectra[0].spectrum_header.pixel_count):
			print("{}: {}".format(i, spectra[0].spec[i]))

		assert (spectra[0].spectrum_header.spectrum_config.vnir == 0)
		assert (spectra[0].spectrum_header.spectrum_config.swir == 1)
		assert (spectra[0].spectrum_header.spectrum_config.radiance == 0)
		assert (spectra[0].spectrum_header.spectrum_config.irradiance == 1)
		assert (spectra[0].spectrum_header.pixel_count == 256)
		assert (spectra[0].spectrum_header.integration_time_ms == 100)

	def test_capture_spectrum_fixed_it_both(self):
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.DARK, 100, 100, 1, 0)
		assert count == 2
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.RADIANCE, 100, 100, 2, 0)
		assert count == 4
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.IRRADIANCE, 450, 450, 0, 1)
		assert count == 4
		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		assert len(slots) == count
		spectra = self.radiometer.download_spectra(slots)

	def test_capture_spectrum_auto_it_VIS(self):
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.DARK, 0, 0, 1, 0)
		assert count == 1
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 0, 0, 2, 0)
		assert count == 2
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 0, 0, 4, 0)
		assert count == 4
		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		assert len(slots) == count
		spectra = self.radiometer.download_spectra(slots)

	def test_capture_spectrum_auto_it_SWIR(self):
		self.radiometer.set_log_level(HypstarLogLevel.DEBUG)
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.RADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.RADIANCE, 0, 0, 2, 0)
		assert count == 2
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.IRRADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.IRRADIANCE, 0, 0, 4, 0)

		count = self.radiometer.capture_spectra(RadiometerType.SWIR, RadiometerEntranceType.DARK, 0, 0, 1, 0)
		assert count == 1

		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		assert len(slots) == count
		spectra = self.radiometer.download_spectra(slots)

	def test_capture_spectrum_auto_it_BOTH(self):
		self.radiometer.set_log_level(HypstarLogLevel.DEBUG)
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.RADIANCE, 0, 0, 2, 0)
		assert count >= 2
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.IRRADIANCE, 0, 0, 4, 0)

		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.DARK, 0, 0, 1, 0)
		assert count >= 1

		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)

		assert len(slots) == count

		spectra = self.radiometer.download_spectra(slots)

	def test_capture_spectrum_auto_it_BOTH_with_reuse(self):
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		if not self.radiometer.hw_info.swir_module_available:
			print('SWIR module not available')
			pass

		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.RADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.RADIANCE, 0, 0, 2, 0, reuse_last_AIT_value=True)
		assert count >= 2
		# rotate to correctly predict count
		self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.IRRADIANCE, 10, 0, 1, 0)
		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.IRRADIANCE, 0, 0, 4, 0, reuse_last_AIT_value=True)

		count = self.radiometer.capture_spectra(RadiometerType.BOTH, RadiometerEntranceType.DARK, 0, 0, 1, 0, reuse_last_AIT_value=True)
		assert count >= 1

		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)

		assert len(slots) == count

		spectra = self.radiometer.download_spectra(slots)

	def test_capture_spectrum_and_convert(self):
		self.radiometer.set_log_level(HypstarLogLevel.DEBUG)

		# self.radiometer.set_SWIR_module_temperature(-5.0)
		count = self.radiometer.capture_spectra(RadiometerType.VIS_NIR, RadiometerEntranceType.DARK, 0, 0, 3, 0)
		slots = self.radiometer.get_last_capture_spectra_memory_slots(count)
		spectra = self.radiometer.download_spectra(slots)
		for i in spectra:
			print(i)
			s = i.convert_to_spectrum_class()  # Type: Spectrum
			s.plot()

	def test_capture_JPEG(self):
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		self.radiometer.capture_JPEG_image(flip=True, mirror=False)
		self.radiometer.set_baud_rate(HypstarSupportedBaudRates.B_6000000)

		img = self.radiometer.download_JPEG_image()
		stream = io.BytesIO(img)
		im = Image.open(stream)
		w, h = im.size
		print("Image size: {} x {}".format(w, h))
		im.show()

	def test_register_callback(self):
		self.radiometer.register_callback()

	def test_enabling_vm(self):
		self.radiometer.VM_enable(True)
		time.sleep(3)
		self.radiometer.VM_enable(False)

	def test_VM_capture_one(self):
		count = 1
		ret_val = self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_VIS, 100, 0.9, scan_count=count)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.IRRADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		assert len(ret_val) == count
		s = ret_val[0].convert_to_spectrum_class()  # Type: Spectrum
		assert (s.header.spectrum_type.radiometer == Radiometer.VIS)
		assert (s.header.spectrum_type.optics == EntranceType.RADIANCE)
		assert (s.header.pixel_count == 2048)
		assert (s.header.exposure_time == 100)
		s.plot()

	def test_VM_capture_many(self):
		count = 23
		ret_val = self.radiometer.VM_measure(RadiometerEntranceType.IRRADIANCE, ValidationModuleLightType.LIGHT_VIS, 32, 1.0, scan_count=count)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.IRRADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		assert len(ret_val) == count
		s = ret_val[0].convert_to_spectrum_class()  # Type: Spectrum
		assert (s.header.spectrum_type.radiometer == Radiometer.VIS)
		assert (s.header.spectrum_type.optics == EntranceType.IRRADIANCE)
		assert (s.header.pixel_count == 2048)
		assert (s.header.exposure_time == 32)
		s.plot()

	def test_VM_capture_default(self):
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		count = 100
		ret_val = self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_VIS, 10, 1.5)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		# ret_val = self.radiometer.VM_measure(RadiometerEntranceType.IRRADIANCE, ValidationModuleLightType.LIGHT_SWIR_1300nm, 100, 0.1)
		assert len(ret_val) == count
		s = ret_val[0].convert_to_spectrum_class()  # Type: Spectrum
		assert (s.header.spectrum_type.radiometer == Radiometer.VIS)
		assert (s.header.spectrum_type.optics == EntranceType.RADIANCE)
		assert (s.header.pixel_count == 2048)
		assert (s.header.exposure_time == 10)
		s.plot()

	def test_VM_capture_negative(self):
		self.radiometer.set_log_level(HypstarLogLevel.TRACE)
		count = -5
		try:
			self.radiometer.VM_measure(RadiometerEntranceType.RADIANCE, ValidationModuleLightType.LIGHT_VIS, 10, 1.5, count)
		except Exception as e:
			assert type(e) == ValueError
			assert str(e) == f'Array length must be >= 0, not {count}'

if __name__ == '__main__':
	unittest.main()
