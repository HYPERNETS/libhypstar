#!/usr/bin/python3

import subprocess, random, pyftdi.usbtools, pyftdi.eeprom, io, datetime

mfr_str = 'UTTO'
sn_prefix = 'RSC5'
product_str = 'Hypstar USB-Serial'
backup_filename = 'backup.ini'
config_filename = 'config.ini'
counter_filename = 'sn_counter.txt'

def update_mfr_prod(e):
	e.set_manufacturer_name(mfr_str)
	e.set_product_name(product_str)

def get_eeprom_data(e):
	s = io.StringIO()
	e.dump_config(s)
	# print(s)
	# parse config into dict for lookups
	print(s.getvalue().strip().split('\n'))
	try:
		d = dict(x.split(': ') for x in s.getvalue().strip().split('\n'))
	# handle empty eeprom - load default values
	except Exception as ex:
		with open (config_filename, 'r') as f:
			e.load_config(f, 'all')
			return get_eeprom_data(e)
	return d

if __name__ == '__main__':
	t = pyftdi.usbtools.UsbTools()
	serials = t.find_all([[0x0403, 0x6014]])
	# generate a warning and abort if more than one found
	if not len(serials):
		print('No RS-485 converters found')
		exit()
	if len(serials) > 1:
		print('More than one FTDI chip found, please disconnect extra devices')
		exit()

	device = t.get_device(serials[0][0])
	e = pyftdi.eeprom.FtdiEeprom()
	e.open(device)
	d = get_eeprom_data(e)

	print('Found board \'{}\', manufacturer \'{}\' with S/N \'{}\''.format(d['product'], d['manufacturer'], d['serial']))

	# back up config
	subprocess.call('ftconf.py ftdi://0x0403:0x6014/1 -o {}'.format(backup_filename), shell=True)

	# don't overwrite existing RSC5xxxx numbers
	if d['serial'].startswith(sn_prefix):
		if d['manufacturer'] == mfr_str:
			print('Detected an already updated converter')
			exit()
		else:
			print('Detected a batch 2 converter, updating manufacturer and product name')
			update_mfr_prod(e)
	else:
		# generate new serial
		week = datetime.date.today().strftime('%y%W')
		idx = 0
		with open(counter_filename, 'a+') as f:
			f.seek(0)
			prev_week, prev_idx = f.readline().split()
			if prev_week == week:
				idx = int(prev_idx) + 1
			f.seek(0)
			f.truncate()
			f.write('{} {}'.format(week, str(idx).zfill(2)))

		sn = sn_prefix + week + str(idx).zfill(2)
		e.set_serial_number(sn)
		update_mfr_prod(e)

		print('New S/N: \'{}\', product string: \'{}\''.format(sn, product_str))
	# e.set_serial_number('FTxxxx')
	# Write to EEPROM
	print('Writing data to the board, please DO NOT DISCONNECT THE BOARD!')
	e.commit(dry_run=False)
	print('Done updating. Old config saved in {}'.format(backup_filename))

	print('Reading back from chip:')
	d = get_eeprom_data(e)
	print('Board \'{}\', manufacturer \'{}\' with S/N \'{}\''.format(d['product'], d['manufacturer'], d['serial']))

	print('Please unplug and reinsert USB-RS485 module from USB port or reboot the system')
	e.close()

