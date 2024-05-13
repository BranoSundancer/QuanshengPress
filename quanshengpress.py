import serial
import struct
import os
import sys
import re
import time
import socket
import bufsock

appname = 'Quanshenmodo'
appdesc = 'Quansheng UV-K5 keypress automation interface'
appauth = 'Branislav Vartik'
appver = '0.1'
applic = 'MIT'
appurl = 'TBD'

# Configuration defaults # FIXME: Allow to change it using script parameters using argparse
tcpport = 4532
bindip = '0.0.0.0'
verbose = 1 # Display rotctl and TRX communication, for FW communication debug use environment variable: set DEBUG=True
vfostep = 1000 # In Hz, the value must match on all *used* VFOs and bands, step below 1000 Hz is not supported yet
vfocurrent = 0 # Default VFO is VFOA

# Init values ###############################################################
sleepafterkeypress = 0.4 # We need to sleep between key presses, otherwise some of them may get lost 
freqmax = 1000000000 - vfostep # Maximum supported frequency, frequencies over 1 Ghz use 7 digits which breaks the code so we don't support it
freqmin = 18000000 # Minimum supported frequency
vfoname = ['VFOA', 'VFOB']
vfofreq = [ 0, 0 ] # Inital VFO frquencies in Hz
vfosplit = 0 # Default split TX mode is off
vfosplittx = vfocurrent # We can TX only with current VFO

# Note: I never programmed in Python, so expect this my first code is ugly like Quasimodo.
#
# Credits:
# * TCP buffered socket server: https://discuss.python.org/t/client-server-tcp-socket-programming/21701/3
# * Communication and obfuscation of commands for the radio: https://github.com/amnemonic/Quansheng_UV-K5_Firmware/blob/main/python-utils/libuvk5.py

Crc16Tab = [0, 4129, 8258, 12387, 16516, 20645, 24774, 28903, 33032, 37161, 41290, 45419, 49548, 53677, 57806, 61935, 4657, 528, 12915, 8786, 21173, 17044, 29431, 25302,
			37689, 33560, 45947, 41818, 54205, 50076, 62463, 58334, 9314, 13379, 1056, 5121, 25830, 29895, 17572, 21637, 42346, 46411, 34088, 38153, 58862, 62927, 50604, 54669, 13907,
			9842, 5649, 1584, 30423, 26358, 22165, 18100, 46939, 42874, 38681, 34616, 63455, 59390, 55197, 51132, 18628, 22757, 26758, 30887, 2112, 6241, 10242, 14371, 51660, 55789,
			59790, 63919, 35144, 39273, 43274, 47403, 23285, 19156, 31415, 27286, 6769, 2640,14899, 10770, 56317, 52188, 64447, 60318, 39801, 35672, 47931, 43802, 27814, 31879,
			19684, 23749, 11298, 15363, 3168, 7233, 60846, 64911, 52716, 56781, 44330, 48395,36200, 40265, 32407, 28342, 24277, 20212, 15891, 11826, 7761, 3696, 65439, 61374,
			57309, 53244, 48923, 44858, 40793, 36728, 37256, 33193, 45514, 41451, 53516, 49453, 61774, 57711, 4224, 161, 12482, 8419, 20484, 16421, 28742, 24679, 33721, 37784, 41979,
			46042, 49981, 54044, 58239, 62302, 689, 4752, 8947, 13010, 16949, 21012, 25207, 29270, 46570, 42443, 38312, 34185, 62830, 58703, 54572, 50445, 13538, 9411, 5280, 1153, 29798,
			25671, 21540, 17413, 42971, 47098, 34713, 38840, 59231, 63358, 50973, 55100, 9939, 14066, 1681, 5808, 26199, 30326, 17941, 22068, 55628, 51565, 63758, 59695, 39368,
			35305, 47498, 43435, 22596, 18533, 30726, 26663, 6336, 2273, 14466, 10403, 52093, 56156, 60223, 64286, 35833, 39896, 43963, 48026, 19061, 23124, 27191, 31254, 2801,
			6864, 10931, 14994, 64814, 60687, 56684, 52557, 48554, 44427, 40424, 36297, 31782, 27655, 23652, 19525, 15522, 11395, 7392, 3265, 61215, 65342, 53085, 57212, 44955,
			49082, 36825, 40952, 28183, 32310, 20053, 24180, 11923, 16050, 3793, 7920]

def crc16_ccitt(data):
	i2 = 0
	for i3 in range(0, len(data)):
		out = Crc16Tab[((i2 >> 8) ^ data[i3]) & 255]
		i2 = out ^ (i2 << 8)
	return 65535 & i2

def crc16_ccitt_le(data):
	crc = crc16_ccitt(data)
	return bytes([crc & 0xFF,]) + bytes([crc>>8,])

def payload_xor(payload):
	XOR_ARRAY = bytes.fromhex('166c14e62e910d402135d5401303e980')
	XOR_LEN   = len(XOR_ARRAY)

	ba=bytearray(payload)
	for i in range(0,len(ba)):
		ba[i] ^= XOR_ARRAY[i%XOR_LEN]
	return bytes(ba)

class uvk5:
	def __enter__(self):
		return self
	def __exit__(self, type, value, traceback):
		pass

	def __init__(self,portName='COM1'):
		
		self.serial = serial.Serial()
		self.serial.baudrate = 38400
		self.serial.timeout=1
		self.serial.port=portName
		self.serial.bytesize=8
		self.serial.parity='N'
		self.serial.stopbits=1
#		self.serial.xonxoff=False
#		self.serial.rtscts=False
#		self.serial.dsrdtr=True
		self.sessTimestamp = b'\x46\x9C\x6F\x64'

		self.CMD_GET_FW_VER   = b'\x14\x05' #0x0514 -> 0x0515

		self.CMD_REBOOT	   = b'\xDD\x05' #0x05DD -> no reply
		self.CMD_WRITE_REG	= b'\x50\x08' #x0850
		self.CMD_READ_REG	 = b'\x51\x08' #x0851
		self.CMD_KEYPRESS	 = b'\x01\x08' #x0801
		
		self.debug = False if os.getenv('DEBUG') is None else True

	def __del__(self):
		self.serial.close()
		return not self.serial.is_open

	def connect(self):
		self.serial.open()
		return self.serial.is_open

	def uart_send_msg(self,msg_dec):
		if self.debug: print('>dec>',msg_dec.hex())
		msg_raw = msg_dec[:4] + payload_xor(msg_dec[4:-2]) + msg_dec[-2:]
		if self.debug: print('>raw>',msg_raw.hex())
		return self.serial.write(msg_raw)

	def uart_receive_msg(self,len):
		if len > 0:
			msg_raw = self.serial.read(len)
			if self.debug: print('<raw<',msg_raw.hex())
			if not re.match(rb'^\xab\xcd.+\xdc\xba',msg_raw):
				if self.debug: print('!not! ABCD+DCBA not in place, reading more bytes...')
				msg_raw = msg_raw + self.serial.read(512)
				if self.debug: print('<raw<',msg_raw.hex())
				if re.match(rb'\xab\xcd.+\xdc\xba',msg_raw):
					msg_raw = re.findall(rb'\xab\xcd.+\xdc\xba',msg_raw)[0]
			if self.debug: print('<raw<',msg_raw.hex())
			msg_dec = msg_raw[:4] + payload_xor(msg_raw[4:-2]) + msg_raw[-2:]
			if self.debug: print('<dec<',msg_dec.hex())
			return msg_dec

	def build_uart_command(self, command_type, command_body=b''):
		cmd = command_type + struct.pack('<H',len(command_body))+ command_body
		cmd_len = struct.pack('<H',len(cmd))
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd =  b'\xAB\xCD' + cmd_len + cmd + cmd_crc + b'\xDC\xBA'
		return cmd

	def get_fw_version(self):
		cmd=self.build_uart_command(self.CMD_GET_FW_VER, self.sessTimestamp)
		self.uart_send_msg(cmd)
		reply = self.uart_receive_msg(128)
		return reply[8:].split(b'\0', 1)[0].decode()

	def reboot(self):
		cmd = self.CMD_REBOOT + b'\x00\x00'
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd= b'\xAB\xCD' + struct.pack('<H',4) + cmd + cmd_crc + b'\xDC\xBA'
		self.uart_send_msg(cmd)
		return True

	def set_current_frequency(self,uint1): # FIXME: Not working
		cmd = self.CMD_WRITE_REG + struct.pack('<H',14) + struct.pack('<H',3) + \
				struct.pack('<H',0x0038) + struct.pack('<H',int(uint1 / 10) % 65536) + \
				struct.pack('<H',0x0039) + struct.pack('<H',int(uint1 / 655360)) + \
				struct.pack('<H',0x0030) + struct.pack('<H',0)
		cmd_len = struct.pack('<H',len(cmd))
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd = b'\xAB\xCD' + cmd_len + cmd + cmd_crc + b'\xDC\xBA'
		self.uart_send_msg(cmd)
		reply = self.uart_receive_msg(256)
		return reply[12:-4]

	def get_current_frequency(self):
		cmd = self.CMD_READ_REG + struct.pack('<H',6) + struct.pack('<H',2) + \
				struct.pack('<H',0x38) + struct.pack('<H',0x39)
		cmd_len = struct.pack('<H',len(cmd))
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd = b'\xAB\xCD' + cmd_len + cmd + cmd_crc + b'\xDC\xBA'
		self.uart_send_msg(cmd)
		reply = self.uart_receive_msg(32)
		if reply:
			return 10 * (struct.unpack('<H',reply[10:12])[0] + 65536 * struct.unpack('<H',reply[26:28])[0])
		else:
			return 0

	def read_register(self,uint1):
		cmd = self.CMD_READ_REG + struct.pack('<H',4) + struct.pack('<H',1) + \
				struct.pack('<H',uint1)
		cmd_len = struct.pack('<H',len(cmd))
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd = b'\xAB\xCD' + cmd_len + cmd + cmd_crc + b'\xDC\xBA'
		self.uart_send_msg(cmd)
		reply = self.uart_receive_msg(16)
		return struct.unpack('<H',reply[10:12])[0]

	def keypress(self,uint1):
		cmd = self.CMD_KEYPRESS + struct.pack('<H',2) + struct.pack('<H',uint1)
		cmd_len = struct.pack('<H',len(cmd))
		cmd_crc = struct.pack('<H',crc16_ccitt(cmd))
		cmd = b'\xAB\xCD' + cmd_len + cmd + cmd_crc + b'\xDC\xBA'
		self.uart_send_msg(cmd)
		time.sleep(sleepafterkeypress)
		reply = self.uart_receive_msg(0)
		return True #hex(struct.unpack('<H',reply[10:12])[0])

def send(sendstr):
	if verbose: print(f'SEND: {sendstr}')
	bs.send(sendstr.encode()+b'\n')
	bs.flush()

def mhz(hz):
	mhz = int(hz/1000000)
	hz = hz - 1000000 * mhz
	khz = int(hz/1000)
	hz = hz - 1000 * khz
	return str(mhz) + '.' + str(khz).zfill(3) + ' ' + str(int(hz/10)).zfill(2)

def press(sequence):
	if verbose: print('PRESS:',sequence)
	for key in sequence.split():
		# Use of if/elif instead match/case is intentional tu support Python <3.10
		if key.isnumeric():
			for num in key:
				radio.keypress(int(num)+32)
		elif key == 'menu':
			radio.keypress(42)
		elif key == 'up':
			radio.keypress(43)
		elif key == 'down':
			radio.keypress(44)
		elif key == 'exit':
			radio.keypress(45)
		elif key == '*': # SCAN
			radio.keypress(46)
		elif key == '#': # FN
			radio.keypress(47)
		elif key == 'ptt':
			radio.keypress(48)
		elif key == 'f2':
			radio.keypress(49)
		elif key == 'f1':
			radio.keypress(50)

def set_freq(vfoid,freq): # FIXME: Check PTT before changing frequency or VFO
		if freqmin <= freq <= freqmax:
			freqsteps = int(round(freq/vfostep,0)) # steps count from 0, not real frequency
			vfofreqsteps = int(round(vfofreq[vfoid] / vfostep,0)) # steps count from 0, not real frequency
			if freqsteps != vfofreqsteps:
				print(f'{vfoname[vfoid]} update: {mhz(vfofreq[vfoid])} -> {mhz(freqsteps*vfostep)} MHz by ', end='', flush=True)
				freqstepsdiff = abs(vfofreqsteps-freqsteps)
				if vfoid != vfocurrent:
					# If we need to change non-current VFO, switch to it there and then back
					press_before = press_after = ' f2 '
				else:
					# If artefact/menu is present, try to cancel it first
					press_before = 'exit exit '
					press_after = ''
				if freqstepsdiff > 6: # Decide effectivity of VFO change by steps vs full
					print('full frequency setting.')
					press(press_before + str(int(freqsteps*vfostep/1000)).zfill(6) + press_after)
				else:
					if freqsteps > vfofreqsteps:
						print(f'stepping UP {freqstepsdiff} times.')
						press(press_before + 'up ' * freqstepsdiff + press_after)
					else:
						print(f'stepping DOWN {freqstepsdiff} times.')
						press(press_before + 'down ' * freqstepsdiff + press_after)
			vfofreq[vfoid] = freqsteps*vfostep
			return True
		else:
			print(f'Frequency {mhz(freq)} MHz out of range ({mhz(freqmin)} - {mhz(freqmax)} MHz).')
			return False

def get_freq(vfoid):
#		freqread = radio.get_current_frequency()
#		set_freq(vfoid,vfofreq[vfoid])
# FIXME: We should check the real frequency only when we didn't set/read the frequency long time ago
#        and overwrite it with our value, because it can be misaligned, but the problem is when RX
#        on crossband is in progress we will read the other frequency. Mybe some registers tells RXbusy?
#        or maybe if we have no frequent updates, we could write the full frequency to reset it to proper value
	return vfofreq[vfoid]


# BEGIN #####################################################################

if len(sys.argv) > 1:
	print(f'{appname} {appver} - {appdesc}')
	print()
	# Connect and read
	with uvk5(sys.argv[1]) as radio:
		if radio.connect():
			# This turns off constant garbage input
			print('Getting FW version: ', end='', flush=True)
			fw_version = radio.get_fw_version()
#			fw_version = 'Testing'
			if fw_version:
				print(fw_version);

				# Selected VFO
				if len(sys.argv) > 2:
					vfocurrent = vfoname.index(sys.argv[2])
				print('Assuming current VFO:',vfoname[vfocurrent])
				# VFO step
				print('Assuming VFO step:',vfostep,'Hz') # FIXME: Set step

				# TRX init
				print('Init: RxMode -> MAIN ONLY') # This is important also for proper initial frequency reading, because crossband RX with active receiving (e.g. monitor) puts in registers non-current frequency
				press('exit exit menu 59 menu 0 menu exit')
				print('Init: F2Shrt -> SWITCH VFO')
				press('menu 25 menu 8 menu exit')
				# VFO read
				print('Getting initial',vfoname[vfocurrent],'frequency: ', end='', flush=True)
				vfofreq[vfocurrent] = radio.get_current_frequency();
				print(mhz(vfofreq[vfocurrent]),'MHz')
				press('f2')
				print('Getting initial',vfoname[1 - vfocurrent],'frequency: ', end='', flush=True)
				vfofreq[1 - vfocurrent] = radio.get_current_frequency();
				print(mhz(vfofreq[1 - vfocurrent]),'MHz')
				press('f2')

				# Open port
				s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
				s.bind((bindip, tcpport))
				s.listen(1)
				print('Wating for connection...', end='', flush=True)
				conn, client_address = s.accept()
				print(f' connected from {client_address[0]}:{client_address[1]}')
				bs = bufsock.bufsock(conn)
				while True:
					bs.flush()
					rigctlcmd = bs.readto(b'\n')
					if rigctlcmd:
						rigctlcmd = rigctlcmd.rstrip(b'\r\n').decode()
						if verbose: print(f'RECV: {rigctlcmd}')
						rigctlcmd = rigctlcmd.split()
						# Use of if/elif instead match/case is intentional tu support Python <3.10

						if rigctlcmd[0] == 'q': # quit # FIXME: \long_commands
							print('Exiting.')
							exit()

						elif rigctlcmd[0] == '_': # get_info # FIXME: dump_caps
							send(appname)

						elif rigctlcmd[0] == 't': # get_ptt
							send('0') # FIXME: Can we read the PTT status?

						elif rigctlcmd[0] == 'F': # set_freq
							if vfosplit:
								if set_freq(1 - vfosplittx,int(rigctlcmd[1])):
									send('RPRT 0')
								else:
									send('RPRT -1')
							else:
								if set_freq(vfocurrent,int(rigctlcmd[1])):
									send('RPRT 0')
								else:
									send('RPRT -1')

						elif rigctlcmd[0] == 'f': # get_freq
							if vfosplit:
								send(str(get_freq(1 - vfosplittx)))
							else:
								send(str(get_freq(vfocurrent)))

						elif rigctlcmd[0] == 'v': # get_vfo # FIXME: Can we read the real selected VFO?
							send(vfoname[vfocurrent])

						elif rigctlcmd[0] == 'V': # set_vfo
							if vfosplit: # We don't allow to change VFO in split mode since we can TX only on current VFO
								send('RPRT -1')
							else:
								if vfocurrent != vfoname.index(rigctlcmd[1]):
									press('f2')
									vfocurrent = vfoname.index(rigctlcmd[1])
								send('RPRT 0')

						elif rigctlcmd[0] == 'S': # set_split_vfo
							vfosplit = int(rigctlcmd[1])
							if vfosplit:
								print('Init: RxMode -> CROSS BAND')
								press('exit exit menu 59 menu 2 menu exit')
								if rigctlcmd[2] == 'Main':
									rigctlcmd[2] = 'VFOA'
								if rigctlcmd[2] == 'Sub':
									rigctlcmd[2] = 'VFOB'
								vfosplittx = vfoname.index(rigctlcmd[2])
								if vfocurrent != vfosplittx:
									press('f2')
									vfocurrent = vfosplittx
							send('RPRT 0')

						elif rigctlcmd[0] == 's': # get_split_vfo
							send(str(vfosplit))
							send(vfoname[vfosplittx])

						elif rigctlcmd[0] == 'I': # set_split_freq
							if vfosplit:
								if set_freq(vfosplittx,int(rigctlcmd[1])):
									send('RPRT 0')
								else:
									send('RPRT -1')
							else:
								send('RPRT 0')

						elif rigctlcmd[0] == 'i': # get_split_freq
							if vfosplit:
								send(str(get_freq(vfosplittx)))
							else:
								send('RPRT -1')

						else:
							print('Unknown command.')
							send('RPRT 0')
					else:
						print("Connection probably closed. Exiting.")
						exit()
			else:
				print('Failed, exiting.')

else:
	print(f'Usage: {os.path.basename(sys.argv[0])} <COMx> [<VFOx>]')
# FIXME: -v -l 0.0.0.0 -p 4532 -c COMx -r {a|b} -f 1000 -b 'keypress [keypress ...]' -a 'keypress [keypress ...]' [-k <keypress> [<keypress> ...]]
	print( '       COMx = serial port, e.g. COM2')
	print( '       VFOx = current VFO, e.g. VFOB')
	print()
	print('Prerequisites:')
	print('* Quansheng UV-K5 flashed with https://github.com/nicsure/quansheng-dock-fw')
	print('* Defined step (min. 1000 Hz) must be configured on all *used* VFOs and *bands*')
	print('* If using TX:')
	print('  * VFOs must be in VFO mode, not memory (MENU 3)')
	print('  * Current VFO must match the state on the radio when program starts')
	print('  * Modulation and CTSS settings must be configured in *used* VFOs and bands')
	print('  * Battery read must be below 8.9V, you can recalibrate it in hidden menu,')
	print('    see https://github.com/egzumer/uv-k5-firmware-custom/wiki/Menu')
	print('* AIOC or modified cable described on https://github.com/nicsure/QuanshengDock')
	print('* Software supporting rigctl protocol e.g. https://github.com/csete/gpredict')
	print()
	print('Notices:')
	print('* Do not change frequency manually. We do not read it back.')
	print('* F1 (lower side) button short press function is initialized as SWITCH VFO')
	print('  and we are unable to return the previous function.') # FIXME: Could be fixed with -a 'menu ...'
	print()
	print(f'{appname} {appver}')
	print(f'Description: {appdesc}')
	print(f'URL: {appurl}')
	print(f'Author: {appauth}')
	print(f'License: {applic}')
