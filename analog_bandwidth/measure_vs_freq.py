import numpy, numpy.fft
from matplotlib import pyplot
import os
import serial

def bufferred_reader():
	pass

def get_reading(comm):
	while True:
		line = comm.readline()
		if not line:
			continue

		#print line
		#continue

		f = line.split()

		if len(f) != 1024 + 2:
			continue

		try:
			data = numpy.array(map(int, f[1:-1]))
		except ValueError:
			continue

		return data

class usbtmc:
	def __init__(self, device):
		self.device = device
		self.f = os.open(device, os.O_RDWR)
	
	def write(self, command):
		os.write(self.f, command);

	def read(self, length=4000):
		return os.read(self.f, length)

	def query(self, command, length=300):
		self.write(command)
		return self.read(length)

	def get_name(self):
		return self.query("*IDN?")

	def send_reset(self):
		self.write("*RST")

	def close(self):
		os.close(self.f)

class SignalGenerator(usbtmc):
	def rf_on(self, freq_hz, power_dbm):
		power_dbm = max(-145, power_dbm)

		self.write("freq %f Hz\n" % (freq_hz,))
		self.write("pow %f dBm\n" % (power_dbm,))
		self.write("outp on\n")

	def rf_off(self):
		self.write("outp off\n")

def main():
	comm = serial.Serial("/dev/ttyUSB0", 115200)
	gen = SignalGenerator("/dev/usbtmc2")

	#data = [0,4095] * 512
	#x = numpy.arange(0, 1024)

	data = [0.0,250e3] * 256
	x = numpy.arange(0, 1e6/2, 1e6/1024) + 1e6/1024;

	pyplot.ion()
	pyplot.clf()
	pyplot.grid()
	pline, = pyplot.plot(x, data)
	pyplot.draw()

	out = open("display2.dat", "w")

	for fexp in numpy.arange(4, 8, 0.01):
		f = 10.0**fexp
	#for f in numpy.arange(100e3, 2e6, 10e3):

		gen.rf_on(f, 0)
		data = get_reading(comm)

		w = numpy.fft.rfft(data)[1:]
		print len(w), max(abs(w))

		n = numpy.argmax(w)
		fm = x[n]

		pline.set_ydata(abs(w))
		pyplot.draw()

		out.write("%f\t%f\t%f\n" % (f, max(data) - min(data), fm))
		out.flush()

	gen.rf_off()

main()
