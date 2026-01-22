"""
    written by Szymon Czerwiński and Marcin Dąbal
"""

from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import serial
import struct

###
# Serial init 
### 
ser = serial.Serial('COM3', 115200, timeout=1)
HEADER = b'\xAA\x55'

###
# Plot init
###

t_data = np.array([])
v_data = np.array([])

Y_MIN, Y_MAX = -0.5, 3.5
X_WINDOW = 2.0  # okno wykresu [s]

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], marker='o', linestyle='-', label="ADC")
freq_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)
ax.set_ylim(Y_MIN, Y_MAX)

# Title and labels 
plt.title('Signal on STM32 DAC')
plt.xlabel('Time [s]')
plt.ylabel('Voltage [V]')

# grid
plt.grid(True, 'major')
plt.grid(True, 'minor', linewidth=0.3)
plt.minorticks_on()

###
# Main loop
###

while True:
    # get message header
    sync = ser.read(2)
    if sync != HEADER:
        continue

    # get signal frequency
    f_raw = ser.read(2)
    if len(f_raw) < 2:
        continue
    f = struct.unpack('<h', f_raw)[0]
    f = float(f)/1000

    # get following data's length
    size_raw = ser.read(2)
    if len(size_raw) < 2:
        continue
    size = struct.unpack('<H', size_raw)[0]

    # get signal samples
    data_raw = ser.read(size * 2)
    if len(data_raw) < size * 2:
        continue
    adc = np.frombuffer(data_raw, dtype='<u2')
    volts = adc / 4095.0 * 3.3

    # time axis
    t_buf = np.linspace(0, X_WINDOW, int(1100*X_WINDOW))
    t_data = t_buf

    # interpolate the sine signal to corretly fit the linear space    
    samples_per_sine = int(1100*X_WINDOW/f)
    interp = interp1d(t_buf[:size], volts, kind='linear', fill_value='extrapolate')
    newsin = interp(np.linspace(0, t_buf[size], samples_per_sine))
    
    # amplitude axis
    v_data = newsin
    for i in range(int(f)):
        v_data = np.append(v_data, newsin)
    v_data = v_data[:int(1100*X_WINDOW)]

    # update the plot
    line.set_data(t_data, v_data)
    ax.set_xlim(t_data[0], t_data[-1])
    freq_text.set_text(f"Częstotliwość ≈ {f:.2f} Hz")


    plt.pause(0.01)
