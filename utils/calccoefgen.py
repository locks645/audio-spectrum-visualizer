import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

def calcrange(n, min, max):
    result = []
    r = (max / min) ** (1 / n)
    for i in range(0, n):
        result.append(min * (r ** i))
    return result

center_ranges = calcrange(10, 20, 8000)
print(center_ranges)

sampling_freq = 10000  # Sampling frequency in Hz
bandwidth_percent = 0.90    # start at 90% bandwidth
order = 3

sos_list = []
for center_freq in center_ranges:
    print(f"bandwidth percent: {bandwidth_percent}")
    bandwidth = center_freq * bandwidth_percent
    #bandwidth = center_freq * bandwidth_percent
    low_cutoff = center_freq - bandwidth / 2
    high_cutoff = center_freq + bandwidth / 2
    sos = signal.butter(order, [low_cutoff, high_cutoff], btype='bandpass', analog=False, output='sos', fs=sampling_freq)
    sos_list.append(sos)
    print(f"coefficients for {center_freq}: \n{sos}\n")
    
    bandwidth_percent *= 0.7

coef_str = "void initialize_coef(void) {\n"
for i, sos in enumerate(sos_list):
    for j, section in enumerate(sos):
        for k, coef in enumerate(section):
            coef_str += f"\tfilters[{i}].coef[{j}][{k}] = float_to_fix({coef});\n"
coef_str += "}"
print(coef_str)
    
for sos in sos_list:
    w, h = signal.sosfreqz(sos, worN=8000, fs=sampling_freq)
    plt.semilogx(w, 20 * np.log10(np.abs(h) + 1e-10), label=f'{center_freq:.2f} Hz')

plt.title('Frequency Responses of Bandpass Filters')
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude [dB]')
plt.grid(True)
plt.legend(loc='best')
plt.show()
