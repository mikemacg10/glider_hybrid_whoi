import numpy as np
import scipy.signal as signal

def generateSignal(fs, startFreq, endFreq, duration, zeroPadTime = 0.5):
    # np array LFM Signal
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    lfm_signal = signal.chirp(t, f0=startFreq, f1=endFreq, t1=duration, method='linear')

    # Zero-pad the signal on either side
    pad_length = int(fs * zeroPadTime)  # Pad with 0.1 seconds of zeros on each side
    lfm_signal = np.pad(lfm_signal, (pad_length, pad_length), 'constant', constant_values=(0, 0))
    t = np.linspace(0, (len(lfm_signal) - 1) / fs, len(lfm_signal), endpoint=False)

    return lfm_signal

def correlateSignals(signal1, signal2, fs=48000):
    correlation = signal.correlate(signal1, signal2, mode="full", method="fft")
    Lags = signal.correlation_lags(signal1.size, signal2.size, mode="full")/fs
    MaxIndex = np.argmax(correlation)
    TimeDelay = Lags[MaxIndex]
    return TimeDelay

def HilbertTransform(TDOA):
    # TDOA is a list of TDOA values
    # returns a list of TDOA values
    pass

def addNoise(signal, snr):
    # TODO: Verify this code
    signal_power = np.sum(np.abs(signal) ** 2) / len(signal)

    # Calculate the power of the noise
    noise_power = signal_power / (10 ** (snr / 10))

    # Generate white noise
    noise = np.random.normal(0, np.sqrt(noise_power), len(signal))

    # Add the noise to the signal
    noisy_signal = signal + noise

    return noisy_signal

def delaySignal(signal, delay, fs):
    delay_samples = int(delay * fs)
    delayed_signal = np.roll(signal, delay_samples)
    return delayed_signal

