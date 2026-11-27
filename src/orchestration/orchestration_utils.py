



def divisor(freq_hz, dt):
    n = round(1 / (freq_hz * dt))
    if n < 1 or abs(n * dt - 1 / freq_hz) > 1e-9:
        raise ValueError("fréquence incompatible avec dt")
    return n # number of ticks needed to have a full period T = 1 / freq_hz