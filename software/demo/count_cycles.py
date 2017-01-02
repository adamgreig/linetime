import struct
import sys

tag_measurement_zc = ord('Z')
tag_measurement_wave = ord('W')
tag_gps_pvt = ord('P')


cycles_31st = 0
cycles_1st = 0
freqs_31st = []
freqs_1st = []


def process_zc(idx):
    global cycles_31st, cycles_1st
    tag, magic1, magic2, magic3 = contents[idx:idx+4]
    if tag != tag_measurement_zc:
        print("invalid zc tag")
        return 1
    if magic1 != 0xEF or magic2 != 0xCD or magic3 != 0xAB:
        print("bad magic")
        return 1
    length = struct.unpack("<I", contents[idx+4:idx+8])[0]
    day, month, tow, freq, rms = struct.unpack(
        "<BBQdd", contents[idx+8:idx+8+length])
    tow /= float(2**32)
    tow /= 1000.0
    if tow >= 518416 or tow < 17:
        cycles_31st += 1
        freqs_31st.append(freq)
    if tow >= 17 and tow < 86417:
        cycles_1st += 1
        freqs_1st.append(freq)
    return length+8
    print("ZC month={} day={} tow={} freq={} rms={}"
          .format(month, day, tow, freq, rms))
    return length+8


def process_wave(idx):
    tag, magic1, magic2, magic3 = contents[idx:idx+4]
    if tag != tag_measurement_wave:
        print("invalid wave tag")
        return 1
    if magic1 != 0xEF or magic2 != 0xCD or magic3 != 0xAB:
        print("bad magic")
        return 1
    length = struct.unpack("<I", contents[idx+4:idx+8])[0]
    return length+8
    data = struct.unpack("<BBQ256h", contents[idx+8:idx+8+length])
    day, month, tow = data[0], data[1], data[2]
    tow /= float(2**32)
    tow /= 1000.0
    print("WAVE month={} day={} tow={}".format(month, day, tow))
    return length+8


def process_pvt(idx):
    tag, magic1, magic2, magic3 = contents[idx:idx+4]
    if tag != tag_gps_pvt:
        print("invalid pvt tag")
        return 1
    if magic1 != 0xEF or magic2 != 0xCD or magic3 != 0xAB:
        print("bad magic")
        return 1
    length = struct.unpack("<I", contents[idx+4:idx+8])[0]
    return length+8
    data = struct.unpack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHHIiI",
                         contents[idx+8:idx+8+length])
    (i_tow, year, month, day, hour, minute, second, valid, t_acc, nano,
     fix_type, flags, reserved1, num_sv, lon, lat, height, h_msl, h_acc, v_acc,
     vel_n, vel_e, vel_d, gspeed, head_mot, s_acc, head_acc, p_dop, reserved2,
     reserved3, head_veh, reserved4) = data
    print("PVT i_tow={} year={} month={} day={} hour={} minute={} second={}"
          " valid={} t_acc={} nano={} fix_type={} flags={} reserved1={}"
          " num_sv={} lon={} lat={} height={} h_msl={} h_acc={} v_acc={}"
          " vel_n={} vel_e={} vel_d={} gpseed={} head_mot={} s_acc={}"
          " head_acc={} p_dop={} reserved2={} reserve3={} head_veh={}"
          " reserved4={}"
          .format(*data))
    return length+8

processors = {
    tag_measurement_zc: process_zc,
    tag_measurement_wave: process_wave,
    tag_gps_pvt: process_pvt,
}

with open(sys.argv[1], "rb") as f:
    contents = f.read()

idx = 0
while idx < len(contents):
    if contents[idx] == 0:
        idx += 1
    elif contents[idx] in processors:
        idx += processors[contents[idx]](idx)
    else:
        print("unhandled byte at idx", idx)
        break

import numpy as np
print("cycles on 31st:", cycles_31st)
print("cycles on 1st:", cycles_1st)
print("avg freq on 31st:", np.mean(freqs_31st))
print("avg freq on 1st:", np.mean(freqs_1st))
print("avg freq, both days:", np.mean(freqs_31st + freqs_1st))

freqs = np.array(freqs_31st + freqs_1st)
freqs = freqs[:(freqs.size//50)*50]
freqs = freqs.reshape((-1, 50)).mean(axis=1)
import matplotlib.pyplot as plt
plt.plot(freqs)
plt.show()
