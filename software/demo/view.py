import struct
import sys

tag_measurement_zc = ord('Z')
tag_measurement_wave = ord('W')
tag_gps_pvt = ord('P')
tag_gps_tp = ord('T')

waveforms = []


def process_zc(idx):
    tag, magic1, magic2, magic3 = contents[idx:idx+4]
    if tag != tag_measurement_zc:
        print("invalid zc tag")
        return 1
    if magic1 != 0xEF or magic2 != 0xCD or magic3 != 0xAB:
        print("bad magic")
        return 1
    length = struct.unpack("<I", contents[idx+4:idx+8])[0]
    week, tow, freq, rms = struct.unpack("<HQdd", contents[idx+8:idx+8+length])
    print("ZC week={} tow={} freq={} rms={}".format(week, tow, freq, rms))
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
    data = struct.unpack("<HQ256h", contents[idx+8:idx+8+length])
    week, tow = data[0], data[1]
    waveform = data[2:]
    global waveforms
    waveforms += waveform
    print("WAVE week={} tow={}".format(week, tow))
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


def process_tp(idx):
    tag, magic1, magic2, magic3 = contents[idx:idx+4]
    if tag != tag_gps_tp:
        print("invalid tp tag")
        return 1
    if magic1 != 0xEF or magic2 != 0xCD or magic3 != 0xAB:
        print("bad magic")
        return 1
    length = struct.unpack("<I", contents[idx+4:idx+8])[0]
    data = struct.unpack("<IIiHBB", contents[idx+8:idx+8+length])
    tow_ms, tow_sub_ms, q_err, week, flags, ref_info = data
    print("TP tow_ms={} tow_sub_ms={} q_err={} week={} flags={} ref_info={}"
          .format(tow_ms, tow_sub_ms, q_err, week, flags, ref_info))
    return length+8


processors = {
    tag_measurement_zc: process_zc,
    tag_measurement_wave: process_wave,
    tag_gps_pvt: process_pvt,
    tag_gps_tp: process_tp,
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
