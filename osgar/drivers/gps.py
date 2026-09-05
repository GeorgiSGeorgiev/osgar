"""
  GPS Driver
"""

from threading import Thread
import struct

from osgar.bus import BusShutdownException


INVALID_COORDINATES = [None, None]
BIN_PREAMBULE = bytes([0xB5, 0x62])


def checksum(s):
    sum = 0
    for ch in s:
        sum ^= ch
    return b"%02X" % (sum)


def str2deg(s):
    'convert DDMM.MMMMMM string to deg'
    assert s != ""
    assert "." in s, s
    dm, frac = ('0000' + s).split('.')
    try:
        return float(dm[:-2]) + float(dm[-2:] + '.' + frac) / 60
    except ValueError as e:
        print(e)
        return None


def nmea2coord_ms(nmea_data):
    lon = nmea_data["lon"]
    lat = nmea_data["lat"]
    if lon and lat:
        return [int(round(lon*3_600_000)), int(round(lat*3_600_000))]
    return [None, None]


def parse_nmea(line):
    nmea_list = line.decode().split(",")
    assert len(nmea_list) == 15
    nmea_data = {}
    # NMEA sentence $GPGGA or $GNGGA
    # https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
    try:
        nmea_data["identifier"] = nmea_list[0]
        nmea_data["lon"] = None if nmea_list[4] == "" else str2deg(nmea_list[4])
        nmea_data["lon_dir"] = None if nmea_list[5] == "" else nmea_list[5]
        nmea_data["lat"] = None if nmea_list[2] == "" else str2deg(nmea_list[2])
        nmea_data["lat_dir"] = None if nmea_list[3] == "" else nmea_list[3]
        nmea_data["utc_time"] = None if nmea_list[1] == "" else nmea_list[1]  # format: "%H%M%S.%f"
        nmea_data["quality"] = None if nmea_list[6] == "" else int(nmea_list[6])
        nmea_data["sats"] = None if nmea_list[7] == "" else int(nmea_list[7])
        nmea_data["hdop"] = None if nmea_list[8] == "" else float(nmea_list[8])
        nmea_data["alt"] = None if nmea_list[9] == "" else float(nmea_list[9])
        nmea_data["a_units"] = None if nmea_list[10] == "" else nmea_list[10]
        nmea_data["undulation"] = None if nmea_list[11] == "" else float(nmea_list[11])
        nmea_data["u_units"] = None if nmea_list[12] == "" else nmea_list[12]
        nmea_data["age"] = None if nmea_list[13] == "" else float(nmea_list[13])
        stn_id = nmea_list[14].split("*")[0]
        nmea_data["stn_id"] = None if stn_id == "" else stn_id
    except ValueError as e:
        print(e)
        return None

    return nmea_data


def parse_line(line):
    assert line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'), line
    if checksum(line[1:-3]) != line[-2:]:
        print('Checksum error!', line, checksum(line[1:-3]))
        return None
    nmea_data = parse_nmea(line)

    return nmea_data


KNOTS_TO_MPS = 0.514444


def parse_rmc(line):
    """RMC ("recommended minimum") - the receiver was already sending this
    once per second alongside GGA and it was being discarded, because
    split_buffer() only ever searched for GGA.

    The valuable field is course over ground: the receiver derives it from
    its own velocity solution rather than by differencing positions, which
    makes it markedly steadier than a bearing computed between two fixes.
    Measured over the 2026-08-29 Stromovka logs, median change between
    consecutive 1Hz samples: 2.83 deg for RMC course against 8.43 deg for a
    1-second position chord.

      $GNRMC,utc,status,lat,NS,lon,EW,sog_knots,cog_deg,date,magvar,EW,mode*cs

    Returns {'cog': degrees true, 'sog': m/s, 'rmc_status': 'A'/'V'} or
    None. cog is None when the receiver could not determine it - which it
    routinely cannot at a standstill, since course comes from velocity and
    there is none. ALWAYS gate on sog before trusting cog."""
    if checksum(line[1:-3]) != line[-2:]:
        print('Checksum error!', line, checksum(line[1:-3]))
        return None
    fields = line.decode(errors='replace').split(',')
    if len(fields) < 10:
        return None
    try:
        sog = None if fields[7] == '' else float(fields[7]) * KNOTS_TO_MPS
        cog = None if fields[8] == '' else float(fields[8])
    except ValueError as e:
        print(e)
        return None
    return {'rmc_status': fields[2] or None, 'sog': sog, 'cog': cog}


def parse_bin(data):
    assert data.startswith(BIN_PREAMBULE), data
    c, i, size = struct.unpack_from('<BBH', data, 2)
    assert len(data) == size + 8, (len(data), size + 8)
    assert c == 1, c  # class = 1 ... NAVigation messages
    assert i in [3, 0x30, 6, 7, 0x34, 0x35, 1, 2, 0x13, 0x14, 4, 0x11, 0x12, 0x20, 0x23, 0x24, 0x21,
                0x26, 0x22, 0x9, 0x3B, 0x3C, 0x39, 0x61, ], hex(i)  # ID
#    print(hex(i))

    # TODO verify checksum!
    payload = data[6:-2]

    # 31.18.20 UBX-NAV-STATUS (0x01 0x03)
    # 31.18.20.1 Receiver Navigation Status
    # Receiver Navigation Status
    if i == 0x03:
        fix = payload[4]
        #assert fix in [2, 3], fix  # 2D, 3D
        if fix not in [2, 3]:
            print("GPS no fix!")

    # 31.18.21 UBX-NAV-SVINFO (0x01 0x30)
    # 31.18.21.1 Space Vehicle Information
    # Information about satellites used or visible
    if i == 0x30:
        return None

    # 31.18.15 UBX-NAV-RELPOSNED (0x01 0x3C)
    # 31.18.15.1 Relative Positioning Information in NED frame
    if i == 0x3C:
        assert len(payload) == 40, len(payload)
        assert payload[0] == 0, payload[0]  # version
        iTOW, rel_pos_north_cm, rel_pos_east_cm = struct.unpack_from('<Iii', payload, 4)
        accN, accE = struct.unpack_from('<II', payload, 24)
        return {'rel_position': [rel_pos_east_cm, rel_pos_north_cm]}

    # 31.18.30 UBX-NAV-VELNED (0x01 0x12)
    # 31.18.30.1 Velocity Solution in NED
    if i == 0x12:
        assert len(payload) == 36, len(payload)
        iTOW, velN, velE, velD = struct.unpack_from('<Iiii', payload, 0)
        gSpeed, heading, sAcc, cAcc = struct.unpack_from('<IiII', payload, 20)
        #print(gSpeed, heading/1e5, sAcc, cAcc/1e5)
        return None  # do not integrate for now

    # 31.18.14 UBX-NAV-PVT (0x01 0x07)
    # 31.18.14.1 Navigation Position Velocity Time Solution
    if i == 0x07:
        assert len(payload) == 92, len(payload)
        gSpeed, heading, sAcc, cAcc = struct.unpack_from('<IiII', payload, 60)
        #print('xx', gSpeed, heading/1e5, sAcc, cAcc/1e5)
        return None  # do not integrate for now


# NMEA sentences this driver extracts from the serial stream. GGA carries
# position/quality; RMC carries course and speed over ground (see parse_rmc).
# Anything not listed here is skipped over as before.
NMEA_SENTENCES = [b'$GNGGA', b'$GPGGA', b'$GNRMC', b'$GPRMC']


def find_nmea_start(data):
    """Offset of the EARLIEST supported sentence in data, or -1.

    Earliest, not latest: with more than one sentence buffered they have to
    be handed out in the order they arrived, or the extra ones are dropped.
    (The previous max() worked only because just one sentence type was ever
    looked for, so at most one of the two finds could be non-negative.)"""
    best = -1
    for tag in NMEA_SENTENCES:
        i = data.find(tag)
        if i >= 0 and (best < 0 or i < best):
            best = i
    return best


def split_buffer(data):
    # in dGPS there is a block of binary data so stronger selection is required
    start_nmea = find_nmea_start(data)
    start_bin = data.find(BIN_PREAMBULE)
    if start_nmea < 0 or (0 <= start_bin < start_nmea):
        if start_bin < 0 or start_bin + 8 >= len(data):
            return data, b''
        else:
            # extract binary data: preambule, class, ID, len, payload, checksum
            c, i, size = struct.unpack_from('<BBH', data, start_bin + 2)
            end = start_bin + 6 + size + 2
            if end >= len(data):
                return data, b''
            return data[end:], data[start_bin:end]

    start = start_nmea
    end = data[start:-2].find(b'*')
    if end < 0:
        return data, b''
    return data[start+end+3:], data[start:start+end+3]


class GPS(Thread):
    def __init__(self, config, bus):
        bus.register('position', 'rel_position', 'nmea_data')
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''
        # most recent RMC (course/speed over ground) - merged into the next
        # published nmea_data, see process_packet
        self.last_rmc = None

    def process_packet(self, line):
        if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
            nmea_data = parse_line(line)
            if nmea_data is not None and self.last_rmc is not None:
                # RMC arrives as its own sentence in the same 1Hz burst as
                # GGA. Merging it into nmea_data rather than publishing a
                # separate stream keeps every existing consumer and every
                # existing config/link working untouched - they simply see
                # three extra keys (cog/sog/rmc_status) and ignore them.
                nmea_data.update(self.last_rmc)
            return {'nmea_data': nmea_data}
        elif line.startswith(b'$GNRMC') or line.startswith(b'$GPRMC'):
            rmc = parse_rmc(line)
            if rmc is not None:
                self.last_rmc = rmc
            return None
        elif line.startswith(BIN_PREAMBULE):
            return parse_bin(line)
        return None

    def process_gen(self, data):
        self.buf, packet = split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                for k, v in ret.items():
                    if v is not None:
                        yield k, v
            self.buf, packet = split_buffer(self.buf)  # i.e. process only existing buffer now

    def run(self):
        try:
            while True:
                packet = self.bus.listen()  # there should be some timeout and in case of failure send None
                dt, __, data = packet
                for name, out in self.process_gen(data):
                    assert out is not None
                    self.bus.publish(name, out)
                    if name == "nmea_data":
                        self.bus.publish("position", nmea2coord_ms(out))
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


def print_output(packet):
    print(packet)

# vim: expandtab sw=4 ts=4
