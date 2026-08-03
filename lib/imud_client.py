"""
imud_client.py — Python client library for imud Stream B binary packets

Quick start:

    from imud_client import ImudClient

    with ImudClient() as client:
        for pkt in client:
            print(f"hdg={pkt.heading_deg:.1f}  rot={pkt.rate_of_turn:.1f} dpm")

For multicast (default hi-rate address 239.255.0.1):

    with ImudClient(port=10111, addr="239.255.0.1") as client:
        ...

For the daemon's TCP stream listener ([stream] tcp_enabled, default port
10112) — lossless framed packets over the network:

    with ImudClient.connect_tcp("boat.local") as client:
        ...
    # equivalently: ImudClient(tcp=("boat.local", 10112))

Requirements: Python 3.8+, standard library only.

Copyright (c) 2026 Richard Simpson
SPDX-License-Identifier: MIT
"""

import socket
import struct
import zlib
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

# ── Protocol constants ────────────────────────────────────────────────────────

IMUD_MAGIC       = 0x494D5544   # "IMUD"
# Wire-layout revision, NOT the release version.  Encoded as major*10 + minor
# of the release that last CHANGED the packet layout — 17 = the layout
# introduced in 1.7 (update-gate health fields); 14 was the 1.4 layout, which
# 1.5 and 1.6 shipped unchanged.  A 1.8 daemon still speaks 17.
# Must equal IMUD_VERSION in include/types.h and lib/imud_client.h; CI's
# version-consistency job checks all three agree.
IMUD_VERSION     = 17
IMUD_PACKET_SIZE = 276

# ── Flags ─────────────────────────────────────────────────────────────────────

class Flags:
    MAG_VALID        = 1 << 0   # mag healthy and calibrated
    MAG_SET_RESET    = 1 << 1   # SET/RESET pulse within last read
    FUSION_CONVERGED = 1 << 2   # MEKF covariance settled
    ACCEL_CAL        = 1 << 3   # accel calibration applied
    GYRO_CAL         = 1 << 4   # gyro bias applied
    MAG_CAL          = 1 << 5   # mag hard/soft-iron cal applied
    MOTION           = 1 << 6   # retired — never set; use quiescence or ENGINE_ON
    FIFO_OVERFLOW    = 1 << 7   # sample gap (FIFO overflow)
    STARTUP              = 1 << 8   # gyro bias estimation in progress
    SHUTDOWN             = 1 << 9   # final packet before clean exit
    DECLINATION_VALID    = 1 << 10  # declination known; true_heading valid
    HEAVE_VALID          = 1 << 11  # heave estimator settled (heave_m/heave_rate valid)
    WAVE_VALID           = 1 << 12  # sea-state stats settled (wave/roll/pitch fields valid)
    ENGINE_ON            = 1 << 13  # engine-vibration detector asserting

    @staticmethod
    def describe(flags: int) -> str:
        """Return a compact string like 'CVM' from a flags bitmask."""
        chars = {
            Flags.FUSION_CONVERGED:  'C',
            Flags.MAG_VALID:         'V',
            Flags.ACCEL_CAL:         'A',
            Flags.GYRO_CAL:          'G',
            Flags.MAG_CAL:           'M',
            Flags.DECLINATION_VALID: 'D',
            Flags.HEAVE_VALID:       'H',
            Flags.STARTUP:           'S',
            Flags.FIFO_OVERFLOW:     '!',
            Flags.SHUTDOWN:          'X',
        }
        return ''.join(c for f, c in chars.items() if flags & f)


# ── Packet struct ─────────────────────────────────────────────────────────────
#
# Wire layout (little-endian, 276 bytes):
#   Offset  Field
#    0      magic          uint32
#    4      version        uint16
#    6      flags          uint16
#    8      ts_wall_ns     uint64
#   16      ts_tai_ns      uint64
#   24      ts_chip_ticks  uint32
#   28      anchor_gen     uint32
#   32      accel[3]       3× float32   (calibrated, m/s²)
#   44      accel_raw[3]   3× float32   (pre-cal, m/s²)
#   56      gyro[3]        3× float32   (bias-corrected, rad/s)
#   68      gyro_raw[3]    3× float32   (pre-bias, rad/s)
#   80      mag[3]         3× float32   (calibrated, µT)
#   92      mag_raw[3]     3× float32   (pre-cal, µT)
#  104      quat[4]        4× float32   [w, x, y, z] body→NED
#  120      pitch          float32      rad, NED
#  124      roll           float32      rad, NED
#  128      yaw            float32      rad, magnetic
#  132      heading_deg    float32      0–360°
#  136      rate_of_turn   float32      deg/min
#  140      temp_c         float32      °C
#  144      cov[9]         9× float32   attitude error covariance (rad²)
#  180      imu_seq        uint32
#  184      declination_deg float32     °E+, 0.0 when DECLINATION_VALID not set
#  188      heave_m        float32     m, + up; 0.0 when heave disabled (v1.1)
#  192      gyro_bias[3]     3× float32  estimated gyro bias, rad/s (v1.2)
#  204      gyro_bias_var[3] 3× float32  gyro-bias variance, (rad/s)² (v1.2)
#  216      heave_rate       float32     m/s, + up (v1.2)
#  220      accel_quiescence float32     EMA of (|a|/g−1)² (v1.2)
#  224      wave_height_m    float32     significant wave height Hs, m (v14)
#  228      wave_period_s    float32     mean zero-crossing period Tz, s (v14)
#  232      roll_period_s    float32     vessel roll period, s (v14)
#  236      roll_amplitude   float32     significant single amplitude 2σ(roll), rad (v14)
#  240      pitch_period_s   float32     vessel pitch period, s (v14)
#  244      pitch_amplitude  float32     significant single amplitude 2σ(pitch), rad (v14)
#  248      mag_anomaly      float32     EMA of ||B|−|B_ref||/|B_ref| (v14)
#  252      mag_residual     float32     EMA of |heading innovation|, rad (v14)
#  256      innov_weight     float32     EMA of Huber weight sqrt(gate/d^2) (v17)
#  260      innov_reject     float32     EMA of gate-reject indicator (v17)
#  264      nis_accel        float32     EMA of accel d^2/2; 1.0 = consistent (v17)
#  268      nis_mag          float32     EMA of mag d^2/dof; 1.0 = consistent (v17)
#  272      crc32          uint32

_STRUCT = struct.Struct('<IHHQQII' + 'f' * 37 + 'I' + 'f' * 22 + 'I')

assert _STRUCT.size == IMUD_PACKET_SIZE, \
    f"struct size mismatch: {_STRUCT.size} != {IMUD_PACKET_SIZE}"


@dataclass
class ImudPacket:
    # Header
    magic:          int
    version:        int
    flags:          int
    ts_wall_ns:     int     # CLOCK_REALTIME, ns
    ts_tai_ns:      int     # CLOCK_TAI, ns
    ts_chip_ticks:  int     # IMU hardware counter ticks
    anchor_gen:     int

    # Accelerometer, m/s²
    accel_x:        float
    accel_y:        float
    accel_z:        float
    accel_raw_x:    float   # pre-calibration
    accel_raw_y:    float
    accel_raw_z:    float

    # Gyroscope, rad/s
    gyro_x:         float   # bias-corrected
    gyro_y:         float
    gyro_z:         float
    gyro_raw_x:     float   # before bias correction
    gyro_raw_y:     float
    gyro_raw_z:     float

    # Magnetometer, µT
    mag_x:          float   # calibrated
    mag_y:          float
    mag_z:          float
    mag_raw_x:      float   # pre-calibration
    mag_raw_y:      float
    mag_raw_z:      float

    # Attitude
    quat_w:         float
    quat_x:         float
    quat_y:         float
    quat_z:         float
    pitch:          float   # rad, NED
    roll:           float   # rad, NED
    yaw:            float   # rad, magnetic
    heading_deg:    float   # 0–360°
    rate_of_turn:   float   # deg/min, + = turning right
    temp_c:         float   # °C

    # Covariance
    cov_0: float; cov_1: float; cov_2: float
    cov_3: float; cov_4: float; cov_5: float
    cov_6: float; cov_7: float; cov_8: float

    # Counters / declination / heave
    imu_seq:          int
    declination_deg:  float   # °E+; 0.0 when DECLINATION_VALID flag not set
    heave_m:          float   # m, + up; 0.0 when the heave estimator is off

    # v1.2 diagnostics — IMU body frame / frame-neutral
    gyro_bias_x:      float   # rad/s
    gyro_bias_y:      float
    gyro_bias_z:      float
    gyro_bias_var_x:  float   # (rad/s)²
    gyro_bias_var_y:  float
    gyro_bias_var_z:  float
    heave_rate:       float   # m/s, + up; 0.0 when heave disabled
    accel_quiescence: float   # EMA of (|a|/g−1)²

    # v14 sea state — 0.0 until WAVE_VALID flag set
    wave_height_m:    float   # significant wave height Hs, m
    wave_period_s:    float   # mean zero-crossing wave period Tz, s; 0.0 = n/a
    roll_period_s:    float   # vessel roll period, s; 0.0 = not rolling
    roll_amplitude:   float   # significant single amplitude 2σ(roll), rad
    pitch_period_s:   float   # vessel pitch period, s; 0.0 = not pitching
    pitch_amplitude:  float   # significant single amplitude 2σ(pitch), rad

    # v14 compass health (see man 5 imud.conf)
    mag_anomaly:      float   # EMA of ||B|−|B_ref||/|B_ref| (unitless)
    mag_residual:     float   # EMA of |heading innovation|, rad

    # v17 MEKF update-gate health
    innov_weight:     float   # EMA of Huber weight sqrt(gate/d^2); 1.0 = no capping
    innov_reject:     float   # EMA of gate-reject indicator; 0.0 = none rejected

    # v17 MEKF measurement-model consistency (rolling NIS, tau ~30 s).
    # Normalised innovation squared, accumulated before the Huber cap and
    # including gate-rejected updates. 1.0 = the filter's covariance matches
    # the innovations it actually sees; > 1.0 = over-confident.
    nis_accel:        float   # accel gravity update, d^2/2
    nis_mag:          float   # mag update, d^2/2 (3-D) or d^2/1 (yaw-only)

    crc32:            int

    # ── Convenience properties ────────────────────────────────────────────

    @property
    def accel(self) -> tuple:
        return (self.accel_x, self.accel_y, self.accel_z)

    @property
    def accel_raw(self) -> tuple:
        return (self.accel_raw_x, self.accel_raw_y, self.accel_raw_z)

    @property
    def gyro(self) -> tuple:
        return (self.gyro_x, self.gyro_y, self.gyro_z)

    @property
    def gyro_raw(self) -> tuple:
        return (self.gyro_raw_x, self.gyro_raw_y, self.gyro_raw_z)

    @property
    def mag(self) -> tuple:
        return (self.mag_x, self.mag_y, self.mag_z)

    @property
    def mag_raw(self) -> tuple:
        return (self.mag_raw_x, self.mag_raw_y, self.mag_raw_z)

    @property
    def quat(self) -> tuple:
        return (self.quat_w, self.quat_x, self.quat_y, self.quat_z)

    @property
    def cov(self) -> tuple:
        return (self.cov_0, self.cov_1, self.cov_2,
                self.cov_3, self.cov_4, self.cov_5,
                self.cov_6, self.cov_7, self.cov_8)

    @property
    def cov_trace(self) -> float:
        return self.cov_0 + self.cov_4 + self.cov_8

    @property
    def true_heading_deg(self) -> Optional[float]:
        """True (geographic) heading in [0, 360), or None if declination unknown."""
        if not (self.flags & Flags.DECLINATION_VALID):
            return None
        return (self.heading_deg + self.declination_deg + 360.0) % 360.0

    @property
    def ts_unix(self) -> float:
        """Wall-clock timestamp as UNIX seconds (float)."""
        return self.ts_wall_ns * 1e-9

    @property
    def flags_str(self) -> str:
        return Flags.describe(self.flags)

    def converged(self) -> bool:
        return bool(self.flags & Flags.FUSION_CONVERGED)

    def __str__(self) -> str:
        s = (
            f"ImudPacket seq={self.imu_seq} "
            f"hdg={self.heading_deg:.1f}° "
            f"pitch={self.pitch * 57.2958:.1f}° "
            f"roll={self.roll * 57.2958:.1f}° "
            f"rot={self.rate_of_turn:.1f} dpm "
            f"flags=[{self.flags_str}]"
        )
        th = self.true_heading_deg
        if th is not None:
            s += f" true_hdg={th:.1f}° (decl={self.declination_deg:+.1f}°)"
        return s


def _parse(buf: bytes) -> Optional[ImudPacket]:
    """Parse and validate raw bytes into an ImudPacket, or return None."""
    if len(buf) != IMUD_PACKET_SIZE:
        return None

    # Validate CRC before full unpack (covers bytes 0..271)
    crc_offset = IMUD_PACKET_SIZE - 4
    computed = zlib.crc32(buf[:crc_offset]) & 0xFFFFFFFF
    stored   = struct.unpack_from('<I', buf, crc_offset)[0]
    if computed != stored:
        return None

    fields = _STRUCT.unpack(buf)
    magic, version = fields[0], fields[1]
    if magic != IMUD_MAGIC or version != IMUD_VERSION:
        return None

    return ImudPacket(*fields)


# ── Client ────────────────────────────────────────────────────────────────────

class ImudClient:
    """
    Blocking receiver for imud Stream B binary packets — UDP (default) or
    a TCP connection to the daemon's [stream] tcp_port.

    Parameters
    ----------
    port : int
        UDP port to listen on (default 10111). Ignored in TCP mode.
    addr : str or None
        If a multicast address (224.0.0.0/4), joins that group.
        Pass None or '' to receive unicast/broadcast. Ignored in TCP mode.
    timeout : float or None
        Socket receive timeout in seconds. None = block forever.
    tcp : (host, port) tuple or None
        When set, connect as a TCP client to the daemon's [stream] TCP
        listener (tcp_enabled in imud.conf; default port 10112) and read
        lossless 276-byte frames instead of receiving UDP datagrams.
    """

    def __init__(self, port: int = 10111, addr: Optional[str] = None,
                 timeout: Optional[float] = None,
                 tcp: Optional[Tuple[str, int]] = None):
        self._port    = port
        self._addr    = addr or ''
        self._timeout = timeout
        self._tcp     = tcp
        self._sock: Optional[socket.socket] = None

    @classmethod
    def connect_tcp(cls, host: str = '127.0.0.1', port: int = 10112,
                    timeout: Optional[float] = None) -> 'ImudClient':
        """Client for the daemon's [stream] TCP listener (open() to connect)."""
        return cls(timeout=timeout, tcp=(host, port))

    def open(self) -> 'ImudClient':
        if self._tcp is not None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self._timeout is not None:
                sock.settimeout(self._timeout)
            sock.connect(self._tcp)
            self._sock = sock
            return self

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Wildcard bind is deliberate: this is a receive-only listener for
        # imud's UDP broadcast/multicast telemetry, and a socket bound to a
        # specific unicast address does not receive broadcast datagrams.
        # Nothing is served; every packet is validated (magic/version/CRC)
        # before use.  Restrict exposure with a firewall if required.
        sock.bind(('', self._port))

        if self._addr:
            try:
                grp = socket.inet_aton(self._addr)
                # Check if multicast (first nibble 0xE = 224.x.x.x – 239.x.x.x)
                if grp[0] >> 4 == 0xE:
                    mreq = grp + socket.inet_aton('0.0.0.0')
                    sock.setsockopt(socket.IPPROTO_IP,
                                    socket.IP_ADD_MEMBERSHIP, mreq)
            except OSError:
                pass

        if self._timeout is not None:
            sock.settimeout(self._timeout)

        self._sock = sock
        return self

    def close(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    def _recv_frame(self) -> Optional[bytes]:
        """TCP mode: read exactly one 276-byte frame (None on timeout)."""
        chunks = []
        got = 0
        while got < IMUD_PACKET_SIZE:
            try:
                chunk = self._sock.recv(IMUD_PACKET_SIZE - got)
            except socket.timeout:
                if got:
                    continue      # mid-frame: keep the framing, keep reading
                return None
            if not chunk:
                raise ConnectionError("imud stream closed")
            chunks.append(chunk)
            got += len(chunk)
        return b''.join(chunks)

    def recv(self) -> Optional[ImudPacket]:
        """
        Receive one valid packet (blocking).

        Silently discards packets that are the wrong size, have a bad magic
        number, wrong version, or fail CRC.

        Returns ImudPacket, or None on socket timeout.
        Raises OSError on socket error, ConnectionError when the TCP/stream
        peer closes.
        """
        if self._sock is None:
            raise RuntimeError("ImudClient is not open — call open() first")
        while True:
            if self._tcp is not None:
                buf = self._recv_frame()
                if buf is None:
                    return None
            else:
                try:
                    buf, _ = self._sock.recvfrom(IMUD_PACKET_SIZE + 1)
                except socket.timeout:
                    return None
            pkt = _parse(buf)
            if pkt is not None:
                return pkt

    def __iter__(self) -> Iterator[ImudPacket]:
        """Iterate over received packets forever (or until timeout returns None)."""
        while True:
            pkt = self.recv()
            if pkt is None:
                return
            yield pkt

    def __enter__(self) -> 'ImudClient':
        return self.open()

    def __exit__(self, *_):
        self.close()


# ── Minimal CLI ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    import argparse

    parser = argparse.ArgumentParser(
        description='Receive and display imud binary stream')
    parser.add_argument('--port', type=int, default=10111)
    parser.add_argument('--addr', default='',
                        help='Multicast group or empty for unicast/broadcast')
    parser.add_argument('--tcp', default='', metavar='HOST[:PORT]',
                        help="Connect to the daemon's [stream] TCP listener "
                             "instead of receiving UDP (default port 10112)")
    args = parser.parse_args()

    if args.tcp:
        host, _, portstr = args.tcp.partition(':')
        tcp_port = int(portstr) if portstr else 10112
        print(f"Connecting to {host}:{tcp_port} (TCP stream)", file=sys.stderr)
        client_ctx = ImudClient.connect_tcp(host, tcp_port)
    else:
        print(f"Listening on UDP port {args.port} "
              f"{'(multicast ' + args.addr + ')' if args.addr else '(unicast/broadcast)'}",
              file=sys.stderr)
        client_ctx = ImudClient(port=args.port, addr=args.addr or None)

    with client_ctx as client:
        for pkt in client:
            print(pkt)
