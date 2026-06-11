# msquic_cellular

This repository implements GBR-CC in MsQuic/secnetperf. GBR-CC uses cellular
scheduler telemetry from the phone modem to guide QUIC uplink pacing rate and
congestion window control.

## GBR-CC Data Path

![GBR-CC data path](docs/gbr-cc/gbr_cc_data_path.svg)

### Why `cellninjia_mobile` can stream in near real time

On phones with Android Linux kernels after 5.4.61, Qualcomm DIAG is no longer
exposed as the `/dev/diag` character device. On those phones, both modem message
reads and buffer-drain commands have to go through diagrouter instead of the
legacy DIAG ioctl device.

The diagrouter open path is in
`tools/cellninjia/cellninjia_mobile/cellninjia_mobile.c`:

- `check_system_version()` probes the abstract Unix socket `\0diag` with
  `socket(AF_UNIX, SOCK_SEQPACKET, 0)` and `connect()`. If that succeeds,
  `cellninjia_mobile` enters socket mode.
- `main()` opens the real diagrouter connection the same way and stores it in
  `diag_sock`. All host DIAG commands from the parser are written to this socket,
  and live modem DIAG messages are read from it.

The drain path is different for the two kernel interfaces:

- **Socket/diagrouter mode, used on newer kernels without `/dev/diag`**: the host
  parser performs the drain operation. Both `diag_get_lte_msquic.py` and
  `diag_get_5g_msquic.py` detect the `Socket mode` welcome message from
  `cellninjia_mobile`, send the socket-mode initialization commands, then start
  `drain_buffer_thread()`. That thread sends
  `DRAIN_BUFFER_COMMAND = b'\x24\x00\x00\x00\x00\x00\x00\x00'` every 100
  microseconds over the TCP bridge. `cellninjia_mobile` forwards those bytes to
  `diag_sock`, so the buffer-drain command reaches diagrouter.
- **Legacy `/dev/diag` mode, used on older kernels**: the phone bridge opens
  `/dev/diag`, switches the DIAG driver into memory-device logging mode, and
  configures the peripheral buffer. Android 10+ uses
  `struct diag_logging_mode_param_t` plus `DIAG_IOCTL_QUERY_CON_ALL`; Android 9
  and older use `struct diag_logging_mode_param_t_9` and the older remote-device
  ioctl path. After the final DIAG configuration pattern `60 00 12 6a 7e`,
  `start_drain_thread()` starts `drain_thread_func()`, which calls
  `DIAG_IOCTL_PERIPHERAL_BUF_DRAIN` every 100 microseconds.

In both modes, `cellninjia_mobile` is not reading a saved DIAG log. It opens the
live DIAG endpoint, immediately forwards each successful DIAG `read()` to the
host TCP client, and prepends an 8-byte Unix timestamp to the raw DIAG payload.
The host parser therefore sees modem telemetry as an online, low-latency stream
rather than an offline dump.

The live forwarding path is:

```text
diagrouter socket or /dev/diag
        -> read() in diag_read_thread()
        -> prepend host read timestamp
        -> write() to TCP clients on port 43555
        -> adb forward to the host parser
```

After this live DIAG stream reaches the host, the LTE or 5G parser connects to
`127.0.0.1:43555`, computes the GBR ratio from BSR demand and grant/PUSCH
allocation, and sends one little-endian `double` to MsQuic through
`/tmp/msquic_cellular_ratio.sock`.

Use one host parser based on the radio mode:

- `tools/cellninjia/diag_get_lte_msquic.py`: LTE DIAG parser.
- `tools/cellninjia/diag_get_5g_msquic.py`: 5G NR DIAG parser.

## Quick Start

### 1. Clone and initialize submodules

Fresh checkout:

```bash
git clone https://github.com/QiangWu769/msquic_cellular.git
cd msquic_cellular
git submodule update --init --recursive
```

Existing checkout:

```bash
cd /home/qwu26/msquic_cellular
git submodule sync --recursive
git submodule update --init --recursive
```

### 2. Build `secnetperf`

```bash
cd /home/qwu26/msquic_cellular
rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DQUIC_BUILD_PERF=ON \
  -DQUIC_ENHANCED_PACKET_LOGGING=ON \
  -DQUIC_TLS_LIB=openssl
cmake --build build --target secnetperf -j"$(nproc)"
```

Expected binary:

```text
/home/qwu26/msquic_cellular/build/bin/Release/secnetperf
```

### 3. Build and install `cellninjia_mobile`

```bash
cd /home/qwu26/msquic_cellular
make -C tools/cellninjia/cellninjia_mobile clean all
make -C tools/cellninjia/cellninjia_mobile push
```

The default Android NDK path in the Makefile is:

```text
/home/qwu26/android-ndk-r26d
```

Override it when needed:

```bash
make -C tools/cellninjia/cellninjia_mobile clean all ANDROID_NDK_HOME=/path/to/android-ndk
```

The push target installs the binary to:

```text
/data/local/tmp/cellninjia_mobile
```

### 4. Start the phone DIAG bridge

Keep this running while collecting cellular telemetry:

```bash
adb forward tcp:43555 tcp:43555
adb shell su -c /data/local/tmp/cellninjia_mobile
```

### 5. Start the host parser

Run exactly one parser for the current radio mode:

```bash
cd /home/qwu26/msquic_cellular
python3 tools/cellninjia/diag_get_lte_msquic.py
```

or:

```bash
cd /home/qwu26/msquic_cellular
python3 tools/cellninjia/diag_get_5g_msquic.py
```

The parser connects to `127.0.0.1:43555`, computes the ratio, and sends it to
MsQuic as one `double` on `/tmp/msquic_cellular_ratio.sock`.

### 6. Run GBR-CC in `secnetperf`

A minimal upload run:

```bash
cd /home/qwu26/msquic_cellular
./build/bin/Release/secnetperf \
  -target:<server-ip> \
  -port:4433 \
  -cc:bbr \
  -upload:20mb \
  -ptput:1 \
  -cellular:1
```

`-cc:bbr` selects the MsQuic congestion-control path where GBR-CC is integrated.
`-cellular:1` enables ratio-driven pacing and cwnd control.

### 7. Run fairness experiments

```bash
cd /home/qwu26/msquic_cellular
./scripts/fairness_gbr_vs_bbr.sh 20mb bbr
```

## Runtime Modes and Signal Format

- `-cellular:0`: receive and log GBR ratio samples without applying control.
- `-cellular:1`: apply GBR ratio to pacing rate and congestion window control.
- Socket path: `/tmp/msquic_cellular_ratio.sock`.
- Packet format: one little-endian `double`, containing only `ratio`.

## Main Files

- `src/perf/lib/cellular_ratio.c`: Unix datagram receiver for ratio samples.
- `src/perf/lib/cellular_ratio.h`: receiver API and shared stats.
- `src/core/bbr.c`: MsQuic congestion-control hook where GBR-CC control is applied.
- `src/perf/lib/SecNetPerfMain.cpp`: `secnetperf` integration and `-cellular` option.
- `tools/cellninjia/diag_get_lte_msquic.py`: LTE DIAG parser for MsQuic.
- `tools/cellninjia/diag_get_5g_msquic.py`: 5G NR DIAG parser for MsQuic.
- `tools/cellninjia/cellninjia_mobile/`: phone-side DIAG bridge source.
- `scripts/fairness_gbr_vs_bbr.sh`: fairness experiment wrapper.
- `analysis/transport/`: transport-log analysis scripts.
- `docs/gbr-cc/`: GBR-CC notes and repository layout documentation.
