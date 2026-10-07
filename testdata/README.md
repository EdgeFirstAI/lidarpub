# LiDAR Test Data

PCAP captures for integration testing LiDAR drivers.

## Files

| File | Sensor | Description |
|------|--------|-------------|
| `e1r_frames.pcap` | Robosense E1R | MSOP packets (port 6699) |
| `os1_frames.pcap` | Ouster OS1-64 | Lidar packets (port 7502), 2048x10 mode |
| `os1_sensor_info.json` | Ouster OS1-64 | Sensor metadata from HTTP API (FW 2.5.3 flat format) |
| `os1_frame0.pcd`, `os1_frame3.pcd` | Ouster OS1-64 | Frames 0 and 3 (counted from the first complete frame) of `os1_frames.pcap` |
| `os1_128_rev7_frames.pcap` | Ouster OS1-128 Rev7 | Lidar packets (port 7502), FW 3.2, 1024x10, RNG15_RFL8_NIR8 |
| `os1_128_rev7_metadata.json` | Ouster OS1-128 Rev7 | Full `/api/v1/sensor/metadata` (FW 3.2 nested format), `udp_dest` cleared |

## Capture Instructions

### Robosense E1R

Robosense packets fit within standard MTU, so a simple port filter works:

```bash
sudo tcpdump -i <interface> -w testdata/e1r_frames.pcap 'udp port 6699' -c 1000
```

### Ouster OS1

Ouster sends ~4KB UDP packets which are IP-fragmented on standard MTU networks.
The PcapSource handles fragment reassembly, but **all fragments must be captured**.

Since only the first fragment contains UDP headers, use a host filter instead of port filter:

```bash
# Configure Ouster to send to this machine
curl -X POST "http://<ouster-ip>/api/v1/sensor/config" \
  -H "Content-Type: application/json" \
  -d '{"udp_dest": "<your-ip>", "lidar_mode": "2048x10", "azimuth_window": [0, 360000]}'

# Wait for sensor to reinitialize (~20 seconds)

# Capture ALL packets from the Ouster (includes all IP fragments)
sudo tcpdump -i <interface> -w testdata/os1_frames.pcap 'host <ouster-ip>' -c 1500

# Save metadata (required for driver initialization)
curl http://<ouster-ip>/api/v1/sensor/metadata/sensor_info > testdata/os1_sensor_info.json
```

**Alternative**: Enable jumbo frames (MTU 9000) on your interface to avoid fragmentation:

```bash
sudo ip link set <interface> mtu 9000
sudo tcpdump -i <interface> -w testdata/os1_frames.pcap 'udp port 7502' -c 500
```

## Running Tests

```bash
cargo test --features pcap
```

Tests will skip gracefully if PCAP files are not present.

### Regenerating the OS1 PCD files

The PCD files are produced by the driver from `os1_frames.pcap`, so regenerate them after any change to Ouster projection:

```bash
cargo test --features pcap --test ouster_xyz_test -- --ignored --nocapture
```

## Notes

- PCAP files are tracked via git-lfs (see `.gitattributes`)
- The PcapSource implementation handles IP fragment reassembly automatically
- Ouster tests may take ~25 seconds due to large packet count and reassembly
