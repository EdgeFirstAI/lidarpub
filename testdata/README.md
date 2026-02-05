# LiDAR Test Data

PCAP captures for integration testing LiDAR drivers.

## Files

| File | Sensor | Description |
|------|--------|-------------|
| `e1r_frames.pcap` | Robosense E1R | MSOP packets (port 6699) |
| `os1_frames.pcap` | Ouster OS1 | Lidar packets (port 7502) |
| `os1_sensor_info.json` | Ouster OS1 | Sensor metadata from HTTP API |

## Capture Instructions

### Robosense E1R

```bash
sudo tcpdump -i <interface> -w testdata/e1r_frames.pcap 'udp port 6699' -c 1000
```

### Ouster OS1

```bash
# Capture packets
sudo tcpdump -i <interface> -w testdata/os1_frames.pcap 'udp port 7502' -c 200

# Save metadata
curl http://<ouster-ip>/api/v1/sensor/metadata/sensor_info > testdata/os1_sensor_info.json
```

## Running Tests

```bash
cargo test --features pcap
```

Tests will skip gracefully if PCAP files are not present.
