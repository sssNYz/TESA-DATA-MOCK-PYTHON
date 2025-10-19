# TESA-DATA-MOCK-PYTHON

A simple drone simulator that sends fake drone data over MQTT.

## What is this?

This tool makes fake drone data. It sends messages about drones flying around. You can use it to test your drone tracking system.

## How to install

1. Install Python 3.7 or newer
2. Install required packages:
```bash
pip install paho-mqtt
```

## How to use

### Basic usage

Run the simulator with default settings:
```bash
python3 drone_mqtt_simulator.py
```

### Two modes

**1. Frames mode (recommended)**
- Makes many drones at once
- Good for testing camera systems
- Default mode

**2. Detections mode**
- Makes one drone flying in a circle
- Good for simple testing

### Common commands

**Start with 5 drones, no fake ones:**
```bash
python3 drone_mqtt_simulator.py --mode frames --num-drones 5 --false-positive-rate 0
```

**Start with 3 drones, some fake ones:**
```bash
python3 drone_mqtt_simulator.py --mode frames --num-drones 3 --false-positive-rate 0.1
```

**Change drone speed:**
```bash
python3 drone_mqtt_simulator.py --speed-range-mps 2 8
```

**Change location (Bangkok area):**
```bash
python3 drone_mqtt_simulator.py --center-lat 13.7563 --center-lon 100.5018
```

## What you will see

The simulator sends JSON messages like this:

```json
{
  "frame_id": 0,
  "timestamp": "2025-10-19T13:19:45.619Z",
  "source_id": "camera-1",
  "objects": [
    {
      "type": "drone",
      "drone_id": "sim-1",
      "timestamp": "2025-10-19T13:19:45.619Z",
      "latitude": 13.7567036,
      "longitude": 100.5054583,
      "altitude_m": 120,
      "speed_mps": 6.47,
      "source_id": "camera-1",
      "confidence": 0.84,
      "bbox": [1573, 486, 32, 21]
    }
  ]
}
```

## Understanding the data

- **drone_id**: Name of drone
  - `sim-1`, `sim-2`, etc. = Real simulated drones
  - `fp-xxxxxx` = Fake drone (false positive)
- **latitude/longitude**: Where the drone is
- **altitude_m**: How high (in meters)
- **speed_mps**: How fast (meters per second)
- **confidence**: How sure the system is (0.0 to 1.0)
- **bbox**: Box on camera image [x, y, width, height]

## All options

```bash
python3 drone_mqtt_simulator.py --help
```

**Main options:**
- `--mode`: frames or detections
- `--num-drones`: How many drones (frames mode)
- `--false-positive-rate`: How often fake drones appear (0.0 to 1.0)
- `--speed-range-mps`: Min and max speed (e.g., "2 8")
- `--center-lat`: Center latitude
- `--center-lon`: Center longitude
- `--radius-m`: Flying area size (meters)
- `--altitude-m`: How high drones fly
- `--interval-s`: Time between messages (seconds)
- `--host`: MQTT server address
- `--port`: MQTT server port
- `--topic`: MQTT topic name
- `--username`: MQTT username
- `--password`: MQTT password

## Examples

**Test with 10 drones, no fake ones, fast speed:**
```bash
python3 drone_mqtt_simulator.py --mode frames --num-drones 10 --false-positive-rate 0 --speed-range-mps 5 15
```

**Test with fake drones sometimes:**
```bash
python3 drone_mqtt_simulator.py --mode frames --num-drones 5 --false-positive-rate 0.2
```

**One drone in circle (old mode):**
```bash
python3 drone_mqtt_simulator.py --mode detections --drone-id "test-drone" --speed-mps 5 --radius-m 100
```

## Troubleshooting

**Problem**: No messages received
- Check MQTT server is running
- Check network connection
- Try: `--host localhost --port 1883`

**Problem**: Too many fake drones
- Use: `--false-positive-rate 0`

**Problem**: Drones too fast/slow
- Use: `--speed-range-mps 1 5` (slow)
- Use: `--speed-range-mps 10 20` (fast)

## Notes

- Default MQTT topic: `drones/frames`
- Default sends every 1 second
- Drones fly in circles or straight lines
- Fake drones (`fp-xxxxxx`) have lower confidence
- Real drones (`sim-1`, `sim-2`, etc.) have higher confidence