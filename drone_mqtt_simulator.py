#!/usr/bin/env python3
"""
Simulate drone data and publish to MQTT.

Modes:
- frames (default): one MQTT message per video frame with many drones inside
- detections: legacy single-drone circular motion (original behavior)

Examples:
    # Frames mode (recommended)
    python drone_mqtt_simulator.py \
        --mode frames --host localhost --topic drones/frames \
        --center-lat 13.7563 --center-lon 100.5018 \
        --num-drones 1 --interval-s 0.5 --radius-m 120

    # Legacy detections mode (original)
    python drone_mqtt_simulator.py \
        --mode detections --host localhost --topic drones/detections \
        --center-lat 13.7563 --center-lon 100.5018 \
        --radius-m 120 --speed-mps 8 --interval-s 0.5
"""

import argparse
import json
import math
import random
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List, Tuple

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:  # pragma: no cover - guides user to install dependency
    print("paho-mqtt is required. Install with `pip install paho-mqtt`.", file=sys.stderr)
    raise


METERS_PER_DEGREE_LAT = 111_320.0

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
VIEW_HALF_WIDTH_M = 600.0  # how many meters from center map to screen edge (internal only)


def meters_per_degree_lon(latitude_deg: float) -> float:
    """Approximate meters per degree of longitude at a given latitude."""
    cos_lat = max(1e-12, abs(math.cos(math.radians(latitude_deg))))
    return METERS_PER_DEGREE_LAT * cos_lat


def position_on_circle(
    center_lat: float, center_lon: float, radius_m: float, angle_rad: float
) -> Tuple[float, float]:
    """
    Compute latitude/longitude for a point on a circle around a center.
    Angle is measured from east and increases counter-clockwise.
    """
    delta_lat = (radius_m * math.sin(angle_rad)) / METERS_PER_DEGREE_LAT
    delta_lon = (radius_m * math.cos(angle_rad)) / meters_per_degree_lon(center_lat)
    return center_lat + delta_lat, center_lon + delta_lon


def latlon_to_m_offsets(
    lat: float, lon: float, center_lat: float, center_lon: float
) -> Tuple[float, float]:
    """Return (dx_east_m, dy_north_m) from center."""
    dy_north_m = (lat - center_lat) * METERS_PER_DEGREE_LAT
    dx_east_m = (lon - center_lon) * meters_per_degree_lon(center_lat)
    return dx_east_m, dy_north_m


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def compute_bbox_and_conf(
    dx_east_m: float, dy_north_m: float, current_speed_mps: float
) -> Tuple[Tuple[int, int, int, int], float]:
    """
    Compute a plausible bbox and confidence based on distance and speed.
    Returns ((x, y, w, h), confidence).
    """
    # Project meters to pixels (simple linear screen model)
    px_per_m_x = (IMAGE_WIDTH / 2) / VIEW_HALF_WIDTH_M
    px_per_m_y = (IMAGE_HEIGHT / 2) / VIEW_HALF_WIDTH_M

    x_center = (IMAGE_WIDTH / 2) + dx_east_m * px_per_m_x + random.gauss(0.0, 5.0)
    y_center = (IMAGE_HEIGHT / 2) - dy_north_m * px_per_m_y + random.gauss(0.0, 5.0)

    distance_m = math.hypot(dx_east_m, dy_north_m)

    # Size shrinks with distance, plus noise
    width_px = 12_000.0 / (distance_m + 50.0) + random.gauss(0.0, 5.0)
    width_px = clamp(width_px, 12.0, 240.0)
    height_px = width_px * 0.66

    # Convert center->top-left, clamp to screen
    x = int(clamp(x_center - width_px / 2.0, 0.0, IMAGE_WIDTH - width_px))
    y = int(clamp(y_center - height_px / 2.0, 0.0, IMAGE_HEIGHT - height_px))
    w = int(min(width_px, IMAGE_WIDTH - x))
    h = int(min(height_px, IMAGE_HEIGHT - y))

    # Confidence model: base - size penalties - speed penalty + jitter
    base = 0.85
    size_penalty = 0.0
    if w < 30:
        size_penalty += 0.15
    if w > 180:
        size_penalty += 0.07
    speed_penalty = clamp((current_speed_mps - 6.0) * 0.02, 0.0, 0.15)
    jitter = random.uniform(-0.05, 0.05)
    confidence = clamp(base - size_penalty - speed_penalty + jitter, 0.30, 0.98)

    return (x, y, w, h), round(confidence, 2)


@dataclass
class DroneState:
    drone_id: str
    type: str
    motion: str  # "circle" | "straight"
    angle_rad: float  # used for circle
    bearing_rad: float  # used for straight
    radius_m: float
    speed_base_mps: float
    lat: float  # only used/updated for straight
    lon: float  # only used/updated for straight
    base_alt_m: float
    wobble_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish simulated drone data to MQTT.")

    # Connection
    parser.add_argument("--host", default="localhost", help="MQTT broker hostname or IP.")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port.")
    parser.add_argument("--qos", type=int, choices=[0, 1, 2], default=0, help="MQTT QoS level.")
    parser.add_argument("--retain", action="store_true", help="Publish messages with the retain flag.")
    parser.add_argument("--username", help="MQTT username, if authentication is required.")
    parser.add_argument("--password", help="MQTT password, if authentication is required.")
    parser.add_argument("--client-id", help="MQTT client ID. Defaults to a generated value.")

    # Mode + topics
    parser.add_argument("--mode", choices=["frames", "detections"], default="frames", help="Publish mode.")
    parser.add_argument("--topic", default="drones/frames", help="MQTT topic to publish to.")

    # Scene
    parser.add_argument("--center-lat", type=float, required=True, help="Latitude of scene center.")
    parser.add_argument("--center-lon", type=float, required=True, help="Longitude of scene center.")
    parser.add_argument("--interval-s", type=float, default=0.5, help="Seconds between published updates.")
    parser.add_argument("--updates", type=int, default=0, help="Total updates to send (0 = run continuously).")

    # Shared motion params
    parser.add_argument("--radius-m", type=float, default=120.0, help="Base orbit radius for circle motion.")
    parser.add_argument("--altitude-m", type=float, default=120.0, help="Base altitude in meters.")
    parser.add_argument("--altitude-wobble-m", type=float, default=0.0, help="Altitude variation amplitude in meters.")

    # Frames mode params
    parser.add_argument("--num-drones", type=int, default=1, help="How many drones per frame.")
    parser.add_argument(
        "--speed-range-mps",
        type=float,
        nargs=2,
        metavar=("MIN", "MAX"),
        default=[3.0, 12.0],
        help="Speed range for drones in m/s (min max).",
    )
    parser.add_argument("--noise-level-m", type=float, default=3.0, help="GPS jitter standard deviation in meters.")
    parser.add_argument("--miss-rate", type=float, default=0.10, help="Probability per frame to miss a real drone.")
    parser.add_argument(
        "--false-positive-rate",
        type=float,
        default=0.03,
        help="Probability per frame to add a false detection.",
    )
    parser.add_argument("--source-id", default="camera-1", help="Source/camera identifier.")
    parser.add_argument("--image-string", default="450697839702995473577", help="Fixed fake image base64 string.")

    # Legacy detections mode params
    parser.add_argument("--drone-id", default="simulated-drone-1", help="Identifier for the single simulated drone.")
    parser.add_argument("--speed-mps", type=float, default=5.0, help="Ground speed (detections mode).")
    parser.add_argument("--start-angle-deg", type=float, default=0.0, help="Starting angle on the orbit (0° = east).")

    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> bool:
    if args.interval_s <= 0:
        print("Update interval must be greater than zero.", file=sys.stderr)
        return False
    if args.radius_m <= 0:
        print("Orbit radius must be greater than zero.", file=sys.stderr)
        return False

    if args.mode == "detections":
        if args.speed_mps <= 0:
            print("Speed must be greater than zero.", file=sys.stderr)
            return False
    else:
        # frames mode
        if args.num_drones < 1:
            print("num-drones must be >= 1.", file=sys.stderr)
            return False
        if len(args.speed_range_mps) != 2 or args.speed_range_mps[0] <= 0 or args.speed_range_mps[0] > args.speed_range_mps[1]:
            print("speed-range-mps must be two numbers: MIN > 0 and MIN <= MAX.", file=sys.stderr)
            return False
        if not (0.0 <= args.miss_rate < 1.0):
            print("miss-rate must be in [0, 1).", file=sys.stderr)
            return False
        if not (0.0 <= args.false_positive_rate < 1.0):
            print("false-positive-rate must be in [0, 1).", file=sys.stderr)
            return False
    return True


def init_frames_states(args: argparse.Namespace) -> List[DroneState]:
    states: List[DroneState] = []
    speed_min, speed_max = args.speed_range_mps

    for i in range(args.num_drones):
        drone_id = f"sim-{i + 1}"
        motion = "circle" if random.random() < 0.6 else "straight"
        speed_base = random.uniform(speed_min, speed_max)
        wobble = args.altitude_wobble_m
        typ = "unknown"

        # random small start offset near center (0..25% of radius)
        start_r = random.uniform(0.0, args.radius_m * 0.25)
        start_theta = random.uniform(0.0, 2 * math.pi)
        offset_lat = (start_r * math.sin(start_theta)) / METERS_PER_DEGREE_LAT
        offset_lon = (start_r * math.cos(start_theta)) / meters_per_degree_lon(args.center_lat)
        start_lat = args.center_lat + offset_lat
        start_lon = args.center_lon + offset_lon

        if motion == "circle":
            angle_rad = random.uniform(0.0, 2 * math.pi)
            bearing_rad = 0.0
            radius = random.uniform(args.radius_m * 0.7, args.radius_m * 1.3)
        else:
            angle_rad = 0.0
            bearing_rad = random.uniform(0.0, 2 * math.pi)
            radius = args.radius_m

        states.append(
            DroneState(
                drone_id=drone_id,
                type=typ,
                motion=motion,
                angle_rad=angle_rad,
                bearing_rad=bearing_rad,
                radius_m=radius,
                speed_base_mps=speed_base,
                lat=start_lat,
                lon=start_lon,
                base_alt_m=args.altitude_m,
                wobble_m=wobble,
            )
        )
    return states


def frames_loop(client: mqtt.Client, args: argparse.Namespace) -> int:
    states = init_frames_states(args)
    frame_id = 0
    dt = args.interval_s
    updates_remaining = args.updates if args.updates > 0 else None

    client.loop_start()
    try:
        while updates_remaining is None or updates_remaining > 0:
            objects = []
            now_iso = datetime.now(timezone.utc).isoformat()

            for st in states:
                # base speed with small per-frame noise
                current_speed = st.speed_base_mps * random.uniform(0.9, 1.1)

                # update position
                if st.motion == "circle":
                    st.angle_rad = (st.angle_rad + (current_speed / st.radius_m) * dt) % (2 * math.pi)
                    lat, lon = position_on_circle(args.center_lat, args.center_lon, st.radius_m, st.angle_rad)
                else:
                    delta_north_m = current_speed * dt * math.cos(st.bearing_rad)
                    delta_east_m = current_speed * dt * math.sin(st.bearing_rad)
                    st.lat = st.lat + (delta_north_m / METERS_PER_DEGREE_LAT)
                    st.lon = st.lon + (delta_east_m / meters_per_degree_lon(st.lat))
                    lat, lon = st.lat, st.lon

                # GPS noise (meters -> degrees)
                if args.noise_level_m > 0.0:
                    noise_north_m = random.gauss(0.0, args.noise_level_m)
                    noise_east_m = random.gauss(0.0, args.noise_level_m)
                    lat += noise_north_m / METERS_PER_DEGREE_LAT
                    lon += noise_east_m / meters_per_degree_lon(lat)

                # altitude wobble
                t = time.time()
                wobble_phase = st.angle_rad if st.motion == "circle" else t
                alt = st.base_alt_m + st.wobble_m * math.sin(wobble_phase)

                # bbox + confidence from distance and speed
                dx_east_m, dy_north_m = latlon_to_m_offsets(lat, lon, args.center_lat, args.center_lon)
                bbox, confidence = compute_bbox_and_conf(dx_east_m, dy_north_m, current_speed)

                # missed detection?
                if random.random() < args.miss_rate:
                    pass  # skip this object this frame
                else:
                    objects.append(
                        {
                            "drone_id": st.drone_id,
                            "type": st.type,
                            "lat": round(lat, 7),
                            "lon": round(lon, 7),
                            "alt_m": round(alt, 2),
                            "speed_mps": round(current_speed, 2),
                            "bbox": [bbox[0], bbox[1], bbox[2], bbox[3]],
                            "confidence": confidence,
                            "timestamp": now_iso,
                        }
                    )

            # false positive(s)
            if random.random() < args.false_positive_rate:
                # put a low-confidence random box near the scene with unknown location
                dx = random.uniform(-VIEW_HALF_WIDTH_M, VIEW_HALF_WIDTH_M)
                dy = random.uniform(-VIEW_HALF_WIDTH_M, VIEW_HALF_WIDTH_M)
                lat_fp = args.center_lat + (dy / METERS_PER_DEGREE_LAT)
                lon_fp = args.center_lon + (dx / meters_per_degree_lon(args.center_lat))
                bbox_fp, conf_fp = compute_bbox_and_conf(dx, dy, current_speed_mps=random.uniform(0.0, 2.0))
                objects.append(
                    {
                        "drone_id": f"fp-{uuid.uuid4().hex[:6]}",
                        "type": "unknown",
                        "lat": round(lat_fp, 7),
                        "lon": round(lon_fp, 7),
                        "alt_m": round(args.altitude_m + random.uniform(-5.0, 5.0), 2),
                        "speed_mps": round(random.uniform(0.0, 2.0), 2),
                        "bbox": [bbox_fp[0], bbox_fp[1], bbox_fp[2], bbox_fp[3]],
                        "confidence": round(clamp(conf_fp, 0.30, 0.50), 2),
                        "timestamp": now_iso,
                    }
                )

            payload = {
                "frame_id": frame_id,
                "timestamp": now_iso,
                "source_id": args.source_id,
                "objects": objects,
                "image_base64": args.image_string,  # fixed fake string as requested
            }

            info = client.publish(args.topic, json.dumps(payload), qos=args.qos, retain=args.retain)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"Publish failed with code {info.rc}", file=sys.stderr)

            frame_id += 1
            if updates_remaining:
                updates_remaining -= 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


def detections_loop(client: mqtt.Client, args: argparse.Namespace) -> int:
    """
    Legacy single-drone circular detections, kept for backward compatibility.
    Publishes to args.topic with fields:
    {drone_id, timestamp, latitude, longitude, altitude_m, speed_mps, radius_m, angle_deg}
    """
    angle = math.radians(args.start_angle_deg) if hasattr(args, "start_angle_deg") else 0.0
    angular_velocity = args.speed_mps / args.radius_m  # radians per second
    updates_remaining = args.updates if args.updates > 0 else None

    client.loop_start()
    try:
        while updates_remaining is None or updates_remaining > 0:
            latitude, longitude = position_on_circle(args.center_lat, args.center_lon, args.radius_m, angle)
            altitude = args.altitude_m + args.altitude_wobble_m * math.sin(angle)

            payload = {
                "drone_id": args.drone_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "latitude": round(latitude, 7),
                "longitude": round(longitude, 7),
                "altitude_m": round(altitude, 2),
                "speed_mps": args.speed_mps,
                "radius_m": args.radius_m,
                "angle_deg": round(math.degrees(angle) % 360.0, 2),
            }

            info = client.publish(args.topic, json.dumps(payload), qos=args.qos, retain=args.retain)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"Publish failed with code {info.rc}", file=sys.stderr)

            angle = (angle + angular_velocity * args.interval_s) % (2 * math.pi)
            if updates_remaining:
                updates_remaining -= 1
            time.sleep(args.interval_s)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        client.loop_stop()
        client.disconnect()
    return 0


def main() -> int:
    args = parse_args()
    if not validate_args(args):
        return 2

    client_id = args.client_id or f"drone-sim-{uuid.uuid4().hex[:10]}"
    client = mqtt.Client(client_id=client_id)
    if args.username:
        client.username_pw_set(args.username, password=args.password)

    try:
        client.connect(args.host, args.port, keepalive=60)
    except OSError as exc:
        print(f"Failed to connect to MQTT broker at {args.host}:{args.port} ({exc}).", file=sys.stderr)
        return 1

    if args.mode == "frames":
        return frames_loop(client, args)
    else:
        # in detections mode, ensure topic default matches legacy behavior if user did not change it
        if args.topic == "drones/frames":
            args.topic = "drones/detections"
        return detections_loop(client, args)


if __name__ == "__main__":
    sys.exit(main())
