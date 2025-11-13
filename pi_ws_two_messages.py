#!/usr/bin/env python3
"""
Stream JPEG frames over WebSocket using a two-message protocol:
1) Frame metadata as JSON text.
2) Raw JPEG bytes.
"""
import argparse
import glob
import json
import os
import time
from datetime import datetime, timezone
from typing import List, Tuple
import math
import random
import uuid
from dataclasses import dataclass

import cv2
from websocket import ABNF, create_connection


# Configuration defaults (override via CLI flags if desired)
WS_URL = "ws://<VPS_IP_OR_DOMAIN>:3000/ws?role=pi&source_id=pi-cam-01"
SOURCE_ID = "pi-cam-01"
FPS = 10
IMAGE_DIR = "./img"
TARGET_HEIGHT = 620
JPEG_QUALITY = 20


def list_images(directory: str) -> List[str]:
    """Return sorted list of supported image file paths in the directory."""
    supported = (".jpg", ".jpeg", ".png")
    pattern = os.path.join(directory, "*")
    files = [
        path
        for path in glob.glob(pattern)
        if os.path.isfile(path) and os.path.splitext(path)[1].lower() in supported
    ]
    return sorted(files)


def load_and_resize(path: str, target_h: int, quality: int) -> Tuple[bytes, int, int]:
    """
    Load an image, resize to target height while preserving aspect ratio,
    and encode as JPEG with the given quality.
    """
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to load image: {path}")

    h, w = image.shape[:2]
    if h != target_h:
        scale = target_h / float(h)
        new_w = max(1, int(round(w * scale)))
        image = cv2.resize(image, (new_w, target_h), interpolation=cv2.INTER_AREA)
        w = new_w
        h = target_h

    success, encoded = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not success:
        raise RuntimeError(f"JPEG encoding failed for: {path}")

    return encoded.tobytes(), w, h


def utc_iso_now() -> str:
    """Return current UTC time in ISO8601 format with Z suffix."""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def send_frame(ws, frame_id: int, jpeg_bytes: bytes, width: int, height: int, objects: List[dict]) -> None:
    """Send frame metadata followed by the binary JPEG payload."""
    meta = {
        "kind": "frame_meta",
        "frame_id": frame_id,
        "timestamp": utc_iso_now(),
        "source_id": SOURCE_ID,
        "image_info": {
            "mime": "image/jpeg",
            "width": width,
            "height": height,
            "quality": JPEG_QUALITY,
        },
        "objects": objects,
    }
    ws.send(json.dumps(meta))
    ws.send(jpeg_bytes, opcode=ABNF.OPCODE_BINARY)


# ---- Drone simulation (merged from drone_mqtt_simulator.py, adapted for WS) ----

METERS_PER_DEGREE_LAT = 111_320.0


def meters_per_degree_lon(latitude_deg: float) -> float:
    """Approximate meters per degree of longitude at a given latitude."""
    cos_lat = max(1e-12, abs(math.cos(math.radians(latitude_deg))))
    return METERS_PER_DEGREE_LAT * cos_lat


def position_on_circle(center_lat: float, center_lon: float, radius_m: float, angle_rad: float) -> Tuple[float, float]:
    """Compute lat/lon for a point on a circle around a center."""
    delta_lat = (radius_m * math.sin(angle_rad)) / METERS_PER_DEGREE_LAT
    delta_lon = (radius_m * math.cos(angle_rad)) / meters_per_degree_lon(center_lat)
    return center_lat + delta_lat, center_lon + delta_lon


def latlon_to_m_offsets(lat: float, lon: float, center_lat: float, center_lon: float) -> Tuple[float, float]:
    """Return (dx_east_m, dy_north_m) from center."""
    dy_north_m = (lat - center_lat) * METERS_PER_DEGREE_LAT
    dx_east_m = (lon - center_lon) * meters_per_degree_lon(center_lat)
    return dx_east_m, dy_north_m


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def compute_bbox_and_conf(
    dx_east_m: float,
    dy_north_m: float,
    current_speed_mps: float,
    image_width: int,
    image_height: int,
    view_half_width_m: float = 600.0,
) -> Tuple[Tuple[int, int, int, int], float]:
    """
    Compute a plausible bbox and confidence based on distance and speed, scaled to the image size.
    Returns ((x, y, w, h), confidence).
    """
    px_per_m_x = (image_width / 2.0) / view_half_width_m
    px_per_m_y = (image_height / 2.0) / view_half_width_m

    x_center = (image_width / 2.0) + dx_east_m * px_per_m_x + random.gauss(0.0, 5.0)
    y_center = (image_height / 2.0) - dy_north_m * px_per_m_y + random.gauss(0.0, 5.0)

    distance_m = math.hypot(dx_east_m, dy_north_m)

    width_px = 12_000.0 / (distance_m + 50.0) + random.gauss(0.0, 5.0)
    width_px = clamp(width_px, 12.0, max(24.0, image_width * 0.15))
    height_px = width_px * 0.66

    x = int(clamp(x_center - width_px / 2.0, 0.0, image_width - width_px))
    y = int(clamp(y_center - height_px / 2.0, 0.0, image_height - height_px))
    w = int(min(width_px, image_width - x))
    h = int(min(height_px, image_height - y))

    base = 0.85
    size_penalty = 0.0
    if w < 30:
        size_penalty += 0.15
    if w > image_width * 0.1:
        size_penalty += 0.07
    speed_penalty = clamp((current_speed_mps - 6.0) * 0.02, 0.0, 0.15)
    jitter = random.uniform(-0.05, 0.05)
    confidence = clamp(base - size_penalty - speed_penalty + jitter, 0.30, 0.98)

    return (x, y, w, h), round(confidence, 2)


@dataclass
class DroneState:
    drone_id: str
    type: str
    motion: str  # "circle" | "straight" | "path"
    angle_rad: float  # used for circle
    bearing_rad: float  # used for straight
    radius_m: float
    speed_base_mps: float
    lat: float  # current position
    lon: float  # current position
    base_alt_m: float
    wobble_m: float
    # For path mode
    start_lat: float = 0.0
    start_lon: float = 0.0
    end_lat: float = 0.0
    end_lon: float = 0.0
    going_forward: bool = True  # True = to end, False = back to start


def init_frames_states(
    num_drones: int,
    center_lat: float,
    center_lon: float,
    radius_m: float,
    altitude_m: float,
    altitude_wobble_m: float,
    speed_min: float,
    speed_max: float,
    motion_mode: str = "random",  # "random", "circle", "straight", "path"
    path_start_lat: float = 0.0,
    path_start_lon: float = 0.0,
    path_end_lat: float = 0.0,
    path_end_lon: float = 0.0,
) -> List[DroneState]:
    states: List[DroneState] = []
    for i in range(num_drones):
        drone_id = f"sim-{i + 1}"
        
        # Pick motion based on mode
        if motion_mode == "random":
            motion = "circle" if random.random() < 0.6 else "straight"
        else:
            motion = motion_mode
        
        speed_base = random.uniform(speed_min, speed_max)
        wobble = altitude_wobble_m
        typ = "unknown"

        # Initialize position based on motion type
        if motion == "path":
            # Start at path_start_lat/lon
            start_lat = path_start_lat
            start_lon = path_start_lon
            angle_rad = 0.0
            bearing_rad = 0.0
            radius = radius_m
        else:
            # Random offset near center
            start_r = random.uniform(0.0, radius_m * 0.25)
            start_theta = random.uniform(0.0, 2 * math.pi)
            offset_lat = (start_r * math.sin(start_theta)) / METERS_PER_DEGREE_LAT
            offset_lon = (start_r * math.cos(start_theta)) / meters_per_degree_lon(center_lat)
            start_lat = center_lat + offset_lat
            start_lon = center_lon + offset_lon
            
            if motion == "circle":
                angle_rad = random.uniform(0.0, 2 * math.pi)
                bearing_rad = 0.0
                radius = random.uniform(radius_m * 0.7, radius_m * 1.3)
            else:  # straight
                angle_rad = 0.0
                bearing_rad = random.uniform(0.0, 2 * math.pi)
                radius = radius_m

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
                base_alt_m=altitude_m,
                wobble_m=wobble,
                start_lat=path_start_lat,
                start_lon=path_start_lon,
                end_lat=path_end_lat,
                end_lon=path_end_lon,
                going_forward=True,
            )
        )
    return states


def generate_objects_for_frame(
    states: List[DroneState],
    dt: float,
    center_lat: float,
    center_lon: float,
    image_width: int,
    image_height: int,
    noise_level_m: float,
    miss_rate: float,
    false_positive_rate: float,
    base_altitude_m: float,
    source_id: str,
) -> List[dict]:
    """Advance states by dt and return object list for this frame."""
    objects: List[dict] = []
    now_iso = utc_iso_now()

    for st in states:
        current_speed = st.speed_base_mps * random.uniform(0.9, 1.1)

        if st.motion == "circle":
            st.angle_rad = (st.angle_rad + (current_speed / st.radius_m) * dt) % (2 * math.pi)
            lat, lon = position_on_circle(center_lat, center_lon, st.radius_m, st.angle_rad)
        elif st.motion == "path":
            # Move toward target (end if going_forward, start if going back)
            target_lat = st.end_lat if st.going_forward else st.start_lat
            target_lon = st.end_lon if st.going_forward else st.start_lon
            
            # Calculate bearing toward target
            dlat = target_lat - st.lat
            dlon = target_lon - st.lon
            
            # Convert to meters for distance check
            dlat_m = dlat * METERS_PER_DEGREE_LAT
            dlon_m = dlon * meters_per_degree_lon(st.lat)
            distance_to_target_m = math.hypot(dlat_m, dlon_m)
            
            # If close to target, switch direction
            if distance_to_target_m < current_speed * dt * 2:  # within 2 frames
                st.going_forward = not st.going_forward
                # Recalculate for new target
                target_lat = st.end_lat if st.going_forward else st.start_lat
                target_lon = st.end_lon if st.going_forward else st.start_lon
                dlat = target_lat - st.lat
                dlon = target_lon - st.lon
                dlat_m = dlat * METERS_PER_DEGREE_LAT
                dlon_m = dlon * meters_per_degree_lon(st.lat)
                distance_to_target_m = math.hypot(dlat_m, dlon_m)
            
            # Move toward target
            if distance_to_target_m > 0.1:
                # Normalize and scale by speed
                bearing_rad = math.atan2(dlon_m, dlat_m)
                delta_north_m = current_speed * dt * math.cos(bearing_rad)
                delta_east_m = current_speed * dt * math.sin(bearing_rad)
                st.lat = st.lat + (delta_north_m / METERS_PER_DEGREE_LAT)
                st.lon = st.lon + (delta_east_m / meters_per_degree_lon(st.lat))
            
            lat, lon = st.lat, st.lon
        else:  # straight
            delta_north_m = current_speed * dt * math.cos(st.bearing_rad)
            delta_east_m = current_speed * dt * math.sin(st.bearing_rad)
            st.lat = st.lat + (delta_north_m / METERS_PER_DEGREE_LAT)
            st.lon = st.lon + (delta_east_m / meters_per_degree_lon(st.lat))
            lat, lon = st.lat, st.lon

        if noise_level_m > 0.0:
            noise_north_m = random.gauss(0.0, noise_level_m)
            noise_east_m = random.gauss(0.0, noise_level_m)
            lat += noise_north_m / METERS_PER_DEGREE_LAT
            lon += noise_east_m / meters_per_degree_lon(lat)

        t = time.time()
        wobble_phase = st.angle_rad if st.motion == "circle" else t
        alt = st.base_alt_m + st.wobble_m * math.sin(wobble_phase)

        dx_east_m, dy_north_m = latlon_to_m_offsets(lat, lon, center_lat, center_lon)
        bbox, confidence = compute_bbox_and_conf(dx_east_m, dy_north_m, current_speed, image_width, image_height)

        if random.random() < miss_rate:
            pass
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

    if random.random() < false_positive_rate:
        dx = random.uniform(-600.0, 600.0)
        dy = random.uniform(-600.0, 600.0)
        lat_fp = center_lat + (dy / METERS_PER_DEGREE_LAT)
        lon_fp = center_lon + (dx / meters_per_degree_lon(center_lat))
        bbox_fp, conf_fp = compute_bbox_and_conf(dx, dy, current_speed_mps=random.uniform(0.0, 2.0), image_width=image_width, image_height=image_height)
        objects.append(
            {
                "drone_id": f"fp-{uuid.uuid4().hex[:6]}",
                "type": "unknown",
                "lat": round(lat_fp, 7),
                "lon": round(lon_fp, 7),
                "alt_m": round(base_altitude_m + random.uniform(-5.0, 5.0), 2),
                "speed_mps": round(random.uniform(0.0, 2.0), 2),
                "bbox": [bbox_fp[0], bbox_fp[1], bbox_fp[2], bbox_fp[3]],
                "confidence": round(clamp(conf_fp, 0.30, 0.50), 2),
                "timestamp": now_iso,
            }
        )

    return objects


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stream local images over WebSocket, with optional drone simulation metadata.")
    parser.add_argument("--ws-url", default=WS_URL, help="WebSocket endpoint URL.")
    parser.add_argument("--source-id", default=SOURCE_ID, help="Camera/source identifier.")
    parser.add_argument("--fps", type=float, default=FPS, help="Frames per second to send.")
    parser.add_argument("--image-dir", default=IMAGE_DIR, help="Directory of source images.")
    # Simulation flags/params
    parser.add_argument("--sim", action="store_true", help="Enable simulated drone detections in metadata.")
    parser.add_argument("--center-lat", type=float, default=13.7563, help="Latitude of scene center (for simulation).")
    parser.add_argument("--center-lon", type=float, default=100.5018, help="Longitude of scene center (for simulation).")
    parser.add_argument("--radius-m", type=float, default=120.0, help="Base orbit radius for circle motion (sim).")
    parser.add_argument("--altitude-m", type=float, default=120.0, help="Base altitude in meters (sim).")
    parser.add_argument("--altitude-wobble-m", type=float, default=0.0, help="Altitude variation amplitude in meters (sim).")
    parser.add_argument("--num-drones", type=int, default=1, help="How many drones per frame (sim).")
    parser.add_argument("--speed-range-mps", type=float, nargs=2, metavar=("MIN", "MAX"), default=[3.0, 12.0], help="Speed range for drones in m/s (sim).")
    parser.add_argument("--noise-level-m", type=float, default=3.0, help="GPS jitter standard deviation in meters (sim).")
    parser.add_argument("--miss-rate", type=float, default=0.10, help="Probability per frame to miss a real drone (sim).")
    parser.add_argument("--false-positive-rate", type=float, default=0.03, help="Probability per frame to add a false detection (sim).")
    parser.add_argument("--motion-mode", type=str, choices=["random", "circle", "straight", "path"], default="random", help="Drone motion mode (sim).")
    parser.add_argument("--path-start-lat", type=float, default=0.0, help="Path start latitude (sim, for path mode).")
    parser.add_argument("--path-start-lon", type=float, default=0.0, help="Path start longitude (sim, for path mode).")
    parser.add_argument("--path-end-lat", type=float, default=0.0, help="Path end latitude (sim, for path mode).")
    parser.add_argument("--path-end-lon", type=float, default=0.0, help="Path end longitude (sim, for path mode).")
    parser.add_argument("--verbose", action="store_true", help="Print debug logs while streaming.")
    parser.add_argument("--reconnect-delay", type=float, default=3.0, help="Seconds to wait before reconnect when WS drops.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    global SOURCE_ID
    SOURCE_ID = args.source_id

    images = list_images(args.image_dir)
    if not images:
        raise FileNotFoundError(f"No supported images found in {args.image_dir}")

    frame_id = 0
    delay = 1.0 / args.fps if args.fps > 0 else 0

    # Prepare simulation state if enabled
    sim_states: List[DroneState] = []
    if args.sim:
        speed_min, speed_max = args.speed_range_mps
        sim_states = init_frames_states(
            num_drones=args.num_drones,
            center_lat=args.center_lat,
            center_lon=args.center_lon,
            radius_m=args.radius_m,
            altitude_m=args.altitude_m,
            altitude_wobble_m=args.altitude_wobble_m,
            speed_min=speed_min,
            speed_max=speed_max,
            motion_mode=args.motion_mode,
            path_start_lat=args.path_start_lat,
            path_start_lon=args.path_start_lon,
            path_end_lat=args.path_end_lat,
            path_end_lon=args.path_end_lon,
        )

    if args.verbose:
        print(f"[pi] Preparing to connect to {args.ws_url} as source_id={SOURCE_ID}")
        print(f"[pi] Found {len(images)} images in {args.image_dir}, fps={args.fps}, delay={delay:.3f}s")
        print(f"[pi] Simulation: {'ON' if args.sim else 'OFF'}")
        if args.sim:
            print(f"[pi] Motion mode: {args.motion_mode}")
            if args.motion_mode == "path":
                print(f"[pi] Path: ({args.path_start_lat:.6f},{args.path_start_lon:.6f}) -> ({args.path_end_lat:.6f},{args.path_end_lon:.6f})")
        print(f"[pi] Reconnect delay: {args.reconnect_delay:.1f}s")

    # Run forever; reconnect if connection drops
    while True:
        ws = None
        try:
            if args.verbose:
                print("[pi] Connecting...")
            ws = create_connection(args.ws_url)
            if args.verbose:
                print("[pi] Connected")
            while True:
                for path in images:
                    jpeg_bytes, width, height = load_and_resize(path, TARGET_HEIGHT, JPEG_QUALITY)
                    objects: List[dict] = []
                    if args.sim:
                        objects = generate_objects_for_frame(
                            states=sim_states,
                            dt=delay if delay > 0 else 0.0,
                            center_lat=args.center_lat,
                            center_lon=args.center_lon,
                            image_width=width,
                            image_height=height,
                            noise_level_m=args.noise_level_m,
                            miss_rate=args.miss_rate,
                            false_positive_rate=args.false_positive_rate,
                            base_altitude_m=args.altitude_m,
                            source_id=SOURCE_ID,
                        )
                    if args.verbose:
                        print(f"[pi] frame {frame_id}: sending meta (objects={len(objects)}) width={width} height={height}")
                    send_frame(ws, frame_id, jpeg_bytes, width, height, objects)
                    if args.verbose:
                        print(f"[pi] frame {frame_id}: sent binary ({len(jpeg_bytes)} bytes)")
                    frame_id += 1
                    if delay > 0:
                        time.sleep(delay)
        except KeyboardInterrupt:
            if args.verbose:
                print("[pi] Stopped by user")
            break
        except Exception as exc:
            if args.verbose:
                print(f"[pi] Connection/send error: {exc}. Reconnecting in {args.reconnect_delay:.1f}s...")
            try:
                if ws is not None:
                    ws.close()
            except Exception:
                pass
            time.sleep(args.reconnect_delay)
            continue
        finally:
            try:
                if ws is not None:
                    ws.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()
