from __future__ import annotations

import json
import os
import socket
import time
from datetime import datetime, timezone
from typing import Any

from dotenv import load_dotenv
import firebase_admin
from firebase_admin import credentials, firestore
from google.auth.exceptions import GoogleAuthError
from google.api_core.exceptions import GoogleAPICallError, RetryError


load_dotenv()


def read_gpsd_location(host: str, port: int, timeout: float) -> dict[str, Any] | None:
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(b'?WATCH={"enable":true,"json":true};\n')
            deadline = time.monotonic() + timeout
            buffer = b''
            while time.monotonic() < deadline:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buffer += chunk
                lines = buffer.split(b'\n')
                buffer = lines.pop()
                for line in lines:
                    try:
                        report = json.loads(line.decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if report.get('class') != 'TPV':
                        continue
                    latitude = report.get('lat')
                    longitude = report.get('lon')
                    if isinstance(latitude, (int, float)) and isinstance(
                        longitude, (int, float)
                    ):
                        return {
                            'latitude': float(latitude),
                            'longitude': float(longitude),
                            'altitude': report.get('alt'),
                            'accuracy': report.get('eph'),
                            'source': 'raspberry_pi_gpsd',
                            'gpsTime': report.get('time'),
                            'sampledAt': datetime.now(timezone.utc),
                            'updatedAt': firestore.SERVER_TIMESTAMP,
                        }
    except OSError as error:
        print(f'GPSD read failed: {error}', flush=True)
    return None


def publish_location(device_ref: Any, location: dict[str, Any]) -> bool:
    try:
        device_ref.set(
            {
                'latestLocation': location,
                'latestLocationSource': 'raspberry_pi_gpsd',
                'lastLocationAt': firestore.SERVER_TIMESTAMP,
            },
            merge=True,
            retry=None,
            timeout=10,
        )
    except (GoogleAPICallError, RetryError, GoogleAuthError, OSError) as error:
        print(f'Firestore GPS publish failed; will retry: {error}', flush=True)
        return False
    print(
        f"GPS published: {location['latitude']}, {location['longitude']}",
        flush=True,
    )
    return True


def run_publisher(device_ref: Any, host: str, port: int,
                  timeout: float, interval: float) -> None:
    retry_delay = 1.0
    while True:
        # Read a fresh fix on every attempt. Never re-stamp an old location
        # as current when the connection recovers.
        location = read_gpsd_location(host, port, timeout)
        if location is not None and not publish_location(device_ref, location):
            time.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, 30.0)
        else:
            if location is not None:
                retry_delay = 1.0
            time.sleep(interval)


def main() -> None:
    service_account = os.getenv('FIREBASE_SERVICE_ACCOUNT')
    device_id = os.getenv('DEVICE_ID', 'cane-001')
    gpsd_host = os.getenv('GPSD_HOST', '127.0.0.1')
    gpsd_port = int(os.getenv('GPSD_PORT', '2947'))
    gps_timeout = float(os.getenv('GPS_TIMEOUT_SECONDS', '2.0'))
    publish_interval = float(os.getenv('GPS_PUBLISH_INTERVAL_SECONDS', '10'))
    if gps_timeout <= 0 or publish_interval <= 0:
        raise ValueError('GPS timeout and publish interval must be positive.')

    if not service_account:
        raise RuntimeError('FIREBASE_SERVICE_ACCOUNT is required.')
    if not firebase_admin._apps:
        firebase_admin.initialize_app(credentials.Certificate(service_account))
    db = firestore.client()
    device_ref = db.collection('devices').document(device_id)

    print(f'Publishing GPSD location for devices/{device_id}', flush=True)
    run_publisher(device_ref, gpsd_host, gpsd_port, gps_timeout, publish_interval)


if __name__ == '__main__':
    main()
