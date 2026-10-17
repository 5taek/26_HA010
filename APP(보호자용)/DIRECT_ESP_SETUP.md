# ESP32 direct Firebase setup

The production paths are:

`ESP32 -> HTTPS Cloud Function -> Firestore + FCM -> Care app`

`Raspberry Pi GPSD -> Firestore device location -> Care app`

`ESP32 -> USB Serial/JTAG fall signals -> Raspberry Pi speaker controller`

The ESP never contains a Firebase service-account key. A per-device token
authenticates the HTTPS request, while the Cloud Function performs privileged
Firestore writes and sends push notifications.

## SDK and dependency versions

Use ESP-IDF **5.5.4** for `FD_KWS`, `FD_KWS_GPS`, `Fall_detection`, and
`Magic_Wand`. This version is recorded in all four dependency locks. Historical
build logs from the original model application do not verify the current
source. For the 2026-10-13 alignment build results, see
[FIRMWARE_ALIGNMENT.md](../FIRMWARE_ALIGNMENT.md). No board flash is performed
by the documentation or build verification workflow. `Magic_Wand` was verified in data collection mode only.
Other IDF versions have not been validated; the manifests' `>=4.1.0`
declaration does not establish compatibility with IDF 4.x. In particular,
`esp-tflite-micro` 1.3.5 requires IDF >=5.0, and the current fall firmwares
also use newer I2C driver APIs and `esp_driver_*` components.

Keep each project's `dependencies.lock` when reproducing the environment:

| Component | Locked version | Projects |
| --- | --- | --- |
| `espressif/esp-tflite-micro` | 1.3.5 | All four |
| `espressif/esp-nn` | 1.2.3 | All four |
| `espressif/esp-dsp` | 1.8.2 | FD_KWS, FD_KWS_GPS |

Some manifest dependencies still use `'*'`. Regenerating the locks may select
newer versions and requires fresh validation. This documentation update does
not change manifests or lock files.

The Care app's current lock requires **Dart >=3.12.0 <4.0.0** and
**Flutter >=3.44.0**. Analysis was verified with Flutter **3.47.2**.
See [the app SDK instructions](app/README.md#sdk-requirements).

## 1. Register the function secret

Choose a long random token for each device. The same token is configured on the
ESP in step 3.

```powershell
cd Care_app
firebase functions:secrets:set DEVICE_TOKENS
```

Enter JSON when prompted:

```json
{"cane-001":"replace-with-at-least-32-random-characters"}
```

## 2. Deploy

```powershell
cd Care_app\functions
npm install
cd ..
firebase deploy --only functions:reportFall
```

Copy the deployed `reportFall` URL. The function is deployed in
`asia-northeast3`.

The Firestore document `devices/cane-001` must already contain a `patientId`
and a non-empty `guardianUids` array. The Care app registration flow creates
that link.

## 3. Configure and build the ESP

```powershell
cd Fall_detection
idf.py menuconfig
```

Under **Fall detection Firebase connection**, set:

- Wi-Fi SSID and password
- deployed `reportFall` URL
- device ID, for example `cane-001`
- the matching device token

Then build the inference firmware:

```powershell
idf.py -p COM6 -DFALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=OFF -DFALL_DETECTION_TEST_MODE=OFF reconfigure build flash
```

Run this command from `Fall_detection` for the standalone firmware. For
restored `FD_KWS`, use a fresh `-B build_fin_current` directory; for preserved
`FD_KWS_GPS`, use `-B build_gps_current`. Their retained build caches may refer
to older source/model versions or paths. Replace `COM6` with the actual port.
These commands are application instructions, not evidence of a completed flash.

All three fall firmware projects default to `FALL_DETECTION_EDGE_IMPULSE_DATA_FORWARDER=OFF`
(fall inference) and `FALL_DETECTION_TEST_MODE=OFF` (normal operation).
The explicit flags above also override values retained in an existing CMake
build cache. Set the data forwarder option to `ON` only when collecting raw
IMU samples for Edge Impulse. For diagnostic logs, set test mode to `ON` and
add `monitor`; normal operation leaves the USB connection available to the
Raspberry Pi controller.

## Event contract

- `safe_fall`: an event was detected, but pickup motion occurred within 10 s
- `danger_fall`: no pickup motion occurred within 10 s

Pickup is an accelerometer movement rule; it does not directly establish
that the user stood up, is uninjured, or is conscious. `FD_KWS` currently sends
only `safe_fall`/`danger_fall`; the server also accepts `normal` for compatible
clients.

`FD_KWS` and the aligned `Fall_detection` use a four-entry RAM queue and at
most three HTTP attempts per report,
without `eventId`/`eventSequence`. It has no persistent event recovery or
ID-based deduplication. Only `FD_KWS_GPS` uses the NVS outbox, stable IDs
and sequence numbers. All three current firmwares have no stationary event gate. See [delivery scope](EVENT_DELIVERY.md).

Only `danger_fall` sends FCM. Both states are written to the global event,
guardian event copies, and the device's latest status.

## Raspberry Pi GPS and speaker control

The Raspberry Pi publishes GPS locations and handles local speaker control.
Both inference firmwares send these raw bytes over USB Serial/JTAG:

| Byte | Meaning | Intended speaker action |
| --- | --- | --- |
| `0xFF` | Fall event detected | Start the local fall warning |
| `0xFE` | Pickup motion detected within the waiting period | Stop the warning |
| `0xFD` | No pickup motion detected within 10 s | Play the danger warning |

The Raspberry Pi speaker controller must interpret these as raw bytes.
In `FD_KWS`, they share the USB connection with the KWS protocol. The GPS
publisher below is a separate process and does not handle speaker signals.
Guardian notifications are sent directly through the ESP HTTPS report and
Cloud Function; the Raspberry Pi handles the local audio response.

```powershell
cd Care_app\raspberry_pi
pip install -r requirements.txt
python gps_location_publisher.py
```

Configure `FIREBASE_SERVICE_ACCOUNT`, `DEVICE_ID`, and optionally
`GPSD_HOST`, `GPSD_PORT`, and `GPS_PUBLISH_INTERVAL_SECONDS`. The publisher
updates `devices/{deviceId}.latestLocation`. When a fall report arrives, the
Cloud Function copies the stored latest Raspberry Pi location into the fall
event. It may be missing or old and is not a reconstruction of the position
at the original fall time. `deviceTimeMs` is uptime, not a wall-clock timestamp.

Firestore write failures are logged and retried with delays from 1 s up to
30 s; each write has a 10 s timeout. Each attempt reads a fresh GPS fix.
Missing fixes leave the last stored location unchanged. `sampledAt` records
when the fix was read, separately from the server write time. The app warns
when the displayed location is more than 2 minutes old or its time is unknown.
Permission and credential errors still require correcting the configuration.

To apply this change, copy the updated `gps_location_publisher.py` to the
Raspberry Pi and restart its publisher process or service. Rebuild and install
the app to enable the stale-location warning. No ESP upload or Cloud Function
deployment is required for this change.
