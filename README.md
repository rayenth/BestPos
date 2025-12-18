IoT GPS Tracking Project

This project collects GPS location data from a mobile phone, sends it through MQTT, stores it in InfluxDB, and visualizes it using Grafana.

The goal is to have near real-time position tracking (latitude, longitude, altitude) on a map, with improved accuracy through sensor fusion.

System Overview

Data flow:

Phone (GPS / OwnTracks)
→ MQTT Broker
→ ESP32 (Kalman filter processing)
→ Telegraf
→ InfluxDB
→ Grafana

Position Estimation Logic

The phone provides raw GPS position data.
An ESP32 processes this data and applies a Kalman filter to improve position estimation by fusing GPS measurements with information from nearby Wi-Fi signals. This allows smoother and more accurate position tracking, especially between GPS updates or in environments where GPS accuracy is degraded.

The filtered position data is then published to the MQTT broker.

Components

Mobile Phone (OwnTracks)
Sends raw GPS data.

ESP32
Performs position estimation using a Kalman filter and Wi-Fi data.

MQTT Broker
Transports position messages.

Telegraf (Docker)
Subscribes to MQTT topics and writes data to InfluxDB.

InfluxDB 2.x
Time-series database storing GPS positions.

Grafana
Visualizes the position data on a map.

Data Format
Measurement
gps_position

Fields

lat : latitude

lon : longitude

alt : altitude

Tags

id : device or person identifier

topic : MQTT topic

MQTT

Example topic:

owntracks/user/Phone


Example payload:

{
  "lat": 35.8067398,
  "lon": 10.6074391,
  "alt": 100
}

Telegraf

Runs in a Docker container

Subscribes to OwnTracks MQTT topics

Parses JSON payloads

Writes GPS fields to InfluxDB

InfluxDB

Bucket: positions

Query language: Flux

Data can be verified using the InfluxDB Web UI (Data Explorer)

Example check query:

from(bucket: "positions")
  |> range(start: -5m)
  |> filter(fn: (r) => r._measurement == "gps_position")
  |> limit(n: 20)

Grafana
Data Source

Type: InfluxDB

Query language: Flux

URL: http://influxdb:8086

Authentication: InfluxDB token

Geomap Query Example
from(bucket: "positions")
  |> range(start: v.timeRangeStart, stop: v.timeRangeStop)
  |> filter(fn: (r) => r._measurement == "gps_position")
  |> filter(fn: (r) =>
    r._field == "lat" or
    r._field == "lon" or
    r._field == "alt"
  )
  |> pivot(
       rowKey: ["_time"],
       columnKey: ["_field"],
       valueColumn: "_value"
     )
  |> keep(columns: ["_time", "lat", "lon", "alt"])

Notes

InfluxDB 2.x uses Flux (not InfluxQL)

Latitude and longitude must be pivoted into columns for Grafana Geomap

Docker networking must use service names instead of localhost

Status

Phone → MQTT: OK

ESP32 processing: OK

MQTT → Telegraf: OK

Telegraf → InfluxDB: OK

InfluxDB → Grafana: OK

Real-time position data is visible on the map.