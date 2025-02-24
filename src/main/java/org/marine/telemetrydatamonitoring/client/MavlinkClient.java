package org.marine.telemetrydatamonitoring.client;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.common.*;
import org.marine.telemetrydatamonitoring.service.TelemetryService;
import org.springframework.stereotype.Component;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Socket;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Component
public class MavlinkClient implements Runnable {
    private final TelemetryService telemetryService;
    private final String missionPlannerHost = "localhost";
    private final int missionPlannerPort = 14550; // TCP Port for Mission Planner
    private final int udpPort = 14557; // First UDP port
    private final int udpPort2 = 14558; // Second UDP port

    // Telemetry data map with all required fields
    private final Map<String, Object> telemetryData = new HashMap<>();

    // Variables for distance calculation and timing
    private Double prevLat = null, prevLon = null;
    private double totalDistance = 0.0;
    private final double homeLat = 16.7745;
    private final double homeLon = 96.1552;

    // Store flight start time in seconds (like time.time() in Python)
    private double startTimeSeconds;

    public MavlinkClient(TelemetryService telemetryService) {
        this.telemetryService = telemetryService;
        initializeTelemetryData();
    }

    private void initializeTelemetryData() {
        telemetryData.put("lat", null);
        telemetryData.put("lon", null);
        telemetryData.put("alt", null);
        telemetryData.put("dist_traveled", 0.0);
        telemetryData.put("wp_dist", null);
        telemetryData.put("dist_to_home", null);
        telemetryData.put("vertical_speed", null);
        telemetryData.put("wind_vel", null);
        telemetryData.put("airspeed", null);
        telemetryData.put("groundspeed", null);
        telemetryData.put("roll", null);
        telemetryData.put("pitch", null);
        telemetryData.put("yaw", null);
        telemetryData.put("toh", null);
        telemetryData.put("tot", null);
        telemetryData.put("time_in_air", null);
        telemetryData.put("time_in_air_min_sec", null);
        telemetryData.put("gps_hdop", null);
        telemetryData.put("battery_voltage", null);
        telemetryData.put("battery_current", null);
        telemetryData.put("ch3percent", null);
        telemetryData.put("ch3out", null);
        telemetryData.put("waypoints", new ArrayList<String>());
    }

    public void startListening() {
        new Thread(this).start();
    }

    @Override
    public void run() {
        startTimeSeconds = System.currentTimeMillis() / 1000.0;

        // Start the telemetry emitter every 1 second
        Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(
                this::emitTelemetry, 1, 1, TimeUnit.SECONDS
        );

        new Thread(this::startUdpListener).start();
        new Thread(this::startUdpListener2).start();
        new Thread(this::startTcpListener).start();
    }

    private void startUdpListener() {
        listenForUdpMessages(udpPort);
    }

    private void startUdpListener2() {
        listenForUdpMessages(udpPort2);
    }

    private void listenForUdpMessages(int port) {
        try (DatagramSocket socket = new DatagramSocket(port)) {
            logInfo("Listening for MAVLink messages on UDP port " + port);
            byte[] buffer = new byte[2048];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            while (true) {
                socket.receive(packet);
                logInfo("Received UDP packet from " + packet.getAddress() + ":" + packet.getPort());
                try (InputStream inputStream = new ByteArrayInputStream(packet.getData(), packet.getOffset(), packet.getLength())) {
                    MavlinkConnection connection = MavlinkConnection.create(inputStream, null);
                    MavlinkMessage<?> message = connection.next();
                    if (message != null) {
                        processTelemetryMessage(message);
                    }
                } catch (Exception e) {
                    logError("Error processing UDP MAVLink message: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            logError("Error in UDP Listener (port " + port + "): " + e.getMessage());
        }
    }

    private void startTcpListener() {
        try (Socket socket = new Socket(missionPlannerHost, missionPlannerPort);
             InputStream inputStream = socket.getInputStream();
             OutputStream outputStream = socket.getOutputStream()) {

            MavlinkConnection connection = MavlinkConnection.create(inputStream, outputStream);
            logInfo("Connected to TCP server at " + missionPlannerHost + ":" + missionPlannerPort);
            while (true) {
                MavlinkMessage<?> message = connection.next();
                if (message != null) {
                    processTelemetryMessage(message);
                }
            }
        } catch (Exception e) {
            logError("Error in TCP Listener: " + e.getMessage());
        }
    }

    private void processTelemetryMessage(MavlinkMessage<?> message) {
        Object payload = message.getPayload();
        double currentTimeSeconds = System.currentTimeMillis() / 1000.0;
        double timeInAir = currentTimeSeconds - startTimeSeconds;
        telemetryData.put("time_in_air", timeInAir);
        // Format time_in_air_min_sec as minutes.seconds (like 2.35 for 2 min 35 sec)
        int minutes = (int) (timeInAir / 60);
        int seconds = (int) (timeInAir % 60);
        telemetryData.put("time_in_air_min_sec", String.format("%d.%02d", minutes, seconds));

        // Process different MAVLink message types
        if (payload instanceof GpsRawInt gps) {
            // Convert GPS raw values (assuming they are in 1e7 format)
            Double lat = gps.lat() / 1e7;
            Double lon = gps.lon() / 1e7;
            Double alt = gps.alt() / 1000.0;
            telemetryData.put("lat", lat);
            telemetryData.put("lon", lon);
            telemetryData.put("alt", alt);
            telemetryData.put("gps_hdop", gps.eph() / 100.0);

            // Calculate distance traveled using Haversine formula if previous exists
            if (prevLat != null && prevLon != null) {
                double distance = calculateDistance(prevLat, prevLon, lat, lon);
                totalDistance += distance;
                telemetryData.put("dist_traveled", totalDistance);
            }
            prevLat = lat;
            prevLon = lon;
            // Calculate distance to home
            double distHome = calculateDistance(lat, lon, homeLat, homeLon);
            telemetryData.put("dist_to_home", distHome);
        } else if (payload instanceof VfrHud vfrHud) {
            telemetryData.put("airspeed", vfrHud.airspeed());
            telemetryData.put("groundspeed", vfrHud.groundspeed());
            telemetryData.put("vertical_speed", vfrHud.climb());
        } else if (payload instanceof Attitude attitude) {
            telemetryData.put("roll", Math.toDegrees(attitude.roll()));
            telemetryData.put("pitch", Math.toDegrees(attitude.pitch()));
            telemetryData.put("yaw", Math.toDegrees(attitude.yaw()));
        } else if (payload instanceof GlobalPositionInt globalPosition) {
            telemetryData.put("alt", globalPosition.relativeAlt() / 1000.0);
            // If needed, you can use heading for something else
        } else if (payload instanceof NavControllerOutput navControllerOutput) {
            telemetryData.put("wp_dist", navControllerOutput.wpDist());
        } else if (payload instanceof MissionCurrent missionCurrent) {
            telemetryData.put("current_wp", missionCurrent.seq());
        } else if (payload instanceof SysStatus sysStatus) {
            telemetryData.put("battery_voltage", sysStatus.voltageBattery() / 1000.0);
            telemetryData.put("battery_current", sysStatus.currentBattery() / 100.0);
        } else if (payload instanceof RcChannels rcChannels) {
            telemetryData.put("ch3out", rcChannels.chan3Raw());
            int chan3 = rcChannels.chan3Raw();
            // Assuming channel value between 1000 and 2000 maps to 0 to 100%
            if (chan3 >= 1000 && chan3 <= 2000) {
                double percent = ((chan3 - 1000) * 100.0) / 1000.0;
                telemetryData.put("ch3percent", Math.round(percent * 100.0) / 100.0);
            } else {
                telemetryData.put("ch3percent", null);
            }
        } else if (payload instanceof MissionItemInt missionItem) {
            @SuppressWarnings("unchecked")
            List<String> waypoints = (List<String>) telemetryData.get("waypoints");
            waypoints.add("Lat: " + (missionItem.x() / 1e7) + ", Lon: " + (missionItem.y() / 1e7));
        } else if (payload instanceof WindCov windCov) {
            // Calculate wind speed from components. Assuming windX() and windY() in cm/s.
            double windX = windCov.windX();
            double windY = windCov.windY();
            double computedWindSpeed = Math.sqrt(windX * windX + windY * windY) / 100.0; // m/s
            telemetryData.put("wind_vel", computedWindSpeed);
        }

        // Calculate tot and toh if groundspeed is available and non-zero.
        Double groundspeed = telemetryData.get("groundspeed") instanceof Number
                ? ((Number) telemetryData.get("groundspeed")).doubleValue() : null;
        Double wp_dist = telemetryData.get("wp_dist") instanceof Number
                ? ((Number) telemetryData.get("wp_dist")).doubleValue() : null;
        Double dist_to_home = telemetryData.get("dist_to_home") instanceof Number
                ? ((Number) telemetryData.get("dist_to_home")).doubleValue() : null;

        if (groundspeed != null && groundspeed > 0) {
            if (wp_dist != null) {
                telemetryData.put("tot", Math.round((wp_dist / groundspeed) * 100.0) / 100.0);
            }
            if (dist_to_home != null) {
                telemetryData.put("toh", Math.round((dist_to_home / groundspeed) * 100.0) / 100.0);
            }
        }

        telemetryService.outputTelemetryData(telemetryData.toString());
    }

    // Haversine formula to calculate distance in meters between two lat/lon points.
    private double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        final double R = 6371000; // Earth radius in meters
        double phi1 = Math.toRadians(lat1);
        double phi2 = Math.toRadians(lat2);
        double deltaPhi = Math.toRadians(lat2 - lat1);
        double deltaLambda = Math.toRadians(lon2 - lon1);
        double a = Math.sin(deltaPhi / 2) * Math.sin(deltaPhi / 2)
                + Math.cos(phi1) * Math.cos(phi2) * Math.sin(deltaLambda / 2) * Math.sin(deltaLambda / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c;
    }

    // Emit telemetry data to the terminal every 1 second
    private void emitTelemetry() {
        System.out.println("\033[1;34m--- Telemetry Data ---\033[0m");
        telemetryData.forEach((key, value) ->
                System.out.printf("\033[92m%-20s\033[0m: %s\n", key, value)
        );
        System.out.println("\033[1;34m----------------------\033[0m\n");
    }

    private void logInfo(String message) {
        System.out.println("\033[92mINFO\033[0m - " + message);
    }

    private void logError(String message) {
        System.err.println("\033[91mERROR\033[0m - " + message);
    }
}
