package org.marine.telemetrydatamonitoring.client;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.ardupilotmega.Wind;
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
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Component
public class MavlinkClient implements Runnable {
    // Service used to handle telemetry data (probably for storage or output)
    private final TelemetryService telemetryService;

    // Constants for mission planner host and ports
    private final String missionPlannerHost = "localhost";
    private final int missionPlannerPort = 14550;
    private final int udpPort = 14557;
    private final int udpPort2 = 14558;

    // Map to store telemetry data
    private final Map<String, Object> telemetryData = new HashMap<>();

    // Variables to calculate movement and distance
    private Double prevLat = null, prevLon = null;
    private double totalDistance = 0.0;
    private double homeLat = 16.7745;
    private double homeLon = 96.1552;

    private double startTimeSeconds;

    // Constructor to initialize telemetryService and data
    public MavlinkClient(TelemetryService telemetryService) {
        this.telemetryService = telemetryService;
        initializeTelemetryData(); // Initializing telemetry data
    }

    // Initialize telemetry data with default values
    private void initializeTelemetryData() {
        telemetryData.put("sysid", null);
        telemetryData.put("alt", null);
        telemetryData.put("dist_traveled", null);
        telemetryData.put("wp_dist", null);
        telemetryData.put("dist_to_home", 0.0);
        telemetryData.put("vertical_speed", 0.0);
        telemetryData.put("groundspeed", 0.0);
        telemetryData.put("wind_vel", 0.0);
        telemetryData.put("airspeed", 0.0);
        telemetryData.put("roll", 0.0);
        telemetryData.put("pitch", 0.0);
        telemetryData.put("yaw", 0.0);
        telemetryData.put("time_in_air", 0.0);
        telemetryData.put("gps_hdop", 0.0);
        telemetryData.put("toh", null);
        telemetryData.put("tot", null);
        telemetryData.put("battery_voltage", 0.0);
        telemetryData.put("battery_current", 0.00);
        telemetryData.put("ch3percent", null);
        telemetryData.put("ch3out", null);
        telemetryData.put("ch9out", 0.00);
        telemetryData.put("ch10out", 0.00);
        telemetryData.put("ch11out", 0.00);
        telemetryData.put("ch12out", 0.00);
        telemetryData.put("waypoints", new ArrayList<String>());
    }

    // Starts the telemetry listening process in a new thread
    public void startListening() {
        new Thread(this).start();
    }

    @Override
    public void run() {
        startTimeSeconds = System.currentTimeMillis() / 1000.0;

        // Setting up periodic task to emit telemetry data every second
        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleAtFixedRate(this::emitTelemetry, 1, 1, TimeUnit.SECONDS);

        // Start UDP and TCP listeners
        new Thread(this::startUdpListener).start();
        new Thread(this::startUdpListener2).start();
        new Thread(this::startTcpListener).start();
    }

    // Listens to UDP messages on specified ports
    private void startUdpListener() {
        listenForUdpMessages(udpPort);
    }

    private void startUdpListener2() {
        listenForUdpMessages(udpPort2);
    }

    // Helper method to listen for UDP messages on a given port
    private void listenForUdpMessages(int port) {
        try (DatagramSocket socket = new DatagramSocket(port)) {
            byte[] buffer = new byte[2048];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            while (true) {
                socket.receive(packet); // Receive incoming UDP packet
                try (InputStream inputStream = new ByteArrayInputStream(packet.getData(), packet.getOffset(), packet.getLength())) {
                    MavlinkConnection connection = MavlinkConnection.create(inputStream, null);
                    MavlinkMessage<?> message = connection.next(); // Read message
                    if (message != null) {
                        processTelemetryMessage(message); // Process message if present
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Starts listening to TCP messages from Mission Planner
    private void startTcpListener() {
        try (Socket socket = new Socket(missionPlannerHost, missionPlannerPort);
             InputStream inputStream = socket.getInputStream();
             OutputStream outputStream = socket.getOutputStream()) {

            MavlinkConnection connection = MavlinkConnection.create(inputStream, outputStream);
            while (true) {
                MavlinkMessage<?> message = connection.next(); // Read TCP message
                if (message != null) {
                    int systemId = message.getOriginSystemId(); // Get system ID
                    telemetryData.put("sysid", systemId);
                    processTelemetryMessage(message); // Process the message
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Processes a MAVLink message and updates telemetry data accordingly
    private void processTelemetryMessage(MavlinkMessage<?> message) {
        Object payload = message.getPayload();


        // Process different types of telemetry data and update the map
        if (payload instanceof GlobalPositionInt globalPositionInt) {
            double currentLat = globalPositionInt.lat() / 1e7;
            double currentLon = globalPositionInt.lon() / 1e7;
            double currentAlt = globalPositionInt.alt() / 1000.0;

            // Calculate distance from home
            double distToHome = calculateDistance(currentLat, currentLon, homeLat, homeLon);

            // Calculate total distance traveled
            if (prevLat != null && prevLon != null) {
                double distance = calculateDistance(prevLat, prevLon, currentLat, currentLon);
                totalDistance += distance;
                telemetryData.put("dist_traveled", totalDistance);
            }

            // Update telemetry data with new values
            telemetryData.put("dist_to_home", distToHome);
            prevLat = currentLat;
            prevLon = currentLon;
            telemetryData.put("lat", currentLat);
            telemetryData.put("lon", currentLon);
            telemetryData.put("alt", currentAlt);
        } else if (payload instanceof VfrHud vfrHud) {
            telemetryData.put("airspeed", vfrHud.airspeed());
            telemetryData.put("groundspeed", vfrHud.groundspeed());
            telemetryData.put("vertical_speed", vfrHud.climb());
        } else if (payload instanceof NavControllerOutput navControllerOutput) {
            telemetryData.put("wp_dist", navControllerOutput.wpDist());
        } else if (payload instanceof Attitude attitude) {
            telemetryData.put("roll", Math.toDegrees(attitude.roll()));
            telemetryData.put("pitch", Math.toDegrees(attitude.pitch()));
            telemetryData.put("yaw", Math.toDegrees(attitude.yaw()));
        } else if (payload instanceof SysStatus sysStatus) {
            telemetryData.put("battery_voltage", sysStatus.voltageBattery() / 1000.0);
            telemetryData.put("battery_current", sysStatus.currentBattery() / 1000.0);
        } else if (payload instanceof GpsRawInt gpsRawInt) {
            telemetryData.put("gps_hdop", gpsRawInt.eph() / 100.0);
        }
        else if (payload instanceof HomePosition homePosition) {
            telemetryData.put("gps_hdop", homePosition.altitude() );
        }
        // Process additional telemetry data from servo and wind data
        else if (payload instanceof ServoOutputRaw servoOutputRaw) {
            telemetryData.put("ch3out", servoOutputRaw.servo3Raw());
            telemetryData.put("ch3percent", String.format("%.2f", ((servoOutputRaw.servo3Raw() - 1000.0) / 1000.0) * 100));
            telemetryData.put("ch9out", servoOutputRaw.servo9Raw());
            telemetryData.put("ch10out", servoOutputRaw.servo10Raw());
            telemetryData.put("ch11out", servoOutputRaw.servo11Raw());
            telemetryData.put("ch12out", servoOutputRaw.servo12Raw());
        } else if (payload instanceof Wind wind) {
            telemetryData.put("wind_vel", wind.speed());
        }

        telemetryService.outputTelemetryData(telemetryData.toString()); // Output telemetry data
    }

    // Emits telemetry data to console every second
    private void emitTelemetry() {
        System.out.println("--- Telemetry Data ---");
        telemetryData.forEach((key, value) -> System.out.println(key + ": " + value)); // Print all telemetry data
    }

    // Calculates distance between two GPS coordinates using Haversine formula
    private double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        final int R = 6371000; // Earth's radius in meters
        double latDistance = Math.toRadians(lat2 - lat1);
        double lonDistance = Math.toRadians(lon2 - lon1);
        double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return R * c; // Returns distance in meters
    }
}
