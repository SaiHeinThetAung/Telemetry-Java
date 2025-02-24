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
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Component
public class MavlinkClient implements Runnable {
    private final TelemetryService telemetryService;

    private final String missionPlannerHost = "localhost";
    private final int missionPlannerPort = 14550; // TCP Port for Mission Planner
    private final int udpPort = 14557; // First UDP port
    private final int udpPort2 = 14558; // Second UDP port

    private final Map<String, Object> telemetryData = new HashMap<>();

    private long startTime;

    public MavlinkClient(TelemetryService telemetryService) {
        this.telemetryService = telemetryService;
        initializeTelemetryData();
    }

    private void initializeTelemetryData() {
        telemetryData.put("lat", 0.0);
        telemetryData.put("lon", 0.0);
        telemetryData.put("alt", 0.0);
        telemetryData.put("dist_traveled", 0);
        telemetryData.put("wp_dist", 0.0);
        telemetryData.put("dist_to_home", 0.0);
        telemetryData.put("vertical_speed", 0.0);
        telemetryData.put("wind_vel", 0.0);
        telemetryData.put("airspeed", 0.0);
        telemetryData.put("groundspeed", 0.0);
        telemetryData.put("roll", 0.0);
        telemetryData.put("pitch", 0.0);
        telemetryData.put("yaw", 0.0);
        telemetryData.put("toh", 0.0);
        telemetryData.put("tot", 0.0);
        telemetryData.put("time_in_air", 0.0);
        telemetryData.put("time_in_air_min_sec", 0.0);
        telemetryData.put("gps_hdop", 0.0);
        telemetryData.put("battery_voltage", 0.0);
        telemetryData.put("battery_current", 0.0);
        telemetryData.put("ch9out", 0.0);
        telemetryData.put("waypoints", new ArrayList<String>());
    }

    public void startListening() {
        new Thread(this).start();
    }

    @Override
    public void run() {
        startTime = System.currentTimeMillis();

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
        try (DatagramSocket datagramSocket = new DatagramSocket(port)) {
            System.out.println("Listening for MAVLink messages on UDP port " + port);

            byte[] buffer = new byte[2048];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            while (true) {
                datagramSocket.receive(packet);
                System.out.println("Received UDP packet from " + packet.getAddress() + ":" + packet.getPort());

                try (InputStream inputStream = new ByteArrayInputStream(packet.getData(), packet.getOffset(), packet.getLength())) {
                    MavlinkConnection connection = MavlinkConnection.create(inputStream, null);
                    MavlinkMessage<?> mavlinkMessage = connection.next();
                    if (mavlinkMessage != null) {
                        processTelemetryMessage(mavlinkMessage);
                    }
                } catch (Exception e) {
                    System.err.println("Error processing UDP MAVLink message: " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.err.println("Error in UDP Listener (port " + port + "): " + e.getMessage());
        }
    }

    private void startTcpListener() {
        try (Socket socket = new Socket(missionPlannerHost, missionPlannerPort);
             InputStream inputStream = socket.getInputStream();
             OutputStream outputStream = socket.getOutputStream()) {

            MavlinkConnection connection = MavlinkConnection.create(inputStream, outputStream);
            System.out.println("Connected to TCP server at " + missionPlannerHost + ":" + missionPlannerPort);

            while (true) {
                MavlinkMessage<?> mavlinkMessage = connection.next();
                if (mavlinkMessage != null) {
                    processTelemetryMessage(mavlinkMessage);
                }
            }
        } catch (Exception e) {
            System.err.println("Error in TCP Listener: " + e.getMessage());
        }
    }

    private void processTelemetryMessage(MavlinkMessage<?> mavlinkMessage) {
        Object payload = mavlinkMessage.getPayload();

        long currentTime = System.currentTimeMillis();
        double timeInAir = (currentTime - startTime) / 1000.0;
        telemetryData.put("time_in_air", timeInAir);
        telemetryData.put("time_in_air_min_sec", Math.round(timeInAir / 60.0 + (timeInAir % 60) / 100.0));

        if (payload instanceof GpsRawInt gpsData) {
            telemetryData.put("lat", gpsData.lat() / 1e7);
            telemetryData.put("lon", gpsData.lon() / 1e7);
            telemetryData.put("alt", gpsData.alt() / 1000.0);
            telemetryData.put("gps_hdop", gpsData.eph() / 100.0);
        } else if (payload instanceof VfrHud vfrHud) {
            telemetryData.put("airspeed", vfrHud.airspeed());
            telemetryData.put("groundspeed", vfrHud.groundspeed());
            telemetryData.put("heading", vfrHud.heading());
            telemetryData.put("vertical_speed", vfrHud.climb());
        } else if (payload instanceof Attitude attitude) {
            telemetryData.put("roll", Math.toDegrees(attitude.roll()));
            telemetryData.put("pitch", Math.toDegrees(attitude.pitch()));
            telemetryData.put("yaw", Math.toDegrees(attitude.yaw()));
        } else if (payload instanceof GlobalPositionInt globalPosition) {
            telemetryData.put("alt", globalPosition.relativeAlt() / 1000.0);
            telemetryData.put("dist_to_home", globalPosition.hdg());
        } else if (payload instanceof NavControllerOutput navControllerOutput) {
            telemetryData.put("wp_dist", navControllerOutput.wpDist());
        } else if (payload instanceof MissionCurrent missionCurrent) {
            telemetryData.put("current_wp", missionCurrent.seq());
        } else if (payload instanceof SysStatus sysStatus) {
            telemetryData.put("battery_voltage", sysStatus.voltageBattery() / 1000.0);
            telemetryData.put("battery_current", sysStatus.currentBattery() / 100.0);
        } else if (payload instanceof RcChannels rcChannels) {
            telemetryData.put("ch9out", rcChannels.chan9Raw());
        } else if (payload instanceof MissionItemInt missionItem) {
            @SuppressWarnings("unchecked")
            List<String> waypoints = (List<String>) telemetryData.get("waypoints");
            waypoints.add("Lat: " + (missionItem.x() / 1e7) + ", Lon: " + (missionItem.y() / 1e7));
        }

        printTelemetryData();
        telemetryService.outputTelemetryData(telemetryData.toString());
    }

    private void printTelemetryData() {
        System.out.println("--- Telemetry Data ---");
        telemetryData.forEach((key, value) -> System.out.println(key + ": " + value));
        System.out.println("----------------------\n");
    }
}
