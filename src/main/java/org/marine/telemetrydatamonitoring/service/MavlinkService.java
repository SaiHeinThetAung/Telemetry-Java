package org.marine.telemetrydatamonitoring.service;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.MavlinkMessage;
import io.dronefleet.mavlink.minimal.Heartbeat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.net.*;

@Service
public class MavlinkService {
    private static final Logger logger = LoggerFactory.getLogger(MavlinkService.class);
    private DatagramSocket udpSocket;
    private MavlinkConnection connection;
    private InetAddress remoteAddress;
    private int remotePort;

    /**
     * Establishes a connection to the Mission Planner using the given host and port.
     *
     * @param host the IP address of the Mission Planner
     * @param port the port to connect to
     */
    public void connect(String host, int port) {
        try {
            logger.info("Attempting to connect to {}:{}", host, port);

            // Create a DatagramSocket for UDP communication
            udpSocket = new DatagramSocket();
            remoteAddress = InetAddress.getByName(host);
            remotePort = port;

            // Set up MavlinkConnection manually (using the socket)
            connection = MavlinkConnection.create(new DatagramInputStream(udpSocket), new DatagramOutputStream(udpSocket));

            logger.info("Connected to Mission Planner at {}:{}", host, port);

            // Start listening for telemetry data
            startTelemetryListener();
        } catch (SocketException | UnknownHostException e) {
            logger.error("Connection failed: {}", e.getMessage());
        }
    }

    /**
     * Starts a separate thread to listen for incoming MAVLink telemetry data.
     */
    private void startTelemetryListener() {
        new Thread(() -> {
            try {
                while (true) {
                    MavlinkMessage<?> mavlinkMessage = connection.next();
                    if (mavlinkMessage != null) {
                        processTelemetryMessage(mavlinkMessage);
                    }
                }
            } catch (IOException e) {
                logger.error("Error reading telemetry data: {}", e.getMessage());
            }
        }, "Telemetry-Listener-Thread").start();
    }

    /**
     * Processes incoming MAVLink messages.
     *
     * @param mavlinkMessage the MAVLink message to process
     */
    private void processTelemetryMessage(MavlinkMessage<?> mavlinkMessage) {
        Object payload = mavlinkMessage.getPayload();
        if (payload instanceof Heartbeat) {
            Heartbeat heartbeat = (Heartbeat) payload;
            logger.info("Received Heartbeat: {}", heartbeat);
        } else {
            logger.info("Received MAVLink message: {}", payload);
        }
    }

    /**
     * Disconnects from the Mission Planner and cleans up resources.
     */
    public void disconnect() {
        try {
            if (udpSocket != null && !udpSocket.isClosed()) {
                udpSocket.close();
                logger.info("Disconnected from Mission Planner.");
            }
        } catch (Exception e) {
            logger.error("Error disconnecting: {}", e.getMessage());
        }
    }

    /**
     * Custom DatagramInputStream for reading UDP data.
     */
    private static class DatagramInputStream extends java.io.InputStream {
        private final DatagramSocket socket;
        private final byte[] buffer = new byte[1024];
        private DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        public DatagramInputStream(DatagramSocket socket) {
            this.socket = socket;
        }

        @Override
        public int read() throws IOException {
            socket.receive(packet);
            return packet.getData()[0]; // Just return the first byte of the packet
        }
    }

    /**
     * Custom DatagramOutputStream for sending UDP data.
     */
    private static class DatagramOutputStream extends java.io.OutputStream {
        private final DatagramSocket socket;
        private final InetAddress remoteAddress;
        private final int remotePort;

        public DatagramOutputStream(DatagramSocket socket) {
            this.socket = socket;
            this.remoteAddress = socket.getInetAddress();
            this.remotePort = socket.getPort();
        }

        @Override
        public void write(int b) throws IOException {
            byte[] data = new byte[] {(byte) b};
            DatagramPacket packet = new DatagramPacket(data, data.length, remoteAddress, remotePort);
            socket.send(packet);
        }
    }
}
