package org.marine.telemetrydatamonitoring;

import org.marine.telemetrydatamonitoring.client.MavlinkClient;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
public class TelemetryDataMonitoringApplication implements CommandLineRunner {
	private final MavlinkClient mavlinkClient;

	public TelemetryDataMonitoringApplication(MavlinkClient mavlinkClient) {
		this.mavlinkClient = mavlinkClient;
	}

	public static void main(String[] args) {
		SpringApplication.run(TelemetryDataMonitoringApplication.class, args);
	}

	@Override
	public void run(String... args) {
		mavlinkClient.startListening();
	}
}
