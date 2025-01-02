package org.marine.telemetrydatamonitoring.controller;

import org.marine.telemetrydatamonitoring.service.MavlinkService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/telemetry")
public class TelemetryController {

    @Autowired
    private MavlinkService mavlinkService;

    @PostMapping("/connect")
    public String connect(@RequestParam String host, @RequestParam int port) {
        mavlinkService.connect(host, port);
        return "Connected to " + host + ":" + port;
    }

    @PostMapping("/disconnect")
    public String disconnect() {
        mavlinkService.disconnect();
        return "Disconnected from Mission Planner.";
    }
}
