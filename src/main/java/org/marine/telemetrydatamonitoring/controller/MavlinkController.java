package org.marine.telemetrydatamonitoring.controller;

import org.marine.telemetrydatamonitoring.service.MavlinkService;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/api/mavlink")
public class MavlinkController {
    private final MavlinkService mavlinkService;

    public MavlinkController(MavlinkService mavlinkService) {
        this.mavlinkService = mavlinkService;
    }

    @PostMapping("/connect")
    public String connectToDrone(@RequestParam String host, @RequestParam int port) {
        mavlinkService.connect(host, port);
        return "Connecting to drone at " + host + ":" + port;
    }

    @PostMapping("/disconnect")
    public String disconnectFromDrone() {
        mavlinkService.disconnect();
        return "Disconnected from drone.";
    }
}
