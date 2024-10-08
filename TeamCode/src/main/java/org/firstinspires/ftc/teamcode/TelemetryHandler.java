package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.HashMap;

public class TelemetryHandler {
    private final LinearOpMode myOpMode;
    private final HashMap<String, String> telemetryData;

    // construtor p initialize o telemetry handler com referencia à telemetria
    public TelemetryHandler(LinearOpMode opMode) {
        this.myOpMode = opMode;
        this.telemetryData = new HashMap<>();
    }

    // update a telemetria
    public void addOrUpdate(String key, String value) {
        telemetryData.put(key, value);
    }

    public void remove(String key) {
        telemetryData.remove(key);
    }

    public void clear() {
        telemetryData.clear();
    }

    public void update() {
        myOpMode.telemetry.clear();  // clears a telemetria antes de add so as novas
        for (String key : telemetryData.keySet()) {
            myOpMode.telemetry.addData(key, telemetryData.get(key));  // add new data
        }
        myOpMode.telemetry.update();  // update telemetry
    }

    public void initializeDefaults() {
        addOrUpdate("Loop Status", "Initializing...");
        addOrUpdate("Arm Position", "Unknown");
        addOrUpdate("Claw Position", "Unknown");
        addOrUpdate("Button Cross", "Not Pressed");
        addOrUpdate("Button Circle", "Not Pressed");
        update();  // Update the telemetry immediately after setting defaults
    }
}
