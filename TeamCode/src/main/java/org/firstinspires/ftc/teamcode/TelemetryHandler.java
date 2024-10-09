package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.HashMap;

/**
 * TelemetryHandler simplifies the management of telemetry data in FTC OpModes.
 * It provides methods to add, update, remove, and display telemetry entries efficiently.
 */
public class TelemetryHandler {
    private final LinearOpMode myOpMode;            // Reference to the OpMode using this handler
    private final HashMap<String, String> telemetryData; // Stores telemetry data as key-value pairs

    /**
     * Constructor initializes the TelemetryHandler with a reference to the current OpMode.
     *
     * @param opMode The LinearOpMode instance that is using this TelemetryHandler.
     */
    public TelemetryHandler(LinearOpMode opMode) {
        this.myOpMode = opMode;
        this.telemetryData = new HashMap<>();
    }

    /**
     * Adds a new telemetry entry or updates an existing one.
     *
     * @param key   The label or name of the telemetry data.
     * @param value The value associated with the key.
     */
    public void addOrUpdate(String key, String value) {
        telemetryData.put(key, value);
    }

    /**
     * Removes a telemetry entry by its key.
     *
     * @param key The label of the telemetry data to remove.
     */
    public void remove(String key) {
        telemetryData.remove(key);
    }

    /**
     * Clears all telemetry data entries.
     */
    public void clear() {
        telemetryData.clear();
    }

    /**
     * Updates the telemetry display with the current data.
     * This method should be called after making changes to the telemetry data.
     */
    public void update() {
        myOpMode.telemetry.clear(); // Clears previous telemetry data
        for (String key : telemetryData.keySet()) {
            myOpMode.telemetry.addData(key, telemetryData.get(key)); // Adds updated data
        }
        myOpMode.telemetry.update(); // Refreshes the telemetry display
    }

    /**
     * Initializes default telemetry entries.
     * This can be customized to include any default values you need.
     */
    public void initializeDefaults() {
        addOrUpdate("Loop Status", "Initializing...");
        addOrUpdate("Arm Position", "Unknown");
        addOrUpdate("Claw Position", "Unknown");
        addOrUpdate("Button Cross", "Not Pressed");
        addOrUpdate("Button Circle", "Not Pressed");
        update(); // Update telemetry to display the default values
    }
}