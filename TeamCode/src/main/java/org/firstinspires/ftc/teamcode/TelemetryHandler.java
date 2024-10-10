package org.firstinspires.ftc.teamcode;

import android.util.Log;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.*;

/**
 * The TelemetryHandler class simplifies the management of telemetry data in FTC OpModes.
 * It allows for adding, updating, removing, and selectively displaying telemetry entries,
 * including nested data structures.
 *
 * This class is designed to be modular and easy to use, accommodating changes in telemetry
 * requirements without significant code modifications.
 */
public class TelemetryHandler {

    /**
     * Reference to the LinearOpMode using this TelemetryHandler.
     */
    private final LinearOpMode myOpMode;

    /**
     * Stores telemetry data as a nested map structure.
     * Keys are String identifiers, and values are either further Map<String, Object>
     * instances (representing nested sections) or the actual telemetry values.
     */
    private final Map<String, Object> telemetryData;

    /**
     * Set of paths to telemetry entries that should be displayed.
     * Paths are represented as dot-delimited strings (e.g., "Robot.Status").
     */
    private final Set<String> displayKeys;

    /**
     * Set of paths to telemetry entries that should be excluded from display.
     */
    private final Set<String> excludeKeys;

    /**
     * Constructs a new TelemetryHandler with a reference to the current OpMode.
     *
     * @param opMode The LinearOpMode instance that is using this TelemetryHandler.
     */
    public TelemetryHandler(LinearOpMode opMode) {
        this.myOpMode = opMode;
        this.telemetryData = new HashMap<>();
        this.displayKeys = new HashSet<>();
        this.excludeKeys = new HashSet<>();
    }

    /**
     * Adds or updates a telemetry entry, regardless of whether it is nested or not.
     * The path represents the hierarchy of keys leading to the value, using dot notation.
     * If the edited entry is currently being displayed, the telemetry display is automatically refreshed.
     *
     * @param value The value to associate with the specified key path.
     * @param path  The dot-delimited path representing nested sections (e.g., "Robot.Arm Position").
     */
    public void edit(Object value, String path) {
        if (path == null || path.isEmpty()) {
            // Log an error if the path is null or empty
            Log.e("TelemetryHandler", "edit() called with null or empty path. No action taken.");
            return; // No path provided; nothing to edit
        }

        // Split the path into individual keys based on the dot delimiter
        String[] keys = path.split("\\.");

        Map<String, Object> currentMap = telemetryData;

        // Traverse or create the nested maps based on the keys
        for (int i = 0; i < keys.length - 1; i++) {
            String key = keys[i];

            // If the section doesn't exist, create a new nested map
            if (!currentMap.containsKey(key)) {
                currentMap.put(key, new HashMap<String, Object>());
            }

            Object nestedObject = currentMap.get(key);

            // Safely cast nestedObject to Map<String, Object> if it is of that type
            if (nestedObject instanceof Map<?, ?>) {
                @SuppressWarnings("unchecked")
                Map<String, Object> nestedMap = (Map<String, Object>) nestedObject;
                currentMap = nestedMap;
            } else {
                // Handle case where the path does not lead to a map
                Log.e("TelemetryHandler", "Invalid structure at " + key + ". Expected a map.");
                return;
            }
        }

        // Add or update the final key-value pair
        String finalKey = keys[keys.length - 1];
        currentMap.put(finalKey, value);

        // Check if the edited path is currently being displayed
        if (shouldDisplay(path)) {
            // If the edited entry is currently displayed, refresh the telemetry
            update();
        }
    }

    /**
     * Removes a specific telemetry entry or section, handling nested structures.
     *
     * @param path The dot-delimited path to the entry or section to remove (e.g., "Robot.Status").
     */
    public void remove(String path) {
        if (path == null || path.isEmpty()) {
            // Log an error if the path is null or empty
            Log.e("TelemetryHandler", "remove() called with null or empty path. No action taken.");
            return; // No path provided; nothing to remove
        }

        String[] keys = path.split("\\.");
        Map<String, Object> currentMap = telemetryData;
        Stack<Map<String, Object>> mapStack = new Stack<>();
        Stack<String> keyStack = new Stack<>();

        // Traverse the nested maps based on the keys
        for (int i = 0; i < keys.length; i++) {
            String currentKey = keys[i];

            if (!currentMap.containsKey(currentKey)) {
                // Key does not exist; nothing to remove
                return;
            }

            mapStack.push(currentMap);
            keyStack.push(currentKey);

            if (i == keys.length - 1) {
                // Last key in the path; remove the entry
                currentMap.remove(currentKey);

                // Refresh telemetry if the removed key was displayed
                String fullPath = String.join(".", Arrays.copyOfRange(keys, 0, i + 1));
                if (displayKeys.contains(fullPath)) {
                    update();
                }

                // Optionally remove empty parent maps
                while (!mapStack.isEmpty()) {
                    Map<String, Object> parentMap = mapStack.pop();
                    String key = keyStack.pop();
                    if (parentMap.get(key) instanceof Map<?, ?>) {
                        @SuppressWarnings("unchecked")
                        Map<String, Object> childMap = (Map<String, Object>) parentMap.get(key);
                        if (childMap.isEmpty()) {
                            parentMap.remove(key);
                        }
                    }
                }
                return;
            } else {
                // Continue traversing deeper into the nested map
                Object nextLevel = currentMap.get(currentKey);

                if (nextLevel instanceof Map<?, ?>) {
                    @SuppressWarnings("unchecked")
                    Map<String, Object> nextMap = (Map<String, Object>) nextLevel;
                    currentMap = nextMap;
                } else {
                    // Handle case where the path does not lead to a map
                    Log.e("TelemetryHandler", "Invalid structure at: " + currentKey + ". Expected a map.");
                    return;
                }
            }
        }
    }

    /**
     * Clears all telemetry data entries and display settings.
     */
    public void clearAll() {
        telemetryData.clear();
        displayKeys.clear();
        excludeKeys.clear();
        update(); // Refresh telemetry display
    }

    /**
     * Updates the telemetry display with the current data based on the display settings.
     * This method should be called after making changes to the telemetry data.
     */
    public void update() {
        myOpMode.telemetry.clear(); // Clears previous telemetry data

        // Display data based on displayKeys and excludeKeys
        displayRecursive(telemetryData, "", new LinkedList<>());

        myOpMode.telemetry.update(); // Refreshes the telemetry display
    }

    /**
     * Recursively displays telemetry data according to the display and exclusion settings.
     *
     * @param currentMap The current level of the nested map.
     * @param prefix     The accumulated key path for display purposes.
     * @param pathStack  The stack of keys representing the current path.
     */
    private void displayRecursive(@NonNull Map<String, Object> currentMap, String prefix, LinkedList<String> pathStack) {
        for (Map.Entry<String, Object> entry : currentMap.entrySet()) {
            String key = entry.getKey();
            Object value = entry.getValue();

            pathStack.add(key);
            String fullPath = String.join(".", pathStack);

            if (excludeKeys.contains(fullPath)) {
                // Skip this key or section
                pathStack.removeLast();
                continue;
            }

            if (value instanceof Map<?, ?>) {
                // Nested map (section)
                @SuppressWarnings("unchecked")
                Map<String, Object> nestedMap = (Map<String, Object>) value;

                if (shouldDisplay(fullPath)) {
                    // Display the entire section
                    displayRecursive(nestedMap, prefix + key + " - ", pathStack);
                }
            } else {
                // Leaf node (actual data)
                if (shouldDisplay(fullPath)) {
                    myOpMode.telemetry.addData(prefix + key, value.toString());
                }
            }

            pathStack.removeLast();
        }
    }

    /**
     * Determines whether a key or path should be displayed based on displayKeys and excludeKeys.
     *
     * @param fullPath The full path of the key (e.g., "Robot.Status").
     * @return True if the key should be displayed; false otherwise.
     */
    private boolean shouldDisplay(String fullPath) {
        if (excludeKeys.contains(fullPath)) {
            // Exclude this path
            return false;
        }
        if (displayKeys.isEmpty()) {
            // If no display keys are specified, display everything except excluded keys
            return true;
        } else {
            // Display only if the key is in displayKeys
            return displayKeys.contains(fullPath);
        }
    }

    /**
     * Checks if the given dot-delimited path exists in the telemetry data.
     *
     * @param path The dot-delimited path to check (e.g., "Robot.Status").
     * @return True if the path exists, false otherwise.
     */
    private boolean pathExists(@NonNull String path) {
        String[] keys = path.split("\\.");
        Map<String, Object> currentMap = telemetryData;

        for (String key : keys) {
            if (currentMap.containsKey(key)) {
                Object value = currentMap.get(key);

                if (value instanceof Map<?, ?>) {
                    // Continue traversing into the nested map
                    @SuppressWarnings("unchecked")
                    Map<String, Object> nestedMap = (Map<String, Object>) value;
                    currentMap = nestedMap;
                } else {
                    // Reached a leaf node
                    return true;
                }
            } else {
                // Key does not exist
                return false;
            }
        }
        return true;
    }

    /**
     * Allows the display of multiple categories or specific entries.
     * You can specify sections, subsections, or individual keys to display.
     *
     * @param paths Variable number of dot-delimited paths representing the data to display.
     *              Each path is a string (e.g., "Robot.Status").
     */
    public void display(@NonNull String... paths) {
        for (String path : paths) {
            if (path != null && !path.isEmpty()) {
                if (pathExists(path)) {
                    displayKeys.add(path);
                } else {
                    Log.e("TelemetryHandler", "display() called with non-existent path: " + path);
                }
            }
        }
        // Update the display when new items are added to be displayed
        update();
    }

    /**
     * Excludes specific sections or keys from being displayed, without removing them from telemetry data.
     * You can specify sections, subsections, or individual keys to exclude.
     *
     * @param paths Variable number of dot-delimited paths representing the data to exclude.
     *              Each path is a string (e.g., "Robot.Debug").
     */
    public void exclude(@NonNull String... paths) {
        for (String path : paths) {
            if (path != null && !path.isEmpty()) {
                if (pathExists(path)) {
                    excludeKeys.add(path);
                } else {
                    Log.e("TelemetryHandler", "exclude() called with non-existent path: " + path);
                }
            }
        }
        // Update the display when items are excluded
        update();
    }

    /**
     * Initializes default telemetry entries.
     * This can be customized to include any default values you need.
     */
    public void initializeDefaults() {
        // Status section
        edit("Initializing...", "Status.Loop Status");

        // Display desired entries
        display(
                "Robot.Arm Position",
                "Robot.Claw Position",
                "Status.Loop Status",
                "Controls.Raw.Raw Left Stick X",
                "Controls.Raw.Raw Left Stick Y",
                "Controls.Raw.Raw Right Stick X",
                "Controls.Raw.Raw Right Stick Y"
        );
    }
}