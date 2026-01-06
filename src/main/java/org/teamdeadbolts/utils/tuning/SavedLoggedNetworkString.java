/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.teamdeadbolts.utils.tuning.ConfigManager.Tuneable;

public class SavedLoggedNetworkString extends LoggedNetworkString implements Tuneable {
    private String key; // This is annoying
    private String lastValue = "";
    private ConfigManager configManager = ConfigManager.getInstance();

    private static final HashMap<String, SavedLoggedNetworkString> INSTANCES = new HashMap<>();

    /**
     * Get an instance of a SavedLoggedNetworkString
     * @param key The key of the value
     * @param defautValue The default value
     * @return An instance of SavedLoggedNetworkString
     */
    public static synchronized SavedLoggedNetworkString get(String key, String defautValue) {
        return INSTANCES.computeIfAbsent(key, k -> new SavedLoggedNetworkString(k, defautValue));
    }

    private SavedLoggedNetworkString(String key, String value) {
        super(key, value);
        this.key = key;
        configManager.registerTunable(this);
    }

    public void initFromConfig() {
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config alue %s\n", key);
            configManager.set(key, get());
        } else {
            Object value = configManager.get(key);
            if (value instanceof String s) {
                System.out.printf("Updating %s to %s\n", key, s);
                super.set(s);
                lastValue = s;
            } else {
                System.out.printf("Warning: %s is of the wrong type\n", key);
            }
        }
    }

    @Override
    public void set(String value) {
        super.set(value);
        configManager.set(this.key, value);
    }

    @Override
    public void periodic() {
        super.periodic();
        String c = get();
        if (!c.equals(this.lastValue)) {
            System.out.printf("Updating %s from the network to: %s\n", key, c);
            this.lastValue = c;
            configManager.set(key, c);
        }
    }
}
