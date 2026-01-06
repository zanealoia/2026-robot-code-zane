/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.teamdeadbolts.utils.tuning.ConfigManager.Tuneable;

public class SavedLoggedNetworkNumber extends LoggedNetworkNumber implements Tuneable {
    private String key; // This is annoying
    private double lastValue = 0.0;
    private ConfigManager configManager = ConfigManager.getInstance();

    private static final HashMap<String, SavedLoggedNetworkNumber> INSTANCES = new HashMap<>();

    /**
     * Get an instance of a SavedLoggedNetworkNumber
     * @param key The key of the value
     * @param defautValue The default value
     * @return An instance of SavedLoggedNetworkNumber
     */
    public static synchronized SavedLoggedNetworkNumber get(String key, double defautValue) {
        return INSTANCES.computeIfAbsent(key, k -> SavedLoggedNetworkNumber.get(k, defautValue));
    }

    private SavedLoggedNetworkNumber(String key, double value) {
        super(key, value);
        this.key = key;
        configManager.registerTunable(this);
    }

    public void initFromConfig() {
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config value %s\n", key);
            configManager.set(key, get());
        } else {
            Object value = configManager.get(key);
            if (value instanceof Double d) {
                System.out.printf("Updating %s to %s\n", key, d);
                super.set(d);
                lastValue = d;
            } else {
                System.out.printf("Warning: %s is of the wrong type\n", key);
            }
        }
    }

    @Override
    public void set(double value) {
        super.set(value);
        configManager.set(this.key, value);
    }

    @Override
    public void periodic() {
        super.periodic();
        double c = get();
        if (c != this.lastValue) {
            System.out.printf("Updating %s from the network to: %s\n", key, c);
            this.lastValue = c;
            configManager.set(key, c);
        }
    }
}
