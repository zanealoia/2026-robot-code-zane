/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class SavedLoggedNetworkString extends LoggedNetworkString {
    private String key; // This is annoying
    private String lastValue = "";
    private ConfigManager configManager = ConfigManager.getInstance();

    public SavedLoggedNetworkString(String key) {
        super(key);
        this.key = key;
        this.initFromConfig();
    }

    public SavedLoggedNetworkString(String key, String value) {
        super(key, value);
        this.key = key;
        this.initFromConfig();
    }

    private void initFromConfig() {
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config alue %s\n", key);
            configManager.set(key, get());
        } else {
            Object value = configManager.get(key);
            if (value instanceof String) {
                String sValue = (String) value;
                System.out.printf("Updating %s to %s\n", key, sValue);
                this.set(sValue);
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
        if (get() != this.lastValue) {
            System.out.printf("Updating %s from the network to: %s\n", key, get());
            this.lastValue = get();
            this.set(this.lastValue);
        }
    }
}
