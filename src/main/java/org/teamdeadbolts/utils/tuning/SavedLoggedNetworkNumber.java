/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SavedLoggedNetworkNumber extends LoggedNetworkNumber {
    private String key; // This is annoying
    private double lastValue = 0.0;
    private ConfigManager configManager = ConfigManager.getInstance();

    public SavedLoggedNetworkNumber(String key) {
        super(key);
        this.key = key;
        this.initFromConfig();
    }

    public SavedLoggedNetworkNumber(String key, double value) {
        super(key, value);
        this.key = key;
        this.initFromConfig();
    }

    private void initFromConfig() {
        if (!configManager.contains(key)) {
            System.out.printf("Creating new config value %s\n", key);
            configManager.set(key, get());
        } else {
            Object value = configManager.get(key);
            if (value instanceof Double) {
                double dValue = (Double) value;
                System.out.printf("Updating %s to %s\n", key, dValue);
                this.set(dValue);
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
        if (get() != this.lastValue) {
            System.out.printf("Updating %s from the network to: %s\n", key, get());
            this.lastValue = get();
            this.set(this.lastValue);
        }
    }
}
