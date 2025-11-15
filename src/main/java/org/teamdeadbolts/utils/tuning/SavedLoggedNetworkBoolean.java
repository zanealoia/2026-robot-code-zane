/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils.tuning;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class SavedLoggedNetworkBoolean extends LoggedNetworkBoolean {
    private String key; // This is annoying
    private boolean lastValue = false;
    private ConfigManager configManager = ConfigManager.getInstance();

    public SavedLoggedNetworkBoolean(String key) {
        super(key);
        this.key = key;
        this.initFromConfig();
    }

    public SavedLoggedNetworkBoolean(String key, boolean value) {
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
            if (value instanceof Boolean) {
                boolean bValue = (Boolean) value;
                System.out.printf("Updating %s to %s\n", key, bValue);
                this.set(bValue);
            } else {
                System.out.printf("Warning: %s is of the wrong type\n", key);
            }
        }
    }

    @Override
    public void set(boolean value) {
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
