/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.utils;

public class MathUtils {
    /**
     * Convert meters per second to rotations per second
     * @param wMPS Wheel velocity (in <strong>m/s</strong>)
     * @param c The circumference of the wheels
     * @return Wheel velocity (in <strong>rotations/s</strong>)
     */
    public static double MPSToRPS(double wMPS, double c) {
        return wMPS / c;
    }

    public static double RPSToMPS(double wRPS, double c) {
        return wRPS * c;
    }
}
