/* The Deadbolts (C) 2025 */
package org.teamdeadbolts.state;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Singleton class to hold the robot state */
public class RobotState {
    private Pose3d robotPose = new Pose3d(); // Field pose of the robot
    private ChassisSpeeds robotSpeeds = new ChassisSpeeds(); // Speed of the robot field rel

    private static RobotState INSTANCE = new RobotState();

    private RobotState() {}

    public static RobotState getInstance() {
        return INSTANCE;
    }

    /*
     =========================== GETTERS =========================
    */

    /**
     * Get the robot pose
     *
     * @return The robot pose
     */
    public Pose3d getRobotPose() {
        return robotPose;
    }

    /**
     * Get the robot velocities
     *
     * @return The robot velocities
     */
    public ChassisSpeeds getRobotVelocities() {
        return robotSpeeds;
    }

    /*
     =========================== SETTERS =========================
    */

    /**
     * Set the robot pose
     *
     * @param newPose The new robot pose
     */
    public void setRobotPose(Pose3d newPose) {
        this.robotPose = newPose;
    }

    /**
     * Set the robot velocities
     *
     * @param newVelocities The new robot velocities
     */
    public void setRobotVelocities(ChassisSpeeds newVelocities) {
        this.robotSpeeds = newVelocities;
    }
}
