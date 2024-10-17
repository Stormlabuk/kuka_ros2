// This class acts as a shared data container between threads.
//
// Volatile properties ensure thread safety.
//
// Synchronised getters and setters ensure thread safety when accessing
// shared variables.

package application;

import com.kuka.roboticsAPI.deviceModel.JointPosition;

public class RobotData {

    // ***** Properties

    private volatile JointPosition latestCommand = null;
    private volatile JointPosition currentJointPositions = null;
    private volatile JointPosition previousJointPositions = null;
    private volatile double[] currentJointVelocities = null;
    private volatile double[] currentTorques = null;
    private volatile boolean moving = false;
    private volatile boolean heartbeatLost = false;

    // ***** Methods

    public synchronized JointPosition getLatestCommand() {
        return latestCommand;
    }

    public synchronized void setLatestCommand(JointPosition latestCommand) {
        this.latestCommand = latestCommand;
    }

    public synchronized JointPosition getCurrentJointPositions() {
        return currentJointPositions;
    }

    public synchronized void setCurrentJointPositions(JointPosition currentJointPositions) {
        this.currentJointPositions = currentJointPositions;
    }

    public synchronized JointPosition getPreviousJointPositions() {
        return previousJointPositions;
    }

    public synchronized void setPreviousJointPositions(JointPosition previousJointPositions) {
        this.previousJointPositions = previousJointPositions;
    }

    public synchronized double[] getCurrentJointVelocities() {
        return currentJointVelocities;
    }

    public synchronized void setCurrentJointVelocities(double[] currentJointVelocities) {
        this.currentJointVelocities = currentJointVelocities;
    }

    public synchronized double[] getCurrentTorques() {
        return currentTorques;
    }

    public synchronized void setCurrentTorques(double[] currentTorques) {
        this.currentTorques = currentTorques;
    }

    public synchronized boolean isMoving() {
        return moving;
    }

    public synchronized void setMoving(boolean moving) {
        this.moving = moving;
    }

    public synchronized boolean isHeartbeatLost() {
        return heartbeatLost;
    }

    public synchronized void setHeartbeatLost(boolean heartbeatLost) {
        this.heartbeatLost = heartbeatLost;
    }
}
