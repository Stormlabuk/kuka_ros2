package application;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;

/**
 * Shared data structure for thread-safe communication between components.
 */
public class RobotDataTool {
    private volatile JointPosition latestCommand = null;
    private volatile JointPosition currentJointPositions = null;
    private volatile JointPosition previousJointPositions = null;
    private volatile double[] currentJointVelocities = null;
    private volatile double[] currentTorques = null;
    private volatile boolean moving = false;
    private volatile boolean heartbeatLost = false;
    private volatile double[] velocityDemand = null;

    private volatile Frame latestCartesianCommand = null;

    private volatile Frame currentCartesianPose = null;        // Flange pose
    private volatile Frame currentToolCartesianPose = null;    // Tool TCP pose

    private volatile boolean paused = false;

    // Joint commands
    public synchronized JointPosition getLatestCommand() {
        return latestCommand;
    }

    public synchronized void setLatestCommand(JointPosition latestCommand) {
        this.latestCommand = latestCommand;
        updateVelocityDemandFromLatestCommand(latestCommand);
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

    public synchronized double[] getVelocityDemand() {
        return velocityDemand;
    }

    public synchronized void setVelocityDemand(double[] velocityDemand) {
        this.velocityDemand = velocityDemand;
    }

    private synchronized void updateVelocityDemandFromLatestCommand(JointPosition jointPos) {
        if (jointPos == null) {
            velocityDemand = null;
            return;
        }

        double[] temp = new double[jointPos.getAxisCount()];
        for (int i = 0; i < jointPos.getAxisCount(); i++) {
            temp[i] = jointPos.get(i);
        }
        velocityDemand = temp;
    }

    // Cartesian commands
    public synchronized Frame getLatestCartesianCommand() {
        return latestCartesianCommand;
    }

    public synchronized void setLatestCartesianCommand(Frame frame) {
        this.latestCartesianCommand = frame;
    }

    public synchronized void resetLatestCartesianCommand() {
        this.latestCartesianCommand = null;
    }

    // Flange Cartesian pose
    public synchronized Frame getCurrentCartesianPose() {
        return currentCartesianPose;
    }

    public synchronized void setCurrentCartesianPose(Frame pose) {
        this.currentCartesianPose = pose;
    }

    // Tool TCP Cartesian pose
    public synchronized Frame getCurrentToolCartesianPose() {
        return currentToolCartesianPose;
    }

    public synchronized void setCurrentToolCartesianPose(Frame pose) {
        this.currentToolCartesianPose = pose;
    }

    // Pause state
    public synchronized boolean isPaused() {
        return paused;
    }

    public synchronized void setPaused(boolean paused) {
        this.paused = paused;
    }
}