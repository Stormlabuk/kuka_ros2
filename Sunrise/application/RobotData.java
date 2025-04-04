package application;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;

/**
 * This class acts as a shared data container between threads.
 *
 * Volatile properties ensure thread safety.
 * Synchronized getters and setters ensure thread safety when accessing shared variables.
 */
public class RobotData {


    private volatile JointPosition latestCommand = null;
    private volatile JointPosition currentJointPositions = null;
    private volatile JointPosition previousJointPositions = null;
    private volatile double[] currentJointVelocities = null;
    private volatile double[] currentTorques = null;
    private volatile boolean moving = false;
    private volatile boolean heartbeatLost = false;
    private volatile double[] velocityDemand = null;
    private volatile Frame latestCartesianCommand = null;

    public synchronized JointPosition getLatestCommand() {
        return latestCommand;
    }


    //When setting latestCommand, also update velocityDemand by converting the JointPosition to a double[].
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

    // for Cartesian commands
    public synchronized Frame getLatestCartesianCommand() {
        return latestCartesianCommand;
    }

    public synchronized void setLatestCartesianCommand(Frame frame) {
        this.latestCartesianCommand = frame;
    }
    
    public synchronized void resetLatestCartesianCommand() {
        this.latestCartesianCommand = null;
    }
}
