package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;

import javax.inject.Inject;
import java.util.Arrays;

/**
 * Velocity-like control using SmartServo by incrementally adjusting joint positions
 * based on desired velocity (interpreted from latestCommand in RobotData).
 */
public class Ros2VelocityControl extends RoboticsAPIApplication {

    @Inject
    private LBR robot;

    private RobotData robotData;
    private TransmitData transmitData;
    private ReceiveData receiveData;
    private HeartBeat heartBeat;
    private Thread transmitThread;
    private Thread receiveThread;
    private Thread heartBeatThread;

    private String ipAddress = "172.31.1.150";
    private int sendPort = 5005;
    private int receivePort = 30300;
    private int heartbeatPort = 30301;
    private volatile boolean running = true;

    private final double Kp = 30;          // Proportional gain
    private final double loopDt = 0.001;    // 1 ms loop

    @Override
    public void initialize() {
        try {
            robotData = new RobotData();
            transmitData = new TransmitData(robotData, robot, ipAddress, sendPort, this);
            receiveData = new ReceiveData(robotData, robot, receivePort, this);
            heartBeat = new HeartBeat(heartbeatPort, this, robotData);
        } catch (Exception e) {
            getLogger().error("Initialization failed: " + e.getMessage(), e);
        }
    }

    @Override
    public void run() {
        transmitThread = new Thread(transmitData);
        receiveThread = new Thread(receiveData);
        heartBeatThread = new Thread(heartBeat);
        transmitThread.start();
        receiveThread.start();
        heartBeatThread.start();

        try {
            JointPosition initialPosition = robot.getCurrentJointPosition();
            SmartServo smartServo = new SmartServo(initialPosition);
            double RelVel = 1.0;
            smartServo.setJointVelocityRel(RelVel);
            getLogger().info(String.valueOf(RelVel));
            robot.moveAsync(smartServo);
            ISmartServoRuntime runtime = smartServo.getRuntime();
            

            while (running) {
                if (robotData.isHeartbeatLost()) {
                    getLogger().info("Heartbeat lost, shutting down.");
                    running = false;
                    break;
                }

                double[] desiredVel = robotData.getVelocityDemand();
                double[] actualVel = robotData.getCurrentJointVelocities();

                if (desiredVel != null && actualVel != null) {
                    double[] velocityError = new double[robot.getJointCount()];
                    double[] newPos = new double[robot.getJointCount()];
                    JointPosition currentPos = robot.getCurrentJointPosition();

                    for (int i = 0; i < robot.getJointCount(); i++) {
                        velocityError[i] = desiredVel[i] - actualVel[i];
                        double delta = Kp * velocityError[i] * loopDt;

                        // Clamp step size
                        double maxStep = 0.1;
                        delta = Math.max(-maxStep, Math.min(maxStep, delta));

                        newPos[i] = currentPos.get(i) + delta;
                    }

                    robotData.setMoving(true);
                    runtime.setDestination(new JointPosition(newPos));

                }

                Thread.sleep((long) (loopDt*1000));
            }
        } catch (Exception e) {
            getLogger().error("Main loop exception: " + e.getMessage(), e);
        } finally {
            shutdown();
        }
    }

    @Override
    public void dispose() {
        shutdown();
        super.dispose();
    }

    private void shutdown() {
        try {
            transmitData.stop();
            receiveData.stop();
            heartBeat.stop();
            transmitThread.join();
            receiveThread.join();
            heartBeatThread.join();
        } catch (InterruptedException e) {
            getLogger().error("Shutdown interruption: " + e.getMessage(), e);
            Thread.currentThread().interrupt();
        }
    }
}
