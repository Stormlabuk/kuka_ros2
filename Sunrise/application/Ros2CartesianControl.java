package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;

import javax.inject.Inject;

public class Ros2CartesianControl extends RoboticsAPIApplication {

    @Inject
    private LBR robot;

    private RobotData robotData;
    private TransmitData transmitData;
    private ReceiveCartesian receiveData;
    private HeartBeat heartBeat;
    private Thread transmitThread;
    private Thread receiveThread;
    private Thread heartBeatThread;
    private String ipAddress = "172.31.1.150";
    private int sendPort = 5005;
    private int receivePort = 30300;
    private int heartbeatPort = 30301;
    private volatile boolean running = true;

    @Override
    public void initialize() {
        try {
            robotData = new RobotData();

            transmitData = new TransmitData(robotData, robot, ipAddress, sendPort, this);
            receiveData = new ReceiveCartesian(robotData, robot, receivePort, this);
            heartBeat = new HeartBeat(heartbeatPort, this, robotData);

            getLogger().info("Initialisation Complete");

        } catch (Exception e) {
            getLogger().error("Initialisation failed: " + e.getMessage(), e);
        }
    }

    @Override
    public void run() {
        transmitThread = new Thread(transmitData);
        receiveThread = new Thread(receiveData);
        heartBeatThread = new Thread(heartBeat);

        transmitThread.start();
        getLogger().info("Transmit thread started");

        receiveThread.start();
        getLogger().info("Receive thread started");

        heartBeatThread.start();
        getLogger().info("Heartbeat thread started");

        try {
            // Start SmartServo in joint space to initialize
            SmartServo smartServo = new SmartServo(robot.getCurrentJointPosition());
            smartServo.setJointVelocityRel(0.2);
            smartServo.setMinimumTrajectoryExecutionTime(0.02);

            IMotionContainer motionContainer = robot.moveAsync(smartServo);
            ISmartServoRuntime runtime = smartServo.getRuntime();

            while (running) {
                if (robotData.isHeartbeatLost()) {
                    getLogger().warn("Heartbeat lost, shutting down");
                    break;
                }

                Frame cartesianTarget = robotData.getLatestCartesianCommand();
                if (cartesianTarget != null) {
                    runtime.updateWithRealtimeSystem();
                    runtime.setDestination(cartesianTarget);
                    robotData.resetLatestCartesianCommand();
                }

                Thread.sleep(10);
            }

            motionContainer.cancel();

        } catch (Exception e) {
            getLogger().error("Exception in main run loop: " + e.getMessage(), e);

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
        transmitData.stop();
        receiveData.stop();
        heartBeat.stop();

        try {
            transmitThread.join();
            receiveThread.join();
            heartBeatThread.join();

        } catch (InterruptedException e) {
            getLogger().error("Thread interruption during shutdown: " + e.getMessage(), e);
            Thread.currentThread().interrupt();
        }

        getLogger().info("Shutdown Complete");
    }
}
