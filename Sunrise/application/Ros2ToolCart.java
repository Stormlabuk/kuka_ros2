package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;

import javax.inject.Inject;

public class Ros2ToolCart extends RoboticsAPIApplication {

    @Inject
    private LBR robot;

    @Inject
    private Tool LoadCell;

    private RobotDataTool robotData;
    private TransmitToolCart transmitData;
    private ReceiveToolCart receiveData;
    private HeartBeatTool heartBeat;
    private PausedStatusTool pauseStatus;

    private Thread transmitThread;
    private Thread receiveThread;
    private Thread heartBeatThread;
    private Thread pauseThread;

    private String ipAddress = "172.31.1.150";
    private int sendPort = 5005;
    private int receivePort = 30300;
    private int heartbeatPort = 30301;
    private int pausePort = 30002;
    private volatile boolean running = true;

    @Override
    public void initialize() {
        try {
            LoadCell.attachTo(robot.getFlange());
            robotData = new RobotDataTool();

            transmitData = new TransmitToolCart(robotData, robot, ipAddress, sendPort, this, LoadCell);
            receiveData = new ReceiveToolCart(robotData, robot, receivePort, this);
            heartBeat = new HeartBeatTool(heartbeatPort, this, robotData);
            pauseStatus = new PausedStatusTool(pausePort, this, robotData);

            getLogger().info("Initialization complete.");
        } catch (Exception e) {
            getLogger().error("Initialization failed: " + e.getMessage(), e);
        }
    }

    @Override
    public void run() {
        transmitThread = new Thread(transmitData);
        receiveThread = new Thread(receiveData);
        heartBeatThread = new Thread(heartBeat);
        pauseThread = new Thread(pauseStatus);

        transmitThread.start();
        receiveThread.start();
        heartBeatThread.start();
        pauseThread.start();

        try {
            SmartServo smartServo = new SmartServo(robot.getCurrentJointPosition());
            smartServo.setJointVelocityRel(0.2);
            smartServo.setMinimumTrajectoryExecutionTime(0.02);

            IMotionContainer motionContainer = robot.moveAsync(smartServo);
            ISmartServoRuntime runtime = smartServo.getRuntime();

            Frame initialPose = robot.getCurrentCartesianPosition(LoadCell.getFrame("TCP"));
            robotData.setLatestCartesianCommand(initialPose);
            getLogger().info("Initial TCP pose set: " + initialPose);

            while (running) {
                if (robotData.isHeartbeatLost()) {
                    getLogger().warn("Heartbeat lost. Stopping motion.");
                    break;
                }

                Frame cartesianTarget = robotData.getLatestCartesianCommand();
                if (cartesianTarget != null) {
                    runtime.setDestination(cartesianTarget);
                }

                Frame currentPose = robot.getCurrentCartesianPosition(LoadCell.getFrame("TCP"));
                robotData.setCurrentToolCartesianPose(currentPose);

                Thread.sleep(1);
            }

            motionContainer.cancel();
        } catch (Exception e) {
            getLogger().error("Run exception: " + e.getMessage(), e);
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
        pauseStatus.stop();

        try {
            transmitThread.join();
            receiveThread.join();
            heartBeatThread.join();
            pauseThread.join();
        } catch (InterruptedException e) {
            getLogger().error("Shutdown interrupted: " + e.getMessage(), e);
            Thread.currentThread().interrupt();
        }

        getLogger().info("Shutdown complete.");
    }
}
