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
    private TransmitCartesian transmitData;
    private ReceiveCartesian receiveData;
    private HeartBeat heartBeat;
    private PausedStatus pauseStatus;

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
            robotData = new RobotData();

            transmitData = new TransmitCartesian(robotData, robot, ipAddress, sendPort, this);
            receiveData = new ReceiveCartesian(robotData, robot, receivePort, this);
            heartBeat = new HeartBeat(heartbeatPort, this, robotData);
            pauseStatus = new PausedStatus(pausePort, this, robotData);

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
        pauseThread = new Thread(pauseStatus);

        transmitThread.start();
        getLogger().info("Transmit thread started");

        receiveThread.start();
        getLogger().info("Receive thread started");

        heartBeatThread.start();
        getLogger().info("Heartbeat thread started");

        pauseThread.start();
        getLogger().info("PauseStatus thread started");

        try {
            SmartServo smartServo = new SmartServo(robot.getCurrentJointPosition());
            smartServo.setJointVelocityRel(0.2);
            smartServo.setMinimumTrajectoryExecutionTime(0.02);

            IMotionContainer motionContainer = robot.moveAsync(smartServo);
            ISmartServoRuntime runtime = smartServo.getRuntime();

            // set initial Cartesian target to current position
            Frame initialPose = robot.getCurrentCartesianPosition(robot.getFlange());
            robotData.setLatestCartesianCommand(initialPose);
            getLogger().info("Initial Cartesian pose set as target: " + initialPose.toString());

            while (running) {
                if (robotData.isHeartbeatLost()) {
                    getLogger().warn("Heartbeat lost, shutting down");
                    break;
                }

                Frame cartesianTarget = robotData.getLatestCartesianCommand();

               // if (!robotData.isPaused()) {
                    //runtime.updateWithRealtimeSystem();

                    if (cartesianTarget != null) {
                        runtime.setDestination(cartesianTarget);
                        //robotData.resetLatestCartesianCommand();
                    } //else {
                        
                        //Frame fallbackPose = robot.getCurrentCartesianPosition(robot.getFlange());
                        //runtime.setDestination(fallbackPose);
                    //}
                //}

                Frame currentPose = robot.getCurrentCartesianPosition(robot.getFlange());
                robotData.setCurrentCartesianPose(currentPose);

                Thread.sleep(1);
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
        pauseStatus.stop();

        try {
            transmitThread.join();
            receiveThread.join();
            heartBeatThread.join();
            pauseThread.join();
        } catch (InterruptedException e) {
            getLogger().error("Thread interruption during shutdown: " + e.getMessage(), e);
            Thread.currentThread().interrupt();
        }

        getLogger().info("Shutdown Complete");
    }
}
