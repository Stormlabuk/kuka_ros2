package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

import javax.inject.Inject;

public class Ros2PositionControl extends RoboticsAPIApplication {

    // ***** Properties

    @Inject
    private LBR robot;                                  // Reference to the robot
    private RobotData robotData;                        // Shared data container between threads
    private TransmitData transmitData;                  // Thread for transmitting data
    private ReceiveData receiveData;                    // Thread for receiving data
    private HeartBeat heartBeat;                        // Thread to monitor connection heartbeat
    private Thread transmitThread;                      // Thread object for transmitting data
    private Thread receiveThread;                       // Thread object for receiving data
    private Thread heartBeatThread;                     // Thread object for heartbeat monitoring
    private String ipAddress = "172.31.1.150";          // PC IP address
    private int sendPort = 5005;                        // Port to send data
    private int receivePort = 30300;                    // Port to receive commands
    private int heartbeatPort = 30301;                  // Port to receive heartbeat
    private volatile boolean running = true;            // Control flag for the main loop

    // ***** Methods

    // Override the run method of the init interface
    @Override
    public void initialize() {
        try {
            // Initialise the shared RobotData object
            robotData = new RobotData();

            // Initialise TransmitData, ReceiveData, and HeartBeat
            transmitData = new TransmitData(robotData, robot, ipAddress, sendPort, this);
            receiveData = new ReceiveData(robotData, robot, receivePort, this);
            heartBeat = new HeartBeat(heartbeatPort, this, robotData); // Pass robotData

            getLogger().info("Initialisation Complete");

        } catch (Exception e) {
            getLogger().error("Initialisation failed: " + e.getMessage(), e);
        }
    }

    // Override the run method of the Runnable interface
    @Override
    public void run() {
        // Start the threads for transmitting, receiving data, and heartbeat monitoring
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
            while (running) {
                // Check if the heartbeat is lost
                if (robotData.isHeartbeatLost()) {
                    getLogger().info("Heartbeat lost, shutting down application.");
                    running = false;
                    break;
                }

                // Check if a new command has been received
                JointPosition latestCommand = robotData.getLatestCommand();

                if (latestCommand != null) {

                    // Move the robot to the new joint positions asynchronously
                    IMotionContainer motion = robot.moveAsync(BasicMotions.ptp(latestCommand));

                    // Reset the command immediately
                    robotData.setLatestCommand(null);

                    // Update moving status
                    robotData.setMoving(true);

                    // Wait for the motion to complete
                    motion.await();

                    // Update moving status
                    robotData.setMoving(false);
                }

                // Sleep to prevent busy waiting
                Thread.sleep(10);
            }

        } catch (Exception e) {
            getLogger().error("Exception in main run loop: " + e.getMessage(), e);

        } finally {
            // Graceful shutdown of threads and resources
            shutdown();
        }
    }

    // Override the kill method of the Runnable interface
    @Override
    public void dispose() {
        // Ensure resources are cleaned up on application exit
        shutdown();
        super.dispose();
    }

    // method for safe sutdown
    private void shutdown() {
        // Stop the threads
        transmitData.stop();
        receiveData.stop();
        heartBeat.stop();

        try {
            // Wait for the threads to finish
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
