// Runs at 1ms. Wraps joint positions, velocities and torques along with robot
// move state into a JSON format and transmits via UDP

package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class TransmitData implements Runnable {
	
	// ***** properties

    private DatagramSocket sendSocket;          // Socket to send UDP packets
    private InetAddress ipAddress;              // IP address to send data to
    private int sendPort;                       // Port to send data to
    private RobotData robotData;                // Shared data container between threads
    private LBR robot;                          // Reference to the robot
    private RoboticsAPIApplication app;         // Reference to the main application for logging
    private volatile boolean running = true;    // Control flag for the thread

    // ***** methods
    
    // Constructor
    public TransmitData(RobotData robotData, LBR robot, String ipAddress, int sendPort, RoboticsAPIApplication app) throws Exception {
        this.robotData = robotData;
        this.robot = robot;
        this.ipAddress = InetAddress.getByName(ipAddress);
        this.sendPort = sendPort;
        this.app = app;

        sendSocket = new DatagramSocket(); // Create a new socket for sending data

    }

    // Stop method
    public void stop() {
        running = false;
        sendSocket.close(); // Close the socket to release resources
    }

    // Override the run method of the Runnable interface
    @Override
    public void run() {
        try {
            

            while (running) {

                // Get current joint positions from the robot
                JointPosition jointPositions = robot.getCurrentJointPosition();

                // Get current torques measured by the robot
                double[] currentTorques = robot.getMeasuredTorque().getTorqueValues();

                // Compute joint velocities based on the difference between current and previous positions
                JointPosition previousPositions = robotData.getPreviousJointPositions();
                double[] jointVelocities = new double[jointPositions.getAxisCount()];

                if (previousPositions != null) {
                    double dt = 0.001; // 1ms interval
                    for (int i = 0; i < jointPositions.getAxisCount(); i++) {
                        jointVelocities[i] = (jointPositions.get(i) - previousPositions.get(i)) / dt;
                    }
                }

                // Update previous joint positions in robotData
                robotData.setPreviousJointPositions(jointPositions);

                // Update current joint positions, velocities, and torques in robotData
                robotData.setCurrentJointPositions(jointPositions);
                robotData.setCurrentJointVelocities(jointVelocities);
                robotData.setCurrentTorques(currentTorques);

                // Get moving status of the robot
                boolean isMoving = robotData.isMoving();

                // Create data object containing joint positions, velocities, torques, and moving status
                JointData jointData = new JointData(jointPositions, jointVelocities, currentTorques, isMoving);

                // Send data via UDP
                byte[] buffer = jointData.toByteArray();
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length, ipAddress, sendPort);
                sendSocket.send(packet);

                // Sleep 10ms
                ThreadUtil.milliSleep(1);
            }
        } catch (Exception e) {
        	
            // Log any exceptions that occur during transmission
            app.getLogger().error("Exception in TransmitData: " + e.getMessage(), e);
        } finally {
        	
            sendSocket.close(); // Ensure the socket is closed
        }
    }

    // helper class for data transmission
    public static class JointData {
        public double[] positions;  // Current joint positions
        public double[] velocities; // Current joint velocities
        public double[] torques;    // Current joint torques
        public boolean moving;      // Robot moving status

        // Constructor to initialise all fields
        public JointData(JointPosition positions, double[] velocities, double[] torques, boolean moving) {
            int n = positions.getAxisCount();
            this.positions = new double[n];
            for (int i = 0; i < n; i++) {
                this.positions[i] = positions.get(i);
            }
            this.velocities = velocities;
            this.torques = torques;
            this.moving = moving;
        }

        // Helper method to round a double to 3 decimal places
        private double roundToThreeDecimals(double value) {
            return Math.round(value * 1000.0) / 1000.0;
        }

        // Method to convert data to byte array for UDP transmission
        public byte[] toByteArray() {
            StringBuilder dataBuilder = new StringBuilder();

            // Append positions, rounding to 3 decimal places
            for (int i = 0; i < positions.length; i++) {
                dataBuilder.append(roundToThreeDecimals(positions[i]));  // Round to 3 decimal places
                if (i < positions.length - 1) {
                    dataBuilder.append(",");
                }
            }
            dataBuilder.append(";");  // Separator between data sections

            // Append velocities, rounding to 3 decimal places
            for (int i = 0; i < velocities.length; i++) {
                dataBuilder.append(roundToThreeDecimals(velocities[i]));  // Round to 3 decimal places
                if (i < velocities.length - 1) {
                    dataBuilder.append(",");
                }
            }
            dataBuilder.append(";");

            // Append torques, rounding to 3 decimal places
            for (int i = 0; i < torques.length; i++) {
                dataBuilder.append(roundToThreeDecimals(torques[i]));  // Round to 3 decimal places
                if (i < torques.length - 1) {
                    dataBuilder.append(",");
                }
            }
            dataBuilder.append(";");

            // Append moving status
            dataBuilder.append(moving ? "1" : "0");

            // Convert to byte array
            return dataBuilder.toString().getBytes();
        }
    }

}
