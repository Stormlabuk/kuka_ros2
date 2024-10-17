// This class continuously listens for UDP incoming transmissions
// Each UDP transmission is parsed and the latestCommand in the 
// class RobotData is updated, to be handled by the main code

package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class ReceiveData implements Runnable {
	
    //******* properties
	
    private DatagramSocket receiveSocket; 		// Socket to receive UDP packets
    private RobotData robotData;          		// Shared data container between threads
    private LBR robot;                    		// Reference to the robot
    private RoboticsAPIApplication app;   		// Reference to the main application for logging
    private volatile boolean running = true;  	// volatile to ensure thread safety, since this is updated by the main class
    
    //******** methods

    // Constructor method
    public ReceiveData(RobotData robotData, LBR robot, int receivePort, RoboticsAPIApplication app) throws Exception {
        this.robotData = robotData;
        this.robot = robot;
        this.app = app;

        // new socket to receive UDP packets on the specified port
        receiveSocket = new DatagramSocket(receivePort);
        
        // Prevent blocking indefinitely, 1000ms timeout
        receiveSocket.setSoTimeout(1000); 
    }

    // Stop method to signal the thread to stop running
    public void stop() {
        running = false;
        receiveSocket.close(); // Close the socket to release resources and unblock any blocking calls
    }

    // Override the run method of the Runnable interface
    @Override
    public void run() {
        byte[] buffer = new byte[1024]; // Buffer to store incoming data
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length); // Datagram packet to receive data

        while (running) {
            try {
            	
            	// Attempt to read from the socket (this will block until data is received or timeout occurs)
                receiveSocket.receive(packet);
                String data = new String(packet.getData(), 0, packet.getLength());
                
                // Parse the data to get joint positions
                String[] jointStrings = data.split(",");
                
                if (jointStrings.length == robot.getJointCount()) {
                	
                	// Convert the string array into a double array for each joint
                    double[] jointValues = new double[robot.getJointCount()];
                    for (int i = 0; i < robot.getJointCount(); i++) {
                        jointValues[i] = Double.parseDouble(jointStrings[i]);
                    }
                    
                    // update the latest command in shared class robotData
                    JointPosition latestCommand = new JointPosition(jointValues);
                    robotData.setLatestCommand(latestCommand);
                } else {
                	
                	// packet not valid (i.e., not received commands for all joints)
                    app.getLogger().warn("Received invalid joint command: " + data);
                }
            } catch (java.net.SocketTimeoutException e) {
            	
                // Timeout occurred, continue listening
            	
            } catch (SocketException e) {
            	
                // Socket was closed, exit the loop if running is false
                if (!running) {
                    break;
                } else {
                    app.getLogger().error("SocketException in ReceiveData: " + e.getMessage(), e);
                }
                
            } catch (Exception e) {
            	
                app.getLogger().error("Exception in ReceiveData: " + e.getMessage(), e);
            }
        }
        
        
    }
}
