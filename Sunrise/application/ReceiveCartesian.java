// This class continuously listens for UDP incoming transmissions
// Each UDP transmission is parsed and the latestCommand in the 
// class RobotData is updated, to be handled by the main code

package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class ReceiveCartesian implements Runnable {

	
	//******* properties
	
    private DatagramSocket receiveSocket;
    private RobotData robotData;
    private LBR robot;				// not actually needed, but left in for posterity
    private RoboticsAPIApplication app;
    private volatile boolean running = true;
    
    //******** methods

    // Constructor method
    public ReceiveCartesian(RobotData robotData, LBR robot, int receivePort, RoboticsAPIApplication app) throws Exception {
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
        receiveSocket.close();	 // Close the socket to release resources and unblock any blocking calls
    }

    // Override the run method of the Runnable interface
    @Override
    public void run() {
        byte[] buffer = new byte[1024];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        while (running) {
            try {
                receiveSocket.receive(packet);
                String data = new String(packet.getData(), 0, packet.getLength()).trim();

                if (data.startsWith("C:")) {
                    String[] parts = data.substring(2).split(",");
                    if (parts.length == 6) {
                        double x = Double.parseDouble(parts[0]);
                        double y = Double.parseDouble(parts[1]);
                        double z = Double.parseDouble(parts[2]);
                        double a = Double.parseDouble(parts[3]);
                        double b = Double.parseDouble(parts[4]);
                        double c = Double.parseDouble(parts[5]);

                        Frame command = new Frame(x, y, z, a, b, c);
                        robotData.setLatestCartesianCommand(command);
                    } else {
                        app.getLogger().warn("Invalid Cartesian command format: " + data);
                    }
                } else {
                    app.getLogger().warn("Received non-Cartesian command: " + data);
                }

            } catch (java.net.SocketTimeoutException e) {
                // Timeout — continue listening
            } catch (SocketException e) {
                if (!running) break;
                app.getLogger().error("SocketException in ReceiveData: " + e.getMessage(), e);
            } catch (Exception e) {
                app.getLogger().error("Exception in ReceiveData: " + e.getMessage(), e);
            }
        }
    }
}
