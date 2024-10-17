// This class runs a heartbeat monitor in its own thread.
// Once a connection is established, this class will check to see if there
// is a chnage in the heartbeat received over UDP in s specific timeframe.

package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.io.IOException;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class HeartBeat implements Runnable {

    // ***** Properties

    private RoboticsAPIApplication app;         // Reference to the main application for logging
    private volatile boolean connected = false;
    private volatile boolean heartbeat = false;
    private volatile boolean previousHeartbeat = false;
    private volatile boolean running = true;
    private volatile long previousTime = System.currentTimeMillis();
    private volatile long currentTime = System.currentTimeMillis();
    private final int hbTimeout = 200;          // Heartbeat timeout in milliseconds
    private final int checkInterval = 25;       // Check interval in milliseconds
    private DatagramSocket socket;
    private RobotData robotData;                // Reference to RobotData

    // ***** Methods

    // Constructor
    public HeartBeat(int port, RoboticsAPIApplication app, RobotData robotData) {
        this.app = app;
        this.robotData = robotData;
        try {
            // Set up the UDP socket
            socket = new DatagramSocket(port);
            socket.setSoTimeout(1000); // 1-second timeout for receive operations
        } catch (SocketException e) {
            app.getLogger().error("SocketException in HeartBeat constructor: " + e.getMessage(), e);
        }
    }

    // Override the run method of the Runnable interface
    @Override
    public void run() {
    	// Buffer size for UDP packets
        byte[] buffer = new byte[1024];          
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        try {
            while (running) {
                if (!connected) {
                    // If not connected, set previousTime = now and then check if connected
                    previousTime = System.currentTimeMillis();

                    // Check to see if connected
                    if (isPacketAvailable(packet)) {
                        // Set connected to true
                        connected = true;
                        app.getLogger().info("Heartbeat Connection Established");

                        // Read the initial heartbeat value
                        heartbeat = readHeartbeat(packet);
                        previousHeartbeat = heartbeat;
                    }
                } else {
                    // If connected, read heartbeat and check for changes
                    currentTime = System.currentTimeMillis();

                    // Read heartbeat
                    if (isPacketAvailable(packet)) {
                        heartbeat = readHeartbeat(packet);

                        // Check for change in heartbeat
                        if (previousHeartbeat != heartbeat) {
                            // Update previousTime
                            previousTime = currentTime;
                        } else {
                            // No change in heartbeat, check if timeout has been exceeded
                            long timeDifference = currentTime - previousTime;
                            if (timeDifference > hbTimeout) {
                                app.getLogger().info("Heartbeat timeout exceeded. Stopping monitoring.");
                                robotData.setHeartbeatLost(true); // Signal the main application
                                running = false;                  // Stop the monitoring thread
                            }
                        }

                        // Update previousHeartbeat
                        previousHeartbeat = heartbeat;
                    } else {
                        // No packet received, check if timeout has been exceeded
                        long timeDifference = currentTime - previousTime;
                        if (timeDifference > hbTimeout) {
                            app.getLogger().info("Heartbeat timeout exceeded due to no data. Stopping monitoring.");
                            robotData.setHeartbeatLost(true); // Signal the main application
                            running = false;                  // Stop the monitoring thread
                        }
                    }
                }

                // Wait for the specified check interval
                try {
                    Thread.sleep(checkInterval);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    app.getLogger().error("Heartbeat thread interrupted: " + e.getMessage(), e);
                }
            }
        } finally {
            // Clean up resources
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
            app.getLogger().info("Heartbeat thread terminated.");
        }
    }

    private boolean isPacketAvailable(DatagramPacket packet) {
        try {
            socket.receive(packet); // This will block until a packet is received or timeout occurs
            return true;
        } catch (SocketTimeoutException e) {
            // No packet received within the timeout period
            return false;
        } catch (IOException e) {
            app.getLogger().error("IOException in HeartBeat: " + e.getMessage(), e);
            return false;
        }
    }

    private boolean readHeartbeat(DatagramPacket packet) {
        String data = new String(packet.getData(), 0, packet.getLength()).trim();
        return Boolean.parseBoolean(data); // Assuming the data is a boolean string "true" or "false"
    }

    public void stop() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close(); // Close the socket to unblock any blocking receive calls
        }
    }
}
