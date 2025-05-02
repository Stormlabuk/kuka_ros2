package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.io.IOException;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class HeartBeatTool implements Runnable {

    private RoboticsAPIApplication app;
    private volatile boolean connected = false;
    private volatile boolean heartbeat = false;
    private volatile boolean previousHeartbeat = false;
    private volatile boolean firstToggleReceived = false;
    private volatile boolean running = true;

    private volatile long previousTime = System.currentTimeMillis();
    private volatile long currentTime = System.currentTimeMillis();

    private final int hbTimeout = 200;      // milliseconds
    private final int checkInterval = 25;   // milliseconds

    private DatagramSocket socket;
    private RobotDataTool robotData;

    public HeartBeatTool(int port, RoboticsAPIApplication app, RobotDataTool robotData) {
        this.app = app;
        this.robotData = robotData;
        try {
            socket = new DatagramSocket(port);
            socket.setSoTimeout(1000);
        } catch (SocketException e) {
            app.getLogger().error("SocketException in HeartBeat constructor: " + e.getMessage(), e);
        }
    }

    @Override
    public void run() {
        byte[] buffer = new byte[1024];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        try {
            while (running) {
                if (!connected) {
                    previousTime = System.currentTimeMillis();

                    if (isPacketAvailable(packet)) {
                        connected = true;
                        app.getLogger().info("Heartbeat connection established");
                        heartbeat = readHeartbeat(packet);
                        previousHeartbeat = heartbeat;
                    }
                } else {
                    currentTime = System.currentTimeMillis();

                    if (isPacketAvailable(packet)) {
                        heartbeat = readHeartbeat(packet);

                        if (!firstToggleReceived && previousHeartbeat != heartbeat) {
                            firstToggleReceived = true;
                            app.getLogger().info("First heartbeat toggle received — starting timeout checks");
                            previousTime = currentTime;
                        } else if (firstToggleReceived && previousHeartbeat != heartbeat) {
                            previousTime = currentTime;
                        } else if (firstToggleReceived) {
                            long timeDiff = currentTime - previousTime;
                            if (timeDiff > hbTimeout) {
                                app.getLogger().warn("Heartbeat timeout exceeded — stopping monitoring.");
                                robotData.setHeartbeatLost(true);
                                running = false;
                            }
                        }

                        previousHeartbeat = heartbeat;
                    } else if (firstToggleReceived) {
                        long timeDiff = currentTime - previousTime;
                        if (timeDiff > hbTimeout) {
                            app.getLogger().warn("Heartbeat timeout due to no data — stopping monitoring.");
                            robotData.setHeartbeatLost(true);
                            running = false;
                        }
                    }
                }

                try {
                    Thread.sleep(checkInterval);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    app.getLogger().error("Heartbeat thread interrupted: " + e.getMessage(), e);
                }
            }
        } finally {
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
            app.getLogger().info("Heartbeat thread terminated.");
        }
    }

    private boolean isPacketAvailable(DatagramPacket packet) {
        try {
            socket.receive(packet);
            return true;
        } catch (SocketTimeoutException e) {
            return false;
        } catch (IOException e) {
            app.getLogger().error("IOException in HeartBeat: " + e.getMessage(), e);
            return false;
        }
    }

    private boolean readHeartbeat(DatagramPacket packet) {
        String data = new String(packet.getData(), 0, packet.getLength()).trim();
        return Boolean.parseBoolean(data);
    }

    public void stop() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }
}
