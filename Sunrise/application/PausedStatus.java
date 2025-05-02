package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.io.IOException;
import com.kuka.roboticsAPI.geometricModel.Frame;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class PausedStatus implements Runnable {

    private final RoboticsAPIApplication app;
    private final RobotData robotData;
    private volatile boolean running = true;
    private DatagramSocket socket;

    // Stores pose at time of pause
    private volatile Frame savedPoseAtPause = null;

    public PausedStatus(int port, RoboticsAPIApplication app, RobotData robotData) {
        this.app = app;
        this.robotData = robotData;

        try {
            socket = new DatagramSocket(port);
            socket.setSoTimeout(10);  // Low timeout for responsiveness
            app.getLogger().info("PausedStatus listener created on port " + port);
        } catch (SocketException e) {
            app.getLogger().error("SocketException in PausedStatus constructor: " + e.getMessage(), e);
        }
    }

    @Override
    public void run() {
        byte[] buffer = new byte[1024];
        DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

        app.getLogger().info("PausedStatus thread started");

        try {
            while (running) {
                if (isPacketAvailable(packet)) {
                    String data = new String(packet.getData(), 0, packet.getLength()).trim().toLowerCase();
                    app.getLogger().info("PausedStatus received: " + data);

                    if (data.equals("pause")) {
                        savedPoseAtPause = robotData.getLatestCartesianCommand();
                        Frame frozenPose = robotData.getCurrentCartesianPose();
                        robotData.setLatestCartesianCommand(frozenPose);
                        robotData.setPaused(true);
                        app.getLogger().info("Motion PAUSED. Holding position: " + frozenPose);
                    } else if (data.equals("resume")) {
                        if (savedPoseAtPause != null) {
                            robotData.setLatestCartesianCommand(savedPoseAtPause);
                            app.getLogger().info("Resuming toward saved target: " + savedPoseAtPause);
                        } else {
                            app.getLogger().warn("Resume received but no saved pose was found.");
                        }
                        robotData.setPaused(false);
                    } else {
                        app.getLogger().warn("Unknown pause command received: " + data);
                    }
                }
            }
        } finally {
            if (socket != null && !socket.isClosed()) {
                socket.close();
            }
            app.getLogger().info("PausedStatus thread terminated.");
        }
    }

    private boolean isPacketAvailable(DatagramPacket packet) {
        try {
            socket.receive(packet);
            return true;
        } catch (SocketTimeoutException e) {
            return false;
        } catch (IOException e) {
            app.getLogger().error("IOException in PausedStatus: " + e.getMessage(), e);
            return false;
        }
    }

    public void stop() {
        running = false;
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }
}
