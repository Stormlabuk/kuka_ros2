package application;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

public class TransmitToolCart implements Runnable {

    private DatagramSocket sendSocket;
    private InetAddress ipAddress;
    private int sendPort;
    private RobotDataTool robotData;
    private LBR robot;
    private Tool tool;
    private RoboticsAPIApplication app;
    private volatile boolean running = true;

    public TransmitToolCart(RobotDataTool robotData, LBR robot, String ipAddress, int sendPort,
                            RoboticsAPIApplication app, Tool tool) throws Exception {
        this.robotData = robotData;
        this.robot = robot;
        this.ipAddress = InetAddress.getByName(ipAddress);
        this.sendPort = sendPort;
        this.app = app;
        this.tool = tool;

        sendSocket = new DatagramSocket();
    }

    public void stop() {
        running = false;
        sendSocket.close();
    }

    @Override
    public void run() {
        try {
            while (running) {
                JointPosition jointPositions = robot.getCurrentJointPosition();
                double[] currentTorques = robot.getMeasuredTorque().getTorqueValues();

                JointPosition previousPositions = robotData.getPreviousJointPositions();
                double[] jointVelocities = new double[jointPositions.getAxisCount()];

                if (previousPositions != null) {
                    double dt = 0.001;
                    for (int i = 0; i < jointPositions.getAxisCount(); i++) {
                        jointVelocities[i] = (jointPositions.get(i) - previousPositions.get(i)) / dt;
                    }
                }

                robotData.setPreviousJointPositions(jointPositions);
                robotData.setCurrentJointPositions(jointPositions);
                robotData.setCurrentJointVelocities(jointVelocities);
                robotData.setCurrentTorques(currentTorques);

                boolean isMoving = robotData.isMoving();

                Frame flangePose = robot.getCurrentCartesianPosition(robot.getFlange());
                Frame toolPose = robot.getCurrentCartesianPosition(tool.getFrame("TCP"));

                robotData.setCurrentCartesianPose(flangePose);
                robotData.setCurrentToolCartesianPose(toolPose);

                byte[] udpBuffer = buildPacket(jointPositions, jointVelocities, currentTorques,
                                               isMoving, flangePose, toolPose);
                DatagramPacket packet = new DatagramPacket(udpBuffer, udpBuffer.length, ipAddress, sendPort);
                sendSocket.send(packet);

                ThreadUtil.milliSleep(1);
            }
        } catch (Exception e) {
            app.getLogger().error("Exception in TransmitToolCart: " + e.getMessage(), e);
        } finally {
            sendSocket.close();
        }
    }

    private byte[] buildPacket(JointPosition positions, double[] velocities, double[] torques,
                               boolean moving, Frame flangePose, Frame toolPose) {
        StringBuilder dataBuilder = new StringBuilder();

        for (int i = 0; i < positions.getAxisCount(); i++) {
            dataBuilder.append(round3(positions.get(i)));
            if (i < positions.getAxisCount() - 1) dataBuilder.append(",");
        }
        dataBuilder.append(";");

        for (int i = 0; i < velocities.length; i++) {
            dataBuilder.append(round3(velocities[i]));
            if (i < velocities.length - 1) dataBuilder.append(",");
        }
        dataBuilder.append(";");

        for (int i = 0; i < torques.length; i++) {
            dataBuilder.append(round3(torques[i]));
            if (i < torques.length - 1) dataBuilder.append(",");
        }
        dataBuilder.append(";");

        dataBuilder.append(moving ? "1" : "0").append(";");

        dataBuilder.append(round3(flangePose.getX())).append(",");
        dataBuilder.append(round3(flangePose.getY())).append(",");
        dataBuilder.append(round3(flangePose.getZ())).append(",");
        dataBuilder.append(round3(flangePose.getAlphaRad())).append(",");
        dataBuilder.append(round3(flangePose.getBetaRad())).append(",");
        dataBuilder.append(round3(flangePose.getGammaRad())).append(";");

        dataBuilder.append(round3(toolPose.getX())).append(",");
        dataBuilder.append(round3(toolPose.getY())).append(",");
        dataBuilder.append(round3(toolPose.getZ())).append(",");
        dataBuilder.append(round3(toolPose.getAlphaRad())).append(",");
        dataBuilder.append(round3(toolPose.getBetaRad())).append(",");
        dataBuilder.append(round3(toolPose.getGammaRad()));

        return dataBuilder.toString().getBytes();
    }

    private double round3(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }
}
