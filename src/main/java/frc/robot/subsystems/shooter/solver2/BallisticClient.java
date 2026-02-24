package frc.robot.subsystems.shooter.solver2;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.SocketChannel;
import java.net.StandardSocketOptions;

public class BallisticClient {
    private static final int MAGIC_NUMBER = 0xABCDABCD;

    // Message Type Enums (as constants)
    private static final byte SOLVE_REQUEST = 1;
    private static final byte SOLVE_RESPONSE = (byte) 129;

    // Struct sizes
    private static final int HEADER_SIZE = 9; // magic(4) + type(1) + length(4)
    private static final int SHOOTER_REQUEST_SIZE = 57;
    private static final int SHOOTER_RESPONSE_SIZE = 25;

    public static class ServerSolverParams {
        public final double relPos0X;
        public final double relPos0Y;
        public final double relPos0Z;
        public final double relVelX;
        public final double relVelY;
        public final double relVelZ;
        public final double v0;
        public final boolean archModeHigh;

        public ServerSolverParams(double relPos0X, double relPos0Y, double relPos0Z,
                double relVelX, double relVelY, double relVelZ,
                double v0, boolean archModeHigh) {
            this.relPos0X = relPos0X;
            this.relPos0Y = relPos0Y;
            this.relPos0Z = relPos0Z;
            this.relVelX = relVelX;
            this.relVelY = relVelY;
            this.relVelZ = relVelZ;
            this.v0 = v0;
            this.archModeHigh = archModeHigh;
        }
    }

    public static class ServerSolverResult {
        public final double azimuth;
        public final double elevation;
        public final double time;
        public final boolean success;

        public ServerSolverResult(double azimuth, double elevation, double time, boolean success) {
            this.azimuth = azimuth;
            this.elevation = elevation;
            this.time = time;
            this.success = success;
        }
    }

    private SocketChannel channel;

    public BallisticClient(String hostname, int port) throws IOException {
        channel = SocketChannel.open(new InetSocketAddress(hostname, port));
        channel.setOption(StandardSocketOptions.TCP_NODELAY, true);
        System.out.println("Connected to ballistic server at " + hostname + ":" + port);
    }

    public ServerSolverResult sendSolve(ServerSolverParams params) {
        ByteBuffer buffer = ByteBuffer.allocate(HEADER_SIZE + SHOOTER_REQUEST_SIZE);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putInt(MAGIC_NUMBER);
        buffer.put(SOLVE_REQUEST);
        buffer.putInt(SHOOTER_REQUEST_SIZE);
        // RelPos0
        buffer.putDouble(params.relPos0X);
        buffer.putDouble(params.relPos0Y);
        buffer.putDouble(params.relPos0Z);
        // RelVel
        buffer.putDouble(params.relVelX);
        buffer.putDouble(params.relVelY);
        buffer.putDouble(params.relVelZ);
        // v0
        buffer.putDouble(params.v0);
        buffer.put((byte) (params.archModeHigh ? 1 : 0)); // 0 archModeLow, 1 archModeHigh
        buffer.flip();

        long startTime = System.nanoTime();
        try {
            channel.write(buffer);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // System.out.println("Shooter request sent.");

        // Read response header
        ByteBuffer headerBuffer = ByteBuffer.allocate(HEADER_SIZE);
        try {
            readFully(channel, headerBuffer);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        headerBuffer.flip();
        headerBuffer.order(ByteOrder.LITTLE_ENDIAN);

        int magic = headerBuffer.getInt();
        byte type = headerBuffer.get();
        int length = headerBuffer.getInt();

        if (magic != MAGIC_NUMBER || type != SOLVE_RESPONSE || length != SHOOTER_RESPONSE_SIZE) {
            System.out.println("Error: Invalid header in solve response. " + magic + " " + type + " " + length);
            return new ServerSolverResult(0, 0, 0, false);
        }

        // Read response payload
        ByteBuffer payloadBuffer = ByteBuffer.allocate(SHOOTER_RESPONSE_SIZE);
        try {
            readFully(channel, payloadBuffer);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        long endTime = System.nanoTime();
        payloadBuffer.flip();
        payloadBuffer.order(ByteOrder.LITTLE_ENDIAN);

        double azimuth = payloadBuffer.getDouble();
        double elevation = payloadBuffer.getDouble();
        double time = payloadBuffer.getDouble();
        boolean success = payloadBuffer.get() != 0;

        double durationMs = (endTime - startTime) / 1_000_000.0;
        System.out.printf("Response received in %.2f ms:\n", durationMs);
        // System.out.printf(" Azimuth: %.2f\n", azimuth);
        // System.out.printf(" Elevation: %.2f\n", elevation);
        // System.out.printf(" Time: %.2f\n", time);
        // System.out.printf(" Success: %s\n", success);
        return new ServerSolverResult(azimuth, elevation, time, success);
    }

    private static void readFully(SocketChannel channel, ByteBuffer buffer) throws IOException {
        while (buffer.hasRemaining()) {
            int bytesRead = channel.read(buffer);
            if (bytesRead == -1) {
                // End of stream was reached unexpectedly.
                throw new IOException("Unexpected end of stream while trying to read " + buffer.capacity() + " bytes.");
            }
        }
    }
}
