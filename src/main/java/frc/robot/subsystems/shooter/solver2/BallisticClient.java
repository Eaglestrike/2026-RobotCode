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
        this(hostname, port, 2000);
    }

    public BallisticClient(String hostname, int port, int timeoutMs) throws IOException {
        channel = SocketChannel.open();
        channel.setOption(StandardSocketOptions.TCP_NODELAY, true);
        channel.socket().connect(new InetSocketAddress(hostname, port), timeoutMs);
        System.out.println("Connected to ballistic server at " + hostname + ":" + port);
    }

    public ServerSolverResult sendSolve(ServerSolverParams params) throws IOException {
        ByteBuffer buffer = ByteBuffer.allocate(HEADER_SIZE + SHOOTER_REQUEST_SIZE);
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.putInt(MAGIC_NUMBER);
        buffer.put(SOLVE_REQUEST);
        buffer.putInt(SHOOTER_REQUEST_SIZE);
        buffer.putDouble(params.relPos0X);
        buffer.putDouble(params.relPos0Y);
        buffer.putDouble(params.relPos0Z);
        buffer.putDouble(params.relVelX);
        buffer.putDouble(params.relVelY);
        buffer.putDouble(params.relVelZ);
        buffer.putDouble(params.v0);
        buffer.put((byte) (params.archModeHigh ? 1 : 0));
        buffer.flip();

        channel.write(buffer);

        ByteBuffer headerBuffer = ByteBuffer.allocate(HEADER_SIZE);
        readFully(channel, headerBuffer);
        headerBuffer.flip();
        headerBuffer.order(ByteOrder.LITTLE_ENDIAN);

        int magic = headerBuffer.getInt();
        byte type = headerBuffer.get();
        int length = headerBuffer.getInt();

        if (magic != MAGIC_NUMBER || type != SOLVE_RESPONSE || length != SHOOTER_RESPONSE_SIZE) {
            throw new IOException("Invalid header: magic=" + magic + " type=" + type + " length=" + length);
        }

        ByteBuffer payloadBuffer = ByteBuffer.allocate(SHOOTER_RESPONSE_SIZE);
        readFully(channel, payloadBuffer);
        payloadBuffer.flip();
        payloadBuffer.order(ByteOrder.LITTLE_ENDIAN);

        double azimuth = payloadBuffer.getDouble();
        double elevation = payloadBuffer.getDouble();
        double time = payloadBuffer.getDouble();
        boolean success = payloadBuffer.get() != 0;

        return new ServerSolverResult(azimuth, elevation, time, success);
    }

    public void close() {
        try {
            if (channel != null && channel.isOpen()) {
                channel.close();
            }
        } catch (IOException ignored) {
        }
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
