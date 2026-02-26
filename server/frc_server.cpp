//g++ -O3 -std=c++17 -o frc_server frc_server.cpp
#include "ballistic_solver_core.hpp"
#include <netinet/tcp.h>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <thread>
#include <string>
#include <functional>

const double GRAVITY = 9.81;           // m/s²
const double DRAG_COEFFICIENT = 0.024; // 0.5*Cd*rho*A/m, Cd=0.47, m=0.215kg (was 0.025)

// The magic number to identify the start of a packet
const uint32_t MAGIC_NUMBER = 0xABCDABCD;

enum class MessageType : uint8_t {
    SOLVE_REQUEST = 1,
    SOLVE_RESPONSE = 129,
};

#pragma pack(push, 1)
// both request and response have the same header
// length is the payload length in bytes
struct PacketHeader {
    uint32_t magic;
    MessageType type;
    uint32_t length;
};

struct ShooterRequest
{
    Vec3 relPos0;
    Vec3 relVel;
    double v0;
    bool archModeHigh; // true for high arch, false for low arch
};

struct ShooterResponse
{
    double azimuth;   // radians
    double elevation; // radians
    double time;
    bool success;
};
#pragma pack(pop)


struct FRCShooterParams : public BallisticParams
{
    FRCShooterParams()
    {
        g = GRAVITY;
        arcMode = ArcMode::Low;

        dt = 0.005;     // Fine RK4 steps for accuracy
        tMax = 3.0;     // Max flight time
        tolMiss = 0.03; // 3cm accuracy target

        maxIter = 20;

        thetaMin = 45.0 * M_PI / 180.0; // 45° minimum elevation
        thetaMax = (90.0 - 14.894799) * M_PI / 180.0; // 75.11° maximum elevation
    }
};

// A generic, multi-threaded TCP server class
class TcpServer {
public:
    using ClientHandler = std::function<void(int, struct sockaddr_in)>;

    TcpServer(int port, ClientHandler handler)
        : port(port), server_fd(-1), client_handler(std::move(handler)) {}

    ~TcpServer() {
        if (server_fd != -1) {
            close(server_fd);
        }
    }

    void run() {
        struct sockaddr_in address;
        int opt = 1;

        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        if (listen(server_fd, 10) < 0) { // Increased backlog for busy servers
            perror("listen");
            exit(EXIT_FAILURE);
        }

        std::cout << "Server listening on port " << port << std::endl;

        while (true) {
            struct sockaddr_in client_address;
            socklen_t client_addrlen = sizeof(client_address);
            int new_socket;

            if ((new_socket = accept(server_fd, (struct sockaddr *)&client_address, &client_addrlen)) < 0) {
                perror("accept");
                continue;
            }

#ifdef SO_NOSIGPIPE
            int option = 1;
            setsockopt(new_socket, SOL_SOCKET, SO_NOSIGPIPE, &option, sizeof(option));
#endif
		int flag = 1;
	setsockopt(new_socket, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

            std::thread client_thread(client_handler, new_socket, client_address);
            client_thread.detach();
        }
    }

private:
    int port;
    int server_fd;
    ClientHandler client_handler;
};


class FRCShooterServer
{
public:
    FRCShooterServer(bool debug = false) : debug_(debug)
    {
        solverParams = FRCShooterParams();
    }

private:
    BallisticParams solverParams;
    bool debug_ = false;

    enum ReadStatus { SUCCESS, RETRY, FAIL };

    ShooterResponse solve(const ShooterRequest &request)
    {
        if (request.archModeHigh)
        {
            solverParams.arcMode = ArcMode::High;
        }
        else
        {
            solverParams.arcMode = ArcMode::Low;
        }
        auto ret = solve_launch_angles(
            request.relPos0,
            request.relVel,
            request.v0,
            DRAG_COEFFICIENT,
            solverParams);
        if (debug_) {
            std::cout << "[DEBUG] Solve request: relPos0=(" << request.relPos0.x << ", " << request.relPos0.y << ", " << request.relPos0.z << "), relVel=(" << request.relVel.x << ", " << request.relVel.y << ", " << request.relVel.z << "), v0=" << request.v0 << std::endl;
            std::cout << "[DEBUG] Solve result: phi=" << ret.phi << ", theta=" << ret.theta << ", tStar=" << ret.tStar << ", success=" << ret.success << std::endl;
        }
        return ShooterResponse{
            ret.phi,
            ret.theta,
            ret.tStar,
            ret.success};
    }

private:
    static ssize_t read_fully(int fd, void* buffer, size_t size) {
        size_t total_read = 0;
        while (total_read < size) {
            ssize_t bytes_read = read(fd, static_cast<char*>(buffer) + total_read, size - total_read);
            if (bytes_read <= 0) {
                // 0 means end of file (graceful disconnect, client has disconnected or closed)
                // (-1) <0 means error, could be EAGAIN or EWOULDBLOCK (timeout), or other errors
                return bytes_read; // Error or closed connection
            }
            total_read += bytes_read;
        }
        return total_read;
    }

    ReadStatus read_message_part(int client_socket, void* buffer, size_t size, const char* part_name) {
        ssize_t bytes_read = read_fully(client_socket, buffer, size);
        if (bytes_read > 0) {
            return ReadStatus::SUCCESS;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            std::cout << "Read timeout waiting for " << part_name << ". Waiting for data..." << std::endl;
            return ReadStatus::RETRY;
        }

        if (bytes_read == 0) {
            // This is a graceful disconnect, not necessarily an error, but it means we should stop.
            std::cout << "Client disconnected while waiting for " << part_name << "." << std::endl;
        } else {
            // This is a real error.
            std::string error_msg = std::string("read ") + part_name + " failed";
            perror(error_msg.c_str());
        }
        return ReadStatus::FAIL;
    }

    void handle_client(int client_socket, struct sockaddr_in address) {
        // Set receive and send timeouts
        struct timeval tv;
        tv.tv_sec = 5; // 5 seconds
        tv.tv_usec = 0;
        if (setsockopt(client_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
            perror("setsockopt(SO_RCVTIMEO) failed");
            close(client_socket);
            return;
        }
        if (setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv) < 0) {
            perror("setsockopt(SO_SNDTIMEO) failed");
            close(client_socket);
            return;
        }

        if (debug_) std::cout << "Client connected from " << inet_ntoa(address.sin_addr) << " on a new thread." << std::endl;

        while (true) {
            PacketHeader header;
            ReadStatus status = read_message_part(client_socket, &header, sizeof(header), "header");
            if (status == ReadStatus::RETRY) continue;
            // If we failed to read the header, break the loop. which will close the connection
            if (status == ReadStatus::FAIL) break;

            if (debug_) {
                            std::cout << "[DEBUG] Received Header: magic=0x" << std::hex << header.magic
                                      << ", type=" << static_cast<int>(header.type)
                                      << ", length=" << std::dec << header.length << std::endl;
                        }

            if (header.magic != MAGIC_NUMBER) {
                // Invalid magic number, desynchronized stream, or network error.
                // close the connection
                std::cerr << "Error: Invalid magic number. Desynchronized stream." << std::endl;
                break;
            }

            bool should_continue = true;
            switch (header.type) {
                case MessageType::SOLVE_REQUEST:
                    should_continue = handle_solve_request(client_socket, header);
                    break;
                default:
                    std::cerr << "Error: Unknown message type: " << static_cast<int>(header.type) << std::endl;
                    should_continue = false; // Stop on unknown type
            }

            if (!should_continue) {
                break; // A handler failed, close the connection
            }
        }
    end_loop:;
        close(client_socket);
    }

    bool handle_solve_request(int client_socket, const PacketHeader& header) {
        if (header.length != sizeof(ShooterRequest)) {
            std::cerr << "Error: Invalid payload length for SOLVE_REQUEST. Expected " << sizeof(ShooterRequest)
                        << ", but got " << header.length << std::endl;
            return false; // Fatal error
        }

        ShooterRequest request;
        ReadStatus status = read_message_part(client_socket, &request, sizeof(request), "ShooterRequest payload");
        if (status != ReadStatus::SUCCESS) return false; // Got payload failure, close connection

        auto start = std::chrono::high_resolution_clock::now();
        ShooterResponse response = solve(request);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        if (debug_) std::cout << "Solver duration: " << duration.count() << " ms" << std::endl;

        return send_response(client_socket, MessageType::SOLVE_RESPONSE, &response, sizeof(response));
    }



    bool send_response(int client_socket, MessageType type, void* payload, uint32_t length) {
        PacketHeader response_header;
        response_header.magic = MAGIC_NUMBER;
        response_header.type = type;
        response_header.length = length;

        if (debug_) {
            std::cout << "Sending response: " << static_cast<int>(type) << " with length " << length << std::endl;
        }

        if (send(client_socket, &response_header, sizeof(response_header), 0) < 0) {
            perror("send response header failed");
            return false;
        }

        if (send(client_socket, payload, length, 0) < 0) {
            perror("send response payload failed");
            return false;
        }
        return true;
    }

public:
    void run(int port) {
        auto handler = [this](int client_socket, struct sockaddr_in address) {
            this->handle_client(client_socket, address);
        };
        TcpServer server(port, handler);
        server.run();
    }
};


int main(int argc, char* argv[]) {
    bool debug = false;
    if (argc > 1 && std::string(argv[1]) == "--debug") {
        debug = true;
        std::cout << "Debug mode enabled." << std::endl;
    }

    FRCShooterServer server(debug);
    server.run(5805);
    return 0;
}
