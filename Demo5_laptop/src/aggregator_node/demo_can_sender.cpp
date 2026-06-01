// demo_can_sender.cpp
// Standalone C++ equivalent of demo_can_sender.py.
// Sends fake aggregator CAN frames over a SocketCAN interface.
//
// Build:  g++ -O2 -o demo_can_sender demo_can_sender.cpp
// Run:    ./demo_can_sender [can_interface]   (default: can0)

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <random>
#include <thread>

// ── CAN IDs (must match AggregatorNode.py) ───────────────────────────────────
static constexpr uint32_t CAN_ID_POSITION  = 0x108;
static constexpr uint32_t CAN_ID_FOREST1   = 0x105;
static constexpr uint32_t CAN_ID_FOREST2   = 0x106;
static constexpr uint32_t CAN_ID_R1_STATUS = 0x107;

// ── Helpers ───────────────────────────────────────────────────────────────────

static int16_t clamp16(int32_t v)
{
    if (v < -32768) return -32768;
    if (v >  32767) return  32767;
    return static_cast<int16_t>(v);
}

// Write a little-endian int16 into buf[offset]
static void write_le16(uint8_t* buf, int offset, int16_t val)
{
    buf[offset]     = static_cast<uint8_t>(val & 0xFF);
    buf[offset + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

// Write a big-endian int16 into buf[offset]  (matches AggregatorNode.py '>hh')
static void write_be16(uint8_t* buf, int offset, int16_t val)
{
    buf[offset]     = static_cast<uint8_t>((val >> 8) & 0xFF);
    buf[offset + 1] = static_cast<uint8_t>(val & 0xFF);
}

// ── Frame builders ────────────────────────────────────────────────────────────

// 0x108  <hhhh>  x_mm, y_mm, z_mm, yaw_cd  (little-endian)
static void build_position(uint8_t out[8],
                            float x, float y, float z, float yaw)
{
    int16_t x_mm   = clamp16(static_cast<int32_t>(x   * 1000.0f));
    int16_t y_mm   = clamp16(static_cast<int32_t>(y   * 1000.0f));
    int16_t z_mm   = clamp16(static_cast<int32_t>(z   * 1000.0f));
    int16_t yaw_cd = clamp16(static_cast<int32_t>(yaw * 100.0f));
    write_le16(out, 0, x_mm);
    write_le16(out, 2, y_mm);
    write_le16(out, 4, z_mm);
    write_le16(out, 6, yaw_cd);
}

// 0x105  forest[0..7] as raw bytes
static void build_forest1(uint8_t out[8], const uint8_t forest[12])
{
    std::memcpy(out, forest, 8);
}

// 0x106  forest[8..11], current_block, status_byte, 0x00, 0x00
static void build_forest2(uint8_t out[8], const uint8_t forest[12],
                           uint8_t current_block,
                           bool lidar_ok, bool camera_ok, bool end)
{
    out[0] = forest[8];
    out[1] = forest[9];
    out[2] = forest[10];
    out[3] = forest[11];
    out[4] = current_block;
    out[5] = (lidar_ok  ? 0x01 : 0x00)
           | (camera_ok ? 0x02 : 0x00)
           | (end       ? 0x04 : 0x00);
    out[6] = 0x00;
    out[7] = 0x00;
}

// 0x107  >hh xxxx   r1_x_mm, r1_y_mm big-endian + 4 padding bytes
//        Matches AggregatorNode.py: struct.pack('>hhxxxx', ...)
static void build_r1_status(uint8_t out[8], float r1_x, float r1_y)
{
    int16_t x_mm = clamp16(static_cast<int32_t>(r1_x * 1000.0f));
    int16_t y_mm = clamp16(static_cast<int32_t>(r1_y * 1000.0f));
    write_be16(out, 0, x_mm);
    write_be16(out, 2, y_mm);
    out[4] = out[5] = out[6] = out[7] = 0x00;
}

// ── CAN socket ────────────────────────────────────────────────────────────────

static int open_can(const char* iface)
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return -1; }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(sock);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return -1;
    }

    return sock;
}

static void send_frame(int sock, uint32_t can_id, const uint8_t data[8])
{
    struct can_frame frame{};
    frame.can_id  = can_id;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    ssize_t n = write(sock, &frame, sizeof(frame));
    if (n != sizeof(frame))
        fprintf(stderr, "Failed to send CAN ID 0x%03X\n", can_id);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    const char* iface = (argc > 1) ? argv[1] : "can0";

    printf("Opening CAN interface: %s\n", iface);
    int sock = open_can(iface);
    if (sock < 0) {
        fprintf(stderr, "Failed to open CAN interface. "
                "Make sure it is up, e.g.:\n"
                "  sudo ip link set %s up type can bitrate 1000000\n", iface);
        return 1;
    }
    printf("CAN interface ready. Sending fake data. Press Ctrl+C to stop.\n");

    std::mt19937 rng(std::random_device{}());
    auto frand = [&](float lo, float hi) {
        return std::uniform_real_distribution<float>(lo, hi)(rng);
    };
    auto irand = [&](int lo, int hi) {
        return std::uniform_int_distribution<int>(lo, hi)(rng);
    };

    uint8_t buf[8];

    while (true)
    {
        // ── Build fake data ───────────────────────────────────────────────
        float x   = frand(-5.0f, 5.0f);
        float y   = frand(-5.0f, 5.0f);
        float z   = 0.0f;
        float yaw = frand(-3.14159f, 3.14159f);

        uint8_t forest[12] = {};
        int num_kfs = irand(0, 4);
        for (int i = 0; i < num_kfs; i++) {
            int block = irand(0, 11);
            static const uint8_t types[] = {1, 2, 9};
            forest[block] = types[irand(0, 2)];
        }

        uint8_t current_block = static_cast<uint8_t>(irand(0, 12));
        bool lidar_ok  = irand(0, 1);
        bool camera_ok = irand(0, 1);
        bool end       = irand(0, 1);

        float r1_x = frand(-5.0f, 5.0f);
        float r1_y = frand(-5.0f, 5.0f);

        // ── Send 4 frames with 10ms inter-frame gap ───────────────────────
        build_position(buf, x, y, z, yaw);
        send_frame(sock, CAN_ID_POSITION, buf);
        printf("Sent 0x%03X (position):  x=%.2f  y=%.2f  yaw=%.2f\n", CAN_ID_POSITION, x, y, yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        build_forest1(buf, forest);
        send_frame(sock, CAN_ID_FOREST1, buf);
        printf("Sent 0x%03X (forest1)\n", CAN_ID_FOREST1);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        build_forest2(buf, forest, current_block, lidar_ok, camera_ok, end);
        send_frame(sock, CAN_ID_FOREST2, buf);
        printf("Sent 0x%03X (forest2):  block=%d  lidar=%d  cam=%d  end=%d\n",
               CAN_ID_FOREST2, current_block, lidar_ok, camera_ok, end);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        build_r1_status(buf, r1_x, r1_y);
        send_frame(sock, CAN_ID_R1_STATUS, buf);
        printf("Sent 0x%03X (r1_status): r1_x=%.2f  r1_y=%.2f\n\n", CAN_ID_R1_STATUS, r1_x, r1_y);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    close(sock);
    return 0;
}
