// Manual, read-only six-axis limit teaching. Never constructs Arm or sends control commands.
#include <damiao_arm_sdk/types.hpp>
#include <damiao_arm_sdk/socketcan_bridge.hpp>
#include <openarm/damiao_motor/dm_motor.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
namespace dm = openarm::damiao_motor;
constexpr double pi = 3.14159265358979323846;
volatile std::sig_atomic_t interrupted = 0;
void signalHandler(int) { interrupted = 1; }
void require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
void cancellation() { require(!interrupted, "用户中止；未生成输出文件。"); }
struct Fd {
    int value{-1};
    ~Fd() { if (value >= 0) ::close(value); }
};
struct Options {
    std::string config, output, interface, serial;
    double margin{3};
    int samples{10}, period{20};
    bool check{false}, show_help{false};
};
void help() {
    std::cout << "Read-only manual six-axis limit calibration (never enables/disables motors)\n"
      "Usage: calibrate_joint_limits --config PATH [--output PATH] [--interface IFACE]\n"
      "  --margin-deg N       positive finite inward margin (default 3)\n"
      "  --samples N          stationary samples, 3..100 (default 10)\n"
      "  --sample-period-ms N sampling interval, 5..1000 (default 20)\n"
      "  --serial PATH        explicit embedded READ_ONLY USB bridge; existing CAN interface required\n"
      "  --check-only         check six disabled motors and firmware ranges; no files written\n"
      "  --help               show help without accessing hardware\n"
      "Output defaults to arm.calibrated.yaml beside the input; existing output is never overwritten.\n";
}
double number(const std::string& text) {
    std::size_t end = 0;
    double value = std::stod(text, &end);
    require(end == text.size() && std::isfinite(value), "Invalid finite numeric argument: " + text);
    return value;
}
Options arguments(int argc, char** argv) {
    Options o;
    bool output = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--check-only") { o.check = true; continue; }
        if (a == "--help") { o.show_help = true; continue; }
        require(a == "--config" || a == "--output" || a == "--interface" || a == "--serial" ||
                a == "--margin-deg" || a == "--samples" || a == "--sample-period-ms", "Unknown argument: " + a);
        require(i + 1 < argc, "Missing value for " + a);
        std::string v = argv[++i];
        require(!v.empty() && v.rfind("--", 0) != 0, "Missing/empty value for " + a);
        if (a == "--config") o.config = v;
        else if (a == "--output") { o.output = v; output = true; }
        else if (a == "--interface") o.interface = v;
        else if (a == "--serial") o.serial = v;
        else {
            const double n = number(v);
            if (a == "--margin-deg") {
                require(n > 0 && std::isfinite(n * pi / 180), "--margin-deg must be finite and positive");
                o.margin = n;
            } else {
                const int low = a == "--samples" ? 3 : 5;
                const int high = a == "--samples" ? 100 : 1000;
                require(n >= low && n <= high && std::floor(n) == n, "Invalid integer range for " + a);
                if (a == "--samples") o.samples = static_cast<int>(n);
                else o.period = static_cast<int>(n);
            }
        }
    }
    require(o.show_help || !o.config.empty(), "--config PATH is required");
    if (!output) o.output = (std::filesystem::path(o.config).parent_path() / "arm.calibrated.yaml").string();
    return o;
}
std::string readText(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    require(input.good(), "Cannot read config: " + path);
    std::ostringstream out; out << input.rdbuf();
    require(!input.bad(), "Config read failed: " + path);
    return out.str();
}
void unusedOutput(const std::string& path) {
    struct stat s{};
    if (::lstat(path.c_str(), &s) == 0) throw std::runtime_error("Output already exists (including symlinks); choose a new --output: " + path);
    require(errno == ENOENT, "Cannot inspect output path: " + path);
}
struct LineLimit { std::size_t start, length; };
using LimitLocations = std::array<std::array<LineLimit, 2>, 6>;
// Accept only an unambiguous block sequence with one scalar min/max per joint.
// Replace scalar tokens, not entire lines, so comments and all other text survive byte-for-byte.
LimitLocations locateLimits(const std::string& source) {
    LimitLocations locations{};
    std::array<std::array<int, 2>, 6> count{};
    const std::regex section(R"(^joints:[ \t]*(?:#.*)?\r?$)");
    const std::regex entry(R"(^( +)-[ \t]+[A-Za-z_][A-Za-z_0-9]*[ \t]*:.*$)");
    const std::regex scalar(R"(^( +)(pos_min|pos_max):[ \t]*([-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)([ \t]*(?:#.*)?\r?)$)");
    bool active = false, seen = false;
    int joint = -1, indent = -1;
    std::size_t offset = 0;
    while (offset < source.size()) {
        const auto end = source.find('\n', offset);
        const auto length = (end == std::string::npos ? source.size() : end) - offset;
        const std::string line = source.substr(offset, length);
        std::smatch m;
        if (std::regex_match(line, section)) {
            require(!seen, "Ambiguous duplicate joints section"); active = seen = true;
        } else if (active) {
            const auto first = line.find_first_not_of(" \t\r");
            if (first == 0 && line[0] != '#') active = false;
            else if (std::regex_match(line, m, entry)) {
                int spaces = static_cast<int>(m[1].length());
                if (indent < 0) indent = spaces;
                require(spaces == indent, "Unsupported nested YAML sequence in joints");
                require(++joint < 6, "Expected exactly six joint YAML entries");
            } else if (line.find("pos_min:") != std::string::npos || line.find("pos_max:") != std::string::npos) {
                // Comments containing these words are not fields.
                if (first != std::string::npos && line[first] != '#') {
                    require(joint >= 0 && std::regex_match(line, m, scalar), "Unsupported/ambiguous position limit YAML scalar");
                    require(static_cast<int>(m[1].length()) == indent + 2, "Unsupported position limit indentation");
                    const int which = m[2] == "pos_min" ? 0 : 1;
                    require(++count[joint][which] == 1, "Duplicate position limit key");
                    locations[joint][which] = {offset + static_cast<std::size_t>(m.position(3)), static_cast<std::size_t>(m.length(3))};
                }
            }
        }
        if (end == std::string::npos) break;
        offset = end + 1;
    }
    require(seen && joint == 5, "Expected exactly six block-style joint entries");
    for (const auto& c : count) require(c[0] == 1 && c[1] == 1, "Each joint needs exactly one pos_min and pos_max");
    return locations;
}
struct State { double position{}, velocity{}; int mos{}, rotor{}; };
class Reader {
    const damiao::ArmConfig& config_;
    Fd socket_;
    std::array<dm::Motor, 6> motors_;
    std::vector<uint8_t> bytes_{std::vector<uint8_t>(8)};
    void inspect(const can_frame& f) const {
        require(!(f.can_id & CAN_ERR_FLAG), "CAN bus fault; operator must stop/check arm and interface");
        require(!(f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)), "Unsupported extended/RTR frame on calibration bus");
    }
    bool receive(can_frame& frame, int timeout) {
        cancellation();
        pollfd p{socket_.value, POLLIN, 0};
        int result = ::poll(&p, 1, timeout);
        if (result < 0 && errno == EINTR) { cancellation(); return false; }
        require(result >= 0, "CAN poll failed");
        if (!result) return false;
        require(!(p.revents & (POLLERR | POLLHUP | POLLNVAL)), "CAN interface disconnected/faulted");
        canfd_frame storage{};
        const auto size = ::read(socket_.value, &storage, sizeof(storage));
        require(size != CANFD_MTU, "CAN FD unsupported: configure classic CAN only");
        require(size == CAN_MTU, "Malformed CAN frame/read failure");
        std::memcpy(&frame, &storage, sizeof(frame));
        inspect(frame);
        return true;
    }
    State decode(std::size_t i, const can_frame& f) {
        require(f.can_dlc == 8, "Malformed motor state length");
        require((f.data[0] & 15) == (config_.joints[i].send_can_id & 15), "State payload motor ID mismatch");
        const unsigned code = f.data[0] >> 4;
        if (code != 0) throw std::runtime_error("Joint " + std::to_string(i + 1) + " is enabled/faulted (code " + std::to_string(code) + "). 操作员必须停止并排查；本工具不会发送停机命令。");
        std::copy(f.data, f.data + 8, bytes_.begin());
        const auto s = dm::CanPacketDecoder::parse_motor_state_data(motors_[i], bytes_);
        require(s.valid && s.t_mos >= 0 && s.t_mos <= 85 && s.t_rotor >= 0 && s.t_rotor <= 85, "Motor temperatures invalid or above 85 C");
        return {s.position, s.velocity, s.t_mos, s.t_rotor};
    }
    bool parameterFrame(const can_frame& f) const {
        // A normal disabled state has D[0] upper nibble zero and D[1] position,
        // but parameter packets explicitly repeat the full 16-bit motor id.
        if (f.can_dlc != 8 || (f.data[2] != 0x33 && f.data[2] != 0x55)) return false;
        for (const auto& j : config_.joints)
            if (f.can_id == j.recv_can_id && (unsigned(f.data[0]) | (unsigned(f.data[1]) << 8)) == j.send_can_id) return true;
        return false;
    }
    void monitor(const can_frame& f) {
        if (parameterFrame(f)) return;
        for (std::size_t i = 0; i < 6; ++i)
            if (f.can_id == config_.joints[i].recv_can_id) { decode(i, f); return; }
    }
    void drain() {
        can_frame f{};
        const auto deadline = Clock::now() + std::chrono::milliseconds(100);
        while (receive(f, 0)) {
            monitor(f);
            require(Clock::now() < deadline, "CAN bus does not drain; stop competing bus clients");
        }
    }
    void send(std::size_t i, uint8_t command, uint8_t rid) {
        drain();
        const auto id = config_.joints[i].send_can_id;
        can_frame f{}; f.can_id = 0x7ff; f.can_dlc = 8;
        f.data[0] = id & 255; f.data[1] = (id >> 8) & 255;
        f.data[2] = command; f.data[3] = rid;
        require(::write(socket_.value, &f, sizeof(f)) == sizeof(f), "CAN read-query transmission failed");
    }
public:
    explicit Reader(const damiao::ArmConfig& c) : config_(c), motors_{{
        {c.joints[0].motor_type,c.joints[0].send_can_id,c.joints[0].recv_can_id},
        {c.joints[1].motor_type,c.joints[1].send_can_id,c.joints[1].recv_can_id},
        {c.joints[2].motor_type,c.joints[2].send_can_id,c.joints[2].recv_can_id},
        {c.joints[3].motor_type,c.joints[3].send_can_id,c.joints[3].recv_can_id},
        {c.joints[4].motor_type,c.joints[4].send_can_id,c.joints[4].recv_can_id},
        {c.joints[5].motor_type,c.joints[5].send_can_id,c.joints[5].recv_can_id}}} {
        socket_.value = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        require(socket_.value >= 0, "Cannot open SocketCAN");
        const can_err_mask_t errors = CAN_ERR_MASK;
        require(::setsockopt(socket_.value, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) == 0, "Cannot subscribe CAN errors");
        // Receive FD too, solely to produce an explicit unsupported diagnostic.
        const int fd = 1;
        require(::setsockopt(socket_.value, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd, sizeof(fd)) == 0, "Cannot configure CAN receiver");
        ifreq request{};
        require(c.can_interface.size() < IFNAMSIZ, "CAN interface name too long");
        std::strcpy(request.ifr_name, c.can_interface.c_str());
        require(::ioctl(socket_.value, SIOCGIFINDEX, &request) == 0, "CAN interface does not exist; create/configure it separately");
        sockaddr_can address{}; address.can_family = AF_CAN; address.can_ifindex = request.ifr_ifindex;
        require(::bind(socket_.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "Cannot bind CAN interface");
    }
    std::array<State, 6> refresh() {
        std::array<State, 6> states{};
        for (std::size_t i = 0; i < 6; ++i) {
            send(i, 0xcc, 0);
            bool found = false;
            auto deadline = Clock::now() + std::chrono::milliseconds(100);
            can_frame f{};
            while (Clock::now() < deadline) {
                if (!receive(f, 5)) continue;
                if (parameterFrame(f)) continue;
                monitor(f);
                if (f.can_id == config_.joints[i].recv_can_id) { states[i] = decode(i, f); found = true; break; }
            }
            if (!found) throw std::runtime_error("Joint " + std::to_string(i + 1) + " fresh state timeout (100 ms); operator must check/stop arm");
        }
        return states;
    }
    double parameter(std::size_t i, uint8_t rid) {
        send(i, 0x33, rid);
        auto deadline = Clock::now() + std::chrono::milliseconds(100);
        can_frame f{};
        while (Clock::now() < deadline) {
            if (!receive(f, 5)) continue;
            const auto id = config_.joints[i].send_can_id;
            if (f.can_id == config_.joints[i].recv_can_id && f.can_dlc == 8 &&
                f.data[0] == (id & 255) && f.data[1] == ((id >> 8) & 255) && f.data[2] == 0x33 && f.data[3] == rid) {
                std::copy(f.data, f.data + 8, bytes_.begin());
                const auto result = dm::CanPacketDecoder::parse_motor_param_data(bytes_);
                require(result.valid && std::isfinite(result.value), "Invalid parameter response");
                return result.value;
            }
            monitor(f);
        }
        throw std::runtime_error("Joint " + std::to_string(i + 1) + " exact parameter RID " + std::to_string(rid) + " timeout; check firmware/IDs and competing CAN clients");
    }
};
// Poll one byte at a time: works for terminals and piped simulation without iostream read-ahead.
std::string prompt(Reader& reader, const std::string& text) {
    std::cout << text << '\n' << std::flush;
    std::string line;
    auto next = Clock::now();
    for (;;) {
        cancellation();
        if (Clock::now() >= next) { reader.refresh(); next = Clock::now() + std::chrono::milliseconds(100); }
        pollfd p{STDIN_FILENO, POLLIN, 0};
        int n = ::poll(&p, 1, 20);
        if (n < 0 && errno == EINTR) continue;
        require(n >= 0, "stdin poll failed");
        if (n == 0) continue;
        require(!(p.revents & (POLLERR | POLLNVAL)), "stdin failure");
        char c;
        require(::read(STDIN_FILENO, &c, 1) == 1, "EOF: 用户中止；未生成输出文件。");
        if (c == '\n') {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            require(line != "q" && line != "Q", "用户中止；未生成输出文件。");
            return line;
        }
        require(line.size() < 128, "Input too long");
        line += c;
    }
}
double capture(Reader& reader, std::size_t joint, const Options& o) {
    std::array<double, 100> positions{};
    for (int n = 0; n < o.samples; ++n) {
        const auto start = Clock::now();
        auto states = reader.refresh();
        require(std::abs(states[joint].velocity) <= 0.15, "选定关节仍在运动 (|velocity| > 0.15 rad/s)；中止。");
        positions[n] = states[joint].position;
        // Continue refreshing during long sample periods so safety is not blind for up to 1 second.
        const auto until = start + std::chrono::milliseconds(o.period);
        while (Clock::now() < until) {
            cancellation();
            std::this_thread::sleep_until(std::min(until, Clock::now() + std::chrono::milliseconds(100)));
            if (Clock::now() < until) {
                const auto extra = reader.refresh();
                require(std::abs(extra[joint].velocity) <= 0.15, "选定关节仍在运动；中止。");
            }
        }
    }
    std::sort(positions.begin(), positions.begin() + o.samples);
    require(positions[o.samples - 1] - positions[0] <= 0.01, "选定关节不静止 (position spread > 0.01 rad)；中止。");
    const int mid = o.samples / 2;
    double value = o.samples % 2 ? positions[mid] : (positions[mid - 1] + positions[mid]) / 2;
    std::cout << "  端点: " << value << " rad / " << value * 180 / pi << " deg\n";
    return value;
}
std::string render(const std::string& source, const LimitLocations& locations, const std::array<std::array<double, 2>, 6>& limits) {
    struct Replacement { LineLimit location; double value; };
    std::array<Replacement, 12> replacements{};
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 2; ++j) replacements[2 * i + j] = {locations[i][j], limits[i][j]};
    std::sort(replacements.begin(), replacements.end(), [](const auto& a, const auto& b) { return a.location.start > b.location.start; });
    std::string result = source;
    for (const auto& r : replacements) {
        std::ostringstream number; number << std::setprecision(17) << r.value;
        result.replace(r.location.start, r.location.length, number.str());
    }
    return result;
}
void save(const Options& o, const std::string& source, const std::string& result) {
    cancellation();
    require(readText(o.config) == source, "Source config changed during teaching; refusing output");
    unusedOutput(o.output);
    // Same-directory temporary and hard-link publication provide atomic, no-clobber creation.
    std::string temp;
    Fd fd;
    for (int n = 0; n < 100; ++n) {
        temp = o.output + ".tmp." + std::to_string(::getpid()) + "." + std::to_string(n);
        fd.value = ::open(temp.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
        if (fd.value >= 0) break;
        require(errno == EEXIST, "Cannot create sibling temporary output");
    }
    require(fd.value >= 0, "Cannot allocate unique temporary output");
    struct Cleanup { const std::string& path; ~Cleanup() { ::unlink(path.c_str()); } } cleanup{temp};
    std::size_t offset = 0;
    while (offset < result.size()) {
        cancellation();
        auto written = ::write(fd.value, result.data() + offset, result.size() - offset);
        if (written < 0 && errno == EINTR) continue;
        require(written > 0, "Writing temporary output failed");
        offset += static_cast<std::size_t>(written);
    }
    require(::fsync(fd.value) == 0, "fsync temporary output failed");
    damiao::ArmConfig::fromYaml(temp).validate();
    cancellation();
    require(readText(o.config) == source, "Source config changed before publication; refusing output");
    require(::link(temp.c_str(), o.output.c_str()) == 0, "Cannot publish output (possibly exists); original untouched");
}
} // namespace
int main(int argc, char** argv) {
    try {
        // Help alone has no configuration or hardware prerequisite.
        if (argc == 2 && std::string(argv[1]) == "--help") { help(); return 0; }
        const auto o = arguments(argc, argv);
        if (o.show_help) { help(); return 0; }
        std::signal(SIGINT, signalHandler); std::signal(SIGTERM, signalHandler);
        auto config = damiao::ArmConfig::fromYaml(o.config);
        config.validate();
        require(config.joint_count == 6 && !config.has_gripper, "Require exactly six active joints and has_gripper false");
        require(!config.enable_fd, "CAN FD unsupported: use classic CAN configuration");
        if (!o.interface.empty()) config.can_interface = o.interface;
        require(!config.can_interface.empty() && config.can_interface.size() < IFNAMSIZ, "CAN interface must be nonempty and shorter than IFNAMSIZ");
        for (const auto& j : config.joints)
            require(j.send_can_id > 0 && j.send_can_id <= 15 && j.recv_can_id <= CAN_SFF_MASK, "Require classic IDs and motor IDs 1..15 for exact state payload matching");
        const std::string source = readText(o.config);
        const auto locations = locateLimits(source);
        unusedOutput(o.output);
        std::unique_ptr<damiao::SocketCanUsbBridge> bridge;
        if (!o.serial.empty()) {
            damiao::BridgeConfig b; b.serial_port = o.serial; b.socketcan_if = config.can_interface;
            b.read_only = true; b.auto_reconnect = false;
            bridge = std::make_unique<damiao::SocketCanUsbBridge>(b); bridge->start();
        }
        Reader reader(config);
        reader.refresh();
        for (std::size_t i = 0; i < 6; ++i) {
            require(reader.parameter(i, 8) == config.joints[i].send_can_id && reader.parameter(i, 7) == config.joints[i].recv_can_id, "Firmware ESC_ID/MST_ID mismatch; correct IDs before calibration");
            const auto limits = dm::MOTOR_LIMIT_PARAMS[static_cast<std::size_t>(config.joints[i].motor_type)];
            const std::array<double, 3> expected{{limits.pMax, limits.vMax, limits.tMax}};
            for (std::size_t k = 0; k < 3; ++k) {
                const double actual = reader.parameter(i, static_cast<uint8_t>(21 + k));
                require(std::abs(actual - expected[k]) <= std::max(1e-5, 1e-4 * std::abs(expected[k])),
                    "Joint " + std::to_string(i + 1) + " firmware PMAX/VMAX/TMAX mismatch: calibration would use wrong radians. Check configured motor_type and SDK firmware compatibility; do not calibrate/change ranges blindly.");
            }
            std::cout << "joint" << i + 1 << " SDK/firmware PMAX=" << limits.pMax << " VMAX=" << limits.vMax << " TMAX=" << limits.tMax << '\n';
        }
        auto states = reader.refresh();
        for (std::size_t i = 0; i < 6; ++i) std::cout << "joint" << i + 1 << " DISABLED error=0 pos=" << states[i].position << " rad vel=" << states[i].velocity << " rad/s MOS=" << states[i].mos << " C rotor=" << states[i].rotor << " C\n";
        if (o.check) return 0;
        std::cout << "\n只读手动示教：本工具仅发送状态/参数读取，不使能、不失能、不设零、不写 Flash。\n"
          "必须可靠支撑重力并固定所有轴；逻辑电源保持开启，六个电机必须失能。\n"
          "每次只缓慢移动指定关节到安全端点；禁止强推机械硬限位。其他轴保持固定。\n"
          "标定后不要设零或断电重启，否则坐标可能改变，需要重新核对/标定。\n"
          "若外部使能/故障/掉线将直接退出；操作员负责停止，本工具不会发停机命令。\n"
          "任意提示输入 q、EOF 或 Ctrl-C 均中止且不生成输出。\n";
        require(prompt(reader, "确认以上安全条件，输入 READY: ") == "READY", "未输入 READY；中止。");
        std::array<std::array<double, 2>, 6> limits{};
        const double margin = o.margin * pi / 180;
        for (std::size_t i = 0; i < 6; ++i) {
            std::array<double, 2> ends{};
            for (int side = 0; side < 2; ++side) {
                require(prompt(reader, "仅移动 joint" + std::to_string(i + 1) + " 到安全端点 " + (side == 0 ? "A" : "B") + "，静止后按 Enter: ").empty(), "端点确认仅接受 Enter 或 q；中止。");
                ends[side] = capture(reader, i, o);
            }
            std::sort(ends.begin(), ends.end());
            const double span = ends[1] - ends[0];
            const auto pmax = dm::MOTOR_LIMIT_PARAMS[static_cast<std::size_t>(config.joints[i].motor_type)].pMax;
            require(span > 2 * margin + 0.01 && span < pmax, "端点跨度过小或疑似编码器跨周；拒绝生成限位。");
            limits[i] = {ends[0] + margin, ends[1] - margin};
            std::cout << "  raw=[" << ends[0] << ", " << ends[1] << "] rad; inward margin=" << o.margin << " deg\n";
        }
        std::cout << "\n最终内缩限位（尚未保存/应用）:\n" << std::setprecision(10);
        for (std::size_t i = 0; i < 6; ++i) std::cout << "joint" << i + 1 << ": [" << limits[i][0] << ", " << limits[i][1] << "] rad = [" << limits[i][0] * 180 / pi << ", " << limits[i][1] * 180 / pi << "] deg\n";
        require(prompt(reader, "审核全部六轴后输入 SAVE 保存新文件: ") == "SAVE", "未输入 SAVE；中止。");
        reader.refresh();
        save(o, source, render(source, locations, limits));
        std::cout << "已生成: " << o.output << "\n原配置未覆盖；未自动应用到 ROS xacro，请人工审核后同步。\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Calibration aborted: " << e.what() << '\n';
        return 1;
    }
}
