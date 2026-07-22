#include "odrive_wheels_driver/can_socket.hpp"

#include <cstring>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace odrive_wheels_driver {

CANSocket::CANSocket(const std::string& interface) : interface_(interface) {
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    return;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    return;
  }
}

CANSocket::~CANSocket() {
  close();
}

bool CANSocket::is_open() const {
  return fd_ >= 0;
}

bool CANSocket::send(uint32_t arb_id, const uint8_t* data, uint8_t dlc) {
  if (fd_ < 0) return false;

  struct can_frame frame{};
  frame.can_id = arb_id;
  frame.can_dlc = dlc;
  if (data && dlc > 0) {
    std::memcpy(frame.data, data, dlc);
  }

  auto nbytes = write(fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool CANSocket::send_rtr(uint32_t arb_id) {
  if (fd_ < 0) return false;

  struct can_frame frame{};
  frame.can_id = arb_id | CAN_RTR_FLAG;
  frame.can_dlc = 8;

  auto nbytes = write(fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

bool CANSocket::recv(struct can_frame& frame, int timeout_ms) {
  if (fd_ < 0) return false;

  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) return false;

  auto nbytes = read(fd_, &frame, sizeof(frame));
  return nbytes == sizeof(frame);
}

int CANSocket::native_handle() const {
  return fd_;
}

void CANSocket::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void CANSocket::set_filter(uint8_t node_id_1, uint8_t node_id_2) {
  if (fd_ < 0) return;

  // Accept frames from both node IDs (match on upper bits of arb_id)
  struct can_filter filters[2];
  filters[0].can_id   = static_cast<uint32_t>(node_id_1) << 5;
  filters[0].can_mask = 0x7E0;  // mask upper 6 bits (node_id field)
  filters[1].can_id   = static_cast<uint32_t>(node_id_2) << 5;
  filters[1].can_mask = 0x7E0;

  setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
}

}  // namespace odrive_wheels_driver
