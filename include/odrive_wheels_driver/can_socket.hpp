#pragma once

#include <cstdint>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>

namespace odrive_wheels_driver {

class CANSocket {
public:
  explicit CANSocket(const std::string& interface);
  ~CANSocket();

  CANSocket(const CANSocket&) = delete;
  CANSocket& operator=(const CANSocket&) = delete;

  bool is_open() const;
  bool send(uint32_t arb_id, const uint8_t* data, uint8_t dlc);
  bool send_rtr(uint32_t arb_id);
  bool recv(struct can_frame& frame, int timeout_ms);
  int native_handle() const;
  void close();

  void set_filter(uint8_t node_id_1, uint8_t node_id_2);

private:
  int fd_ = -1;
  std::string interface_;
};

}  // namespace odrive_wheels_driver
