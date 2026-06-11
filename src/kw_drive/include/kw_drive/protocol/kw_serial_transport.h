#ifndef KW_DRIVE_PROTOCOL_KW_SERIAL_TRANSPORT_H
#define KW_DRIVE_PROTOCOL_KW_SERIAL_TRANSPORT_H

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace kw
{

struct CanFrame
{
  uint32_t can_id{0};
  std::array<uint8_t, 8> data{};
};

class KwSerialTransport
{
public:
  using FrameCallback = std::function<void(const CanFrame &)>;

  KwSerialTransport(std::string serial_port, uint32_t serial_baud);
  ~KwSerialTransport();

  void setFrameCallback(FrameCallback cb);
  void sendCanFrame(uint32_t can_id, const std::array<uint8_t, 8> & data);

private:
  void openSerial();
  void configureSerial(uint32_t baud);
  void readLoop();
  void processRx();

  std::string serial_port_;
  uint32_t serial_baud_{921600};
  int fd_{-1};

  std::array<uint8_t, 30> tx_frame_{};
  std::vector<uint8_t> rx_buffer_;

  std::mutex write_mutex_;
  std::mutex callback_mutex_;
  FrameCallback frame_callback_;

  std::thread read_thread_;
  std::atomic<bool> stop_thread_{false};
};

}  // namespace kw

#endif
