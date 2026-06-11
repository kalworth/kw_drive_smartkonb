#include "kw_drive/protocol/kw_serial_transport.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace kw
{
namespace
{

speed_t toTermiosBaud(uint32_t baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default:
      throw std::invalid_argument("unsupported serial_baud; use 921600 for KW Drive");
  }
}

void writeAll(int fd, const uint8_t * data, size_t size)
{
  size_t written = 0;
  while (written < size) {
    const ssize_t ret = ::write(fd, data + written, size - written);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(std::string("serial write failed: ") + std::strerror(errno));
    }
    written += static_cast<size_t>(ret);
  }
}

}  // namespace

KwSerialTransport::KwSerialTransport(std::string serial_port, uint32_t serial_baud)
: serial_port_(std::move(serial_port)), serial_baud_(serial_baud)
{
  if (serial_port_.empty()) {
    serial_port_ = "/dev/ttyACM0";
  }

  tx_frame_ = {
    0x55, 0xaa, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  openSerial();
  configureSerial(serial_baud_);
  read_thread_ = std::thread(&KwSerialTransport::readLoop, this);

  std::cout << "Open KW serial adapter successful: " << serial_port_
            << " @ " << serial_baud_ << std::endl;
}

KwSerialTransport::~KwSerialTransport()
{
  stop_thread_ = true;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void KwSerialTransport::openSerial()
{
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    throw std::runtime_error("failed to open " + serial_port_ + ": " + std::strerror(errno));
  }
}

void KwSerialTransport::configureSerial(uint32_t baud)
{
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    throw std::runtime_error(std::string("tcgetattr failed: ") + std::strerror(errno));
  }

  cfmakeraw(&tio);
  tio.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tio.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tio.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tio.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tio.c_cflag |= CS8;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;

  const speed_t termios_baud = toTermiosBaud(baud);
  cfsetispeed(&tio, termios_baud);
  cfsetospeed(&tio, termios_baud);

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    throw std::runtime_error(std::string("tcsetattr failed: ") + std::strerror(errno));
  }
  tcflush(fd_, TCIOFLUSH);
}

void KwSerialTransport::setFrameCallback(FrameCallback cb)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  frame_callback_ = std::move(cb);
}

void KwSerialTransport::sendCanFrame(uint32_t can_id, const std::array<uint8_t, 8> & data)
{
  std::array<uint8_t, 30> frame = tx_frame_;
  frame[13] = static_cast<uint8_t>(can_id & 0xff);
  frame[14] = static_cast<uint8_t>((can_id >> 8) & 0xff);

  for (size_t i = 0; i < data.size(); ++i) {
    frame[21 + i] = data[i];
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  writeAll(fd_, frame.data(), frame.size());
}

void KwSerialTransport::readLoop()
{
  std::array<uint8_t, 256> buf{};
  while (!stop_thread_) {
    const ssize_t ret = ::read(fd_, buf.data(), buf.size());
    if (ret > 0) {
      rx_buffer_.insert(rx_buffer_.end(), buf.begin(), buf.begin() + ret);
      processRx();
      continue;
    }
    if (ret < 0 && errno != EINTR && errno != EAGAIN) {
      std::cerr << "[Error] serial read failed: " << std::strerror(errno) << std::endl;
      return;
    }
  }
}

void KwSerialTransport::processRx()
{
  constexpr uint8_t header = 0xaa;
  constexpr uint8_t feedback_tag = 0x11;
  constexpr uint8_t tail = 0x55;
  constexpr size_t frame_len = 16;

  size_t i = 0;
  while (i + frame_len <= rx_buffer_.size()) {
    if (rx_buffer_[i] != header || rx_buffer_[i + frame_len - 1] != tail) {
      ++i;
      continue;
    }

    CanFrame frame{};
    frame.can_id =
      static_cast<uint32_t>(rx_buffer_[i + 3]) |
      (static_cast<uint32_t>(rx_buffer_[i + 4]) << 8) |
      (static_cast<uint32_t>(rx_buffer_[i + 5]) << 16) |
      (static_cast<uint32_t>(rx_buffer_[i + 6]) << 24);
    for (size_t j = 0; j < frame.data.size(); ++j) {
      frame.data[j] = rx_buffer_[i + 7 + j];
    }

    FrameCallback cb;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      cb = frame_callback_;
    }
    if (cb && rx_buffer_[i + 1] == feedback_tag) {
      cb(frame);
    }

    i += frame_len;
  }

  if (i > 0) {
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(i));
  }
}

}  // namespace kw
