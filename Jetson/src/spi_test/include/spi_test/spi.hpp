#ifndef KIKS_IO_SPI_HPP_
#define KIKS_IO_SPI_HPP_

extern "C" {
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}
#include <cstdint>
#include <memory>
#include <string_view>

namespace kiks::io::spi {

using Data = spi_ioc_transfer;

class Base {
protected:
  Base(std::string_view dev_name) { dev_ = std::make_shared<Dev>(dev_name); }

  Base(const Base &spi_base) { dev_ = spi_base.dev_; }

  Base(Base &&spi_base) { dev_ = std::move(spi_base.dev_); }

  const Base &operator=(const Base &spi_base) {
    dev_ = spi_base.dev_;
    return *this;
  }

  const Base &operator=(const Base &&spi_base) {
    dev_ = std::move(spi_base.dev_);
    return *this;
  }

  class Dev {
  public:
    Dev(std::string_view dev_name) noexcept {
      fd_ = open(dev_name.data(), O_RDWR);
      opend_ = (fd_ >= 0);
    }

    ~Dev() {
      if (opend_) {
        close(fd_);
      }
    }

    inline int get_fd() const noexcept { return fd_; }

    inline bool is_opend() const noexcept { return opend_; }

    inline void set_max_speed_hz(const std::uint32_t max_speed_hz) noexcept {
      ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_hz);
    }

    inline void set_bits_per_word(const std::uint8_t bits_per_word) noexcept {
      ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    }

    inline void set_mode(const int mode) const noexcept {
      ioctl(fd_, SPI_IOC_WR_MODE32, &mode);
    }

    inline void set_lsb(const std::uint8_t lsb) const noexcept {
      ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &lsb);
    }

  private:
    int fd_;
    bool opend_;
  };

  std::shared_ptr<Dev> dev_;

public:
  inline bool is_opend() const noexcept { return dev_->is_opend(); }

  inline void
  set_max_speed_hz(const std::uint32_t max_speed_hz) const noexcept {
    dev_->set_max_speed_hz(max_speed_hz);
  }

  inline void
  set_bits_per_word(const std::uint8_t bits_per_word) const noexcept {
    dev_->set_bits_per_word(bits_per_word);
  }

  inline void set_mode(const int mode) const noexcept { dev_->set_mode(mode); }

  inline void set_lsb(const std::uint8_t lsb) const noexcept {
    dev_->set_lsb(lsb);
  }
};

class Master : public Base {
public:
  inline Master(std::string_view dev_name) : Base(dev_name) {}

  inline void write(Data *spi_datas, std::size_t count = 1) const noexcept {
    ioctl(dev_->get_fd(), SPI_IOC_MESSAGE(count), spi_datas);
  }

  template <class Datas>
  inline void write(const Datas &spi_datas) const noexcept {
    ioctl(dev_->get_fd(), SPI_IOC_MESSAGE(spi_datas.size()), spi_datas.data());
  }
};

} // namespace kiks::io::spi

#endif // KIKS_IO_SPI_HPP_