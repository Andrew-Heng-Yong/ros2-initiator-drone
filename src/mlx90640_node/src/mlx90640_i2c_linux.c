// Linux i2c-dev transport for the Melexis MLX90640 reference API.
#include "MLX90640_I2C_Driver.h"
#include "mlx90640_node/mlx90640_i2c_linux.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int i2c_fd = -1;
static char i2c_device[64] = "/dev/i2c-1";

void MLX90640_I2CSetDevice(const char *device) {
  if (device != NULL && strlen(device) < sizeof(i2c_device)) {
    strncpy(i2c_device, device, sizeof(i2c_device) - 1);
    i2c_device[sizeof(i2c_device) - 1] = '\0';
  }
}

void MLX90640_I2CInit(void) {
  if (i2c_fd >= 0) return;
  i2c_fd = open(i2c_device, O_RDWR | O_CLOEXEC);
  if (i2c_fd < 0) perror("MLX90640: cannot open I2C device");
}

static int select_slave(uint8_t address) {
  MLX90640_I2CInit();
  if (i2c_fd < 0 || ioctl(i2c_fd, I2C_SLAVE, address) < 0) return -MLX90640_I2C_NACK_ERROR;
  return MLX90640_NO_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                      uint16_t nMemAddressRead, uint16_t *data) {
  if (select_slave(slaveAddr) != MLX90640_NO_ERROR) return -MLX90640_I2C_NACK_ERROR;
  // SMBus adapters commonly limit I2C_RDWR transfers; 32 words is portable.
  while (nMemAddressRead > 0) {
    const uint16_t words = nMemAddressRead > 32 ? 32 : nMemAddressRead;
    uint8_t address_bytes[2] = {(uint8_t)(startAddress >> 8), (uint8_t)startAddress};
    uint8_t bytes[64];
    struct i2c_msg messages[2] = {
      {.addr = slaveAddr, .flags = 0, .len = 2, .buf = address_bytes},
      {.addr = slaveAddr, .flags = I2C_M_RD, .len = (uint16_t)(words * 2), .buf = bytes},
    };
    struct i2c_rdwr_ioctl_data transaction = {.msgs = messages, .nmsgs = 2};
    if (ioctl(i2c_fd, I2C_RDWR, &transaction) < 0) return -MLX90640_I2C_NACK_ERROR;
    for (uint16_t i = 0; i < words; ++i)
      data[i] = (uint16_t)((bytes[2 * i] << 8) | bytes[2 * i + 1]);
    data += words;
    startAddress += words;
    nMemAddressRead -= words;
  }
  return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  if (select_slave(slaveAddr) != MLX90640_NO_ERROR) return -MLX90640_I2C_NACK_ERROR;
  uint8_t bytes[4] = {(uint8_t)(writeAddress >> 8), (uint8_t)writeAddress,
                      (uint8_t)(data >> 8), (uint8_t)data};
  return write(i2c_fd, bytes, sizeof(bytes)) == (ssize_t)sizeof(bytes)
             ? MLX90640_NO_ERROR : -MLX90640_I2C_WRITE_ERROR;
}

int MLX90640_I2CGeneralReset(void) { return MLX90640_NO_ERROR; }
void MLX90640_I2CFreqSet(int freq) { (void)freq; }
