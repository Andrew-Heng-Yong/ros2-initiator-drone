#ifndef MLX90640_NODE__MLX90640_I2C_LINUX_H_
#define MLX90640_NODE__MLX90640_I2C_LINUX_H_

#ifdef __cplusplus
extern "C" {
#endif

// Selects the Linux i2c-dev endpoint before the MLX90640 reference API opens it.
void MLX90640_I2CSetDevice(const char *device);

#ifdef __cplusplus
}
#endif

#endif  // MLX90640_NODE__MLX90640_I2C_LINUX_H_
