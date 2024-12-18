	
#include "CAN.hpp"

void init(const std::string& CANDevice)
{
    this->_canFd     = open(i2cDevice.c_str(), O_RDWR);

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode." << std::endl;
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Failed to set bits per word." << std::endl;
        close(fd);
        return -1;
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Failed to set max speed." << std::endl;
        close(fd);
        return -1;
    }
}

uint8_t ReadByte(addr)
{

}
		senddata = [CAN_READ, addr, 0]
		# print(f"Enviando: {senddata}")  # Verifica o que está sendo enviado
		res = self.SPI.xfer2(senddata)
		# print(f"Recebido: {res}")  # Verifica o que foi recebido
		return int(res[2])

	def WriteByte(self, addr, data):
		senddata = [CAN_WRITE, addr]
		if isinstance(data, int):
			senddata.append(data)
		else:
			senddata = senddata + data
		# print(senddata)
		self.SPI.writebytes(senddata) 