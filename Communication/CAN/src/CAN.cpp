	
#include "CAN.hpp"

void init(const std::string& CANDevice)
{
    this->_canFd     = open(CANDevice.c_str(), O_RDWR);

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;

    if (ioctl(this->_canFd, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MODE, &mode) < 0) {
        std::cerr << "Failed to set SPI mode." << std::endl;
        close(this->_canFd);
        return -1;
    }

    if (ioctl(this->_canFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(this->_canFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "Failed to set bits per word." << std::endl;
        close(this->_canFd);
        return -1;
    }

    if (ioctl(this->_canFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 || ioctl(this->_canFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
        std::cerr << "Failed to set max speed." << std::endl;
        close(this->_canFd);
        return -1;
    }
}
void sendByte(addr, )
{
	uint8_t sendData = [CAN_READ, addr, 0];
	int res = ioctl(fd, SPI_IOC_MESSAGE(1), &(sendData));
	return int(res[2])
}


uint8_t readByte(addr)
{

}
	# print(f"Recebido: {res}")  # Verifica o que foi recebido


	def WriteByte(self, addr, data):
		senddata = [CAN_WRITE, addr]
		if isinstance(data, int):
			senddata.append(data)
		else:
			senddata = senddata + data
		# print(senddata)
		self.SPI.writebytes(senddata) 