	
#include "CAN.hpp"

void CAN::init(const std::string& CANDevice)
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
void CAN::sendInfo(addr, uint8_t *tx)
{

    struct spi_ioc_transfer tr[1] = {
    {
        .tx_buf = (unsigned long)tx,
        .len = len,
        .delay_usecs = 10,
    },
    };
    errno = 0;
    ioctl(fd, SPI_IOC_MESSAGE(1), &(tr));
    if (errno != 0)
    {
        // failed
    }
}


uint8_t *CAN::readInfo(addr, uint8_t *tx)   
{
    struct spi_ioc_transfer tr[2] = {
    {
        .tx_buf = (unsigned long)tx,
        .len = tx_len,
        .delay_usecs = 10,
    },
    {
        .rx_buf = (unsigned long)rx,
        .len = rx_len,
        .delay_usecs = 10,
    },
    };
    errno = 0;
    ioctl(fd, SPI_IOC_MESSAGE(2), &(tr));
    if (errno != 0)
    {
        // failed
    }
    return rx;
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





