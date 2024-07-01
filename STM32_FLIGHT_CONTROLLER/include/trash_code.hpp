////////////////////////////////////////////////////////////////////////////////////////////////

// void nrf_spi_setup2(){
//   SPIClass nrfSPI(PA7, PA6, PA5);
//   pinMode(CSNPIN, OUTPUT);
//   nrfSPI.begin(CSNPIN);
//   nrfSPI.setBitOrder(MSBFIRST);
//   nrfSPI.setDataMode(SPI_MODE0);
// }

// void nrf_write_reg(uint8_t reg, uint8_t data){
//   uint8_t buf[2];
//   buf[0] = reg| (1<<5);
//   buf[1] = data;

//   digitalWrite(CSNPIN, LOW);    //Pull csn to select nrf device
//   nrfSPI.beginTransaction(nrfSPISettings);
//   nrfSPI.transfer(CSNPIN, buf, SPI_CONTINUE);
//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device

// }

// void nrf_write_reg_multi(uint8_t reg, uint8_t *data, int size){
//   uint8_t buf[2];
//   buf[0] = reg | (1 << 5);
//   // buf[1] = data;

//   digitalWrite(CSNPIN, LOW);    //Pull csn to select nrf device
//   nrfSPI.beginTransaction(nrfSPISettings);
//   nrfSPI.transfer(CSNPIN, buf, sizeof(buf), SPI_CONTINUE);
//   nrfSPI.transfer(CSNPIN, data, sizeof(data), SPI_CONTINUE);
//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device
// }

// uint8_t nrf_read_reg(uint8_t Reg){
//   uint8_t data = 0;
//   digitalWrite(CSNPIN, LOW);    //Pull csn to select nrf device
//   nrfSPI.beginTransaction(nrfSPISettings);
//   nrfSPI.transfer(CSNPIN, Reg, SPI_CONTINUE);
//   data = nrfSPI.transfer(CSNPIN, 0xff, SPI_CONTINUE);
//   // HAL_SPI_Receive();
//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device

//   return data;
// }

// void nrf_read_reg_multi(uint8_t reg, uint8_t *data, int size){
//   digitalWrite(CSNPIN, LOW);    //Pull csn to select nrf device
//   nrfSPI.beginTransaction(nrfSPISettings);
//   nrfSPI.transfer(CSNPIN, reg, SPI_CONTINUE); // Send the register address

//   for(int i = 0; i < size; i++) {
//     data[i] = nrfSPI.transfer(CSNPIN, 0xFF, SPI_CONTINUE); // Read data from the register
//   }

//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device
// }

///////////////////////////////////////////////////////////////////////////////////////////////////
// void SPI1_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* Peripheral clock enable */
//   __HAL_RCC_SPI1_CLK_ENABLE();

//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   /**SPI1 GPIO Configuration
//   PA5     ------> SPI1_SCK
//   PA6     ------> SPI1_MISO
//   PA7     ------> SPI1_MOSI
//   */
//   GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
//   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   hspi1.Instance = SPI1;
//   hspi1.Init.Mode = SPI_MODE_MASTER;
//   hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//   hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//   hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//   hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//   hspi1.Init.NSS = SPI_NSS_SOFT;
//   hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//   hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//   hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//   hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//   hspi1.Init.CRCPolynomial = 10;
//   if (HAL_SPI_Init(&hspi1) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// void CS_Select (void)
// {
// 	digitalWrite(CSNPIN, LOW);
// }

// void CS_UnSelect (void)
// {
// 	digitalWrite(CSNPIN, HIGH);
// }

// void CE_Enable (void)
// {
// 	digitalWrite(CEPIN, LOW);
// }

// void CE_Disable (void)
// {
// 	digitalWrite(CEPIN, HIGH);
// }

// // write a single byte to the particular register
// void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
// {
// 	uint8_t buf[2];
// 	buf[0] = Reg|1<<5;
// 	buf[1] = Data;

// 	// Pull the CS Pin LOW to select the device
// 	CS_Select();

// 	HAL_SPI_Transmit(NRF24_SPI, buf, 2, 1000);

// 	// Pull the CS HIGH to release the device
// 	CS_UnSelect();
// }

// //write multiple bytes starting from a particular register
// void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
// {
// 	uint8_t buf[2];
// 	buf[0] = Reg|1<<5;
// //	buf[1] = Data;

// 	// Pull the CS Pin LOW to select the device
// 	CS_Select();

// 	HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
// 	HAL_SPI_Transmit(NRF24_SPI, data, size, 1000);

// 	// Pull the CS HIGH to release the device
// 	CS_UnSelect();
// }

// uint8_t nrf24_ReadReg (uint8_t Reg)
// {
// 	uint8_t data=0;

// 	// Pull the CS Pin LOW to select the device
// 	CS_Select();

// 	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
// 	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

// 	// Pull the CS HIGH to release the device
// 	CS_UnSelect();

// 	return data;
// }

// /* Read multiple bytes from the register */
// void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
// {
// 	// Pull the CS Pin LOW to select the device
// 	CS_Select();

// 	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
// 	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

// 	// Pull the CS HIGH to release the device
// 	CS_UnSelect();
// }

// // send the command to the NRF
// void nrfsendCmd (uint8_t cmd)
// {
// 	// Pull the CS Pin LOW to select the device
// 	CS_Select();

// 	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

// 	// Pull the CS HIGH to release the device
// 	CS_UnSelect();
// }

// void nrf24_reset(uint8_t REG)
// {
// 	if (REG == STATUS)
// 	{
// 		nrf24_WriteReg(STATUS, 0x00);
// 	}

// 	else if (REG == FIFO_STATUS)
// 	{
// 		nrf24_WriteReg(FIFO_STATUS, 0x11);
// 	}

// 	else {
// 	nrf24_WriteReg(CONFIG, 0x08);
// 	nrf24_WriteReg(EN_AA, 0x3F);
// 	nrf24_WriteReg(EN_RXADDR, 0x03);
// 	nrf24_WriteReg(SETUP_AW, 0x03);
// 	nrf24_WriteReg(SETUP_RETR, 0x03);
// 	nrf24_WriteReg(RF_CH, 0x02);
// 	nrf24_WriteReg(RF_SETUP, 0x0E);
// 	nrf24_WriteReg(STATUS, 0x00);
// 	nrf24_WriteReg(OBSERVE_TX, 0x00);
// 	nrf24_WriteReg(CD, 0x00);
// 	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
// 	nrf24_WriteRegMulti(RX_ADDR_P0, rx_addr_p0_def, 5);
// 	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
// 	nrf24_WriteRegMulti(RX_ADDR_P1, rx_addr_p1_def, 5);
// 	nrf24_WriteReg(RX_ADDR_P2, 0xC3);
// 	nrf24_WriteReg(RX_ADDR_P3, 0xC4);
// 	nrf24_WriteReg(RX_ADDR_P4, 0xC5);
// 	nrf24_WriteReg(RX_ADDR_P5, 0xC6);
// 	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
// 	nrf24_WriteRegMulti(TX_ADDR, tx_addr_def, 5);
// 	nrf24_WriteReg(RX_PW_P0, 0);
// 	nrf24_WriteReg(RX_PW_P1, 0);
// 	nrf24_WriteReg(RX_PW_P2, 0);
// 	nrf24_WriteReg(RX_PW_P3, 0);
// 	nrf24_WriteReg(RX_PW_P4, 0);
// 	nrf24_WriteReg(RX_PW_P5, 0);
// 	nrf24_WriteReg(FIFO_STATUS, 0x11);
// 	nrf24_WriteReg(DYNPD, 0);
// 	nrf24_WriteReg(FEATURE, 0);
// 	}
// }

// void NRF24_Init (void)
// {
// 	// disable the chip before configuring the device
// 	CE_Disable();

// 	// reset everything
// 	nrf24_reset (0);

// 	nrf24_WriteReg(CONFIG, 0);  // will be configured later

// 	nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

// 	nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

// 	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

// 	nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

// 	nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

// 	nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

// 	// Enable the chip after configuring the device
// 	CE_Enable();

// }

// // set up the Tx mode

// void NRF24_TxMode (uint8_t *Address, uint8_t channel)
// {
// 	// disable the chip before configuring the device
// 	CE_Disable();

// 	nrf24_WriteReg (RF_CH, channel);  // select the channel

// 	nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Write the TX address

// 	// power up the device
// 	uint8_t config = nrf24_ReadReg(CONFIG);
// //	config = config | (1<<1);   // write 1 in the PWR_UP bit
// 	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
// 	nrf24_WriteReg (CONFIG, config);

// 	// Enable the chip after configuring the device
// 	CE_Enable();
// }

// // transmit the data

// uint8_t NRF24_Transmit (uint8_t *data)
// {
// 	uint8_t cmdtosend = 0;

// 	// select the device
// 	CS_Select();

// 	// payload command
// 	cmdtosend = W_TX_PAYLOAD;
// 	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

// 	// send the payload
// 	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

// 	// Unselect the device
// 	CS_UnSelect();

// 	HAL_Delay(1);
//   delayMicroseconds(10);

// 	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

// 	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
// 	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
// 	{
// 		cmdtosend = FLUSH_TX;
// 		nrfsendCmd(cmdtosend);

// 		// reset FIFO_STATUS
// 		nrf24_reset (FIFO_STATUS);

// 		return 1;
// 	}

// 	return 0;
// }

// void NRF24_RxMode (uint8_t *Address, uint8_t channel)
// {
// 	// disable the chip before configuring the device
// 	CE_Disable();

// 	nrf24_reset (STATUS);

// 	nrf24_WriteReg (RF_CH, channel);  // select the channel

// 	// select data pipe 2
// 	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
// 	en_rxaddr = en_rxaddr | (1<<2);
// 	nrf24_WriteReg (EN_RXADDR, en_rxaddr);

// 	/* We must write the address for Data Pipe 1, if we want to use any pipe from 2 to 5
// 	 * The Address from DATA Pipe 2 to Data Pipe 5 differs only in the LSB
// 	 * Their 4 MSB Bytes will still be same as Data Pipe 1
// 	 *
// 	 * For Eg->
// 	 * Pipe 1 ADDR = 0xAABBCCDD11
// 	 * Pipe 2 ADDR = 0xAABBCCDD22
// 	 * Pipe 3 ADDR = 0xAABBCCDD33
// 	 *
// 	 */
// 	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
// 	nrf24_WriteReg(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address

// 	nrf24_WriteReg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2

// 	// power up the device in Rx mode
// 	uint8_t config = nrf24_ReadReg(CONFIG);
// 	config = config | (1<<1) | (1<<0);
// 	nrf24_WriteReg (CONFIG, config);

// 	// Enable the chip after configuring the device
// 	CE_Enable();
// }

// uint8_t isDataAvailable (int pipenum)
// {
// 	uint8_t status = nrf24_ReadReg(STATUS);

// 	if ((status&(1<<6))&&(status&(pipenum<<1)))
// 	{

// 		nrf24_WriteReg(STATUS, (1<<6));

// 		return 1;
// 	}

// 	return 0;
// }

// void NRF24_Receive (uint8_t *data)
// {
// 	uint8_t cmdtosend = 0;

// 	// select the device
// 	CS_Select();

// 	// payload command
// 	cmdtosend = R_RX_PAYLOAD;
// 	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

// 	// Receive the payload
// 	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

// 	// Unselect the device
// 	CS_UnSelect();

// 	// HAL_Delay(1);
//   delayMicroseconds(10);

// 	cmdtosend = FLUSH_RX;
// 	nrfsendCmd(cmdtosend);
// }

// // Read all the Register data
// void NRF24_ReadAll (uint8_t *data)
// {
// 	for (int i=0; i<10; i++)
// 	{
// 		*(data+i) = nrf24_ReadReg(i);
// 	}

// 	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+10), 5);

// 	nrf24_ReadReg_Multi(RX_ADDR_P1, (data+15), 5);

// 	*(data+20) = nrf24_ReadReg(RX_ADDR_P2);
// 	*(data+21) = nrf24_ReadReg(RX_ADDR_P3);
// 	*(data+22) = nrf24_ReadReg(RX_ADDR_P4);
// 	*(data+23) = nrf24_ReadReg(RX_ADDR_P5);

// 	nrf24_ReadReg_Multi(RX_ADDR_P0, (data+24), 5);

// 	for (int i=29; i<38; i++)
// 	{
// 		*(data+i) = nrf24_ReadReg(i-12);
// 	}

// }

/*
void send_telemetry2() {
  telemetry_loop_counter++;

  data_tx.payload = telemetry_send_byte;

  if (telemetry_loop_counter == 125)telemetry_loop_counter = 0;
  radio.stopListening();



  radio.startListening();
}

void read_telemetry() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&data_rx, sizeof(data_rx));
    radio.stopListening();
  }
}



bool RF24::write(const void* buf, uint8_t len, const bool multicast=true)
{
    bool startTx = true;
    const uint8_t writeType = multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD;
    const void* buf = buf;
    uint8_t data_len = len;
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    uint8_t blank_len = !data_len ? 1 : 0;
    data_len = rf24_min(data_len, payload_size);
    blank_len = static_cast<uint8_t>(payload_size - data_len);
    digitalWrite(CEPIN, HIGH);


    digitalWrite(CEPIN, LOW);
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}






void RF24::write_register(uint8_t reg, uint8_t value, bool is_cmd_only)
{
    if (is_cmd_only) {
        _spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
        csn(LOW);
        status = _spi->transfer(W_REGISTER | reg);
        csn(HIGH);
      _spi->endTransaction();
    }
    else {
        _spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
        csn(LOW);
        status = _spi->transfer(W_REGISTER | reg);
        _spi->transfer(value);
        csn(HIGH);
      _spi->endTransaction();
    }
}





inline void RF24::beginTransaction()
{
    _spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
    csn(LOW);
}

inline void RF24::endTransaction()
{
    csn(HIGH);
    _spi->endTransaction();
}

*/

/*
"T"
"M"
Longitude int32
Latitude  int32
error
flight mode
return_to_home_step
battery voltage
temperature
roll
pitch
yaw
start
altitude
pressure
takeoff throttle
compass heading
heading lock
num sats
fix type
adjustable setting1
adjustable setting2
adjustable setting3
adjustable setting4
check byte
Loop time
Telemetry time
Speed
PID roll
PID yaw
PID pitch
PID Alt
SW A-D int32
VR A-B int32
ch1-4 int32
*/