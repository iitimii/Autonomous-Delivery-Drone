void telem_setup() {
    radio.begin();            
    radio.setChannel(120);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN); 
    radio.setAutoAck(false);
    radio.openWritingPipe(addresses[0]); 
    // radio.openReadingPipe(1, addresses[1]);
    radio.stopListening();
}


void send_telemetry() {

  switch (telemetry_loop_counter){

    case 0:
    telemetry_send_signature = 'T';
    telemetry_send_payload1 = loop_time_count;
    telemetry_send_payload2 = lon_gps_actual;
    telemetry_send_payload3 = lat_gps_actual;
    telemetry_send_payload4 = number_used_sats;
    telemetry_send_payload5 = P;
    telemetry_send_payload6 = T;
    break;


  case 1:
    telemetry_send_signature = 'I';
    telemetry_send_payload1 = gyro_pitch;
    telemetry_send_payload2 = gyro_roll;
    telemetry_send_payload3 = channel_1;
    telemetry_send_payload4 = channel_2;
    telemetry_send_payload5 = channel_3;
    telemetry_send_payload6 = error;
    break;
  

  }




  data_tx.signature = telemetry_send_signature;
  data_tx.payload1 = telemetry_send_payload1;
  data_tx.payload2 = telemetry_send_payload2;
  data_tx.payload3 = telemetry_send_payload3;
  data_tx.payload4 = telemetry_send_payload4;
  data_tx.payload5 = telemetry_send_payload5;
  data_tx.payload6 = telemetry_send_payload6;

  telemetry_loop_counter++;
  if (telemetry_loop_counter >= 2)telemetry_loop_counter = 0;                             
  radio.stopListening();
  radio.write(&data_tx, sizeof(TXData));//);
  radio.startListening();
}



// void telem_setup2(){
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
//   nrfSPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
//   nrfSPI.transfer(CSNPIN, buf, SPI_CONTINUE);
//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device

// }



// void nrf_write_reg_multi(uint8_t reg, uint8_t *Data, int size){
//   uint8_t buf[2];
//   buf[0] = reg | (1 << 5);
//   // buf[1] = data;

//   digitalWrite(CSNPIN, LOW);    //Pull csn to select nrf device
//   nrfSPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
//   nrfSPI.transfer(CSNPIN, buf, sizeof(buf), SPI_CONTINUE);
//   nrfSPI.endTransaction();
//   digitalWrite(CSNPIN, HIGH);    //Pull csn to release nrf device
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