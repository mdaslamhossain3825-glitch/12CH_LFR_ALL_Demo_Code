void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;

  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    //map analog value x
    //analog to digital
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {    //black line - normal mode.
        sensorDigital[i] = 1;  //black line =1
      } else {
        sensorDigital[i] = 0;  //black line =0 (black line -inverse mode)
      }
    } else {
      if (inverseON == 0) {    //white line - normal mode
        sensorDigital[i] = 0;  //white line =0 - normal mode
      } else {
        sensorDigital[i] = 1;  //white line - inverse mode.
      }
    }

    sumOnSensor += sensorDigital[i];
    sensorWight += sensorDigital[i] * WeightValue[i];
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void analog_value_test() {
  while (1) {
    read_black_line();
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(String(sensorADC[i]) + "  ");
    }
    Serial.println();
  }
}

void Bit_Sensor_Show() {
  while (1) {
    read_black_line();  //take sensor reading

    Serial.print("Bit Sensor in Decimal: " + String(bitSensor));  //Decimal Value Show
    Serial.print(" | Bit Sensor in Binary: ");

    for (int i = (sensorNumber - 1); i >= 0; i--) {  //display bitsensor data in binary
      Serial.print(String(bitRead(bitSensor, i)));
    }
    Serial.println();
  }
}