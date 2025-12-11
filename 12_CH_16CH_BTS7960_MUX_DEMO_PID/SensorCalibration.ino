void calibrateSensor() {
  // Initialize calibration limits
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Set the maximum ADC value for each sensor to 0 (lowest possible)
    Min_ADC[i] = 1024;  // Set the minimum ADC value for each sensor to 1024 (highest possible, since ADC range is 0–1023)
  }
  // Define rotation directions for calibration sweeps
  int rotationDirections[][2] = {
    { -1, 1 },  // Rotate robot slightly to the left
    { 1, -1 },  // Rotate robot slightly to the right
    { 1, -1 },  // Repeat rotation to the right for better calibration coverage
    { -1, 1 }   // Rotate left again to cover the full sensor range
  };

  // Perform calibration sweeps (4 passes)
  for (int step = 0; step < 4; step++) {
    // Rotate the robot slowly in the defined direction for this step
    motor(base_speed * 0.4 * rotationDirections[step][0],
          base_speed * 0.4 * rotationDirections[step][1]);

    // Sweep through several readings while rotating
    for (int sweep = 0; sweep < 500; sweep++) {
      // Read each sensor one by one
      for (int i = 0; i < sensorNumber; i++) {
        selectChannel(i);                    // Select which sensor channel to read (if using multiplexer)
        sensorADC[i] = analogRead(SIG_PIN);  // Read the analog value from the sensor

        // Update the maximum and minimum ADC values recorded for this sensor
        Max_ADC[i] = max(Max_ADC[i], sensorADC[i]);
        Min_ADC[i] = min(Min_ADC[i], sensorADC[i]);
      }
    }
    motor(0, 0);  // Stop the motors after each rotation step
    delay(200);   // Small pause before changing rotation direction
  }

  // Store calibrated Reference, Max, and Min values in EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.0;  // Calculate mid-point (reference) value

    EEPROM.write(i, Reference_ADC[i] / 4);           // Store reference value (divided by 4 because EEPROM stores 0–255)
    delay(5);                                        // Small delay for EEPROM write stability
    EEPROM.write(i + sensorNumber, Max_ADC[i] / 4);  // Store maximum calibrated value
    delay(5);
    EEPROM.write(i + (sensorNumber * 2), Min_ADC[i] / 4);  // Store minimum calibrated value
    delay(5);
  }
}

// Function to load calibration data from EEPROM
void LoadCalibration() {
  // Print header labels for serial monitor (sensor indices)
  Serial.print("S0: ");
  Serial.print(", S1: ");
  Serial.print(", S2: ");
  Serial.print(", S3: ");
  Serial.print(", S4: ");
  Serial.print(", S5: ");
  Serial.print(", S6: ");
  Serial.print(", S7: ");
  Serial.println();

  // Load the stored calibration data for each sensor
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;                 // Read and restore reference value
    Max_ADC[i] = EEPROM.read(i + sensorNumber) * 4;        // Read and restore maximum value
    Min_ADC[i] = EEPROM.read(i + (sensorNumber * 2)) * 4;  // Read and restore minimum value

    Serial.print(String(Reference_ADC[i]) + ",  ");  // Print loaded reference value
  }
  Serial.println();  // Move to new line after printing all sensor values
}
