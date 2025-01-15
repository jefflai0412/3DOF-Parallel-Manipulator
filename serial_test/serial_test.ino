void setup() {
  // Start the serial communication
  Serial.begin(115200); // Set baud rate to 115200; you can adjust if needed.
  Serial.println("Ready to receive and echo data...");
}

void loop() {
  // Check if data is available on the serial input
  if (Serial.available() > 0) {
    // Read the incoming data
    String receivedData = Serial.readStringUntil('\n');
    
    // Echo the received data back to the serial output
    Serial.print("Esp32 Received: ");
    Serial.println(receivedData);
    
    // Optionally, you can perform additional actions with the data here
  }
}
