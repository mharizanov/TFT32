//WiFiClientSecure espClient;   // use this , if using TLS
WiFiClient espClient;
PubSubClient client(espClient);

String clientId;
long lastMsg = 0;
char msg[50];
int value = 0;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)topic[strlen(mqtt_intopic)-1] == '1' && (char)payload[0] == '1')  set_switch_state(1,1);
  if ((char)topic[strlen(mqtt_intopic)-1] == '1' && (char)payload[0] == '0')  set_switch_state(1,0);

  if ((char)topic[strlen(mqtt_intopic)-1] == '2' && (char)payload[0] == '1')  set_switch_state(2,1);
  if ((char)topic[strlen(mqtt_intopic)-1] == '2' && (char)payload[0] == '0')  set_switch_state(2,0);

}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection ");
    Serial.print("; Client ID = ");Serial.print(clientId);Serial.print(" ...");
        
    // Attempt to connect
    int connection_result = false;
    if (mqtt_user =="" ) {
      connection_result = client.connect(clientId.c_str());
    } else {
      connection_result = client.connect(clientId.c_str(),mqtt_user, mqtt_pass);
    }

    if (connection_result) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(mqtt_outtopic, "starting..");
      // ... and resubscribe
      client.subscribe(mqtt_intopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqtt_loop() {

  if (!client.connected()) {
    mqtt_reconnect();
  }
  client.loop();
}

void mqtt_task( void * parameter )
{
   while(1) {
      mqtt_loop();
      delay(50);
   }
    vTaskDelete( NULL ); // Should never be reached
}

void mqtt_setup() {
    
  // Create a random client ID
  clientId = "TFT32-";
  clientId += String(random(0xffff), HEX);
    
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);

  xTaskCreate(
                    mqtt_task,        /* Task function. */
                    "mqtt",           /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    5,                /* Priority of the task. */
                    NULL);            /* Task handle. */
}

