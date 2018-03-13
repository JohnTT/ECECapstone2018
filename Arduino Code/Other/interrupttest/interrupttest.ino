SoftwareTimer blinkTimer;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  blinkTimer.begin(10, blink_timer_callback);
  blinkTimer.start();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  Serial.println("Test");
}
