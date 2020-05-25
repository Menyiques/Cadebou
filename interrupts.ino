
void IRAM_ATTR onTime() {
  portENTER_CRITICAL_ISR(&timerMux);
  interrupts++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
 
void setup_interrupts() {
   // Configure Prescaler to 80, as our timer runs @ 80Mhz
  // Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
  timer = timerBegin(0, 80, true);                
  timerAttachInterrupt(timer, &onTime, true);    
  // Fire Interrupt every 50000 ticks, so 20 times per second (every 50 millis)
  timerAlarmWrite(timer, 50000, true);      
  timerAlarmEnable(timer);
}
