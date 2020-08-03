
void IRAM_ATTR onTime() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (interrupts==0){interrupts=1;}
  portEXIT_CRITICAL_ISR(&timerMux);
}
 
void setupInterrupts() {
   // Configure Prescaler to 80, as our timer runs @ 80Mhz
  // Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
  timer = timerBegin(0, 80, true);                
  timerAttachInterrupt(timer, &onTime, true);    
  // Fire Interrupt every 50000 ticks, so 20 times per second (every 50 millis)
  timerAlarmWrite(timer, 25000, true);      //Cada 25 ms (40 veces por segundo)
  timerAlarmEnable(timer);
}
