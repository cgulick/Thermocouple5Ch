




 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  TimerIRQCounter++;
  TimerInterruptFlag = TRUE;
  ErrorToggle++;
  ErrorToggle&=1;
  SecondHandCount++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

void TimerIrqSetup()
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
}


void ClearTimerIrqFlag()
{
  portENTER_CRITICAL(&timerMux);
  TimerInterruptFlag = FALSE;
  portEXIT_CRITICAL(&timerMux);
}
