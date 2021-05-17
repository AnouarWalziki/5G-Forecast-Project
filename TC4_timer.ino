// Set timer TC4 to call the TC4_Handler every 0.625ms: (1 * (29999 + 1)) / 48MHz = 0.625ms
void setup() {
  PORT->Group[PORTA].DIRSET.reg = PORT_PA21;               // Set D7 as a digital output
  
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |                 // Enable GCLK0 for TC4 and TC5
        GCLK_CLKCTRL_GEN_GCLK0 |             // Select GCLK0 at 48MHz
        GCLK_CLKCTRL_ID_TC4_TC5;             // Feed GCLK0 output to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);                       // Wait for synchronization

  TC4->COUNT16.CC[0].reg = 29999;                          // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;             // Enable TC4 overflow (OVF) interrupts

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC |     // Reset timer on the next prescaler clock
        TC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 1, 48MHz/1 = 48MHz
        TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode 
        TC_CTRLA_MODE_COUNT16;         // Set the timer to 16-bit mode      
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC4 timer
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

void loop() {}

void TC4_Handler()                                         // Interrupt Service Routine (ISR) for timer TC4
{      
  //if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)       // Optionally check for overflow (OVF) interrupt      
  //{   
    PORT->Group[PORTA].OUTTGL.reg = PORT_PA21;             // Toggle the D7 output
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
  //}
}