
#ifndef BEENHERE
#include "SDT.h"
#endif

/*****
  Purpose: Determine which UI button was pressed

  Parameter list:
    int valPin            the ADC value from analogRead()

  Return value;
    int                   -1 if not valid push button, index of push button if valid
*****/
int ProcessButtonPress(int valPin) {
#ifdef M0JTSBUTTONINJECT
  return valPin;
#else
  int switchIndex;

  if (valPin == BOGUS_PIN_READ) {  // Not valid press
    return -1;
  }
  if (valPin == MENU_OPTION_SELECT && menuStatus == NO_MENUS_ACTIVE) {
    NoActiveMenu();
    return -1;
  }
  for (switchIndex = 0; switchIndex < NUMBER_OF_SWITCHES; switchIndex++) {
    if (abs(valPin - EEPROMData.switchValues[switchIndex]) < WIGGLE_ROOM)  // ...because ADC does return exact values every time
    {
      return switchIndex;
    }
  }
  return -1;  // Really should never do this
#endif
}

/*****
  Purpose: Check for UI button press. If pressed, return the ADC value

  Parameter list:
    int vsl               the value from analogRead in loop()\

  Return value;
    int                   -1 if not valid push button, ADC value if valid
*****/
int ReadSelectedPushButton() {
#ifdef M0JTSBUTTONINJECT
  int returnValue = M0JTSButtonPressed;
  M0JTSButtonPressed = -1;
  return returnValue;
#else
  minPinRead = 0;
  int buttonReadOld = 1023;

  while (abs(minPinRead - buttonReadOld) > 3) {  // do averaging to smooth out the button response
    minPinRead = analogRead(BUSY_ANALOG_PIN);
    buttonRead = .1 * minPinRead + .9 * buttonReadOld;  // See expected values in next function.
    buttonReadOld = buttonRead;
  }
  //Serial.print("In ReadSelectedPushButton: minPinRead = ");
  //Serial.println(minPinRead);

  if (buttonRead >= EEPROMData.switchValues[0] + WIGGLE_ROOM) {  //AFP 10-29-22   too large?
    return -1;
  }
  minPinRead = buttonRead;
  MyDelay(100L);
  //("    minPinRead = ");
  //Serial.println(minPinRead);
  return minPinRead;
#endif
}

/*****
  Purpose: Function is designed to route program control to the proper execution point in response to
           a button press.

  Parameter list:
    int vsl               the value from analogRead in loop()

  Return value;
    void
*****/
void ExecuteButtonPress(int val) {
  /*
  if (val == 1) {                           // If they selected Menu Up
    DrawMenuDisplay();
  }

  if (val == MENU_OPTION_SELECT && menuStatus == NO_MENUS_ACTIVE) {  // Pressed Select with no primary/secondary menu selected
    NoActiveMenu();
    return;
  } else {
    menuStatus = PRIMARY_MENU_ACTIVE;
  }
    menuStatus = PRIMARY_MENU_ACTIVE;
*/
  MyDelay(100L);
  //("top of ExecuteButtonPress(),     val = ");
 //Serial.println(val);

  switch (val) {
    case MENU_OPTION_SELECT:  // 0
      break;

      /*  

      const char *topMenus[] = { "CW Options", "RF Set", "VFO Select",
                           "EEPROM", "AGC", "Spectrum Options",
                           "Noise Floor", "Mic Gain", "Mic Comp",
                           "EQ Rec Set", "EQ Xmt Set", "Calibrate", "Bearing" };

int (*functionPtr[])() = { &CWOptions, &RFOptions, &VFOSelect,
                           &EEPROMOptions, &AGCOptions, &SpectrumOptions,
                           &ButtonSetNoiseFloor, &MicGainSet, &MicOptions,
                           &EqualizerRecOptions, &EqualizerXmtOptions, &IQOptions, &BearingMaps };

*/
    case MAIN_MENU_UP:                      // 1          11/16/23 JJP                                     Button 1
      DrawMenuDisplay();                    // Draw selection box and primary menu
      SetPrimaryMenuIndex();                // Scroll through primary indexes and select one
      if (mainMenuWindowActive == false) {  // Was Main Menu choice cancelled?}
        mainMenuWindowActive = false;
        EraseMenus();
        RedrawDisplayScreen();
        ShowFrequency();
        DrawFrequencyBarValue();
        break;
      }
      SetSecondaryMenuIndex();  // Use the primary index selection to redraw the secondary menu and set its index

      secondaryMenuChoiceMade = functionPtr[mainMenuIndex]();

      tft.fillRect(1, SPECTRUM_TOP_Y + 1, 513, 379, RA8875_BLACK);  // Erase Menu box
                                                                    //      DrawSpectrumDisplayContainer();
      EraseMenus();
      RedrawDisplayScreen();
      ShowFrequency();
      DrawFrequencyBarValue();

      break;

    case BAND_UP:  // 2         Now calls ProcessIQData and Encoders calls                    Button 2
      EraseMenus();
      if (currentBand < 5) digitalWrite(bandswitchPins[currentBand], LOW);  // Added if so unused GPOs will not be touched.  KF5N October 16, 2023.
      ButtonBandIncrease();
      if (currentBand < 5) digitalWrite(bandswitchPins[currentBand], HIGH);
      BandInformation();
      NCOFreq = 0L;
      DrawBandWidthIndicatorBar();  // AFP 10-20-22
      SetFreq();
      ShowSpectrum();
      break;

    case ZOOM:  // 3
      menuStatus = PRIMARY_MENU_ACTIVE;
      //      EraseMenus();
      ButtonZoom();
      break;

    //case MAIN_MENU_DN:                                // 4
    case RESET_TUNING:
//      Serial.println("Reset Tuning selected.");
      ResetTuning();
      break;

    case BAND_DN:  // 5
      EraseMenus();
      ShowSpectrum();  //Now calls ProcessIQData and Encoders calls
      if (currentBand < 5) digitalWrite(bandswitchPins[currentBand], LOW);
      ButtonBandDecrease();
      if (currentBand < 5) digitalWrite(bandswitchPins[currentBand], HIGH);
      BandInformation();
      NCOFreq = 0L;
      DrawBandWidthIndicatorBar();  //AFP 10-20-22
      break;

    case SET_MODE:  // 6
                    //      EraseMenus();
                    //      ButtonFilter();
      ButtonMode();
      ShowSpectrumdBScale();
      break;

    case DEMODULATION:  // 7
      EraseMenus();
      ButtonDemodMode();
      break;

    case MAIN_TUNE_INCREMENT:  // 8
//      Serial.println("Tune increment selected.");
      ButtonFreqIncrement();
      break;

    case NOISE_REDUCTION:  // 9
//      Serial.print("val = ");
//      Serial.println(val);
      ButtonNR();
      break;

    case NOTCH_FILTER:  // 10
      ButtonNotchFilter();
      UpdateNotchField();
      break;

    case FINE_TUNE_INCREMENT:  // 11
      UpdateIncrementField();
      break;

    case FILTER:  // 12
      ButtonFilter();
      break;

    case DECODER_TOGGLE:  // 13
//      Serial.println("Decoder Toggle selected.");
      decoderFlag = !decoderFlag;
      UpdateDecoderField();
      break;

    case UNUSED_1:  // 14

      break;

    case UNUSED_2:  // 15
      break;

    case UNUSED_3:  // 16
      if (calOnFlag == 0) {
        ButtonFrequencyEntry();
      }
      break;

    case UNUSED_4:  // 17
 
      
      break;

    default:
      break;
  }
}


/*****
  Purpose: To process a band decrease button push

  Parameter list:
    void

  Return value:
    void
*****/
void ButtonFreqIncrement() {
  tuneIndex--;
  if (tuneIndex < 0)
    tuneIndex = MAX_FREQ_INDEX - 1;
  freqIncrement = incrementValues[tuneIndex];
  DisplayIncrementField();
}


/*****
  Purpose: Error message if Select button pressed with no Menu active

  Parameter list:
    void

  Return value;
    void
*****/
void NoActiveMenu() {
  tft.setFontScale((enum RA8875tsize)1);
  tft.setTextColor(RA8875_RED);
  tft.setCursor(10, 0);
  tft.print("No menu selected");

  menuStatus = NO_MENUS_ACTIVE;
  mainMenuIndex = 0;
  secondaryMenuIndex = 0;
}
