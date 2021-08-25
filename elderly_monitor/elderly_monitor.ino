/*Copyright (C) 2021 Telit Communications S.p.A. Italy - All Rights Reserved.*/
/*    See LICENSE file in the project root for full license information.     */

/**
  @file
    elderly_monitor.ino

  @brief
    Elder tracker demo

  @details
    Telit Charlie Evaluation Kit: Elder tracker

    This example shows how to enable the onboard features,
    and how to use the embedded BOSCH Sensor
    in order to build a simple elder people tracker


  @version
    1.0.0

  @note

  @author
    Cristina Desogus

  @date
    07/10/2021
*/

#include <telit_bma400.h>
#include <TLTMDM.h>

#define USER_PB 6 /* SAMD21 PIN nr. 7 */
#define USER_LD 7 /* SAMD21 PIN nr. 21 */

#define DEBUG 0

ME310* _me310 = new ME310();
TLTSMS sms(_me310);

/*When NMEA_DEBUG is false Unsolicited NMEA is disabled*/
/*NMEA is true*/
TLTGNSS gnss(_me310, true);

/*STEP COUNTER*/
struct bma400_dev bma;
struct bma400_sensor_conf accel_setting[2];
struct bma400_int_enable int_en[2];

/*SMS destination number*/
char remoteNum[20] = "+39XXXXXXXXX";

char txtMsg[200];

uint8_t buttonState = 0;
uint8_t prevButtonState = 0;
unsigned long lastButtonPress = 0;
unsigned long lastButtonDebounce = 0;
uint8_t buttonDebounceDelay = 50;
uint16_t buttonAlarmDelay = 1000;
unsigned long loopTime = 0;
int8_t rslt = 0;

uint32_t step_count = 0;
uint8_t act_int = 0;
uint16_t int_status = 0;

void setup()
{
  memset(accel_setting, 0, sizeof(struct bma400_sensor_conf));

  Serial.begin(115200);
  _me310->begin(115200);
  delay(1000);
  _me310->powerOn();
  delay(5000);
  Serial.println("Start elderly monitoring application");
  delay(1000);
  Serial.print("Initializing GNSS");
  while(!gnss.setGNSSConfiguration())
  {
    Serial.print(".");
  }
  Serial.println(" is completed successfully");
  Serial.println("BMA400 Setting");
  if(bma400Setting())
  {
    Serial.println("BMA400 is set OK");
    switchOperatorSelection(false);
    delay(3000);
    Serial.println("Press USR button at any time to send an emergency SMS");
  }
}

void loop()
{
  bool res = true;
  loopTime = millis();

  rslt = bma400_get_interrupt_status(&int_status, &bma);
  if(rslt == 0)
  {
    if (int_status & BMA400_ASSERTED_STEP_INT)
    {
      rslt = bma400_get_steps_counted(&step_count, &act_int, &bma);
    }
    delay(10);
  }
  res = checkUserButton();
  if(res == true)
  {
    if(gnss.setGNSSConfiguration())
    {
      if(switchOperatorSelection(false))
      {
        Serial.println("Ready for another run!");
      }
    }
  }
  delay(10);
}

bool checkUserButton()
{
  bool ret = false;
  uint8_t tempButtonState = digitalRead(USER_PB);
  /* Let's debounce the button in order to have proper reading */
  if( tempButtonState != prevButtonState )
  {
    lastButtonDebounce = loopTime;
  }
  if((loopTime - lastButtonDebounce) > buttonDebounceDelay)
  {
    /* Regardless from the reading state, it has been there for longer
     * than the debounce delay, so take it as the actual current state:
     */

    /* If the button state has changed */
    if(tempButtonState != buttonState)
    {
      buttonState = tempButtonState;

      /* If the button is pressed (LOW)
      * toggle the led ON and set the last button press time
      */
      if(buttonState == LOW)
      {
        lastButtonPress = loopTime;
        digitalWrite(USER_LD, HIGH);
      }
      else
      {
        digitalWrite(USER_LD, LOW);
      }
      ret = false;
    }
    else
    {
      if(buttonState == LOW && loopTime - lastButtonPress > buttonAlarmDelay)
      {
        ret = true;
        int messageSize = 0;
        /* -----
        * By keeping the button pressed for buttonAlarmDelay,
        * an SMS is sent to a custom number
        */
        Serial.println( "Button alert." );
        Serial.println("Waiting...");

        GNSSInfo gnssInfo = gnss.getGNSSData();

        if(gnss.unsetGNSSConfiguration())
        {
          delay(3000);

          if(switchOperatorSelection(true))
          {
            sendMessage(gnssInfo);
          }
          else
          {
            Serial.println("\nERROR\n");
          }
        }
      }
    }
  }
  prevButtonState = tempButtonState;
  return ret;
}

bool bma400Setting()
{
  bool ret = false;
  rslt = bma400_interface_init(&bma, BMA400_I2C_INTF);
  if(rslt == 0)
  {
    rslt = bma400_soft_reset(&bma);
    if(rslt == 0)
    {
      rslt = bma400_init(&bma);
      if(rslt == 0)
      {
        accel_setting[0].type = BMA400_STEP_COUNTER_INT;
        accel_setting[1].type = BMA400_ACCEL;

        rslt = bma400_get_sensor_conf(accel_setting, 2, &bma);
        if(rslt == 0)
        {
          accel_setting[0].param.step_cnt.int_chan = BMA400_INT_CHANNEL_1;
          accel_setting[1].param.accel.odr = BMA400_ODR_100HZ;
          accel_setting[1].param.accel.range = BMA400_RANGE_2G;
          accel_setting[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

          rslt = bma400_set_sensor_conf(accel_setting, 2, &bma);
          if(rslt == 0)
          {
            rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, &bma);
            if(rslt == 0)
            {
              int_en[0].type = BMA400_STEP_COUNTER_INT_EN;
              int_en[0].conf = BMA400_ENABLE;
              int_en[1].type = BMA400_LATCH_INT_EN;
              int_en[1].conf = BMA400_ENABLE;
              rslt = bma400_enable_interrupt(int_en, 2, &bma);
              if(rslt == 0)
              {
                ret = true;
              }
            }
          }
        }
      }
    }
  }
 return ret;
}

bool switchOperatorSelection(bool enable)
{
  bool ret = false;
  ME310::return_t rc;
  if(enable)
  {
    rc = _me310->operator_selection(0);
    if(rc == ME310::RETURN_VALID)
    {
      delay(5000);
      _me310->read_network_registration_status();
      while ((strcmp(_me310->buffer_cstr(1), "+CREG: 0,1") != 0) &&  (strcmp(_me310->buffer_cstr(1), "+CREG: 0,5") != 0))
      {
        delay(3000);
        _me310->read_network_registration_status();
      }
      delay(1000);
      ret = true;
    }
  }
  else
  {
    rc = _me310->operator_selection(2);
    if(rc == ME310::RETURN_VALID)
    {
      ret = true;
    }
  }
  return ret;
}

void sendMessage(GNSSInfo gnssInfo)
{
  String strStepCount = String(step_count);
  String startString = "My position is: ";
  String actString;

  sms.setMessageFormat(1);
  memset(txtMsg, 0, sizeof(txtMsg));
  strncat(txtMsg, startString.c_str(), startString.length());
  strncat(txtMsg, gnssInfo.latitude.c_str(), gnssInfo.latitude.length());
  strncat(txtMsg, " ", 1);
  strncat(txtMsg, gnssInfo.longitude.c_str(), gnssInfo.longitude.length());
  strncat(txtMsg, " Steps: ", 12);
  strncat(txtMsg, strStepCount.c_str(), strStepCount.length());
  strncat(txtMsg, " ", 1);
  switch (act_int)
  {
    case BMA400_STILL_ACT:
      actString = "Activity: Still";
      break;
    case BMA400_WALK_ACT:
      actString = "Activity: Walking";
      break;
    case BMA400_RUN_ACT:
      actString =  "Activity: Running";
      break;
  }

  strncat(txtMsg, actString.c_str(), actString.length());
  sms.beginSMS(remoteNum);
  sms.print(txtMsg);
  int rc = sms.endSMS();
  if(rc != 0)
  {
    Serial.println("\nERROR SMS was not sent successfully");
  }
  else
  {
    Serial.println("\nSMS was sent successfully!\n");
    Serial.print("Message payload: ");
    Serial.println(txtMsg);
  }
}
