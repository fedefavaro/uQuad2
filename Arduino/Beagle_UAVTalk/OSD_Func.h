
//------------------ Heading and Compass ----------------------------------------

static char buf_show[12];
const char buf_Rule[36] = {0xc2,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc4,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc3,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0,
                           0xc5,0xc0,0xc0,0xc1,0xc0,0xc0,0xc1,0xc0,0xc0};
void setHeadingPattern()
{
  int start;
  start = round((osd_heading * 36)/360);
  start -= 5;
  if(start < 0) start += 36;
  for(int x=0; x <= 10; x++){
    buf_show[x] = buf_Rule[start];
    if(++start > 35) start = 0;
  }
  buf_show[11] = '\0';
}


void updateTravelDistance(void)
{
  static unsigned long loopTimer = 0;

  if (loopTimer + MEASURE_PERIOD <= millis()) {
    if (osd_groundspeed > 1.0) {
      osd_travel_distance += osd_groundspeed * (float) (millis() - loopTimer) / 1000.0;
    }
    loopTimer = millis();
  }
}

