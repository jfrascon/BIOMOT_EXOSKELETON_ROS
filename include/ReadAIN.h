/**
  * Authors: Elisa Piñuela Martín. epinuela@externas.sescam.jccm.es
             Juan Francisco Rascón Crespo. jfrascon@gmail.com
  * Code for reading Analog GPIO from BeagleBone Black. This easy function
  * is use for reading values provided by the foot switch resistor.
  */

#include <stdio.h>
#include <unistd.h>

unsigned int Read_AnalogData(unsigned int ch_num) {
  // Read_AnalogData(ch_num)
  // ch_num: 0-7
  FILE *file = NULL;
  char buf[70];
  unsigned int analogValue = 0;

  sprintf(buf, "/sys/devices/ocp.3/helper.15/AIN%d", ch_num);

  file = fopen(buf, "r");
  fscanf(file, "%d", &analogValue);

  fclose(file);
  return analogValue;
}
