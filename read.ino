/* function to read with running average over 5 values */ 
double running_average(double value) {
  total = total - readings[index];
  readings[index] = value;
  total = total + readings[index];
  index = index + 1;
  if (index >= numReadings) {
    index = 0;
  }
  return total / numReadings;
}
