#ifndef EXTRA_H
#define EXTRA_H
char * TimeToString(uint32_t t)
{
  static char str[8];
  sprintf(str, "%02d:%02d:%02d", t / 3600000 , (t / 60000) % 60, (t / 1000) % 60);
  return str;
}
#endif
