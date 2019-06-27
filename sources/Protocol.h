#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "MwiiSSerial.h"

void serialCom();
void debugmsg_append_str(const char *str);

//sserial
#if (GPS)
  void sserialCom();
#endif

#endif /* PROTOCOL_H_ */
