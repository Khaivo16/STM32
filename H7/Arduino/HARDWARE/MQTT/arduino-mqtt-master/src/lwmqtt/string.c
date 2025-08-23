#include <string.h>

#include "lwmqtt.h"


lwmqtt_string_t lwmqtt_string(const char *str) {
	lwmqtt_string_t string_t={0,NULL};
	
  // check for null
  if (str == NULL) {
    string_t.len=0; string_t.data=NULL;		
    return string_t;
  }

  // get length
  uint16_t len = (uint16_t)strlen(str);

  // check zero length
  if (len == 0) {	
		string_t.len=0; string_t.data=NULL;		
    return string_t;
  }

	string_t.len=len; string_t.data=(char *)str;
	
  return (lwmqtt_string_t)string_t;
}

int lwmqtt_strcmp(lwmqtt_string_t a, const char *b) {
  // get string of b
  lwmqtt_string_t b_str = lwmqtt_string(b);

  // return if both are zero length
  if (a.len == 0 && b_str.len == 0) {
    return 0;
  }

  // return if lengths are different
  if (a.len != b_str.len) {
    return -1;
  }

  // compare memory of same length
  return strncmp(a.data, b_str.data, a.len);
}
