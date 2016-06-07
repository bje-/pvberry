#define __ASSERT_USE_STDERR

#include <assert.h>

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  // transmit diagnostic informations through serial link. 
  Serial.print(__file + String(":") + __lineno + String(": ") + __func + String(": Assertion `"));
  Serial.println(__sexp + String("' failed."));
  Serial.flush();

  while (1)
    {
      digitalWrite(13, HIGH);
      delay(250);
      digitalWrite(13, LOW);
      delay(250);
    }
}
