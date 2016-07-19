// myassert.h: Handy assert function for the Arduino.
//
// Copyright 2016 Ben Elliston
//
// PVBerry is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.

// PVBerry is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

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

void unreachable()
{
  int unreachable = 0;
  assert (unreachable);
  __builtin_unreachable();
}
