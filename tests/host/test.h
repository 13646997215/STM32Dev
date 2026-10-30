#ifndef TEST_H
#define TEST_H
#include "course.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
static unsigned checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; if(failures<=20)printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#x); } } while(0)
#define NEAR(a,b,e) CHECK(fabs((double)(a)-(double)(b))<=(e))
#define DONE() do {printf("checks=%u failures=%u\n",checks,failures);return failures?1:0;} while(0)
#endif
