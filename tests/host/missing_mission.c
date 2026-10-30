#include "course.h"
#include <string.h>
void Config_Default(RobotConfig*c){memset(c,0,sizeof(*c));}
int Mission_Start(Mission*m,uint8_t p,const MissionInput*i,const RobotConfig*c){(void)m;(void)p;(void)i;(void)c;return 0;}
void Mission_Update(Mission*m,const MissionInput*i,const RobotConfig*c){(void)m;(void)i;(void)c;}
