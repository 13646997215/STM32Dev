#include "test.h"
int main(void){RobotConfig c;Pose p;FgCapture f[2]={{0},{0}};int8_t d[2]={1,1};uint8_t trust[2]={1,1};MotionOutput o;float x,y;
 Config_Default(&c);Pose_Reset(&p,0,0,0,f);CHECK(p.valid);
 f[0].total=f[1].total=738;Pose_Update(&p,f,d,trust,0,&c);NEAR(p.x,.067*COURSE_PI,1e-5);NEAR(p.y,0,1e-6);
 f[0].total=f[1].total=1476;d[0]=d[1]=-1;Pose_Update(&p,f,d,trust,0,&c);NEAR(p.x,0,1e-5);
 trust[0]=0;f[0].total++;Pose_Update(&p,f,d,trust,0,&c);CHECK(!p.valid);
 f[0].total=f[1].total=0xfffffffdU;Pose_Reset(&p,0,0,0,f);f[0].total=f[1].total=2;trust[0]=1;d[0]=d[1]=1;Pose_Update(&p,f,d,trust,0,&c);NEAR(p.x,5*.067*COURSE_PI/738,1e-6);
 p.x=p.y=0;p.yaw=0;o=Motion_Goto(&p,.5f,0,0,0,&c);CHECK(o.v>0);NEAR(o.w,0,1e-6);CHECK(!o.reached);
 p.yaw=COURSE_PI/2;o=Motion_Goto(&p,.5f,0,0,0,&c);CHECK(o.v==0&&o.w<0);
 p.x=.5f;p.yaw=0;o=Motion_Goto(&p,.5f,0,0,0,&c);CHECK(o.reached&&o.v==0&&o.w==0);
 p.x=.5f;p.yaw=0;o=Motion_Goto(&p,0,0,0,1,&c);CHECK(o.v<0&&fabs(o.w)<1e-5);
 p.x=1;p.y=2;p.yaw=COURSE_PI/2;Motion_WheelPoint(&p,.1f,.07f,&x,&y);NEAR(x,.93,1e-6);NEAR(y,2.1,1e-6);
 DONE();}
