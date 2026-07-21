#include "xunji.h"
extern void xunji_set_speed(float a);
extern void xunji_yuandizuozhuan(float a);
void WJ_task_zhiujiao(void)//直角转弯加普通弯道
{
        xunji_all();
        if(xunji_flag==0)
        {
            xunji_set_speed(WJ_xunji_fangxing());
        }
        else if (xunji_flag==1)
        {
            if(xunji_yanshi>=200)
            {xunji_yuandizuozhuan(-30);}
            else 
            {
                xunji_set_speed(WJ_xunji_fangxing());
            }
        }
}