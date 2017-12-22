#include "ven_tc1117.h"
#include "ven_8digi.h"

void
ven_tc1117(u32 delay) {
    u32 table[][4] = {{1, 0, 0, 0}, {1, 0, 1, 0}, {0, 0, 1, 0},
                           {0, 1, 1, 0}, {0, 1, 0, 0}, {0, 1, 0, 1},
                           {0, 0, 0, 1}, {1, 0, 0, 1}};
    void This(u32 step) {
        A1 = table[step][0];
        A2 = table[step][1];
        B1 = table[step][2];
        B2 = table[step][3];
    }
    void Idle() {
        A1 = 0;
        A2 = 0;
        B1 = 0;
        B2 = 0;
    }
    u32 cycle[][2] = {
        {0, 1000},  {1560, 844}, {173, 827}, {195, 805}, {222, 778}, {258, 741},
        {309, 691}, {382, 618},  {500, 500}, {707, 293}, {1000, 0},
    };
    //每一次相位变化
    for (u32 s = 0; s < 8; s++) {
        static u32 T = 1000;
        static u32 x = 120, t = 0, tt = 50;
        t++;
        if (t > tt) {
            if (x < 350) {
                x += 10;
            }
            if (x >= 350) {
                x += 2;
                tt *= 1.1;
            }
            t = 0;
        }
        //读取占空比数组
        if (x < T) {
            for (u32 p = 0; p < 11; p++) {
                //解析每一次占空比
                //占
                for (u32 bsy = 0; bsy < cycle[p][0] / x; bsy++) {
                    // ven_8digi_DEC(x);
                    This(s);
                }
                //空
                for (u32 idle = 0; idle < cycle[p][1] / x; idle++) {
                    // ven_8digi_DEC(x);
                    Idle();
                }
            }
        } else {
            for (u32 p = 0; p < 11; p++) {
                //解析每一次占空比
                //占
                for (u32 bsy = 0; bsy < cycle[p][0] / x; bsy++) {
                    // ven_8digi_DEC(x);
                    This(s);
                }
            }
        }
    }
}
void
ven_MotorRunInType() {
    while (1) {
        ven_tc1117(13);
    }
}

void
ven_showMotor() {
    ven_MotorRunInType();
    ;
}
