#include "../make/stm32f103c8t6.h"
#include "string.h"
#include "system_util.h"
int main() {
        openUart1();
        // char cmd[] = "Hello world\n";
        // char len = strlen(cmd);
        // while (1) {
        // for (int i = 0; i < len; i++) {
        // uart1SendByte(cmd[i]);
        // }
        // }
        char *cmd = "Hello evenardo!\n";
        while (1) {
                uart1SendString(cmd);
                delay();
        }
}
