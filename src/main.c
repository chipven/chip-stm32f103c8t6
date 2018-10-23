#include "../make/stm32f103c8t6.h"
#include "string.h"
#include "system_util.h"
int main() {
        openUart1();
        char *cmd = "Hello evenardo!\n";
        while (1) {
                uart1SendString(cmd);
                delay();
        }
}