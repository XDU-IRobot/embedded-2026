#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>

void SD_Card_info(void);
bool Save_Params_To_SD(float Pitch[4], float Yaw[4]);
void Load_Params_From_SD(float Pitch[4], float Yaw[4]);

#endif //SD_CARD_H
