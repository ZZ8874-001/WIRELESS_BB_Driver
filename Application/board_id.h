#ifndef BOARD_ID_H
#define BOARD_ID_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_NUM 4
#define BOARD_ID_WORD_COUNT 3U
#define BOARD_ID_INVALID ((int8_t)-1)

typedef struct
{
    uint32_t words[BOARD_ID_WORD_COUNT];
} BoardID_t;

bool BoardID_Read(BoardID_t *board_id);
int8_t BoardID_Detect(void);
bool BoardID_IsValid(int8_t board_id);

#endif
