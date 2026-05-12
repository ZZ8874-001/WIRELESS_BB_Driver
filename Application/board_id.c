#include "board_id.h"

#define STM32_UID_WORD0_ADDR ((uint32_t)0x1FFFF7ACU)
#define STM32_UID_WORD1_ADDR ((uint32_t)0x1FFFF7B0U)
#define STM32_UID_WORD2_ADDR ((uint32_t)0x1FFFF7B4U)

volatile BoardID_t g_board_id_snapshot = {{0U, 0U, 0U}};
volatile int8_t g_board_id_last_result = BOARD_ID_INVALID;

static const BoardID_t kKnownBoardIDs[BOARD_NUM] =
{
    {{0x80090020U, 0x8006000EU, 0x00100014U}},
    {{0x58304302U, 0x564E4317U, 0x53524301U}},
    {{0x00100014U, 0x53524301U, 0x20393038U}},
    {{0x00050023U, 0x43534317U, 0x20353437U}},
};

static bool BoardID_Equals(const BoardID_t *lhs, const BoardID_t *rhs)
{
    if ((lhs == 0) || (rhs == 0))
    {
        return false;
    }

    for (uint32_t i = 0; i < BOARD_ID_WORD_COUNT; ++i)
    {
        if (lhs->words[i] != rhs->words[i])
        {
            return false;
        }
    }

    return true;
}

bool BoardID_Read(BoardID_t *board_id)
{
    if (board_id == 0)
    {
        return false;
    }

    board_id->words[0] = *(const volatile uint32_t *)STM32_UID_WORD0_ADDR;
    board_id->words[1] = *(const volatile uint32_t *)STM32_UID_WORD1_ADDR;
    board_id->words[2] = *(const volatile uint32_t *)STM32_UID_WORD2_ADDR;

    g_board_id_snapshot.words[0] = board_id->words[0];
    g_board_id_snapshot.words[1] = board_id->words[1];
    g_board_id_snapshot.words[2] = board_id->words[2];

    return true;
}

int8_t BoardID_Detect(void)
{
    BoardID_t current_board_id;

    if (!BoardID_Read(&current_board_id))
    {
        g_board_id_last_result = BOARD_ID_INVALID;
        return BOARD_ID_INVALID;
    }

    for (int8_t i = 0; i < BOARD_NUM; ++i)
    {
        if (BoardID_Equals(&current_board_id, &kKnownBoardIDs[i]))
        {
            g_board_id_last_result = i;
            return i;
        }
    }

    g_board_id_last_result = BOARD_ID_INVALID;
    return BOARD_ID_INVALID;
}

bool BoardID_IsValid(int8_t board_id)
{
    return (board_id >= 0) && (board_id < BOARD_NUM);
}
