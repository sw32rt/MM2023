#ifndef SOUND_H_
#define SOUND_H_

void g_SoundTask(void *argument);
void g_SoundInit(void);

typedef enum scoreIndex
{
    SCORE_INDEX_0 = 0,
    SCORE_INDEX_1,
    SCORE_INDEX_2,
}scoreIndex;

typedef struct score_t
{
    uint16_t* pScore;
    uint16_t length;
}score_t;

#endif /* SOUND_H_ */
