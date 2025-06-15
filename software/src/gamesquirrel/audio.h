
#include <stdint.h>
#include <stdbool.h>

// FIXME do we need to force 32 bit alignment?

typedef struct AudioSample AudioSample;
struct AudioSample
{
    int16_t right;
    int16_t left;
};

void AudioInit(void);

bool AudioStart(AudioSample *samples, int count);
bool AudioComplete(AudioSample **samples_out, int *count_out);

