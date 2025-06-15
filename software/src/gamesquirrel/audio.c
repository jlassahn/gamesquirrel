
#include "gamesquirrel/audio.h"
#include "stm32h523xx.h"
#include <string.h>

#include <math.h>
#include "gamesquirrel/core.h"

static AudioSample sample_table[1024] __attribute__((aligned (4)));
static bool running = false;

// FIXME design and implement audio API

void GPDMA1_Channel5_IRQHandler(void)
{
    // FIXME react to done flag specifically
    // if (GPDMA1_Channel5->CSR & 0x100)

    GPDMA1_Channel5->CBR1 = 1024*4;
    GPDMA1_Channel5->CSAR = (uint32_t)sample_table;
    GPDMA1_Channel5->CDAR = (uint32_t)&DAC1->DHR12LD;

    GPDMA1_Channel5->CFCR = 0x7F00; // clear all flags

    GPDMA1_Channel5->CCR =
        0x00C00000 |  // high priority
        0x00000100 | // Transfer complete interrupt enable
        // 0x00000002 | // RESET
        0x00000001; // Enable

}

void AudioInit(void)
{
    DAC1->MCR = 0x02028202;
    DAC1->CR = 0x00010001; // start out untriggered

    while ((DAC1->SR & 0x08000800) != 0x08000800)
    {}

    // data is signed 16 bit
    DAC1->DHR12LD =
        (0x0000 << 16) | //channel 2
        (0x0000 << 0); //channel 1

    // set to triggered mode for DMA
    // Trigger 12 is lptim2 ch1
    // Only channel 1 DMA is enabled because the DMA will use the Dual DAC mode
    DAC1->CR = 0x00331033;
    DAC1->DHR12LD =
        (0x7FFF << 16) | //channel 2
        (0x7FFF << 0); //channel 1

    for (int i=0; i<1024; i++)
    {
        sample_table[i].left = (int16_t)(sin(i*(3.1416/64))*32767);
        sample_table[i].right = (int16_t)(sin(i*(3.1416/128))*32767);
    }

    NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);
}


bool AudioStart(AudioSample *samples, int count)
{
    if (running)
        return false;

    // DMA channel setup
    // source increment, 32 bit transfers, source 16 byte burst dest 4 byte burst
    GPDMA1_Channel5->CTR1 = 0x0002003A;
    GPDMA1_Channel5->CTR2 = 0x00000402; // destination req DAC1 ch1
    GPDMA1_Channel5->CBR1 = 1024*4;
    GPDMA1_Channel5->CSAR = (uint32_t)sample_table;
    GPDMA1_Channel5->CDAR = (uint32_t)&DAC1->DHR12LD;
    GPDMA1_Channel5->CTR3 = 0;
    GPDMA1_Channel5->CBR2 = 0;
    GPDMA1_Channel5->CLLR = 0;

    GPDMA1_Channel5->CFCR = 0x7F00; // clear all flags

    GPDMA1_Channel5->CCR =
        0x00C00000 |  // high priority
        0x00000100 | // Transfer complete interrupt enable
        // 0x00000002 | // RESET
        0x00000001; // Enable

    running = true;

    return false;
}

bool AudioComplete(AudioSample **samples_out, int *count_out)
{
    *samples_out = NULL;
    *count_out = 0;
    return false;
}

