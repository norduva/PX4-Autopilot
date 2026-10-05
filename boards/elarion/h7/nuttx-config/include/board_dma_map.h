/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

// DMAMUX1 Using at most 8 Channels on DMA1 --------   Assigned
#define DMAMAP_SPI2_RX    DMAMAP_DMA12_SPI2RX_0     // 1 DMA1:39 IMUs
#define DMAMAP_SPI2_TX    DMAMAP_DMA12_SPI2TX_0     // 2 DMA1:40 IMUs
#define DMAMAP_SPI3_RX    DMAMAP_DMA12_SPI3RX_0     // 3 DMA1:61 Flash / microSD
#define DMAMAP_SPI3_TX    DMAMAP_DMA12_SPI3TX_0     // 4 DMA1:62 Flash / microSD
// Timer 3 (DMAMAP_DMA12_TIM3UP_0)                  // 5 DMA1:27 TIM3UP/TIM3CH1-4  Motors 1-2
// Timer 5 (DMAMAP_DMA12_TIM5UP_0)                  // 6 DMA1:59 TIM5UP/TIM5CH1-4  Motors 3-6
// Timer 4 (DMAMAP_DMA12_TIM4UP_0)                  // 7 DMA1:32 TIM4UP/TIM4CH1-4  Motors 7-8
//                                                  // 8 spare

// DMAMUX1 Using at most 8 Channels on DMA2 --------   Assigned
#define DMAMAP_UART7_RX   DMAMAP_DMA12_UART7RX_1    // 1 DMA2:79 RC
#define DMAMAP_UART7_TX   DMAMAP_DMA12_UART7TX_1    // 2 DMA2:80 RC
#define DMAMAP_USART1_RX  DMAMAP_DMA12_USART1RX_1   // 3 DMA2:41 TELEM1
#define DMAMAP_USART1_TX  DMAMAP_DMA12_USART1TX_1   // 4 DMA2:42 TELEM1
#define DMAMAP_USART3_RX  DMAMAP_DMA12_USART3RX_1   // 5 DMA2:45 GPS1
#define DMAMAP_USART3_TX  DMAMAP_DMA12_USART3TX_1   // 6 DMA2:46 GPS1
#define DMAMAP_UART8_RX   DMAMAP_DMA12_UART8RX_1    // 7 DMA2:81 TELEM2
#define DMAMAP_UART8_TX   DMAMAP_DMA12_UART8TX_1    // 8 DMA2:82 TELEM2
