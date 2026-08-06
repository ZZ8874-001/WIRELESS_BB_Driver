# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

```bash
make clean && make -j            # clean rebuild
make -j                          # incremental build
```

Output: `build/WireLess_BB_Driver.elf/.hex/.bin`
Target: STM32F334C8, Cortex-M4 + hardware FPU, `-O2`, gcc-arm-none-eabi.

## Architecture

This is a **synchronous buck converter firmware** (bare metal, no RTOS) for STM32F334. The HRTIM generates complementary PWM with dead-time, and a wireless module provides remote enable/control via UART.

### Scheduling

All real-time work runs in `HAL_TIM_PeriodElapsedCallback` (TIM2, 10 kHz):

```
TIM2 ISR (every 100 µs):
  Bsp_ADC_ProcessSample()   ← integer-LPF raw ADC, convert to float once
  Buck_Boost_Task()          ← state machine, NFB control, PWM update
  Detect_Task()              ← timeout watchdogs (every 10th tick = 1 kHz)
```

ADC is triggered by HRTIM Timer C at ~100 kHz, DMA'ed directly into circular buffers. The ISR just polls the latest values — no ADC interrupt processing.

### Key modules

| Layer | Files | Role |
|-------|-------|------|
| Control | `Application/bb_control.c` | Buck state machine (Error→SoftStart→Buck), NFB gain control, V/I dual-loop |
| Detection | `Application/detect_task.c/h` | 7+ timeout-event (TOE) watchdogs, each with configurable ms threshold |
| Board ID | `Application/board_id.c/h` | STM32 UID → board index, per-board calibration coefficients |
| ADC BSP | `Bsp/bsp_adc.c/h` | DMA circular buffers, integer first-order LPF, OVP analog watchdog |
| UART BSP | `Bsp/bsp_uart.c/h` | DMA + IDLE-interrupt wireless protocol, debug data injection |
| DWT | `Bsp/bsp_dwt.c/h` | Cycle-count microsecond delay/timing |
| Filters | `Commponents/filter32.c/h` | Float (float) and integer (int32_t Q15) first-order LPF; window/IIR |
| Controller | `Commponents/Controller/` | PID, fuzzy PID, feedforward, LDOB, tracking differentiator (mostly unused) |
| CubeMX | `Core/Src/*.c`, `WireLess_BB_Driver.ioc` | HAL peripheral init — **never hand-edit driver sections** |

### Control Loop

The buck uses **non-linear feedback (NFB)**, not PID:

```
Vout/Vin → voltage_gain_measure → gain error → sin() × Kp → voltage_gain_NFB_
Iout error → sin() × Kp                                     → current_gain_NFB_
Both compete (min/avg) → final_gain → duty = constrain(gain, 0.05, 0.95)
```

Sink-current protection: `Is_Buck_OutputVoltage_Allowed()` — hysteresis comparator (disable when Vout > Vin×1.05, release ≤ Vin×1.02). Acts via `Update_Buck_Output_Enable()` on `HRTIM_OUTPUT_TB1|TB2`.

### Critical register-level code

- `Bsp/bsp_uart.c`: Direct register writes override HAL UART/DMA init — **do not remove CR1=0 clearing without understanding OVER8/baud implications**.
- `Bsp/bsp_adc.c`: DMA1_Ch1/Ch4 interrupts are disabled (HTIE + TCIE cleared); EOS callback is empty — DMA circular mode handles all buffering.
- `Application/bb_control.c`:100: `HRTIM_TIMERID_TIMER_C` must be included in `WaveformCountStart` for ADC triggering.

### CubeMX changes that need software sync

- ADC trigger source (Timer A→C): `Bsp_ADC_Init` DMA targets may need buffer-size alignment.
- USART oversampling (16×→8×): `Bsp_UART_Init` must include `USART_CR1_OVER8` in CR1 restore.
- New board entries: `BOARD_NUM` in `board_id.h`, calibration arrays in `bsp_adc.c`, UID table in `board_id.c`.
