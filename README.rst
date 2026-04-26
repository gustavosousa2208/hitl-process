HIL Process Plant for WeAct STM32G0B1 Core
##########################################

This Zephyr application emulates the nonlinear bicopter-like seesaw plant
described in [docs/plant-model.md](docs/plant-model.md).

What it does
************

- Receives two ESC command PWMs as inputs.
- Integrates the nonlinear plant model at ``50 ms``.
- Exposes the simulated angle as a potentiometer-like analog voltage.
- Keeps the Zephyr console enabled on the board UART.

Current hardware mapping
************************

- Console UART: ``USART1`` on ``PA9``/``PA10`` at ``115200``
- Left ESC PWM input: ``PB4`` (`TIM3_CH1`)
- Right ESC PWM input: ``PB14`` (`TIM15_CH1`)
- Simulated angle analog output: ``PA4`` (`DAC1_OUT1`, 0 to 3.3 V)

Model assumptions
*****************

- The incoming ESC signal is the duty-based PWM described in the plant notes,
  around ``490.2 Hz``.
- ``1.0 ms`` high time means zero command.
- ``1.5 ms`` high time means full command.
- Values above ``1.5 ms`` are saturated, which tolerates ESC initialization
  pulses up to ``2.0 ms``.
- The analog output is scaled to reproduce the original potentiometer
  preprocessing:

  - ``adc ~= 512 + theta * (516 / pi)``
  - then remapped to the MCU 12-bit DAC range

Build
*****

Build for the WeAct board with:

.. code-block:: console

   west build -b weact_stm32g0b1_core

Notes
*****

If you want different pins, input timing, or output scaling, adjust
[`boards/weact_stm32g0b1_core.overlay`](boards/weact_stm32g0b1_core.overlay)
and [`src/main.c`](src/main.c).
