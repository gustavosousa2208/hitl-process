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

- Console UART: ``USART3`` on ``PD8``/``PD9`` at ``115200``
- Left ESC PWM input: ``PB4`` (`TIM3_CH1`, ST Zio ``CN7`` pin ``19`` / ``D25``)
- Right ESC PWM input: ``PA0`` (`TIM5_CH1`, ST morpho ``CN11`` pin ``28``)
- Simulated angle analog output: ``PA4`` (`DAC1_OUT1`, ST Zio ``CN7`` pin ``17`` / ``D24``, 0 to 3.3 V)

Model assumptions
*****************

- The incoming ESC signal is PWM at ``50 Hz`` (``20 ms`` period).
- ``1.0 ms`` high time means zero command.
- ``2.0 ms`` high time means full command.
- Values between ``1.0 ms`` and ``2.0 ms`` are mapped linearly to
  ``0`` to ``100 %`` throttle.
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
