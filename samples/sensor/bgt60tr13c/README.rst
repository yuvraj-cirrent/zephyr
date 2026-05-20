.. _bgt60tr13c-sample:

BGT60TR13C 60 GHz Radar Sensor Sample
#####################################

Overview
********

This sample demonstrates how to use the Infineon XENSIV™ BGT60TR13C 60 GHz FMCW radar
sensor on the CY8CKIT-062S2-AI board. The sensor communicates over SPI and provides
a FIFO interface for retrieving radar frame data at regular intervals.

The sample periodically reads the FIFO status and prints the number of available
samples along with the first sample value.

Requirements
************

- `CY8CKIT-062S2-AI PSoC 6 AI Evaluation Kit <https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-ai/>`_
  with integrated BGT60TR13C radar sensor (U6)
- Zephyr with BGT60TR13C sensor driver enabled
- Working SPI and GPIO peripheral drivers

Building and Running
********************

Build the sample for the cy8ckit_062s2_ai board:

.. code-block:: console

   west build -b cy8ckit_062s2_ai/cy8c624abzi_s2d44 -p always samples/sensor/bgt60tr13c

Flash and run:

.. code-block:: console

   west flash
   west monitor

Expected Output
***************

The sample will print the radar FIFO status every second:

.. code-block:: console

   BGT60TR13C sample started on bgt60tr13c@0
   BGT60TR13C Radar Report
   ======================
   FIFO sample count: 128
   First FIFO sample: 0x1a2b

   BGT60TR13C Radar Report
   ======================
   FIFO sample count: 256
   First FIFO sample: 0x3c4d

Pin Configuration
*****************

The BGT60TR13C sensor is connected to the PSoC6 MCU on the CY8CKIT-062S2-AI as follows:

- **SPI Interface (SCB6)**:
  - MOSI: P12[0]
  - MISO: P12[1]
  - CLK:  P12[2]
  - CS:   P12[3] (GPIO-based, active-low)

- **Control Signals**:
  - IRQ (Data-Ready): P11[0]
  - Reset (active-low): P11[1]

Interpreting Results
********************

**FIFO sample count**: The number of 16-bit samples available in the radar's internal FIFO.
Valid range is typically 0-4096.

**First FIFO sample**: The raw 16-bit value of the first sample in the FIFO. This
represents the complex radar return data and must be post-processed by the application
to extract range, Doppler, and presence information.

Advanced Usage
**************

For production applications, implement:

1. **Continuous data streaming**: Use the interrupt trigger mode to automatically
   fetch FIFO data when the data-ready signal asserts.

2. **Real-time DSP**: Apply FFT and CFAR (Constant False Alarm Rate) processing to
   the raw FIFO samples to detect targets and measure their range, velocity, and RCS.

3. **Sensor configuration**: Write to the BGT60TR13C configuration registers to adjust
   chirp parameters, TX power, and integration time for your use case.

Refer to the BGT60TR13C datasheet and the Infineon sensor-xensiv-bgt60trxx library
for register definitions and advanced features.

See Also
********

- :ref:`sensor` — Zephyr Sensor Subsystem
- :ref:`drivers/sensor/infineon/bgt60tr13c` — BGT60TR13C Driver Documentation
