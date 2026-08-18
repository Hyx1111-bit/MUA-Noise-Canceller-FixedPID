<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This design implements a fixed-gain PID controller for suppressing noise in Multi-Unit Activity (MUA) neural
signals, targeting a cortical visual prosthetic application.

Two 8-bit unsigned parallel data streams are received every clock cycle:

- **Brain Signal (BS)**, on `ui_in[7:0]` — the measured/recorded neural signal
- **Desired Signal (DS)**, on `uio_in[7:0]` — the reference/target signal

Both streams pass through a `Data_Receiver` module, which latches the incoming data synchronously on the rising
edge of `clk`. The latched outputs are then passed through an additional register stage to suppress glitches
before further processing.

Since the ADC data arrives as unsigned values (0–255), each channel is converted to a signed representation
(-128 to +127) using offset-binary mapping, where the ADC midpoint (128) maps to zero. This conversion
(`Unsigned2Signed_Converter`) is required so that the PID error calculation can correctly represent both positive
and negative deviations from the desired signal.

The two signed values (BS and DS) are fed into `PID_Controller_Integrated_Scale`, a fixed-gain PID controller
(Kp, Ki, and Kd are constant values, not adaptive). Each clock cycle, the controller computes:

- **Error** = Desired Signal − Brain Signal
- **P-term** = Kp × error
- **I-term** = Ki × (sum of the last 16 error values, sliding window)
- **D-term** = Kd × (error[n] − error[n-1])

The three terms are summed and scaled to produce an 8-bit signed stimulus output.

Finally, this signed stimulus value is converted back to unsigned format (`Signed2Unsigned_Converter`, inverse
offset-binary mapping) so that it can drive an external DAC, and is output on `uo_out[7:0]`.

The `uio_out` and `uio_oe` ports are not used in this design and are tied to 0.

## How to test

1. Apply a clock on `clk` at approximately 10 kHz (the design was sized to process MUA signals with a bandwidth
   of ~5 kHz, satisfying the Nyquist criterion).
2. Hold `rst_n` low for at least one clock cycle to reset the design, then release it high.
3. Drive `ena` high to enable the design.
4. On each rising edge of `clk`, present the current Brain Signal sample (unsigned, 0–255) on `ui_in[7:0]` and
   the current Desired Signal sample (unsigned, 0–255) on `uio_in[7:0]`.
5. After a few clock cycles of pipeline latency, observe the stimulus output on `uo_out[7:0]` (unsigned, 0–255,
   with 128 representing zero/baseline stimulation).
6. To verify correct PID behaviour, apply a step or ramp difference between the Brain Signal and Desired Signal
   inputs and confirm that the stimulus output moves in the direction that would drive the error toward zero.
## External hardware

Two external ADCs (mounted on the FPGA development board) are expected to provide the Brain Signal and Desired
Signal samples as parallel 8-bit unsigned data on `ui_in` and `uio_in` respectively. The stimulus output on
`uo_out` is intended to drive an external DAC for delivering the stimulation signal.
