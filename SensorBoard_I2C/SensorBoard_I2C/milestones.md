# Lab 6: Milestones

Lab 6: Building the Rest of the FSM, with a Tool

# Milestone 1: The package as shipped
10 points

This is a gate, not a warm-up.

Until the smoke test passes, you cannot tell a broken state machine from a broken toolchain. Do not start generating until it does.

Requirements:

1. Build a bitstream.

   Show the TA the last lines of:

   build/build.log

   It must show:
   - the board string
   - both positive slack numbers
   - EXIT_CODE=0

2. Run:

   python/smoke_test.py

   It must produce:

   RESULT: ALL PASS

   All twelve checks must pass.

3. Run:

   python/i2c_first_frame.py

   It must find all six devices across the two buses.

4. Run:

   ./build.sh --sim

   From the last frame of the log, explain to the TA:
   - what the slave is doing to SDA
   - why the STOP never happens


# Milestone 2: Finish the transaction, one frame at a time
30 points

## How the 30 points split

20 points:
A working read, based on the four gates below.

10 points:
The step log, filled in as you go, with attempt counts and what was missing each time.

The log is not paperwork attached to the real deliverable.

It is the only record of how you broke the job up, the whole post lab is built on it, and it cannot be reconstructed afterwards.

A pair who never got 0xCB but kept an honest log of where it broke earns most of those 10 points.

A pair with a working machine and no log does not.

Extend:

hdl/I2C_Transmit.v

from a single frame into a complete register read, following the method on the Lab 6 page and the plan you wrote in pre-lab question 3.


## What it has to do when you are finished

Gate 1: Simulates clean

Evidence:

RESULT: ALL PASS

from:

./build.sh --sim


Gate 2: Builds with positive slack

Evidence:

WNS and WHS are both positive in:

build/build.log


Gate 3: Reads the ID register

Evidence:

result1[7:0] = 0xCB

from the ADT7420.


Gate 4: Handles an absent device

Evidence:

Address:

0x49

must give:

error = 1

and the state must return to:

0


## Log your steps as you go

Fill this in while you work, not afterwards.

Use the step numbers from your pre-lab plan.

The table should contain:

Step (from your plan)
Attempts to pass
What was missing from attempt 1

Example:

Step 1:
Finish frame 2, the register pointer

Attempts to pass:
?

What was missing from attempt 1:
For example, did not say which file

Step 2:
...

Step 3:
...

Continue for all steps in your plan.


## Keep every prompt

Keep every prompt verbatim, including the ones that did not work.

Start a fresh conversation for this lab so the transcript contains only this work.

The prompts that failed are the interesting data, and the post lab asks for them.


## If you cannot get it working

Say so and show how far you got.

Report:
- the last step that passed
- the step that did not pass
- the prompts you tried on it

That is a finding and it is worth marks.

Do not report a version you never got to read 0xCB as if it worked.


# Milestone 3: Measure it against the one you wrote by hand
20 points

You now have two implementations of the same protocol:

1. The one you wrote state by state in Lab 5.
2. The one you just generated in Lab 6.

Fill the comparison table on the bench.

Every number must come from a named artifact, not from reading the code.


## Metrics

### I2C_Transmit LUT and FF

Source:

build/post_synth_utilization_hier.rpt

Use the row:

u_i2c


### States and the encoding

Source:

The FSM extraction report in the synthesis log.

Do not count states by eye.


### Measured SCL

Use:

TICK_DIVIDE

and your state count.

Confirm the result on the:

SCL_0

and:

SDA_0

through-holes.


### t_LOW and t_HIGH

Determine them using the same method.

Check both against:

ADT7420 Table 2, page 5


### Time for a complete read

Use:

result2 / 100.8

in microseconds.


### Recovers from a NACK or a wrong bus

Evidence:

status()['state']

returns to:

0


## Before you compare times, check the duty cycle

If the two versions run SCL at different frequencies, their transaction times are not comparable.

In that case, you are measuring bus speed, not state machines.

Normalise them.

Set both to the same SCL before you time either.

Report the duty cycle you measured for each.

A version whose t_LOW and t_HIGH differ has changed the bus, whatever else it changed.


# Demo and Submission

Upload two separate files to this assignment with exactly these names:

lab6_I2C_Transmit.v

Contents:
Your finished generated state machine.


lab6_python.py

Contents:
The Python you used to drive and read it.


Your post-lab PDF and your prompt transcript go to the separate:

Lab 6 - Post Lab

assignment.


# Rules

## Code uploaded, then demonstrated

Result:

Graded normally on the seven checks below.


## Files missing or misnamed, noticed at the demo

Result:

Fix it there and then.

The TA waits.

No penalty.


## No code submitted by the deadline

Result:

Milestone scores 0, regardless of the demonstration.


## Demonstration and upload both late

Result:

10% per hour, applied to the milestone as a whole.


## Due

The first 60 minutes of your next lab session.

This is the same deadline as the post lab.


# What the TA Checks

## Check 0

The TA opens this assignment in Canvas before starting.

Required:

Both code files are submitted and correctly named.

Nothing else happens until they are.


## Check 1

The TA looks at:

build/build.log

Required:

- board string correct
- WNS positive
- WHS positive
- EXIT_CODE=0


## Check 2

The TA watches:

python/smoke_test.py

run.

Required:

RESULT: ALL PASS

All twelve checks pass.


## Check 3

The TA watches your finished design read the ID register.

Required:

result1[7:0]

prints:

0xCB


## Check 4

The TA sets the address to:

0x49

and reruns.

Required:

error = 1

and the state returns to:

0

rather than hanging.


## Check 5

The TA looks at your step log.

Required:

- filled in during the session
- attempt counts included
- what was missing from attempt 1 included

This row is worth 10 of the 30 Milestone 2 points.


## Check 6

The TA looks at your Milestone 3 table.

Required:

- both versions measured
- SCL stated for each
- duty cycle stated for each


# Important note

Check 4 is the one that separates a working machine from a lucky one.

A design that only ever meets a device that answers has never had its error path executed.