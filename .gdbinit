# GDB cannot automatically discover these settings, but they are available from
# the OpenOCD output, so we can set them here.

set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

target extended-remote localhost:3333
