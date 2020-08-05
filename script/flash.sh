#!/bin/bash
stm32flash -b 460800 -R -i dtr,rts,-dtr $1 -w $2
