#!/bin/bash
stm32flash -R -i dtr,rts,-dtr $1 -w $2
