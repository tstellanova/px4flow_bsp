#!/usr/bin/env python

import array

# this simple python script converts hex-encoded
# CSV pixel values to raw binary values in order to create a RAW image file.
# This was used for verifying pixel transfer from the camera sensor
with open('sessions/live_image01.csv', 'rt') as input:
    text = input.read()
    entries = text.split(',')
    values = [int(x, 16) for x in entries]

with open('sessions/live_out01.raw', 'wb') as out:
    raw_values = array.array('B', values) # one byte
    raw_values.tofile(out)
