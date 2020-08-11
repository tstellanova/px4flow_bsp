#!/usr/bin/env python

import base64

# this simple python script converts base64-encoded
# pixel values to raw binary values in order to create a RAW image file.
# This was used for verifying pixel transfer from the camera sensor
with open('sessions/out18.b64', 'r') as input:
    data = input.read()
    decoded = base64.b64decode(data)

with open('sessions/out18.raw', 'wb') as out:
    out.write(decoded)
