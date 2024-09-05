#!/bin/bash
wget -R "index.*","dataset-calib*" -m -np -nH -c --no-check-certificate -erobots=off https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/

# optionally verify md5 sums:
cd tumvi/exported/euroc/512_16
md5sum -c *.md5

#wget -R "index.*","dataset-calib*" -m -np -nH -c --no-check-certificate -erobots=off https://cdn3.vision.in.tum.de/tumvi/exported/euroc/1024_16/

## optionally verify md5 sums:
#cd tumvi/exported/euroc/1024_16
#md5sum -c *.md5


