#!/bin/sh

rsync -e ssh -rpzv _build/html/* scott@vehicles.caltech.edu:/var/www/scott/btsynth/
