#!/bin/bash
./make_calib_board.py --type apriltag --nx 3 --ny 3 --tsize 0.03 --tspace 0.5 --marginx 0.162 --marginy 0.25 --color Black --borderbits 2 --startid 4
 #boarderbits of 2 is more stable as outside line is tracked
