#!/bin/bash
python3 train_dnn.py --data_resolution $1 --batch_size $2 --model $3 --directory $4 --shuffle
