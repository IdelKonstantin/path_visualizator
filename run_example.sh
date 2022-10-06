#!/bin/bash

cd ./path_example
cmake . && make 
cd ..
./path_example/example | ./viz.py