#!/bin/bash

g++ test.cpp -o test && ./test | ./viz.py
