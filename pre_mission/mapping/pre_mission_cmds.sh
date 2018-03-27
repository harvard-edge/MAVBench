#!/bin/bash
echo s 10

echo f 0 0 1 3
echo s 3
echo f 0 0 1 3
echo s 3.1


echo f -1 0 .2 2
echo s 2.1
echo f -1 0 .2 2
echo s 2.1
echo r

echo f 0 -1 .2 2
echo s 2.1
echo f 0 -1 .2 2
echo s 2.1
echo r


echo f 1 0 .2 2
echo s 2.1
echo f 1 0 .2 2
echo s 2.1
echo r


echo f 0 1 .2 2
echo s 2.1
echo f 0 1 .2 2
echo s 2.1
echo r


echo f -0.5 0 .4 4
echo s 4.5


echo p
echo c
