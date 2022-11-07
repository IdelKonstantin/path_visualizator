#!/usr/bin/env python3
#coding: utf-8

import subprocess
import os
import argparse

##############################################################################################
parser = argparse.ArgumentParser(description='drone exploration path visio on Yandex maps')

parser.add_argument('--lat', action='store', dest='latitude',
        help='latitude of polygon center',
        required=True)

parser.add_argument('--long', action='store', dest='longitude', 
        help='longitude of polygon center',
        required=True)

parser.add_argument('--file', action='store', dest='file_name', 
         help='path to file with points',
         required=True)

latitude = parser.parse_args().latitude
longitude = parser.parse_args().longitude
file_name = parser.parse_args().file_name

##############################################################################################

def getRawPathStrings(pointFileName):
    
    rawCoordinateStrings = []

    with open(pointFileName, 'r') as f:
        for line in f:
            rawCoordinateStrings.append(line)

    return rawCoordinateStrings

def getCoordinates(rawCoordinateStrings):

    coordinatesList = []

    for rawString in rawCoordinateStrings[:-1]:
        latRaw, longRaw = rawString.split(", ")
        longRaw = longRaw[:-1]

        lat_ = round(float(latRaw) * 0.0000001, 6)
        long_ = round(float(longRaw) * 0.0000001, 6)

        coordinateString = '[' + str(lat_) + ', ' + str(long_) + '],'
        coordinatesList.append(coordinateString)   

    return coordinatesList

def makePolylineJS(latitude, longitude, coordinates):

    JStext = 'ymaps.ready(init);\n'
    JStext += 'function init() {\n'
    JStext += 'var myMap = new ymaps.Map("map", {\n'
    JStext += str('center: [' + latitude + ', ' + longitude + '],\n')
    JStext += 'zoom: 17}, {\n'
    JStext += "searchControlProvider: 'yandex#search'});\n"
    JStext += 'var myPolyline = new ymaps.Polyline([\n'

    for coordinateLine in coordinates:
    	JStext += str(coordinateLine + '\n')

    JStext += '], {balloonContent: "Ломаная линия"}, {\n'
    JStext += 'balloonCloseButton: false,\n'
    JStext += 'strokeColor: "#000000",\n'
    JStext += 'strokeWidth: 4,\n'
    JStext += 'strokeOpacity: 0.5});\n'
    JStext += 'myMap.geoObjects.add(myPolyline);}\n'

    with open('polyline.js', 'w') as f:
    	f.write(JStext)

def showDronePath():

    subprocess.Popen("firefox ./polyline.html", shell=True).wait()

##############################################################################################

coordinatesList = getCoordinates(getRawPathStrings(file_name))
makePolylineJS(latitude, longitude, coordinatesList)
showDronePath()