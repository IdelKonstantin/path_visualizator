#!/usr/bin/env python3
#coding: utf-8

import subprocess
import os
import argparse

##############################################################################################
LOG_FILENAME_PRREFIX = 'logRasp'
LATITUDE_MARKER = 'Широта: '
LONGITUDE_MARKER = '\tДолгота: '

##############################################################################################
parser = argparse.ArgumentParser(description='drone exploration path visio on Yandex maps')

parser.add_argument('--lat', action='store', dest='latitude',
        help='latitude of polygon center',
        required=True)

parser.add_argument('--long', action='store', dest='longitude', 
        help='longitude of polygon center',
        required=True)

parser.add_argument('--dir', action='store', dest='logs_dir', 
        help='path to logs',
        required=True)

latitude = parser.parse_args().latitude
longitude = parser.parse_args().longitude
logs_dir = parser.parse_args().logs_dir

##############################################################################################

def getRaspberryLogsList(logs_dir):

    rawFileList = [os.path.join(logs_dir, x) for x in os.listdir(logs_dir)]
    raspLogsFileList = []

    if rawFileList:

        for file in rawFileList:
            if file.find(LOG_FILENAME_PRREFIX) != -1:
                raspLogsFileList.append(file)

    return sorted(raspLogsFileList, key = os.path.getmtime)

def getRawPathStrings(logFileList):
    
    rawCoordinateStrings = []

    for log in logFileList:
        with open(log, 'r') as f:
            for line in f:
                if(line.find(LATITUDE_MARKER) != -1):
                    rawCoordinateStrings.append(line)

    return rawCoordinateStrings

def getCoordinates(rawCoordinateStrings):

    coordinatesList = []

    for rawLine in rawCoordinateStrings:

        latIndexStart = rawLine.find(LATITUDE_MARKER) + len(LATITUDE_MARKER)
        latIndexEnd = rawLine.find(LONGITUDE_MARKER)

        longIndex = latIndexEnd + len(LONGITUDE_MARKER)

        lat_ = rawLine[latIndexStart:latIndexEnd]
        long_ = rawLine[longIndex:-1]

        coordinateString = '[' + lat_ + ', ' + long_ + '],'
        coordinatesList.append(coordinateString)

    return coordinatesList

def makePolylineJS(latitude, longitude, coordinates):

    JStext = 'ymaps.ready(init);\n'
    JStext += 'function init() {\n'
    JStext += 'var myMap = new ymaps.Map("map", {\n'
    JStext += str('center: [' + latitude + ', ' + longitude + '],\n')
    JStext += 'zoom: 20}, {\n'
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

coordinatesList = getCoordinates(getRawPathStrings(getRaspberryLogsList(logs_dir)))

makePolylineJS(latitude, longitude, coordinatesList)
showDronePath()