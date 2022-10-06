#!/usr/bin/env python3
#coding: utf-8

import fileinput
import matplotlib.patches
import matplotlib.pyplot as plt

STATIC_HOLES_INDEX = 0
DYNAMIC_HOLES_INDEX = 1
PATH_INDEX = 2

################################################################################
#
# Настройки отображения

POLYGON_X_SIZE = 15000
POLYGON_Y_SIZE = 15000
FONT_SIZE = 8
FONT_GAP = 0.3
NODE_RADIUS = 0.2
ROUND_DIGITS = 1
SHOW_ANNOTATIONS = False

################################################################################

def makeHolesArray(rawInput, index, makerStart):

    rawHoles = rawInput[index]

    markerEnd = ")]"
    delimeter = ");("

    holesString = rawHoles[len(makerStart):-len(markerEnd)] 
    holesStrings = holesString.split(delimeter)

    holesArray = []

    for hole in holesStrings:
        currentHoleCoords = hole.split(",")
        holesArray.append(currentHoleCoords)

    return holesArray

def makePathArray(rawInput, pathIndex):

    pathArray = []

    pathRawData = rawInput[pathIndex + 1:]

    for pathPoint in pathRawData:
        pathArray.append(pathPoint.split(":"))

    return pathArray

################################################################################

def drawGrid():
    plt.xlim(-1, POLYGON_X_SIZE)
    plt.ylim(-1, POLYGON_Y_SIZE)
    plt.grid()

def defineAxes():
    axes = plt.gca()
    axes.set_aspect("equal")
    return axes

def plotRestrictionHoles(axes, holesContainer, color_):

    holePolys = []

    for hole in holesContainer:

        holePoly = []

        for pseudoPoint in hole:

            rawPoint = pseudoPoint.split(":")
            rawPoint[0] = float(rawPoint[0])
            rawPoint[1] = float(rawPoint[1])

            holePoly.append(tuple(rawPoint))

        holePolys.append(holePoly)

    for hole in holePolys:
        polyHole = matplotlib.patches.Polygon(hole, color = color_)
        axes.add_patch(polyHole)

def plotArrow(axes, begPoint, endPoint, time, radAngle, status):

    plotPoint(axes, begPoint[0], begPoint[1], time, radAngle, status)

    x0, y0 = begPoint
    xn, yn = endPoint

    arrow_dx = xn - x0
    arrow_dy = yn - y0

    arrow = matplotlib.patches.Arrow(x0, y0, arrow_dx, arrow_dy, width=0.2, color="b")
    axes.add_patch(arrow)

def plotPoint(axes, x, y, time, radAngle, status):

    annotation = str(round(float(time), ROUND_DIGITS)) + "/" + str(round(float(radAngle), ROUND_DIGITS)) + "/" + status
    
    node = matplotlib.patches.Circle((x, y), radius=NODE_RADIUS, fill=False)
    axes.add_patch(node)

    if SHOW_ANNOTATIONS == True:
        plt.text(x, y + FONT_GAP, annotation, horizontalalignment="center")

def plotPath(axes, path):
    
    statusDict = {
        'CHECKPOINT_REACHED':'CR',
        'CHECKPOINT_NOT_AVAILABLE':'CNA',
        'MOVE':'M',
        'SEARCH_TARGET':'ST',
        'BYPASSING_OBSTACLE':'BO',
        'NOT_SET':'NS'   
    }

    rangeLen = len(path)

    for index in range(rangeLen):
        if index < rangeLen - 1:
            
            time = path[index][0]
            xbeg = float(path[index][1])
            ybeg = float(path[index][2])
            angle = path[index][3]

            shortStatusName = statusDict[path[index][4]]

            xend = float(path[index + 1][1])
            yend =float(path[index + 1][2])

            plotArrow(axes, [xbeg, ybeg], [xend, yend], time, angle, shortStatusName)
        else:

            time = path[index][0]
            xLast = float(path[index][1])
            yLast = float(path[index][2])
            angle = path[index][3]

            shortStatusName = statusDict[path[index][4]]    
            plotPoint(axes, xLast, yLast, time, angle, shortStatusName)

def setFontSize():
    matplotlib.rcParams.update({'font.size': FONT_SIZE})

################################################################################

rawInput = []
for line in fileinput.input():
   rawInput += line.split()

staticHoles = makeHolesArray(rawInput, STATIC_HOLES_INDEX, "holes_static=[(")
dynamicHoles = makeHolesArray(rawInput, DYNAMIC_HOLES_INDEX, "holes_dynamic=[(")
path = makePathArray(rawInput, PATH_INDEX)

drawGrid()
axes = defineAxes()

setFontSize()
plotRestrictionHoles(axes, staticHoles, "g")
plotRestrictionHoles(axes, dynamicHoles, "r")
plotPath(axes, path)

plt.show()
