#ifndef DRON_H
#define DRON_H

#include <stdint.h>
#include <vector>
#include <list>
#include <string>

#include "core.h"

class Dron
{
public:
    Dron(double xPos, double yPos);

    void setTask(double x, double y, TASK task);
    inline void currentPos(double &x, double &y) const { x=xPos; y=yPos; }
    inline TASK currentTask() const { return taskStack.back(); }
    inline double currentCourse() const { return course; }
    inline int32_t currentBypassDirection() const { return bypassDirection; }
    void run(double modelingTime, const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles);

private:
//    const double ceilPointsSize=1000;   // размер элемента ячейки - 1000м
//    const double radiusTargetZone=50;   // радиус зоны, в которую необходимо зайти,
//                                        // чтобы считалось, что задание выполнено - 50м
    const double dispersionMax=10;      // предельное отклонение от координат последней точки
    const double runTime=1;             // размер шага по времени - 1с
    const double lidarDistance=20;      // дальность лидара - 20м
    const int dronLength=5;             // длина дрона - 5м
    const int dronWidth=3;              // ширина дрона - 3м
    const double velForw=5;             // скорость "вперед" - 5м/с
    const double velBack=3;             // скорость "назад" - 3м/с

    const int turningRadius;            // радиус разворота дрона - 4м - расcчитывается

    double course=0;
    double xPos, yPos; // в координатах ячейки
    double xEndPos, yEndPos;
    double lastTime=0;
    double pathLengthLocal=0;
    double pathLengthSummary=0;


    double currenLength=0;              // текущее расстояние до конечной точкой
    double currentAzimuth=0;            // текущий азимут из конечной точки в текущую
    double maneuverAccumulation=0;      // индикатор петли при обходе препятствия
    int32_t bypassDirection=0;          // направление обхода препятствия. Выбирается рандомно,
                                        // (-1) - в сторону уменьшения курсового угла
                                        // 0 - нет обхода
                                        // 1 - в сторону возрастания курсового угла
    bool directionHasChanged=false;     // признак смены направления при обходе препятствия
    std::list<TASK> taskStack;

    bool analysisCourse(double &nc, double dl,
                        const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles);
    void maneuverDataReset();
    void maneuverDataSet();
    void taskReset();
    double azimuth(double &ds);
};

#endif // DRON_H
