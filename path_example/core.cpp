#include <cmath>
#include <cfloat>

#include "core.h"

bool checkPointInStaticBlockages(int32_t x, int32_t y, const CEIL &staticBlockages)
{
    for(const RECT &ceil : staticBlockages)
    {
        if(x>=ceil.xMin && x<=ceil.xMin && y>=ceil.yMin && y<=ceil.yMin)
            return true;
    }
    return false;
}

bool checkPointInDynamicBlockages(double x, double y, const OBSTACLES &dynamicObstacles)
{
    for(const Obstacle &obstacle : dynamicObstacles)
    {
        const RECT &ceil=obstacle.boundingRect;
        if(x>=ceil.xMin && x<=ceil.xMin && y>=ceil.yMin && y<=ceil.yMin &&
           checkPointInPolygon(x, y, obstacle.polygon))
            return true;
    }
    return false;
}

bool checkPointInPolygon(double x, double y, const POLYGON &polygon)
{
    int32_t n=0;
    const int64_t sz=polygon.size();
    double k, b, x1, y1, x2, y2, xp;

    for(int64_t i=0; i<sz; i++)
    {
        const std::pair<double, double> &next=polygon[(i+1)%sz];

        x1=polygon[i].first-x;
        y1=polygon[i].second-y;

        x2=next.first-x;
        y2=next.second-y;

        if(x1==0 && y1==0) // совпадение с узлом
            return true;

        if(x1<0 && x2<0) // на обратной стороне луча
            continue;

        if(y2==0) // узел на пути луча: рассматриваем только (x1, y1)
            continue;

        if(std::abs(y1-y2)<1e-12)
        {
            if(std::abs(y1)<1e-12 && SIGN_2S(x1)!=SIGN_2S(x2)) // на ребре
                return true;
            continue; // иначе ребро не учитывается
        }

        if(x1==x2) // вертикаль
            xp=x1;
        else
        {
            k=(y2-y1)/(x2-x1);
            b=y1-k*x1;
            xp=-b/k;
        }
        n+=xp>=0 && xp>std::min(x1, x2) && xp<std::max(x1, x2);
    }
    return n%2;
}

void generateBlockages(const int32_t szX, const int32_t szY, OBSTACLES &dynamicObstacles)
{
    int32_t n; // - количество сторон
    int32_t attempt;
    double x0, y0, x, y, r, a, da;
    POLYGON polygon;
    RECT boundingRect;

    auto rnd=[](double min, double max)->double
    {
        double d=max-min;
        return min+d*rand()/RAND_MAX; // от min до max
    };

    do
    {
        x0=rnd(0, szX);
        y0=rnd(0, szY);

        if(checkPointInDynamicBlockages(x0, y0, dynamicObstacles))
            continue;

        a=0;
        attempt=0;
        n=rnd(3, 8); // от 3 до 8
        polygon.clear();
        polygon.resize(n);
        for(int32_t i=0; i<n;)
        {
            da=(M_PIm2-a)/(n-i);
            a=rnd(a, a+da);     // азимут
            r=rnd(1, 15);       // дальность
            x=x0+r*cos(a);
            y=y0+r*sin(a);

            if(checkPointInDynamicBlockages(x, y, dynamicObstacles))
            {
                if(++attempt>10)
                    break;
                continue;
            }
            polygon[i]={x, y};
            i++;
        }
    } while(attempt>10);

    bool firstWrite=true;;
    for(const std::pair<double, double> &point : polygon)
    {
        if(firstWrite || boundingRect.xMin>point.first)  boundingRect.xMin=point.first;
        if(firstWrite || boundingRect.yMin>point.second) boundingRect.yMin=point.second;
        if(firstWrite || boundingRect.xMax<point.first)  boundingRect.xMax=point.first;
        if(firstWrite || boundingRect.yMax<point.second) boundingRect.yMax=point.second;
        firstWrite=false;
    }
    dynamicObstacles.push_back({polygon, boundingRect});
}

//LOCALMAP createMapFromBlocages(double x0, double y0, double course, double lidarDistance, const BLOCAGES &blockages)
//{
//    LOCALMAP map;
//    const int angleDisp=30; // максимальное отклонение от осевого направления в гр
//    const int nA=angleDisp*2+1; // количество отсчетов
//    const int nR=lidarDistance;
//    double az, x, y;
//    bool forbidden;

//    map.resize(nA);

//    double ccc=RTOG(course);

//    for(int a=0; a<nA; a++)
//    {
//        az=course+(a-angleDisp)*M_PId180;
//        map[a].resize(nR);
//        forbidden=false;

//        double aaa=RTOG(az);

//        for(int r=0; r<nR; r++)
//        {
//            if(!forbidden)
//            {
//                x=(r+1)*cos(az)+x0;
//                y=(r+1)*sin(az)+y0;
//                forbidden=checkPointInBlockages(x, y, blockages);
//            }
//            map[a][r]=forbidden ? MAPTYPES::FIRBIDDEEN : MAPTYPES::INTERMEDIATE;
//        }
//    }
//    return map;
//}

// azimuth - в радианах
MAPTYPES typePoint(double x0, double y0, double azimuth, int32_t r,
                   const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles)
{
    double x, y;

    x=r*cos(azimuth)+x0;
    y=-r*sin(azimuth)+y0;
    return checkPointInStaticBlockages(x, y, staticBlockages) ||
              checkPointInDynamicBlockages(x, y, dynamicObstacles)
            ? MAPTYPES::FIRBIDDEEN
            : MAPTYPES::INTERMEDIATE;
}

// course - в градусах
//void createMapFromBlocages(double x0, double y0, int32_t course, int32_t dr, int32_t da,
//                           int32_t widthSector, double lidarDistance,
//                           const CEIL &staticBlockages, const BLOCAGES &dynamicBlockages,
//                           LOCALMAP &map)
//{
//    const int nR=CEILING(lidarDistance);
//    int32_t szMap=map.size(), indexZero=szMap/2, amin, amax;
//    MAPTYPES type;

//    amin=MAX(0, indexZero+da-widthSector/2);
//    amax=MIN(szMap-1, indexZero+da+widthSector/2);

//    for(int32_t a=amin; a<=amax; a++)
//    {
//        if(map[a].size())
//            continue;

//        map[a].resize(nR);
//        type=MAPTYPES::UNKNOWN;
//        for(int r=0; r<nR; r++)
//        {
//            if(type!=MAPTYPES::FIRBIDDEEN)
//                type=typePoint(x0, y0, course+a-indexZero, r+dr, staticBlockages, dynamicBlockages);
//            map[a][r]=type;
//        }
//    }
//}

