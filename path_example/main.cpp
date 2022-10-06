#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <memory>
#include <cstring>

#include "dron.h"
#include "core.h"

#include <time.h>
std::string file="arr_"+std::to_string(time(nullptr))+".txt";

inline int32_t fromIndex(int32_t nX, int32_t y, int32_t x)
{
    return y*nX+x;
}

std::string toStr(int64_t data)
{
    char ss[100];
    sprintf(ss, "%4lld", data);
    return std::string(ss)+"  ";
}

template<typename F>
void print_cycle(int32_t nX, int32_t nY, std::string txt, F functor)
{
    FILE *f;
    int32_t index;

    f=fopen(file.c_str(), "at");

    fprintf(f,"%s\n", txt.data());

    for(int y=0; y<nY; y++)
    {
        for(int x=0; x<nX; x++)
        {
            index=fromIndex(nX, y, x);
            fprintf(f,"%s", functor(index, y, x).data());
        }
        fprintf(f,"\n");
    }
    fclose(f);
}


inline void wr_if_greater(int32_t &recv, int32_t src)
{
    if(recv==-1 || recv<src) recv=src;
};

inline void wr_if_less(int32_t &recv, int32_t src)
{
    if(recv==-1 || recv>src) recv=src;
};

inline void wr_if_near_to_middle_on_left(int32_t &recv, int32_t src, int32_t middle)
{
    if(recv==-1 || (src<middle && (recv<src || recv>middle))) recv=src;
};

inline void wr_if_near_to_middle_on_right(int32_t &recv, int32_t src, int32_t middle)
{
    if(recv==-1 || (src>middle && (recv>src || recv<middle))) recv=src;
};

enum class POINT_TYPE : int
{// -1 - запрещенная, 0 - промежуточная, 1 - начальная, 2 - конечная
    FORBIDDEN=-1, INTERMEDIATE=0, STARTING=1, ENDING=2
};

inline int32_t length(int32_t dy, int32_t dx)
{
    static int32_t lengthOfLocalTransition[9]=
    {
        14, 10, 14,
        10,  0, 10,
        14, 10, 14,
    };

    return lengthOfLocalTransition[dy*3+dx+4];// (dy+1)*3+dx+1=3*dy+3+dx+1=3*dy+dx+4
}

struct ZoneInfo
{
    int64_t penalty=0;
};

struct ItemConstArray : ZoneInfo
{
    POINT_TYPE pointType=POINT_TYPE::INTERMEDIATE;
};

struct ItemPoint
{
    ItemPoint()=default;
    ItemPoint(int32_t x, int32_t y) : x(x), y(y) { }
    int32_t x=-1, y=-1;
};

struct ItemProcessingArray
{
    int64_t penaltyOfRoute=-1;  // накопленный штраф. -1 - не рассчитывалось
    int32_t lengthOfRoute=-1;   // накопленный путь.  -1 - не рассчитывалось
    ItemPoint prevPoint;
};

struct Rect
{
    Rect()=default;
    Rect(const ItemPoint &upper_left, const ItemPoint &lower_right) :
        upper_left(upper_left), lower_right(lower_right) {}

    ItemPoint upper_left, lower_right;
    inline bool inRect(int32_t y, int32_t x)
    {
        return y>=upper_left.y && y<=lower_right.y &&
                x>=upper_left.x && x<=lower_right.x;
    }
};

void inflateArea(const int32_t nX, const int32_t nY, const int32_t nP,
                 ItemConstArray *constArray,
                 Rect areaIn, Rect areaOut,
                 ItemProcessingArray *src, ItemProcessingArray *dest)
{
    const int32_t indexXmax=nX-1, indexYmax=nY-1;
    int32_t index1, index2;
    int32_t x, y;
    int32_t x_env, y_env;
    int32_t x_env_min, x_env_max, y_env_min, y_env_max;
    int32_t dl;
    int64_t penalty;
    bool modify, write;
    int32_t middleX, middleY;
    ItemProcessingArray *tmp, *dest_=dest;

    auto inflate_=[=](Rect &area, int32_t direction)
    {
        area.upper_left.x=MAX(0, area.upper_left.x-direction);
        area.upper_left.y=MAX(0, area.upper_left.y-direction);

        area.lower_right.x=MIN(indexXmax, area.lower_right.x+direction);
        area.lower_right.y=MIN(indexYmax, area.lower_right.y+direction);
    };
    auto inflate=[=](Rect &area) { inflate_(area, 1); };
    auto deflate=[=](Rect &area) { inflate_(area, -1); };

//    print_cycle(nX, nY, "ItemConstArray-pointType",
//                [&](int index, int y, int x)->std::string
//    {
//        const ItemConstArray &item=(reinterpret_cast<ItemConstArray *>(constArray))[index];
//        return toStr(static_cast<int>(item.pointType))+" ";
//    });

//    print_cycle(nX, nY, "\nItemProcessingArray-lengthOfRoute \nP="+toStr(-1),
//                [&](int index, int y, int x)->std::string
//    {
//        const ItemProcessingArray &item=src[index];
//        return toStr(static_cast<int>(item.lengthOfRoute))+" ";
//    });

    for(int32_t p=0; p<nP; p++)
    {
        Rect areaIn_l, areaOut_l;

//        deflate(areaIn); // уменьшаем внутреннюю область
        inflate(areaOut); // увеличиваем внешнюю область

//        middleX=(areaOut.upper_left.x+areaOut.lower_right.x)/2;
//        middleY=(areaOut.upper_left.y+areaOut.lower_right.y)/2;

        modify=false;

        for(y=areaOut.upper_left.y; y<=areaOut.lower_right.y; y++)
        {
            y_env_min=MAX(0, y-1);
            y_env_max=MIN(indexYmax, y+1);

            for(x=areaOut.upper_left.x; x<=areaOut.lower_right.x; x++)
            {
//                if(areaIn.inRect(y, x))
//                    continue;

                index1=y*nX+x;

                if(constArray[index1].pointType==POINT_TYPE::STARTING ||
                   constArray[index1].pointType<POINT_TYPE::INTERMEDIATE)
                    continue;

                x_env_min=MAX(0, x-1);
                x_env_max=MIN(indexXmax, x+1);

                dest[index1]=src[index1];
                write=false;
                for(y_env=y_env_min; y_env<=y_env_max; y_env++)
                {
                    for(x_env=x_env_min; x_env<=x_env_max; x_env++)
                    {
                        if(y==y_env && x==x_env)
                            continue;

                        index2=y_env*nX+x_env;
                        if(constArray[index2].pointType==POINT_TYPE::ENDING ||
                           constArray[index2].pointType<POINT_TYPE::INTERMEDIATE ||
                           src[index2].penaltyOfRoute<0)
                            continue;

                        penalty=src[index2].penaltyOfRoute+constArray[index2].penalty;
                        if(dest[index1].penaltyOfRoute>=0 && dest[index1].penaltyOfRoute<penalty)
                            continue;

                        dl=src[index2].lengthOfRoute+length(y_env-y, x_env-x);
                        if(dest[index1].penaltyOfRoute==penalty && dest[index1].lengthOfRoute<=dl)
                            continue;

                        write=true;
                        dest[index1].lengthOfRoute=dl;
                        dest[index1].penaltyOfRoute=penalty;
                        dest[index1].prevPoint.x=x_env;
                        dest[index1].prevPoint.y=y_env;
                    }
                }
                if(write)
                {
                    wr_if_less(areaOut_l.upper_left.y, y);
                    wr_if_less(areaOut_l.upper_left.x, x);

                    wr_if_greater(areaOut_l.lower_right.x, x);
                    wr_if_greater(areaOut_l.lower_right.y, y);

//                    wr_if_near_to_middle_on_left(areaIn_l.upper_left.x, x, middleX);
//                    wr_if_near_to_middle_on_left(areaIn_l.upper_left.y, y, middleY);

//                    wr_if_near_to_middle_on_right(areaIn_l.lower_right.x, x, middleX);
//                    wr_if_near_to_middle_on_right(areaIn_l.lower_right.y, y, middleY);

                    modify=true;
                }
            }   //x
        }   //y

        if(!modify)
            break;

//        print_cycle(nX, nY, "\nItemProcessingArray-lengthOfRoute \nP="+toStr(p),
//                    [&](int index, int y, int x)->std::string
//        {
//            const ItemProcessingArray &item=dest[index];
//            return toStr(static_cast<int>(item.lengthOfRoute))+" ";
//        });

        tmp=src;
        src=dest;
        dest=tmp;

        areaOut=areaOut_l;
//        areaIn=areaIn_l;

    }   //p
    if(dest_!=dest)
        ::memcpy(dest, src, static_cast<uint64_t>(nX*nY));
}

bool createConstArray(int x_beg, int y_beg, int nX, int nY, int *arrayArea, ItemConstArray *constArray)
{
    const int32_t index_beg=y_beg*nX+x_beg;
    bool ret=false;

    for(int y=0; y<nY; y++)
        for(int x=0; x<nX; x++)
        {
            const int32_t index=y*nX+x;
            constArray[index].penalty=arrayArea[index];
            if(index==index_beg)
                constArray[index].pointType=POINT_TYPE::STARTING;
            else
                if(!arrayArea[index])
                {
                    constArray[index].pointType=POINT_TYPE::ENDING;
                    ret=true;
                }
                else
                    if(arrayArea[index]==-1)
                        constArray[index].pointType=POINT_TYPE::FORBIDDEN;
                    else constArray[index].pointType=POINT_TYPE::INTERMEDIATE;
        }
    return ret;
}

template<typename F>
void reconstructPath(const int32_t nX, const int32_t nY,
                     ItemProcessingArray *src, ItemConstArray *constArray,
                     int32_t offsetX, int32_t offsetY,
                     std::vector<std::pair<int32_t, int32_t>> &path,
                     F functor)
{
    int32_t penalty, penalty_l, length;
    std::vector<std::pair<int32_t, int32_t>> endPoints;

    for(int y=0; y<nY; y++)
        for(int x=0; x<nX; x++)
        {
            const int32_t index=y*nX+x;
            if(constArray[index].pointType!=POINT_TYPE::ENDING || src[index].lengthOfRoute==-1)
                continue;

            penalty_l=src[index].penaltyOfRoute+constArray[index].penalty;
            if(endPoints.empty() || penalty>penalty_l ||
               (penalty==penalty_l && length>src[index].lengthOfRoute) ||
               (penalty==penalty_l && length==src[index].lengthOfRoute))
            {
                if(penalty>penalty_l || length>src[index].lengthOfRoute)
                    endPoints.clear();
                endPoints.push_back({x, y});
               penalty=penalty_l;
               length=src[index].lengthOfRoute;
            }
        }
    if(endPoints.empty())
        return;

    int32_t index=(endPoints.size()>1) ? functor(endPoints) : 0;
    std::deque<ItemPoint> route;
    ItemPoint point;

    if(index<=0 || index>=int32_t(endPoints.size())) index=0;
    point={ endPoints[index].first, endPoints[index].second };
    do
    {
        route.push_front(point);
        const int32_t index=point.y*nX+point.x;
        point=src[index].prevPoint;
    }
    while(point.x!=-1);

    path.resize(route.size()-1);
    index=0;
    for(const ItemPoint &item : route)
    {
        if(index)
            path[index-1]={ item.x+offsetX, item.y+offsetY };
        index++;
    }
    return;
}

bool check(const std::pair<int32_t, int32_t> &startingPoint,
           const std::vector<std::vector<int64_t>> &region)
{
    const int32_t nYR=region.size();

    if(!nYR || startingPoint.second<0 || startingPoint.second>=nYR)
        return false;

    const int32_t nXR=region[0].size();

    return nXR && startingPoint.first>=0 && startingPoint.first<nXR;
}

// внутренняя функция
template<typename FEP, typename SEP>
std::vector<std::pair<int32_t, int32_t>>
routeCalculationInternal(const std::pair<int32_t, int32_t> &startingPoint,
                         const std::vector<std::vector<int64_t>> &region,
                         std::pair<int32_t, int32_t> topLeft,
                         std::pair<int32_t, int32_t> bottomRight,
                         FEP fillEndPoins, SEP selectorEndPoins)
{
    if(!check(startingPoint, region))
        return {};

    const int32_t nYR=region.size();
    const int32_t nXR=region[0].size();
    bool ret=false;

    if(topLeft.first<0) topLeft.first=0;
    if(bottomRight.first>=nXR) bottomRight.first=nXR;

    if(topLeft.second<0) topLeft.second=0;
    if(bottomRight.second>=nYR) bottomRight.second=nYR;

    const int32_t nX=bottomRight.first-topLeft.first+1;
    const int32_t nY=bottomRight.second-topLeft.second+1;
    const int32_t nP=((nX-2)*nY/2.+1)+1;
    const int32_t szArray=nX*nY;

    const int32_t &x_beg=startingPoint.first-topLeft.first,
                  &y_beg=startingPoint.second-topLeft.second;
    const int32_t index_beg=y_beg*nX+x_beg;

    const Rect areaBegs={{x_beg, y_beg}, {x_beg, y_beg}};
    ItemConstArray *constArray=new ItemConstArray[szArray];

    for(int y=0; y<nY; y++)
        for(int x=0; x<nX; x++)
        {
            const int32_t index=fromIndex(nX, y, x);
            const int32_t &val=region[y+topLeft.second][x+topLeft.first];

            constArray[index].penalty=val;
            constArray[index].pointType=val==-1
                    ? POINT_TYPE::FORBIDDEN
                    : POINT_TYPE::INTERMEDIATE;
        }
    ret=fillEndPoins(nX, nY, constArray);

    std::vector<std::pair<int32_t, int32_t>> res;
    if(ret)
    {
        ItemProcessingArray *src=new ItemProcessingArray[szArray],
                *dest=new ItemProcessingArray[szArray];

        src[index_beg].lengthOfRoute=0;
        src[index_beg].penaltyOfRoute=0;
        constArray[index_beg].pointType=POINT_TYPE::STARTING;
        inflateArea(nX, nY, nP, constArray, areaBegs, areaBegs, src, dest);
        reconstructPath(nX, nY, dest, constArray,
                        topLeft.first, topLeft.second, res, selectorEndPoins);

        delete [] src;
        delete [] dest;
    }

    delete [] constArray;
    return res;
}

// первый вариант - для патрулирования в зоне
// startingPoint - координаты начальной точки в region,
//                 startingPoint.first<->x, startingPoint.second<->y
// region - область в которой строится маршрут - region[y][x]
// topLeft, bottomRight - координаты зоны в районе
// isEndPoints - функтор, проверки, является ли точка с координатами (x, y) конечной
//               возвращаемое значение - true, если точка с координатами (x, y) является конечной
//               прототип: bool f(int32_t x, int32_t y, int64_t val), val - значение region[y][x]
// selectorEndPoins - функтор выбора конечной точки маршрута.
//                    Вызывается при наличии нескольких равных путей
//                    возвращаемое значение - индекс лучшей точки из вектора endPoints
//                    прототип: int f(const std::vector<std::pair<int32_t, int32_t>> &endPoints)
template<typename IEP, typename SEP>
std::vector<std::pair<int32_t, int32_t>>
routeCalculation(const std::pair<int32_t, int32_t> &startingPoint,
                 const std::vector<std::vector<int64_t>> &region,
                 const std::pair<int32_t, int32_t> &topLeft,
                 const std::pair<int32_t, int32_t> &bottomRight,
                 IEP isEndPoints, SEP selectorEndPoins)
{
    return routeCalculationInternal(startingPoint, region, topLeft, bottomRight,
    [&isEndPoints, &region, &topLeft]
    (int32_t nX, int32_t nY, ItemConstArray *array)->bool
    {
        bool ret=false;
        for(int y=0; y<nY; y++)
            for(int x=0; x<nX; x++)
                if(isEndPoints(x+topLeft.first, y+topLeft.second, region[y][x]))
                {
                    const int32_t index=fromIndex(nX, y, x);
                    array[index].pointType=POINT_TYPE::ENDING;
                    ret=true;
                }
        return ret;
    },
    selectorEndPoins);
}

// второй вариант - для выхода в одну из заданных точек
// region - область в которой строится маршрут - region[y][x]
// startingPoint - координаты начальной точки в region,
//                 startingPoint.first<->x, startingPoint.second<->y
// endPoints - координаты конечных точки в region
// selectorEndPoins - функтор выбора конечной точки маршрута.
//                    Вызывается при наличии нескольких равных путей
//                    возвращаемое значение - индекс лучшей точки из вектора endPoints
//                    прототип: int f(const std::vector<std::pair<int32_t, int32_t>> &endPoints)
template<typename SEP>
std::vector<std::pair<int32_t, int32_t>>
routeCalculation(const std::pair<int32_t, int32_t> &startingPoint,
                 const std::vector<std::pair<int32_t, int32_t>> &endPoints,
                 const std::vector<std::vector<int64_t>> &region,
                 const std::pair<int32_t, int32_t> &topLeft,
                 const std::pair<int32_t, int32_t> &bottomRight,
                 SEP selectorEndPoins)
{
    return routeCalculationInternal(startingPoint, region, topLeft, bottomRight,
    [&endPoints](int32_t nX, int32_t nY, ItemConstArray *array)->bool
    {
        bool ret=false;
        for(const std::pair<int32_t, int32_t> &item : endPoints)
        {
            if(item.first>=0 && item.first<nX && item.second>=0 && item.second<nY)
            {
                const int32_t index=fromIndex(nX, item.second, item.first);
                array[index].pointType=POINT_TYPE::ENDING;
                ret=true;
            }
        }
        return ret;
    },
    selectorEndPoins);
}

enum  class Command : int32_t
{
    // команды
    GO, ZONE_CHANGED, RESET_ZONE, RETURN, GO_TO_POINT, ABORT,
    // действия
    WAITING, GO_TO_PATROL_ZONE, GO_TO_HOME, TRANSITION_TO_POINT, PATROL_ZONE,
    // запрсы
    QUERY_UPDATE_ZONE
};

// допущение - размеры района не должны изменятся, зона патрулирования может плавать
// startingPoint - стартовая точка в системе координат района
// topLeft, bottRight - координаты (min, max) зоны патрулирования в системе координат района
// moveController - внешняя система управления
//                  прототип:
//                  Command moveController(const std::pair<int32_t, int32_t> &currentPosition,
//                                         std::pair<int32_t, int32_t> &nextPosition,
//                                         int32_t indexNextPosition,
//                                         const std::vector<std::pair<int32_t, int32_t>> &localPath,
//                                         Command currentCommand)
//                  параметр nextPosition, - может быть изменен, если возвращена команда GO_TO_POINT
template<typename MC>
void run(const std::pair<int32_t, int32_t> &startingPoint,
         std::pair<int32_t, int32_t> &pointOfReturn,
         std::pair<int32_t, int32_t> &topLeft,
         std::pair<int32_t, int32_t> &bottomRight,
         std::vector<std::vector<int64_t>> &region,
         MC moveController)
{
    std::pair<int32_t, int32_t> currentPosition=startingPoint;
    std::vector<std::pair<int32_t, int32_t>> localPath;
    Command currentCommand=Command::WAITING;
    bool ignoreKeyValue=true;
    std::pair<int32_t, int32_t> nextPosition;

    auto randomizePoint=[](const std::vector<std::pair<int32_t, int32_t>> &foundPoints)->int32_t
    {
        int32_t sz=foundPoints.size();
        return rand()%sz;
    };

    auto runController=
    [moveController, &ignoreKeyValue, &currentPosition, &nextPosition, &region]
    (const std::vector<std::pair<int32_t, int32_t>> &route, Command currentCommand)->Command
    {
        Command ret=currentCommand, local;
        int32_t sz=int32_t(route.size());

        for(int index=0; index<sz; index++)
        {
            if(!ignoreKeyValue)
                region[currentPosition.second][currentPosition.first]+=1;

            nextPosition=route[index];
            local=moveController(currentPosition, nextPosition, index, route, currentCommand);
            if(local!=Command::GO)
            {
                ret=local;
                break;
            }
            currentPosition=route[index];
        }
        return ret;
    };

    auto findNextPoint=[&region, &topLeft, &bottomRight, &currentPosition, randomizePoint]()
            ->std::vector<std::pair<int32_t, int32_t>>
    {
        std::vector<std::pair<int32_t, int32_t>> foundPoints, localPath;
        int32_t x, y, index, sz, dl=-1;
        int32_t d[8]={2, 1, 2, 1, 1, 2, 1, 2};
        int32_t dc[8][2]=
        {
            {-1, -1}, {0, -1}, {1, -1},
            {-1,  0}, {1,  0},
            {-1,  1}, {0,  1}, {1,  1}
        };

        for(int32_t index=0; index<8; index++)
        {
            x=currentPosition.first+dc[index][0];
            y=currentPosition.second+dc[index][1];
            if(x<topLeft.first || y<topLeft.second ||
               x>bottomRight.first || y>bottomRight.second)
                continue;
            if(!region[y][x] && (dl==-1 || dl>=d[index]))
            {
                if(dl>d[index])
                {
                    dl=d[index];
                    foundPoints.clear();
                }
                foundPoints.push_back({x, y});
            }
        }
        sz=foundPoints.size();
        if(sz)
        {
            index=rand()%sz;
            return localPath={foundPoints[index]};
        }
        else
        {
            localPath=
            routeCalculation(currentPosition, region, topLeft, bottomRight,
            [&region](int32_t x, int32_t y, int64_t)->bool
            {
                return !region[y][x];
            },
            [randomizePoint](const std::vector<std::pair<int32_t, int32_t>> &foundPoints)->int32_t
            {
                return randomizePoint(foundPoints);
            });
        }

        return localPath;
    };

    if(!check(currentPosition, region))
        return;

    const int32_t nY=region.size();
    const int32_t nX=region[0].size();
    const std::pair<int32_t, int32_t> topLeftRegion={0, 0}, bottomRightRegion={nX-1, nY-1};

//    srand(time(nullptr));

    while(true)
    {
        switch(currentCommand)
        {
        case Command::WAITING   :
            currentCommand=moveController(currentPosition, currentPosition, -1, {}, currentCommand);
            continue;
        case Command::GO_TO_PATROL_ZONE  :  // действие - выход в зону патрулирования
            ignoreKeyValue=true;
            //    проверка на попадание в зону патрулирования
            if(currentPosition.first<topLeft.first || currentPosition.second<topLeft.second ||
               currentPosition.first>bottomRight.first || currentPosition.second>bottomRight.second)
            {
                localPath=
                routeCalculation(currentPosition, region, topLeftRegion, bottomRightRegion,
                [&topLeft, &bottomRight](int32_t x, int32_t y, int64_t)->bool
                {
                    bool onEdgeX=x==topLeft.first  || x==bottomRight.first,
                         onEdgeY=y==topLeft.second || y==bottomRight.second;
                    return (onEdgeX && y>=topLeft.second && y<=bottomRight.second) ||
                           (onEdgeY && x>=topLeft.first && x<=bottomRight.first) ;
                },
                [randomizePoint](const std::vector<std::pair<int32_t, int32_t>> &foundPoints)->int32_t
                {
                    return randomizePoint(foundPoints);
                });
            }
            break;
        case Command::GO_TO_HOME    :   // действие - возврат
            ignoreKeyValue=true;
            localPath=
            routeCalculation(currentPosition, {pointOfReturn}, region, topLeftRegion, bottomRightRegion,
            [randomizePoint](const std::vector<std::pair<int32_t, int32_t>> &foundPoints)->int32_t
            {
                return randomizePoint(foundPoints);
            });
            break;
        case Command::TRANSITION_TO_POINT    :  // действие - переход в заданную точку в зоне патрулирования
            ignoreKeyValue=false;
            localPath=
            routeCalculation(currentPosition, {nextPosition}, region, topLeft, bottomRight,
            [randomizePoint](const std::vector<std::pair<int32_t, int32_t>> &foundPoints)->int32_t
            {
                return randomizePoint(foundPoints);
            });
            break;
        case Command::PATROL_ZONE   :   // действие - патрулирование в зоне
            ignoreKeyValue=false;
            localPath=findNextPoint();
            if(localPath.empty())
            {
                currentCommand=Command::QUERY_UPDATE_ZONE;
                continue;
            }
            break;
        case Command::QUERY_UPDATE_ZONE : // запрос на изменение штрафного функционала
            currentCommand=moveController(currentPosition, currentPosition, -1, {}, currentCommand);
            continue;
        case Command::ZONE_CHANGED  : // изменились границы зоны патрулирования
            currentCommand=Command::GO_TO_PATROL_ZONE;
            continue;
        case Command::RESET_ZONE    : // сброс штрафного функционала стандартным алгоритмом
            // позже
            currentCommand=Command::GO_TO_PATROL_ZONE;
            continue;
        case Command::RETURN        : // команда на возврат
            currentCommand=Command::GO_TO_HOME;
            continue;
        case Command::GO_TO_POINT        :
            currentCommand=Command::TRANSITION_TO_POINT; // команда на переход в другую точку в зоне патрулирования
            continue;
        case Command::ABORT         : // прерывание расчета (для отладки)
            return;
        default                     :
            currentCommand=Command::GO_TO_PATROL_ZONE;
            continue;
        }

//        print_cycle(localPath.size(), 1, "\n local path: ",
//        [&](int index, int y, int x)->std::string
//        {
//            (void)x; // dummy warnings
//            (void)y; // dummy warnings

//            const std::pair<int32_t, int32_t> &item=localPath[index];
//            std::string str;

//            if(index && !(index%10)) str="\n";

//            str+=" ("+toStr(static_cast<int>(item.first))+
//                    ", "+toStr(static_cast<int>(item.second))+
//                    ") ";
//            return str;
//        });

        currentCommand=runController(localPath, currentCommand);
        if(currentCommand==Command::GO_TO_PATROL_ZONE || currentCommand==Command::TRANSITION_TO_POINT)
            currentCommand=Command::PATROL_ZONE;
        if(currentCommand==Command::GO_TO_HOME)
            break;

//        print_cycle(nX, nY, "\nRegion-penalty",
//                    [&](int index, int y, int x)->std::string
//        {
//            int64_t val=region[y][x];
//            return toStr(val)+" ";
//        });

    }
}

template<typename F>
void modeling(Dron &dron, double &modelingTime,
              const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles,
              F dronMoveController)
{
    double x, y;
    TASK task;

    while(true)
    {
        dron.run(modelingTime, staticBlockages, dynamicObstacles);
        dron.currentPos(x, y);
        task=dron.currentTask();
        dronMoveController(x, y, ROUND(RTOG(dron.currentCourse())), task);
        if(task==TASK::NOT_SET)
            break;
        if(task>TASK::BEGIN_MOVE_GROUP && task<TASK::END_MOVE_GROUP)
            modelingTime+=1;
    }
}

int main()
{
    const double ceilSize=1000;         // размер ячейки сетки - 1км
    // Допущение - region - это прямоугольный массив
    // Если это не так, то надо решать как расположены строки в массиве относительно друг друга
    std::vector<std::vector<int64_t>> region=
    {//   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14
/* 0 */ { 0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 1 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 2 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 3 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 4 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0,-1,-1, 0 },
/* 5 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0,-1,-1, 0 },
/* 6 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 7 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 8 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/* 9 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0 },
/*10 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0,-1,-1, 0 },
/*11 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0,-1, 0 },
/*12 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0,-1, 0 },
/*13 */ { 0, 0,-1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0,-1, 0 },
/*14 */ { 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
    };

    const int32_t nY=region.size();     // размер региона по Y
    const int32_t nX=region[0].size();  // размер региона по X (ограничение: Y>0)
    const int32_t szY=nY*ceilSize;      // размер региона по Y
    const int32_t szX=nX*ceilSize;      // размер региона по X (ограничение: Y>0)

    OBSTACLES dynamicObstacles;
    CEIL staticObstacles;

// запись статических препятствий
    for(int32_t y=0; y<nY; y++)
        for(int32_t x=0; x<nX; x++)
            if(region[y][x]==-1)
            {
                double xMin=x*ceilSize, yMin=y*ceilSize;
                staticObstacles.push_back({ xMin, yMin, xMin+ceilSize, yMin+ceilSize });
            }

//    генерация динамических препятствий
    dynamicObstacles.push_back({{{500, 1450}, {550, 1550}, {450, 1550}}, {450, 1450, 550, 1550}}); // тест треугольник
//    dynamicBlockages.push_back({{450, 1250}, {400, 1350}, {500, 1450},
//                                {600, 1350}, {550, 1250}, {700, 1350},
//                                {500, 1550}, {300, 1350}}); // тест подкова - не проходимо!

//    генерация динамических препятствий
    const int32_t blockageNumber=50; // количество препятствий
    for(int32_t i=0; i<blockageNumber; i++)
    {
        generateBlockages(szX, szY, dynamicObstacles);
    }

    std::vector<std::pair<int32_t, int32_t>> fullPath;

    // начальная точка
    std::pair<int32_t, int32_t> begPoint={0, 0};

    // границы района патрулирования (район включает границу)
    std::pair<int32_t, int32_t> topLeft={10, 0}, bottomRight={14, 14};
    // новый район патрулирования (район включает границу)
    std::pair<int32_t, int32_t> topLeft2={7, 9}, bottomRight2={9, 14};

    // точки входа в район патрулирования
    // в координатах общего района, возможно удобнее будет в координатах района патрулирования
    std::vector<std::pair<int32_t, int32_t>> endPoints={{10, 6}, {10, 7}, {10, 8}};

    // точка возврата
    std::pair<int32_t, int32_t> homePoint={0, 8};

    fullPath.push_back(begPoint);

    Dron dron(0, 500); // дрон на середине
    double modelingTime=0;

    ////////////////////////////// Вывод исходных данных //////////////////////
    bool fwPl=true;
    std::cout<<"holes_static=[";
    for(const RECT &rect : staticObstacles)
    {
        if(!fwPl) std::cout<<",";
        std::cout<<"("
                 <<rect.xMin<<":"<<rect.yMin
                 <<","<<rect.xMin<<":"<<rect.yMax
                 <<","<<rect.xMax<<":"<<rect.yMax
                 <<","<<rect.xMax<<":"<<rect.yMin
                 <<")";
        fwPl=false;
    }
    std::cout<<"]"<<std::endl;

    fwPl=true;
    std::cout<<"holes_dynamic=[";
    for(const Obstacle &obstacle : dynamicObstacles)
    {
        if(!fwPl) std::cout<<";";
        std::cout<<"(";
        bool fwPt=true;
        for(const std::pair<double, double> &point : obstacle.polygon)
        {
            if(!fwPt) std::cout<<",";
            std::cout<<point.first<<":"<<point.second;
            fwPt=false;
        }
        std::cout<<")";
        fwPl=false;
    }
    std::cout<<"]"<<std::endl;
    ///////////////////////////////////////////////////////////////////////////

    std::cout<<"path="<<std::endl;
    run(begPoint, homePoint, topLeft, bottomRight, region,
    [begPoint, &fullPath, &topLeft, &topLeft2, &bottomRight, &bottomRight2,
     &staticObstacles, &dynamicObstacles, &dron, &modelingTime]
        (const std::pair<int32_t, int32_t> &currentPosition,
        std::pair<int32_t, int32_t> &nextPosition, int32_t indexNextPosition,
        const std::vector<std::pair<int32_t, int32_t>> &localPath, Command currentCommand)
        ->Command
    {
        switch(currentCommand)
        {
        case Command::WAITING :
            return Command::ZONE_CHANGED;
        case Command::QUERY_UPDATE_ZONE :
            if(topLeft.first==topLeft2.first)
                return Command::RETURN;
            topLeft=topLeft2;
            bottomRight=bottomRight2;
            return Command::ZONE_CHANGED;
        case Command::GO_TO_PATROL_ZONE  :
        case Command::GO_TO_HOME         :
        case Command::TRANSITION_TO_POINT:
            if(currentPosition.first==begPoint.first && currentPosition.second==begPoint.second)
            {
                dron.setTask((currentPosition.first+0.5)*1000, (currentPosition.second+0.5)*1000, TASK::MOVE);
                modeling(dron, modelingTime, staticObstacles, dynamicObstacles,
                [&modelingTime](double x, double y, double course, TASK task)
                {
                    std::cout<<modelingTime<<":"<<x<<":"<<y<<":"<<course<<":"<<taskName(task)<<std::endl;
                });
            }
            dron.setTask((nextPosition.first+0.5)*1000, (nextPosition.second+0.5)*1000, TASK::MOVE);
            modeling(dron, modelingTime, staticObstacles, dynamicObstacles,
            [&modelingTime](double x, double y, double course, TASK task)
            {
                std::cout<<modelingTime<<":"<<x<<":"<<y<<":"<<course<<":"<<taskName(task)<<std::endl;
            });
            break;
        case Command::PATROL_ZONE        :
            dron.setTask((nextPosition.first+0.5)*1000, (nextPosition.second+0.5)*1000, TASK::SEARCH_TARGET);
            modeling(dron, modelingTime, staticObstacles, dynamicObstacles,
            [&modelingTime](double x, double y, double course, TASK task)
            {
                std::cout<<modelingTime<<":"<<x<<":"<<y<<":"<<course<<":"<<taskName(task)<<std::endl;
            });
            break;
        default : break;

        }

        fullPath.push_back(nextPosition);
        return Command::GO;
    });

//    print_cycle(fullPath.size(), 1, "\n full path: ",
//    [&](int index, int y, int x)->std::string
//    {
//        (void)x, (void)y;
//        const std::pair<int32_t, int32_t> &item=fullPath[index];
//        std::string str;

//        if(index && !(index%10)) str="\n";

//        str+=" ("+toStr(static_cast<int>(item.first))+
//                ", "+toStr(static_cast<int>(item.second))+
//                ") ";
//        return str;
//    });

    return 0;
}
