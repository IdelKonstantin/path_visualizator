#include <math.h>
#include <stdio.h>
#include "dron.h"

Dron::Dron(double xPos, double yPos)
    : turningRadius(CEILING(std::hypot(dronLength, dronWidth)/2.)+1.)
    , xPos(xPos)
    , yPos(yPos)
{
    taskReset();
}

void Dron::setTask(double x, double y, TASK task)
{
    taskReset();

    if(task>TASK::BEGIN_MOVE_GROUP && task<TASK::END_MOVE_GROUP)
    {
        xEndPos=x;
        yEndPos=y;

        currentAzimuth=azimuth(currenLength);
        course=GTOR(ROUND(RTOG(CORNER_0_2pi(currentAzimuth+M_PI))));
        taskStack.push_back(currenLength<velForw // за 1 сек
                            ? TASK::CHECKPOINT_REACHED_PENDING : task);
    }
}

void Dron::run(double modelingTime, const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles)
{
    double dt=modelingTime-lastTime;
    double nc, ds, arc=0, da;
    TASK curreentTask=taskStack.back();

    double courseCurr=RTOG(course), x=xPos, y=yPos;
//    double ccc3, ccc1, ccc2, ccc4, ccc5, ccc6;
    // modelingTime>=578

    lastTime=modelingTime;

    switch(curreentTask)
    {
    case TASK::CHECKPOINT_REACHED_PENDING :
        taskStack.back()=TASK::CHECKPOINT_REACHED;
        break;
    case TASK::CHECKPOINT_NOT_AVAILABLE :
    case TASK::CHECKPOINT_REACHED       :
        taskReset();
        break;
    case TASK::NOT_SET                  : break;
    case TASK::MOVE                     :
    case TASK::SEARCH_TARGET              :
    case TASK::BYPASSING_OBSTACLE       :
//        ccc=RTOG(course);

        ds=dt*velForw;
        ds=MIN(ds, currenLength);
        if(ds>1e-4)
        {
            xPos+=ROUND(ds*cos(course)); // cos(M_PIm2-course) ->  cos(course)
            yPos+=-ROUND(ds*sin(course));// sin(M_PIm2-course) -> -sin(course)
            pathLengthLocal+=ds;
            pathLengthSummary+=ds;
        }
        if((curreentTask==TASK::BYPASSING_OBSTACLE && currenLength<dispersionMax) ||
           ABS(currenLength-ds)<1e-1)
        {
            taskReset();
            taskStack.push_back(TASK::CHECKPOINT_REACHED);
            break;
        }

        double anew;

        anew=azimuth(currenLength);

//        ccc1=RTOG(course);
//        ccc2=RTOG(currentAzimuth);
//        ccc3=RTOG(anew);

        arc=ARC_in_2pi(currentAzimuth, anew);
        if(arc>M_PI) arc=-(M_PIm2-arc);
        currentAzimuth=anew;

//        ccc4=RTOG(arc);

        maneuverAccumulation+= arc;
        da=ABS(maneuverAccumulation)-M_PIm2;
        if(da>=0) // проверка на петлю // modelingTime>=577
        {
            taskReset();
            taskStack.push_back(TASK::CHECKPOINT_NOT_AVAILABLE);
        }

//        nc=curreentTask==TASK::BYPASSING_OBSTACLE
//                ? course-bypassDirection*M_PId4
//                : M_PI+currentAzimuth;
        nc=M_PI+currentAzimuth;
        nc=CORNER_0_2pi(nc);

//        ccc5=RTOG(nc);

        analysisCourse(nc, currenLength, staticBlockages, dynamicObstacles);
        course=GTOR(ROUND(RTOG(nc)));

//        ccc6=RTOG(course);




        break;
    default : break;
    }


//if(modelingTime>=100)
//{
//    FILE *f;
//    f=fopen("position.txt", "at");
//    fprintf(f,"%7.1lf"\
//              "\t%9.3lf\t%9.3lf"\
//              "\t%8.4lf"\
//              "\t%9.3lf\t%9.3lf"\
//              "\t%24s\t%9.3lf\t%9.3lf\t%7.2lf\t%7.2lf\n",
//            modelingTime, x, y, courseCurr, xPos, yPos, taskName(curreentTask).data(),
//            xEndPos, yEndPos, RTOG(arc), RTOG(maneuverAccumulation));
//    fclose(f);
//}
}

bool Dron::analysisCourse(double &nc, double dl,
                          const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles)
{
    LOCALMAP localMap;

    auto half_sectorG=[this](double distance)->int32_t
    {
        return CEILING(RTOG(asin(turningRadius/(turningRadius+distance))));
    };

    static const int32_t sector_max=2*half_sectorG(0)+1;

    auto analysisOnCourse=[dl, nc, &half_sectorG, &localMap,
                           &staticBlockages, &dynamicObstacles, this](int64_t da)->int32_t
    {
        int64_t szA, szR=MIN(CEILING(lidarDistance), CEILING(dl));
        int index0, index;
        int64_t half_wa, minA, maxA;
        int ncG;

        szA=localMap.size();
        if(!szA)
            return -1;

        index0=szA/2;
        index=index0+da;

        if(index<0 || index>=szA)
            return -1;

        ncG=ROUND(RTOG(nc))-index0;
        for(int64_t r=0; r<szR; r++)
        {
            half_wa=half_sectorG(r);
            minA=index-half_wa;
            maxA=index+half_wa;
            if(minA<0 || maxA>=szA)
                return -1;

            for(int64_t a=minA; a<=maxA; a++) // r>=17
            {
                if(!localMap[a].size())
                    localMap[a].assign(szR, MAPTYPES::UNKNOWN);
                if(localMap[a][r]==MAPTYPES::UNKNOWN)
                    localMap[a][r]=typePoint(xPos, yPos, GTOR(ncG+a), turningRadius+r,
                                             staticBlockages, dynamicObstacles);
                if(localMap[a][r]==MAPTYPES::FIRBIDDEEN)
                {
                    for(; r<szR; r++)
                        localMap[a][r]=MAPTYPES::FIRBIDDEEN;
                    return 0;
                }
            }
        }
        return 1;
    };

    auto findFreeDirection=[analysisOnCourse, &localMap](int32_t direction)->int64_t
    {
        int64_t a, ret=-1;
        int64_t szMap=localMap.size(), half_szA=szMap/2;

        for(int64_t index=1; index<=half_szA; index++)
        {
            a=index*direction; // index>=95
            if(half_szA+a<0 || half_szA+a>=szMap)
                return -1;

            ret=analysisOnCourse(a);
            if(ret==1)
                break;
        }
        return ret==1 ? a*direction : -1;
    };

    int64_t a=-1;

    localMap.resize(359+2*sector_max); // -179-amax .. 0 .. 179+amax, 0 соотв. amax+180 или size()/2

    // проверяем наличие прохода
    if(/*taskStack.back()!=TASK::BYPASSING_OBSTACLE && */analysisOnCourse(0)==1)
    {
        maneuverDataReset();
        return true;
    }

    maneuverDataSet();
    a=findFreeDirection(bypassDirection);
    if(a==-1 && !directionHasChanged) // допускается только одна смена направления обхода
    {
        directionHasChanged=true;
        bypassDirection*=-1; // смена направления, если тупик
        a=findFreeDirection(bypassDirection);
    }

    if(a<0)
    {
        maneuverDataReset();
        taskStack.push_back(TASK::CHECKPOINT_NOT_AVAILABLE);
    }

    if(a>0) nc=CORNER_0_2pi(nc+GTOR(a));
    return a>0;
}

void Dron::maneuverDataReset()
{
    if(taskStack.back()==TASK::BYPASSING_OBSTACLE)
        taskStack.pop_back();
    directionHasChanged=false;
    bypassDirection=0;
}

void Dron::maneuverDataSet()
{
    if(taskStack.back()!=TASK::BYPASSING_OBSTACLE)
    {
        taskStack.push_back(TASK::BYPASSING_OBSTACLE);
        directionHasChanged=false;
        bypassDirection=2*(rand()%2)-1; // выбираем направление - рандомно
    }
}

void Dron::taskReset()
{
    pathLengthLocal=0;
    currenLength=0;
    currentAzimuth=0;
    maneuverAccumulation=0;
    maneuverDataReset();
    taskStack.clear();
    taskStack.push_back(TASK::NOT_SET);
}

double Dron::azimuth(double &ds)
{
    double dx=xPos-xEndPos, dy=yPos-yEndPos, a;

    ds=std::hypot(dx, dy),
    a=acos(dx/ds);
    if(dy>0)
        a=M_PIm2-a;
    return a;
}
