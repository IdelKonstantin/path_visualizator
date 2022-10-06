#ifndef CORE_H
#define CORE_H

#include <vector>
#include <string>
#include <inttypes.h>

#ifndef MAX
#  define MAX(a,b)      (((a)>(b))  ?   (a)     :   (b))
#endif
#ifndef MIN
#  define MIN(a,b)      (((a)<(b))  ?   (a)     :   (b))
#endif

#define ABS(a)          (((a)<0)    ?   (-(a))  :   (a))
#define SIGN_2S(a)      (((a)<0)    ?   (-1)    :   1)              // знак (-1, +1)
#define ROUND(a)        (int64_t((a)+0.5*SIGN_2S(a)))				// округление к целому
#define FLOOR(a)        (int64_t(a)+((a)<int64_t(a) ? -1 : 0))      // БМЦ
#define CEILING(a)      (-FLOOR(-(a)))								// ББЦ

//#define M_PI        3.1415926535897932384626433832795
#define M_PIm2		6.283185307179586476925286766559				// 2*Pi
#define M_PId2		1.5707963267948966192313216916398				// Pi/2
#define M_PId4		0.78539816339744830961566084581988				// Pi/4
#define M_PId180	0.017453292519943295769236907684886				// Pi/180
#define M_180dPI	57.295779513082320876798154814105				// 180/Pi

#define GTOR(a)		((a)*M_PId180)
#define RTOG(a)		((a)*M_180dPI)

// операции с углами
#define CORNER_in_2pi(corner)   ((corner)-static_cast<long long>((corner)/M_PIm2)*M_PIm2)
#define CORNER_in_360(corner)   ((corner)-static_cast<long long>((corner)/360.0)*360)

#define CORNER_0_2pi(corner)    (CORNER_in_2pi(corner)+((corner)<0 ? M_PIm2 : 0))
#define CORNER_0_360(corner)    (CORNER_in_360(corner)+((corner)<0 ? 360    : 0))

#define PART_0_2pi(corner1, corner2, part) CORNER_0_2pi(corner1+ARC_in_2pi(corner1, corner2)*part)
#define PART_0_360(corner1, corner2, part) CORNER_0_360(corner1+ARC_in_360(corner1, corner2)*part)

#define BISECTOR_0_2pi(corner1, corner2) PART_0_2pi(corner1, corner2, 0.5)
#define BISECTOR_0_360(corner1, corner2) PART_0_360(corner1, corner2, 0.5)

inline static double ARC_in_2pi(double corner1, double corner2)
{
    corner1=CORNER_in_2pi(corner1);
    corner2=CORNER_in_2pi(corner2);
    double sub=corner2-corner1;
    double sub_c=CORNER_0_2pi(sub+M_PIm2);
    if(sub_c<1e-8)
        sub_c=ABS(sub)>1e-8 ? M_PIm2 : 0;
    return sub_c;
}

inline static double ARC_in_360(double corner1, double corner2)
{
    corner1=CORNER_in_360(corner1);
    corner2=CORNER_in_360(corner2);
    double sub=corner2-corner1;
    double sub_c=CORNER_0_360(sub+360);
    if(sub_c<1e-8)
        sub_c=ABS(sub)>1e-8 ? 360 : 0;
    return sub_c;
}

enum class MAPTYPES : int
{
    UNKNOWN, INTERMEDIATE, FIRBIDDEEN
};

struct RECT
{
    double xMin, yMin, xMax, yMax;
};

typedef std::vector<std::pair<double, double>> POLYGON;
typedef std::vector<RECT> CEIL;
typedef std::vector<std::vector<MAPTYPES>> LOCALMAP;

struct Obstacle
{
    POLYGON polygon;
    RECT boundingRect;
};

typedef std::vector<Obstacle> OBSTACLES;

bool checkPointInStaticBlockages(int32_t x, int32_t y, const CEIL &staticBlockages);
bool checkPointInDynamicBlockages(double x, double y, const OBSTACLES &dynamicObstacles);
bool checkPointInPolygon(double x, double y, const POLYGON &polygon);
void generateBlockages(const int32_t szX, const int32_t szY, OBSTACLES &dynamicObstacles);

MAPTYPES typePoint(double x0, double y0, double azimuth, int32_t r,
                   const CEIL &staticBlockages, const OBSTACLES &dynamicObstacles);
//карта с шагом 1м и 1гр, в полярной СК (азимут, дальность)->тип точки
// азимут az0 - угол между OX и направлением на некоторую точку (отсчет - CW)
//void createMapFromBlocages(double x0, double y0, int32_t course, int32_t dr,
//                           int32_t da, int32_t widthSector, double lidarDistance,
//                           const CEIL &staticBlockages, const BLOCAGES &dynamicBlockages,
//                           LOCALMAP &map);
//LOCALMAP createMapFromBlocages(double x0, double y0, double az0, double lidarDistance, const BLOCAGES &blockages);

//double minimumDistanceToPolygonBorder(double x0, double y0, double x, double y,
//                                      const std::vector<std::pair<double, double>> &polygon);


#define TASK_LIST                           \
    /* разделитель группы состояние */      \
    DECL_TASK(BEGIN_STATE_GROUP)            \
    /*  */                                  \
    DECL_TASK(NOT_SET)                      \
    DECL_TASK(GO_TO_PATROL_ZONE)            \
    DECL_TASK(GO_TO_HOME)                   \
    DECL_TASK(TRANSITION_TO_POINT)          \
    DECL_TASK(PATROL_ZONE)                  \
    DECL_TASK(CHECKPOINT_REACHED)           \
    DECL_TASK(CHECKPOINT_NOT_AVAILABLE)     \
    DECL_TASK(CHECKPOINT_REACHED_PENDING)   \
    /*  */                                  \
    DECL_TASK(END_STATE_GROUP)              \
    /*  */                                  \
    /* разделитель группы движение*/        \
    DECL_TASK(BEGIN_MOVE_GROUP)             \
    /*  */                                  \
    DECL_TASK(MOVE)                         \
    DECL_TASK(SEARCH_TARGET)                  \
    DECL_TASK(BYPASSING_OBSTACLE)           \
    /*  */                                  \
    DECL_TASK(END_MOVE_GROUP)               \
    /*  */                                  \
    /* разделитель группы комманды*/        \
    DECL_TASK(BEGIN_COMMAND_GROUP)          \
    /**/                                    \
    DECL_TASK(RETURN)                       \
    DECL_TASK(GO_TO_POINT)                  \
    DECL_TASK(SET_PATROL_ZONE)              \
    DECL_TASK(ABORT) /* прерывание расчета (для отладки)*/ \
    DECL_TASK(WAITING)                      \
    /*  */                                  \
    DECL_TASK(END_COMMAND_GROUP)            \
    /*  */                                  \
    /* разделитель группы запросы */        \
    DECL_TASK(BEGIN_QUERY_GROUP)            \
    /**/                                    \
    DECL_TASK(QUERY_UPDATE_ZONE)            \
    /*  */                                  \
    DECL_TASK(END_QUERY_GROUP)              \
    /*  */                                  \

enum class TASK : int
{
#define DECL_TASK(t)  t,
TASK_LIST
#undef DECL_TASK
};

static std::string taskName(TASK task)
{
    switch(task)
    {
#define DECL_TASK(t)  case TASK::t : return #t;
TASK_LIST
#undef DECL_TASK
    }
    return {};
}

#endif // CORE_H
