#include <ros/ros.h>
#include <mission_handler_2021/IsPointInPolygon.h>

bool isPointInPolygon(double x, double y, const geometry_msgs::Polygon& poly){
    struct Pos{
        double x, y;
    };

    static auto calc_triangle_area = [](const Pos& f,const Pos& s,const Pos& t)->double { //first, second, third
        double area = fabs((f.x*(s.y-t.y) + s.x*(t.y-f.y) + t.x*(f.y-s.y))/2);
        return area;
    };
    static double EPSILON = 0.1;

    if (!poly.points.size()) return false;

    Pos p{x,y};
    const int N_POINT = (int)poly.points.size();
    double sum_of_poly = 0;
    double sum_of_poly_with_point = 0;

    //calc sum of poly
    Pos t_anchor{poly.points.front().x, poly.points.front().y};
    for(int i = 1; i < N_POINT - 1; ++i){
        Pos t1{poly.points[i].x, poly.points[i].y};
        Pos t2{poly.points[i+1].x, poly.points[i+1].y};
        sum_of_poly += 
            calc_triangle_area(t_anchor, t1, t2);            
    }

    //calc sum of poly with point
    for(int i = 0 ; i < N_POINT; ++i){
        Pos t1{poly.points[i % N_POINT].x, poly.points[i % N_POINT].y};
        Pos t2{poly.points[(i + 1) % N_POINT].x, poly.points[(i + 1) % N_POINT].y};
        sum_of_poly_with_point += 
            calc_triangle_area(p, t1, t2);
    }    

    if (fabs(sum_of_poly - sum_of_poly_with_point) <= EPSILON) return true; //the point is in first triangle
    else return false;
}