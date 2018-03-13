#include "Trajectory.h"
#define INF 0x3f3f3f3f

struct Area { // 节点的匿名域
    double xMin, xMax, yMin, yMax, t;
};

class AnonyArea {
    std::string *id;
    unsigned int size;
    unsigned int length;
    std::vector<Area> area;

   public:
    AnonyArea(int);
    AnonyArea(TrajectorySet);
    Area getArea(int);
    int getLength();
    double countArea();
    std::string* getID();
};

AnonyArea::AnonyArea(int k) {
    id = new std::string[k];
    size = k;
    length = 0;
}

AnonyArea::AnonyArea(std::vector<Trajectory> T) {
    size = T.size();
    id = new std::string[size];
    length = T[0].getLength();
    for (int i = 0; i < size; i++) id[i] = T[i].getId();
    for (int i = 0; i < length; i++) {
        double xMax, xMin, yMax, yMin;
        Coord flag = T[0].getCoord(i);
        xMax = flag.x;
        xMin = xMax;
        yMax = flag.y;
        yMin = yMax;
        for (auto track : T) {
            Coord cod = track.getCoord(i);
            xMax = (cod.x > xMax) ? cod.x : xMax;
            xMin = (cod.x < xMin) ? cod.x : xMin;
            yMax = (cod.y > yMax) ? cod.y : yMax;
            yMin = (cod.y < yMin) ? cod.y : yMin;
        }
        Area newArea = {xMin, xMax, yMin, yMax, flag.t};
        area.push_back(newArea);
    }
}

double AnonyArea::countArea() { //TODO 考虑匿名域重叠!
    double count=0,x1=-INF,x2=INF,y1=-INF,y2=INF;
    for(const auto& ar:this->area) {
        double temp=(ar.xMax - ar.xMin) * (ar.yMax - ar.yMin);
        count += temp;
    }
    return count;
}

Area AnonyArea::getArea(int i) {
    return area[i];
}

int AnonyArea::getLength() {
    return length;
}

std::string *AnonyArea::getID() {
    return id;
}
