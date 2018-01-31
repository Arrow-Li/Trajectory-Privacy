#include "Trajectory.h"

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
    double countArea();
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
    double count=0;
    for(const auto& ar:this->area)
        count+=(ar.xMax-ar.xMin)*(ar.yMax-ar.yMin);
    return count;
}
