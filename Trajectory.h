#include <set>
#include <string>
#include <vector>

struct Coord {
    double x, y, t;
};

struct Vaw {
    std::string id, id_connect;
    double weight;
};

class Trajectory {
    std::string id;
    unsigned int length;
    std::vector<Coord> cod;
   public:
    Trajectory();
    Trajectory(const Trajectory &p) {
        id = p.id;
        length = p.length;
        cod = p.cod;
    }
    Trajectory(std::string, std::vector<Coord>);
    const Trajectory &operator=(const Trajectory &);
    void clear();
    std::string getId();
    Coord getCoord(int);
    unsigned int getLength();
    void insertNode(unsigned  int, double);
    void syncTrajectory(std::set<double> &);
    void areaTrack(double &, double &, double &, double &);
    friend void write_data(std::vector<Trajectory> &); //TODO temp
    friend std::vector<Trajectory> EqualTrack(std::vector<Trajectory> &,
                                              double);
    friend double getTrackDis(Trajectory, Trajectory);
    friend double getTrackCos(Trajectory, Trajectory);
    friend bool slcover(Trajectory, Trajectory, int, double, double &);
};

Trajectory::Trajectory() {
    id = "";
    length = 0;
}

Trajectory::Trajectory(std::string id, std::vector<Coord> cod) {
    this->id = id;
    this->length = (unsigned int)cod.size();
    this->cod = cod;
}

const Trajectory &Trajectory::operator=(const Trajectory &other) {
    if (this != &other) {
        if (this->length != 0) this->clear();
        if (other.length != 0) {
            this->id = other.id;
            this->length = other.length;
            this->cod.assign(other.cod.begin(),other.cod.end());
        }
    }
    return *this;
}

std::string Trajectory::getId() { return id; }

Coord Trajectory::getCoord(int x) { return cod[x]; }

unsigned int Trajectory::getLength() { return this->length; }

void Trajectory::clear() {
    id = "";
    length = 0;
    cod.clear();
}

void Trajectory::areaTrack(double &xmin, double &xmax, double &ymin,
                           double &ymax) {
    xmin = cod[0].x;
    xmax = cod[0].x;
    ymin = cod[0].y;
    ymax = cod[0].y;
    for (int i = 0; i < length; ++i) {
        if (cod[i].x <= xmin) xmin = cod[i].x;
        if (cod[i].x >= xmax) xmax = cod[i].x;
        if (cod[i].y <= ymin) ymin = cod[i].y;
        if (cod[i].y >= ymax) ymax = cod[i].y;
    }
}

void Trajectory::insertNode(unsigned int i, double t) {
    double newX,newY,ratio;
    Coord newCoord;
    if(i==0){ //插头结点
        ratio=(t-cod[i].t)/(cod[i+1].t-cod[i].t);
        newX=cod[i].x+ratio*(cod[i+1].x-cod[i].x);
        newY=cod[i].y+ratio*(cod[i+1].y-cod[i].y);
        newCoord={newX,newY,t};
        cod.insert(cod.begin(),newCoord);
    }
    else {
        if(i>=length) {//插尾部
            ratio=(t-cod[i-2].t)/(cod[i-1].t-cod[i-2].t);
            newX=cod[i-2].x+ratio*(cod[i-1].x-cod[i-2].x);
            newY=cod[i-2].y+ratio*(cod[i-1].y-cod[i-2].y);
            newCoord={newX,newY,t};
        }
        else {
            ratio=(t-cod[i-1].t)/(cod[i].t-cod[i-1].t);
            newX=cod[i-1].x+ratio*(cod[i].x-cod[i-1].x);
            newY=cod[i-1].y+ratio*(cod[i].y-cod[i-1].y);
            newCoord={newX,newY,t};
        }
        cod.insert(cod.begin()+i,newCoord);
    }
    length++;
}

void Trajectory::syncTrajectory(std::set<double>& timeLine) { //TODO exit code -1073741819 (0xC0000005)
    unsigned int i=0;
    for(auto t:timeLine){
        if(cod[i].t!=t)
            insertNode(i,t);
        i++;
    }
}