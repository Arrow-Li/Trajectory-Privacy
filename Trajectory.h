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
    unsigned long length;
    // vector<Coord> cod;
   public:
    std::vector<Coord> cod;
    Trajectory();
    Trajectory(std::string, std::vector<Coord>);
    Trajectory(const Trajectory &p) {
        id = p.id;
        length = p.length;
        cod = p.cod;
    }
    const Trajectory &operator=(const Trajectory &);
    void clear();
    std::string getId();
    unsigned long getLength();
    void areaTrack(double &, double &, double &, double &);
    friend void syncTrajectory(Trajectory &, std::set<double> &);
    friend void insert_cod(Trajectory &, int, double, bool);
    friend std::vector<Trajectory> Equal_tracks(std::vector<Trajectory> &,
                                                double);
    friend double distance_track(Trajectory, Trajectory);
    friend double cos_angle(Trajectory, Trajectory);
    friend bool slcover(Trajectory, Trajectory, int, double, double &);
};

Trajectory::Trajectory() {
    id = "";
    length = 0;
}

Trajectory::Trajectory(std::string id, std::vector<Coord> cod) {
    this->id = id;
    this->length = cod.size();
    this->cod = cod;
}

const Trajectory &Trajectory::operator=(const Trajectory &other) {
    if (this != &other) {
        if (this->length != 0) this->clear();
        if (other.length != 0) {
            this->id = other.id;
            this->length = other.length;
            this->cod = other.cod;
        }
    }
    return *this;
}

std::string Trajectory::getId() { return id; }

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

unsigned long Trajectory::getLength() { return this->length; }