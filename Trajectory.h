#include <set>
#include <vector>
#include <string>

using namespace std;

struct Coord
{
    double x, y, t;
};

struct Vaw
{
    string id, id_connect;
    double weight;
};

class Trajectory
{
    string id;
    unsigned long length;
    //vector<Coord> cod;
public:
    vector<Coord> cod;
    Trajectory();
    Trajectory(string, vector<Coord>);
    Trajectory(const Trajectory &p) {
        id = p.id;
        length = p.length;
        cod = p.cod;
    }
    string getid();
    void clear();
    const Trajectory& operator=(const Trajectory&);
    void area_track(double&, double&, double&, double&);
    unsigned long length_Tra();
    friend void sync_track(Trajectory &, set<double> &);
    friend void insert_cod(Trajectory &, int, double, bool);
    friend vector<Trajectory> Equal_tracks(vector<Trajectory> &, double);
    friend double distance_track(Trajectory, Trajectory);
    friend double cos_angle(Trajectory, Trajectory);
    friend bool slcover(Trajectory, Trajectory, int, double, double&);
};

unsigned long Trajectory::length_Tra() {
    return this->length;
}