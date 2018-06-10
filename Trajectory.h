#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include <set>
#include <string>
#include <vector>
#define TrajectorySet std::vector<Trajectory>

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
    void insertNode(unsigned int, double, Coord *);
    void generateNode(std::vector<Coord> *, Coord *, Coord *, unsigned int &);
    void syncTrajectory(unsigned int &, unsigned int &, int);
    void areaTrack(double &, double &, double &, double &);
    friend std::vector<Trajectory> EqualTrack(std::vector<Trajectory> &);
    friend double getTrackDis(Trajectory &, Trajectory &);
    friend double getTrackCos(Trajectory &, Trajectory &);
    friend bool slCover(Trajectory &, Trajectory &, int, double, double &);
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
            this->cod.assign(other.cod.begin(), other.cod.end());
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
    for (const auto &coord : cod) {
        if (coord.x <= xmin) xmin = coord.x;
        if (coord.x >= xmax) xmax = coord.x;
        if (coord.y <= ymin) ymin = coord.y;
        if (coord.y >= ymax) ymax = coord.y;
    }
}

void Trajectory::insertNode(unsigned int i, double t, Coord *c = NULL) {
    double newX, newY, ratio;
    Coord newCoord;
    if (i == 0) {  //插头结点
        ratio = (t - c->t) / (cod[i].t - c->t);
        newX = c->x + ratio * (cod[i].x - c->x);
        newY = c->y + ratio * (cod[i].y - c->y);
        newCoord = {newX, newY, t};
        cod.insert(cod.begin() + i, newCoord);
    } else {
        if (i >= length) {  //插尾部
            ratio = (t - cod[i - 2].t) / (cod[i - 1].t - cod[i - 2].t);
            newX = cod[i - 2].x + ratio * (cod[i - 1].x - cod[i - 2].x);
            newY = cod[i - 2].y + ratio * (cod[i - 1].y - cod[i - 2].y);
            newCoord = {newX, newY, t};
            cod.push_back(newCoord);
        } else {
            ratio = (t - cod[i - 1].t) / (cod[i].t - cod[i - 1].t);
            newX = cod[i - 1].x + ratio * (cod[i].x - cod[i - 1].x);
            newY = cod[i - 1].y + ratio * (cod[i].y - cod[i - 1].y);
            newCoord = {newX, newY, t};
            cod.insert(cod.begin() + i, newCoord);
        }
    }
    length++;
}

void Trajectory::generateNode(std::vector<Coord> *newCod, Coord *front, Coord *back, unsigned int &t) {
    double newX, newY, ratio;
    Coord newCoord;
    if (t < front->t) {  // 两点之前
        ratio = (front->t - t) / (back->t - t);
        newX = (front->x - ratio * back->x) / (1 - ratio);
        newY = (front->y - ratio * back->y) / (1 - ratio);
    }
    if (t > back->t) {  // 两点之后
        ratio = (back->t - front->t) / (t - front->t);
        newX = (back->x + (ratio - 1) * front->x) / ratio;
        newY = (back->y + (ratio - 1) * front->y) / ratio;
    } else {  // 两点之间
        ratio = (t - front->t) / (back->t - front->t);
        newX = front->x + ratio * (back->x - front->x);

        newY = front->y + ratio * (back->y - front->y);
    }
    newCoord = {newX, newY, (double)t};
    newCod->push_back(newCoord);
}

void Trajectory::syncTrajectory(unsigned int &startT, unsigned int &endT, int gap) {
    std::vector<Coord> newCod;
    unsigned int i = 0;

    for (unsigned int timeLine = startT; timeLine <= endT; timeLine += gap) {
        while (i < this->length) {
            if (timeLine < cod[i].t) break;
            i++;
        }
        if (i == 0)
            generateNode(&newCod, &cod[0], &cod[1], timeLine);
        else {
            if (i == this->length)
                generateNode(&newCod, &cod[i - 2], &cod[i - 1], timeLine);
            else
                generateNode(&newCod, &cod[i - 1], &cod[i], timeLine);
        }
    }
    newCod.swap(this->cod);
    std::vector<Coord>().swap(newCod);
    this->length = (unsigned int)cod.size();
}

#endif  //_TRAJECTORY_H