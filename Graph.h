#include <math.h>
#include <memory.h>
#include <algorithm>
#include "Matrix.h"
#include "Trajectory.h"

class Graph {
    TrajectorySet V;
    Matrix WE;
    void visit(int, bool *, std::vector<int> &);

   public:
    Graph(int size) : WE(size){};
    Graph(TrajectorySet &t);
    Graph(const Graph &G) : WE(G.WE) { V = G.V; }
    int countV();
    void insertV(Trajectory);
    void insertE(int, int, double);
    double weight(int, int);
    void show();  // TODO temp
    double minE(Trajectory &, Trajectory &);
    int find(std::string);
    void deleteV(std::string);
    double compare(Graph G1, Graph G2, std::string &, std::string &);
    std::string refind(int);
    std::vector<std::string> linkV(std::string);
    Trajectory getV(int);
    TrajectorySet &getT();
    int next(int, int);
    Graph create_child(std::vector<int>);
    std::vector<Graph> DFS(int);
    void count_area(int, double &, double &, double &, double &);
    friend void merge_graph(Graph &, Graph &, std::string, std::string, double);
    friend Graph createTG(TrajectorySet &, double, double, double, double);
};

Graph::Graph(TrajectorySet &t) : WE(t.size()) { V = t; }

void Graph::show() { WE.show(); }

int Graph::countV() { return V.size(); }

void Graph::insertV(Trajectory t) { V.push_back(t); }

void Graph::insertE(int i, int j, double w) { WE.setValue(i, j, w); }

double Graph::weight(int i, int j) { return WE.getValue(i, j); }

int Graph::find(std::string id) {
    int i;
    for (i = 0; i < V.size(); i++) {
        if (V[i].getId() == id) break;
    }
    if (i == V.size()) return -1;
    return i;
}

std::string Graph::refind(int x) { return V[x].getId(); }

Trajectory Graph::getV(int x) { return V[x]; }

TrajectorySet &Graph::getT() { return this->V; }

std::vector<std::string> Graph::linkV(std::string a) {
    std::vector<std::string> v;
    int x = this->find(a);
    for (int i = 0; i < V.size(); ++i) {
        if (i == x) continue;
        if (WE.getValue(x, i) != INF) v.push_back(this->refind(i));
    }
    return v;
}

void Graph::deleteV(std::string x) {
    if (this->find(x) == -1) return;
    WE.del(this->find(x));
    V.erase(V.begin() + this->find(x));
}

double Graph::minE(Trajectory &v1, Trajectory &v2) {
    double min = INF;
    int x = 0, y = 0;

    for (int i = 0; i < V.size(); ++i) {
        for (int j = i + 1; j < V.size(); ++j) {
            if (WE.getValue(i, j) < min && WE.getValue(i, j) > 0) {
                min = WE.getValue(i, j);
                x = i;
                y = j;
            }
        }
    }
    if (x != y) {
        v1 = V[x];
        v2 = V[y];
    }
    return min;
}

Graph Graph::create_child(std::vector<int> id) {
    TrajectorySet T;
    for (int i = 0; i < id.size(); ++i) T.push_back(V[id[i]]);
    Graph G(T);
    for (int i = 0; i < id.size(); ++i) {
        for (int j = 0; j < i + 1; ++j) {
            G.WE.setValue(i, j, WE.getValue(id[i], id[j]));
        }
    }
    return G;
}

std::vector<Graph> Graph::DFS(int vi) {
    int n = this->countV(), i = vi;
    bool visited[n];
    std::vector<int> child;
    std::vector<Graph> G;
    memset(visited, false, sizeof(visited));
    do {
        if (!visited[i]) {
            visit(i, visited, child);
            sort(child.begin(), child.end());
            G.push_back(create_child(child));
            child.clear();
        }
        i = (i + 1) % n;
    } while (i != vi);
    return G;
}

int Graph::next(int i, int j = -1) {
    int n = this->countV();
    if (i >= 0 && i < n && j >= -1 && j < n && i != j) {
        for (int k = j + 1; k < n; ++k) {
            if (weight(i, k) > 0 && weight(i, k) < INF) return k;
        }
    }
    return -1;
}

void Graph::visit(int vi, bool *visited, std::vector<int> &child) {
    visited[vi] = true;
    child.push_back(vi);
    for (int i = next(vi); i != -1; i = next(vi, i)) {
        if (!visited[i]) {
            visit(i, visited, child);
        }
    }
}

double Graph::compare(Graph G1, Graph G2, std::string &id,
                      std::string &id_connect) {
    double min_w = INF;
    for (int i = 0; i < G1.countV(); ++i) {
        for (int j = 0; j < G2.countV(); ++j) {
            double temp_w =
                this->weight(find(G1.V[i].getId()), find(G2.V[j].getId()));
            if (temp_w != INF /*&&temp_w!=0*/) {
                if (temp_w < min_w) {
                    min_w = temp_w;
                    id = G1.V[i].getId();
                    id_connect = G2.V[j].getId();
                }
            }
        }
    }
    return min_w;
}

void Graph::count_area(int t, double &x_max, double &x_min, double &y_max,
                       double &y_min) {
    // TODO 计算匿名域面积
    for (int j = 0; j < this->countV(); ++j) {
        t %= this->V[j].getLength();
        x_min = (this->V[j].getCoord(t).x < x_min) ? this->V[j].getCoord(t).x : x_min;
        x_max = (this->V[j].getCoord(t).x > x_max) ? this->V[j].getCoord(t).x : x_max;
        y_min = (this->V[j].getCoord(t).y < y_min) ? this->V[j].getCoord(t).y : y_min;
        y_max = (this->V[j].getCoord(t).y > y_max) ? this->V[j].getCoord(t).y : y_max;
    }
    return;
}

/*
bool pcmp(const double &d1, const double &d2){
    if(d1<d2)
        return true;
    return false;
}

Coord fuse(std::vector<Coord>& point){
    double x[4]={a.x,b.x,c.x,d.x},y[4]={a.y,b.y,c.y,d.y};
    qsort(x,4, sizeof(x[0]),pcmp);
    qsort(y,4, sizeof(y[0]),pcmp);
    Coord result;
    result.x=(x[0]+x[y])/2;
    result.y=(y[0]+y[y])/2;
    result.t=(a.t+b.t+c.t+d.t)/4;
    return result;
}
 */