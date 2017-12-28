#include "iostream"
#include "Graph.h"
bool same_element(string, vector<Trajectory>&);
void merge_graph(Graph&, Graph&, string, string, double);
bool scmp(const Vaw &,const Vaw &);

Trajectory::Trajectory() {
    id = "";
    length = 0;
}

Trajectory::Trajectory(string id, vector<Coord> cod) {
    this->id = id;
    this->length = cod.size();
    this->cod = cod;
}

const Trajectory& Trajectory::operator=(const Trajectory& other) {
    if (this != &other) {
        if (this->length != 0)
            this->clear();
        if (other.length != 0) {
            this->id = other.id;
            this->length = other.length;
            this->cod = other.cod;
        }
    }
    return *this;
}

string Trajectory::getid() {
    return id;
}

void Trajectory::clear() {
    id = ""; length = 0;
    cod.clear();
}

void Trajectory::area_track(double& xmin, double& xmax, double& ymin, double& ymax) {
    xmin = cod[0].x; xmax = cod[0].x; ymin = cod[0].y; ymax = cod[0].y;
    for (int i = 0; i < length; ++i) {
        if (cod[i].x <= xmin)
            xmin = cod[i].x;
        if (cod[i].x >= xmax)
            xmax = cod[i].x;
        if (cod[i].y <= ymin)
            ymin = cod[i].y;
        if (cod[i].y >= ymax)
            ymax = cod[i].y;
    }
}

/*
void sync_track(Trajectory &p, Trajectory &q, bool flag = false) {
    for (int i = 0; i < q.length; ++i)
    {
        for (int j = 0; j < p.length; ++j)
        {
            if (q.cod[i].t == p.cod[j].t)
                break;
            if (q.cod[i].t < p.cod[j].t) {
                double x_new, y_new;
                x_new = p.cod[j].x + (p.cod[j + 1].x - p.cod[j].x) * (q.cod[i].t - p.cod[j].t) / (p.cod[j + 1].t - p.cod[j].t);
                y_new = p.cod[j].y + (p.cod[j + 1].y - p.cod[j].y) * (q.cod[i].t - p.cod[j].t) / (p.cod[j + 1].t - p.cod[j].t);
                Coord cod_new = {x_new, y_new, q.cod[i].t};
                p.cod.insert(p.cod.begin() + j, cod_new);
                p.length++;
                break;
            }
            if (j == p.length - 1 && (q.cod[i].t > p.cod[j].t)) {
                double x_new, y_new;
                x_new = p.cod[j].x + (p.cod[j].x - p.cod[j - 1].x) * (q.cod[i].t - p.cod[j].t) / (p.cod[j].t - p.cod[j - 1].t);
                y_new = p.cod[j].y + (p.cod[j].y - p.cod[j - 1].y) * (q.cod[i].t - p.cod[j].t) / (p.cod[j].t - p.cod[j - 1].t);
                Coord cod_new = {x_new, y_new, q.cod[i].t};
                p.cod.insert(p.cod.begin() + j + 1, cod_new);
                p.length++;
            }
        }
    }
    if (flag)
        return;
    sync_track(q, p, true);
}
*/

void insert_cod(Trajectory &p, int i, double t, bool special = false) {
    double x_new, y_new;
    if (special) {
        if (i == 0) {
            x_new = (p.cod[i].x * (p.cod[i + 1].t - t) - p.cod[i + 1].x * (p.cod[i].t - t)) / (p.cod[i + 1].t - p.cod[i].t);
            y_new = p.cod[i + 1].y + (p.cod[i + 1].t - t) * (p.cod[i].y - p.cod[i + 1].y) / (p.cod[i + 1].t - p.cod[i].t);
            Coord new_cod = {x_new, y_new, t};
            p.cod.insert(p.cod.begin(), new_cod);
        }
        else {
            x_new = p.cod[i - 1].x + (p.cod[i].x - p.cod[i - 1].x) * (t - p.cod[i - 1].t) / (p.cod[i].t - p.cod[i - 1].t);
            y_new = p.cod[i - 1].y + (p.cod[i].y - p.cod[i - 1].y) * (t - p.cod[i - 1].t) / (p.cod[i].t - p.cod[i - 1].t);
            Coord new_cod = {x_new, y_new, t};
            p.cod.insert(p.cod.begin() + i + 1, new_cod);
        }
    }
    else {
        x_new = p.cod[i].x + (p.cod[i + 1].x - p.cod[i].x) * (t - p.cod[i].t) / (p.cod[i + 1].t - p.cod[i].t);
        y_new = p.cod[i].y + (p.cod[i + 1].y - p.cod[i].y) * (t - p.cod[i].t) / (p.cod[i + 1].t - p.cod[i].t);
        Coord new_cod = {x_new, y_new, t};
        p.cod.insert(p.cod.begin() + i + 1, new_cod);
    }
    p.length++;
    return;
}

void sync_track(Trajectory &p, set<double> &time_line) {
    int i = 0;
    for (auto t : time_line) {
        if (t < p.cod[0].t) { //插入轨迹前
            insert_cod(p, 0, t, true);

            /*
            Coord new_cod={p.cod[0].x,p.cod[0].y,t};
            p.cod.insert(p.cod.begin(),new_cod);
            p.length++;
            */

            continue;
        }
        if (t > p.cod[p.length - 1].t) { //插入轨迹后
            insert_cod(p, p.length - 1, t, true);

            /*
            Coord new_cod={p.cod[p.length-1].x,p.cod[p.length-1].y,t};
            p.cod.insert(p.cod.begin()+p.length,new_cod);
            p.length++;
            */

            i++;
            continue;
        }
        while (i < p.length) {
            if (t == p.cod[i].t)
                break;
            if (t < p.cod[i + 1].t && t > p.cod[i].t) { //插入轨迹中
                insert_cod(p, i, t);
                i++;
                break;
            }
            i++;
        }
        if (t == p.cod[i].t)
            continue;
    }
}

vector<Trajectory> Equal_tracks(vector<Trajectory> &T, double tp) {
    vector<Trajectory> equ_T;
    double s_t1, s_t2, end_t1, end_t2;
    set<double> time_line;
    //tp*=T[0].cod[T[0].length-1].t-T[0].cod[0].t;
    s_t1 = T[0].cod[0].t-tp;
    s_t2 = T[0].cod[0].t+tp;
    end_t1 = T[0].cod[T[0].length-1].t-tp;
    end_t2 = T[0].cod[T[0].length-1].t+tp;
    vector<Trajectory>::iterator it=T.begin();
    while(it!=T.end()){
        int l=(*it).length-1;
        if (((*it).cod[0].t >= s_t1 && (*it).cod[0].t <= s_t2) && ((*it).cod[l].t >= end_t1 && (*it).cod[l].t <= end_t2)){
            equ_T.push_back(*it);
            for (int i = 0; i <= l; ++i)
                time_line.insert((*it).cod[i].t); //生成时间轴
            T.erase(it);
        }
        else
            it++;
    }
    for (int i = 0; i < equ_T.size(); ++i) {
        sync_track(equ_T[i], time_line);
    }
    return equ_T;
}

double cos_angle(Trajectory p, Trajectory q) {
    int ignore=1;
    double cos_value = 0;
    for (int i = 0; i < p.length - 1; ++i) {
        double tmp0 = (p.cod[i + 1].x - p.cod[i].x) * (q.cod[i + 1].x - q.cod[i].x);
        tmp0 += (p.cod[i + 1].y - p.cod[i].y) * (q.cod[i + 1].y - q.cod[i].y);
        double tmp1 = sqrt(pow((p.cod[i + 1].x - p.cod[i].x), 2) + pow((p.cod[i + 1].y - p.cod[i].y), 2));
        tmp1 *= sqrt(pow((q.cod[i + 1].x - q.cod[i].x), 2) + pow((q.cod[i + 1].y - q.cod[i].y), 2));
        if(tmp0==0||tmp1==0){ //忽略时间间隔内未移动的点
            ignore++;
            continue;
        }
        cos_value += fabs(tmp0 / tmp1); //cos要取绝对值？ 两个线段夹角在0到90度
    }
    cos_value /= p.length - ignore;
    return cos_value;
}

double distance_track(Trajectory p, Trajectory q) {
    double dis = 0;
    for (int i = 0; i < p.length; ++i) {
        double tmp = pow((p.cod[i].x - q.cod[i].x), 2) + pow((p.cod[i].y - q.cod[i].y), 2);
        dis += sqrt(tmp);
    }
    dis /= p.length;
    return dis;
}

Matrix distance_matrix(vector<Trajectory> &TEC, double &max, double &min) {
    int n = TEC.size();
    Matrix TDM(n, n);
    max = 0; min = INF;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            TDM.set(i, i, 0);
            double dis = distance_track(TEC[i], TEC[j]);
            TDM.set(i, j, dis);
            TDM.set(j, i, dis);
            if (TDM.weight(i, j) >= max) {
                max = TDM.weight(i, j);
            }
            if (TDM.weight(i, j) <= min && i != j) {
                min = TDM.weight(i, j);
            }
        }
    }
    return TDM;
}

bool slcover(Trajectory p, Trajectory q, int s, double lambda, double &cpq) {
    double xmax, xmin, ymax, ymin;
    int count = 0;
    cpq = cos_angle(p, q);
    if (cpq < cos(lambda) || cpq > 1)
        return false;
    q.area_track(xmin, xmax, ymin, ymax);
    for (int i = 0; i < p.length; ++i) {
        if (count >= s)
            return true;
        if ((p.cod[i].x >= xmin && p.cod[i].x <= xmax) || (p.cod[i].y >= ymin && p.cod[i].y <= ymax)) //并还是或?
            count++;
    }
    return count >= s ? true : false;
}

double EW_Cons(double cpq, int i, int j, Matrix &TDM, double alpha, double beta, double max, double min) {
    double w = alpha * (1 - cpq);
    if (max == min)
        return w;
    else {
        w += beta * (TDM.weight(i, j) - min) / (max - min);
        return w;
    }
}

Graph TDM_Cons(vector<Trajectory> &TEC, double s, double lambda, double alpha, double beta) {  //如何随机？
    Graph TG(TEC);
    double max, min,cos_t, length=TEC[0].length_Tra();
    Matrix TDM(distance_matrix(TEC, max, min));
    for (int i = 0; i < TG.V.size(); ++i) {
        for (int j = i + 1; j < TG.V.size(); ++j) {
            if (slcover(TG.V[i], TG.V[j], s, lambda, cos_t))
                TG.insertE(i, j, EW_Cons(cos_t, i, j, TDM, alpha, beta, max, min));
            else
                TG.insertE(i, j, INF);
        }
    }
    return TG;
}

double Anony_track(double &IL, vector<Trajectory> &TEC, int k, double s, double lambda, double alpha, double beta, int &ti) {
    Graph TG(TDM_Cons(TEC, s, lambda, alpha, beta)),*V;
    vector<Graph> G(TG.DFS(0)),S;
    vector<Vaw> W;
    double n_TEC=TEC.size(),TSR=0;
    //隐匿Size<k的连通分量
    bool tag[G.size()],uc= true;
    memset(tag, true, sizeof(tag));
    for (int i = 0; i < G.size(); ++i)
        tag[i]= G[i].countV() > k ? true:false;
    //隐匿End
    Graph *TG_copy=new Graph(TG);
    while (TG.countV() > 0) {
        for (int i = 0; i < G.size(); ++i) {
            if(!tag[i]&&uc){ //跳过规模小于k的连通分量
                tag[i]= true;
                continue;
            }
            if (G[i].countV() >= k) {
                Trajectory v1, v2;
                double min_e=G[i].minE(v1, v2); //在 TG还是G[i]? 中寻找权最小的边(v1,v2)
                if(v1.length_Tra()==0&&v2.length_Tra()==0){
                    vector<Graph> UCG=G[i].DFS(0);
                    G.erase(G.begin()+i);
                    G.insert(G.end(),UCG.begin(),UCG.end());
                    i--;uc= false;
                    continue;
                }
                V=new Graph(k);
                V->insertV(v1);V->insertV(v2);
                V->insertE(0,1,min_e);
                while (V->countV() < k) {
                    for (int j = 0; j < V->countV(); ++j) {
                        vector<string> Array_Link = G[i].linkV(V->refind(j));
                        for (int p = 0; p < Array_Link.size(); ++p) {
                            if (!same_element(Array_Link[p], V->getT())) {
                                Vaw temp = {V->refind(j),Array_Link[p], G[i].weight(G[i].find(V->refind(j)), G[i].find(Array_Link[p]))};
                                W.push_back(temp);
                            }
                        }
                    }
                    //生成一个k-匿名集V
                    if(W.size()<=0) //无点相关的情况
                        break;
                    sort(W.begin(), W.end(), scmp); //从小到大排序
                    V->insertV(G[i].getV(G[i].find(W[0].id_connect)));
                    V->insertE(V->find(W[0].id),V->find(W[0].id_connect),W[0].weight);
                    W.clear();
                }
                for (int j = 0; j < V->countV(); ++j) { //在TG删除V的节点及其关联边
                    TG.deleteV(V->refind(j));
                    G[i].deleteV(V->refind(j));
                }
                S.push_back(*V);
                delete V;
            }
            else {
                double cost=INF;
                int min_cost_pos;
                string id,id_connect;
                for (int j = 0; j < S.size(); ++j) {
                    string t_id,t_id_con;
                    double temp_cost=TG_copy->compare(G[i],S[j],t_id,t_id_con);
                    if(temp_cost<cost){
                        cost=temp_cost;
                        min_cost_pos=j;
                        id=t_id;
                        id_connect=t_id_con;
                    }
                }
                if(cost!=INF)
                    merge_graph(G[i], S[min_cost_pos], id, id_connect, cost);
                //在TG中删除G[i]
                if(TG.countV()<=0)
                    break;
                for (int l = 0; l < G[i].countV(); ++l)
                    TG.deleteV(G[i].refind(l));
            }
        }
    }
    vector<Graph>::iterator it=S.begin();
    while(it!=S.end()){ //k-匿名集轨迹数目<k
        if((*it).countV()>=k){
            it++;
            continue;
        }
        double cost=INF;
        int min_cost_pos;
        string id,id_connect;
        for (int j = 0; j < S.size(); ++j) {
            if(it==S.begin()+j||(*it).countV()+S[j].countV()>=2*k) //匿名集节点数是不是不能超过2k?
                continue;
            string t_id,t_id_con;
            double temp_cost=TG_copy->compare((*it),S[j],t_id,t_id_con);
            if(temp_cost<cost){
                cost=temp_cost;

                min_cost_pos=j;
                id=t_id;
                id_connect=t_id_con;
            }
        }
        if(cost!=INF)
            merge_graph((*it), S[min_cost_pos], id, id_connect, cost);
        S.erase(it);
    }
    delete TG_copy;
    //TODO 信息损失
    //TODO IL+=XXX
    //IL
    if(S.size()==0){
        ti++;
        return 1;
    }
    double x_max=S[0].getV(0).cod[ti%S[0].getV(0).length_Tra()].x,x_min=x_max,y_max=S[0].getV(0).cod[ti%S[0].getV(0).length_Tra()].y,y_min=y_max;
    for(auto g:S)
        g.count_area(ti,x_max,x_min,y_max,y_min);
    ti++;
    IL+=((x_max-x_min)*(y_max-y_min));
    //TSR
    for(auto i:S)
        TSR += i.countV();
    TSR=(n_TEC-TSR)/n_TEC;
    return TSR;
}

bool same_element(string id, vector<Trajectory>& v) {
    for (int i = 0; i < v.size(); ++i) {
        if (id == v[i].getid())
            return true;
    }
    return false;
}

void merge_graph(Graph& G1, Graph& G2, string id, string id_connect,double w) {
    //G1合并到G2
    G2.V.insert(G2.V.end(), G1.V.begin(), G1.V.end());
    G2.WE.plus(G1.countV(), G1.WE);
    G2.WE.set(G2.find(id),G2.find(id_connect),w);
    return;
}

bool scmp(const Vaw &v1,const Vaw &v2){
    if(v1.weight<v2.weight)
        return true;
    return false;
}