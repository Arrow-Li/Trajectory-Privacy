#include "Graph.h"
#include "Anonymous.h"
bool same_element(std::string, TrajectorySet &);
void merge_graph(Graph &, Graph &, std::string, std::string, double);
bool scmp(const Vaw &, const Vaw &);

TrajectorySet EqualTrack(TrajectorySet &T) {
    TrajectorySet equalT;
    double tp, startT = 0, endT = 0;
    std::set<double> timeLine;
    for (const auto &track : T) {
        startT += track.cod[0].t / T.size();
        endT += track.cod[track.length - 1].t / T.size();
    }
    tp = (endT - startT) * 0.04;  //TODO 范围约束参数
reCreateTimeLine:
    TrajectorySet::iterator it = T.begin();
    while (it != T.end()) {
        double coordStartT = (*(*it).cod.begin()).t;
        double coordEndT = (*((*it).cod.end() - 1)).t;
        if ((coordStartT >= startT - tp && coordStartT <= startT + tp) &&
            (coordEndT >= endT - tp && coordEndT <= endT + tp)) {
            equalT.push_back(*it);
            for (const auto &coord : (*it).cod)
                timeLine.insert(coord.t);  //生成时间轴
            T.erase(it);
        } else
            it++;
    }
    if (equalT.size() < 1) {  //如果等价类数量为0,重新生成时间轴
        tp *= 2;  //TODO 范围增长参数
        goto reCreateTimeLine;  // ReSync
    }
    for (auto &et : equalT) {
        et.cod.reserve(timeLine.size());  //减少Vector自增长次数
        et.syncTrajectory(timeLine);
    }
    return equalT;
}

double getTrackCos(Trajectory p, Trajectory q) {
    int ignore = 0;
    double cosValue = 0;
    for (int i = 0; i < p.length - 1; ++i) {
        double num =
            (p.cod[i + 1].x - p.cod[i].x) * (q.cod[i + 1].x - q.cod[i].x) +
            (p.cod[i + 1].y - p.cod[i].y) * (q.cod[i + 1].y - q.cod[i].y);
        double denom = sqrt(pow((p.cod[i + 1].x - p.cod[i].x), 2) +
                            pow((p.cod[i + 1].y - p.cod[i].y), 2)) *
                       sqrt(pow((q.cod[i + 1].x - q.cod[i].x), 2) +
                            pow((q.cod[i + 1].y - q.cod[i].y), 2));
        if (num == 0 || denom == 0) {  //忽略时间间隔内未移动的点
            ignore++;
            continue;
        }
        cosValue += fabs(num / denom);  //cos要取绝对值？ 两个线段夹角在0到90度
    }
    cosValue /= p.length - ignore;
    return cosValue;
}

double getTrackDis(Trajectory p, Trajectory q) {
    double dis = 0;
    for (int i = 0; i < p.length; ++i) {
        double tmp = pow((p.cod[i].x - q.cod[i].x), 2) +
                     pow((p.cod[i].y - q.cod[i].y), 2);
        dis += sqrt(tmp);
    }
    dis /= p.length;
    return dis;
}

Matrix getDisMatrix(TrajectorySet &TEC, double &max, double &min) {
    int n = TEC.size();
    Matrix TDM(n);
    max = 0;
    min = INF;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            TDM.setValue(i, j, getTrackDis(TEC[i], TEC[j]));
            if (TDM.getValue(i, j) >= max) max = TDM.getValue(i, j);
            if (TDM.getValue(i, j) <= min) min = TDM.getValue(i, j);
        }
    }
    return TDM;
}

bool slCover(Trajectory p, Trajectory q, int s, double lambda, double &cpq) {
    double xMax, xMin, yMax, yMin;
    int count = 0;
    cpq = getTrackCos(p, q); //cos(x) 弧度
    if (cpq < cos(lambda) || cpq > 1) return false;
    q.areaTrack(xMin, xMax, yMin, yMax);
    for (const auto & coord : p.cod) {
        if (count >= s) return true;
        if ((coord.x >= xMin && coord.x <= xMax) ||
            (coord.y >= yMin && coord.y <= yMax))
            count++;
    }
    return count >= s ? true : false;
}

double EW_Cons(double cpq, int i, int j, Matrix &TDM, double alpha, double beta,
               double max, double min) {
    double w = alpha * (1 - cpq);
    if (max == min)
        return w;
    else {
        w += beta * (TDM.getValue(i, j) - min) / (max - min);
        return w;
    }
}

Graph createTG(TrajectorySet &TEC, double s, double lambda, double alpha,
               double beta) {  //如何随机？
    Graph TG(TEC);
    double trackDisMax, trackDisMin, trackCos;
    Matrix TDM(getDisMatrix(TEC, trackDisMax, trackDisMin));
    for (int i = 0; i < TG.V.size(); ++i) {
        for (int j = i + 1; j < TG.V.size(); ++j) {
            if (slCover(TG.V[i], TG.V[j], s, lambda, trackCos))
                TG.insertE(i, j,
                           EW_Cons(trackCos, i, j, TDM, alpha, beta, trackDisMax, trackDisMin));
        }
    }
    return TG;
}

double AnonyTrack(double &InfoLoss, TrajectorySet &TEC, int k, double s,
                  double lambda, double alpha, double beta, int &ti) {
    Graph TG(createTG(TEC, s, lambda, alpha, beta)), *V;
    std::vector<AnonyArea> AnonyTrackSet;
    std::vector<Graph> G(TG.DFS(0)), S;
    std::vector<Vaw> W;
    double n_TEC = TEC.size(), TSR = 0;
    //隐匿Size<k的连通分量
    bool tag[G.size()], uc = true;
    memset(tag, true, sizeof(tag));
    for (int i = 0; i < G.size(); ++i)
        tag[i] = G[i].countV() > k ? true : false;
    //隐匿End
    Graph *TG_copy = new Graph(TG);
    while (TG.countV() > 0) {
        for (int i = 0; i < G.size(); ++i) {
            if (!tag[i] && uc) {  //跳过规模小于k的连通分量
                tag[i] = true;
                continue;
            }
            if (G[i].countV() >= k) {
                Trajectory v1, v2;
                double min_e =
                    G[i].minE(v1, v2);  //TODO 在TG还是G[i]?中寻找权最小的边(v1,v2)
                if (v1.getLength() == 0 && v2.getLength() == 0) {
                    std::vector<Graph> UCG = G[i].DFS(0);
                    G.erase(G.begin() + i);
                    G.insert(G.end(), UCG.begin(), UCG.end());
                    i--;
                    uc = false;
                    continue;
                }
                V = new Graph(k);
                V->insertV(v1);
                V->insertV(v2);
                V->insertE(0, 1, min_e);  // TODO k=1越界？
                while (V->countV() < k) {
                    for (int j = 0; j < V->countV(); ++j) {
                        std::vector<std::string> Array_Link =
                            G[i].linkV(V->refind(j));
                        for (int p = 0; p < Array_Link.size(); ++p) {
                            if (!same_element(Array_Link[p], V->getT())) {
                                Vaw temp = {
                                    V->refind(j), Array_Link[p],
                                    G[i].weight(G[i].find(V->refind(j)),
                                                G[i].find(Array_Link[p]))};
                                W.push_back(temp);
                            }
                        }
                    }
                    //生成一个k-匿名集V
                    if (W.size() <= 0)  //无点相关的情况
                        break;
                    sort(W.begin(), W.end(), scmp);  //从小到大排序
                    V->insertV(G[i].getV(G[i].find(W[0].id_connect)));
                    V->insertE(V->find(W[0].id), V->find(W[0].id_connect),
                               W[0].weight);
                    W.clear();
                }
                for (int j = 0; j < V->countV();
                     ++j) {  //在TG删除V的节点及其关联边
                    TG.deleteV(V->refind(j));
                    G[i].deleteV(V->refind(j));
                }
                S.push_back(*V);
                delete V;
            } else {
                double cost = INF;
                int min_cost_pos;
                std::string id, id_connect;
                for (int j = 0; j < S.size(); ++j) {
                    std::string t_id, t_id_con;
                    double temp_cost =
                        TG_copy->compare(G[i], S[j], t_id, t_id_con);
                    if (temp_cost < cost) {
                        cost = temp_cost;
                        min_cost_pos = j;
                        id = t_id;
                        id_connect = t_id_con;
                    }
                }
                if (cost != INF)
                    merge_graph(G[i], S[min_cost_pos], id, id_connect,
                                cost);  // id-G1 id_connect-G2
                //在TG中删除G[i]
                if (TG.countV() <= 0) break;
                for (int l = 0; l < G[i].countV(); ++l)
                    TG.deleteV(G[i].refind(l));
            }
        }
    }
    std::vector<Graph>::iterator it = S.begin();
    while (it != S.end()) {  // k-匿名集轨迹数目<k
        if ((*it).countV() >= k) {
            it++;
            continue;
        }
        double cost = INF;
        int min_cost_pos;
        std::string id, id_connect;
        for (int j = 0; j < S.size(); ++j) {
            if (it == S.begin() + j ||
                (*it).countV() + S[j].countV() >=
                    2 * k)  //匿名集节点数是不是不能超过2k?
                continue;
            std::string t_id, t_id_con;
            double temp_cost = TG_copy->compare((*it), S[j], t_id, t_id_con);
            if (temp_cost < cost) {
                cost = temp_cost;

                min_cost_pos = j;
                id = t_id;
                id_connect = t_id_con;
            }
        }
        if (cost != INF)
            merge_graph((*it), S[min_cost_pos], id, id_connect, cost);
        S.erase(it);
    }
    delete TG_copy;
    // TODO 信息损失
    // TODO InfoLoss+=XXX
    // InfoLoss
    if (S.size() == 0) {
        ti++;
        return 1;
    }
    for(auto anonyArea:S) // 生成轨迹匿名域集合
        AnonyTrackSet.push_back(AnonyArea(anonyArea.getT()));
    S.clear();
    ti++;
    for(auto anonySet:AnonyTrackSet)
        InfoLoss+=anonySet.countArea();
    // TSR
    for (auto i : S) TSR += i.countV();
    TSR = (n_TEC - TSR) / n_TEC;
    return TSR;
}

bool same_element(std::string id, TrajectorySet &v) {
    for (int i = 0; i < v.size(); ++i) {
        if (id == v[i].getId()) return true;
    }
    return false;
}

void merge_graph(Graph &G1, Graph &G2, std::string id, std::string id_connect,
                 double w) {
    // G1合并到G2
    G2.V.insert(G2.V.end(), G1.V.begin(), G1.V.end());
    G2.WE.merge(G2.find(id_connect), G1.find(id), w, G1.WE);
    return;
}

bool scmp(const Vaw &v1, const Vaw &v2) {
    if (v1.weight < v2.weight) return true;
    return false;
}