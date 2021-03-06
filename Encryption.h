#include "Anonymous.h"
#include "Graph.h"
#include "iomanip"

time_t noSec(time_t, bool);
void eraseFast(TrajectorySet &, std::string);
bool same_element(std::string, TrajectorySet &);
void merge_graph(Graph &, Graph &, std::string, std::string, double);

TrajectorySet EqualTrack(TrajectorySet &T) {
    TrajectorySet equalT;
    time_t startT = (time_t)T[0].cod[0].t,
           endT = (time_t)T[0].cod[T[0].length - 1].t, minStartT = 0,
           maxEndT = 0;
    double tp = ((endT - startT) * 0.05);  // TODO 范围约束参数
    //std::set<double> timeLine;
    /*
    for (const auto &track : T) {
        startT += track.cod[0].t / T.size();
        endT += track.cod[track.length - 1].t / T.size();
    }
     */
    // TODO 等价类划分 修改
reCreateTimeLine:
    TrajectorySet::iterator it = T.begin();
    while (it != T.end()) {
        time_t coordStartT = (time_t)(*(*it).cod.begin()).t;
        time_t coordEndT = (time_t)(*((*it).cod.end() - 1)).t;
        if ((coordStartT >= startT - tp && coordStartT <= startT + tp) &&
            (coordEndT >= endT - tp && coordEndT <= endT + tp)) {
            if (coordStartT < minStartT || minStartT == 0)
                minStartT = coordStartT;
            if (coordEndT > maxEndT) maxEndT = coordEndT;
            equalT.push_back(*it);
            eraseFast(T, (*it).id);
        } else
            ++it;
    }
    if (equalT.size() < 1) {    //如果等价类数量为0,重新生成时间轴
        tp *= 1.2;              // TODO 范围增长参数
        goto reCreateTimeLine;  // ReSync
    }
    if (equalT.size() == 1) return equalT;
    minStartT = noSec(minStartT, true);  // 时间轴头尾取整
    maxEndT = noSec(maxEndT, false);
    // timeLine.erase(++timeLine.find(maxEndT), timeLine.end());  // 精简时间轴
    // timeLine.erase(timeLine.begin(), timeLine.find(minStartT));
    for (auto &et : equalT) {
        et.cod.reserve((unsigned long)(maxEndT - minStartT) / 30 +
                       1);  //减少Vector自增长次数
        et.syncTrajectory((unsigned int &)minStartT, (unsigned int &)maxEndT,
                          30);
    }
    return equalT;
}

double getTrackCos(Trajectory &p, Trajectory &q) {
    int ignore = 1;
    double tmp0, tmp1, tmp2, tmp3, up, down, TrackLen = 0, cosValue = 0;
    for (int i = 0; i < p.length - 1; ++i) {
        tmp0 = p.cod[i + 1].x - p.cod[i].x;
        tmp1 = q.cod[i + 1].x - q.cod[i].x;
        tmp2 = p.cod[i + 1].y - p.cod[i].y;
        tmp3 = q.cod[i + 1].y - q.cod[i].y;
        up = (tmp0) * (tmp1) + (tmp2) * (tmp3);
        if (up == 0) {  //忽略时间间隔内未移动的点
            //ignore++;
            continue;
        }

        tmp0 = sqrt(tmp0 * tmp0 + tmp2 * tmp2);
        tmp1 = sqrt(tmp1 * tmp1 + tmp3 * tmp3);
        TrackLen += (tmp0 + tmp1)/ 2.0;
        down = 1.0 / tmp0 + 1.0 / tmp1;

        //down = sqrt((tmp0 * tmp0 + tmp2 * tmp2) * (tmp1 * tmp1 + tmp3 * tmp3));

        cosValue += (up * down * 0.5);  // 向量夹角[0,180]
    }
    cosValue /= TrackLen;
    return cosValue;
}

double getTrackDis(Trajectory &p, Trajectory &q) {
    double dis = 0, tmp0, tmp1;
    for (int i = 0; i < p.length; ++i) {
        tmp0 = (p.cod[i].x - q.cod[i].x), tmp1 = (p.cod[i].y - q.cod[i].y);
        tmp0 *= tmp0, tmp1 *= tmp1;
        dis += sqrt(tmp0 + tmp1);
    }
    dis /= p.length;
    return dis;
}

Matrix getDisMatrix(TrajectorySet &TEC, double &max, double &min) {
    int n = TEC.size();
    double tmpValue;
    Matrix TDM(n);
    max = 0, min = INF;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            tmpValue = getTrackDis(TEC[i], TEC[j]);
            TDM.setValue(i, j, tmpValue);
            if (tmpValue >= max) max = tmpValue;
            if (tmpValue <= min) min = tmpValue;
        }
    }
    return TDM;
}

bool slCover(Trajectory &p, Trajectory &q, double s, double lambda, double &cpq) {
    int count = 0;
    double xMax, xMin, yMax, yMin, length = p.getLength();
    cpq = getTrackCos(p, q);  // cos(x) 弧度
    if (cpq < cos(lambda) || cpq > 1) return false;
    q.areaTrack(xMin, xMax, yMin, yMax);
    for (const auto &coord : p.cod) {
        if ((coord.x < xMin || coord.x > xMax) ||
            (coord.y < yMin || coord.y > yMax))
            count++;
        if (count/length >= s) return false;
    }
    return count/length >= s ? false : true;
}

double calculateW(double cpq, double trackDis, double alpha, double beta,
                  double max, double min) {
    double w = alpha * (1 - cpq);
    if (max == min)
        return w;
    else {
        w += beta * (trackDis - min) / (max - min);
        return w;
    }
}

Graph createTG(TrajectorySet &TEC, double s, double lambda, double alpha,
               double beta) {
    Graph TG(TEC);
    int SizeTG = TG.V.size();
    double trackDisMax, trackDisMin, trackCos;
    Matrix TDM(getDisMatrix(TEC, trackDisMax, trackDisMin));
    for (int i = 0; i < SizeTG; ++i) {
        for (int j = i + 1; j < SizeTG; ++j) {
            if (slCover(TG.V[i], TG.V[j], s, lambda, trackCos))
                TG.insertE(i, j,
                           calculateW(trackCos, TDM.getValue(i, j), alpha, beta,
                                      trackDisMax, trackDisMin));
        }
    }
    return TG;
}

bool AnonyDoneCheck(bool *run, int len){
    for (int i = 0; i < len ; ++i){
        if (run[i])
            return false;
    }
    return true;
}

bool VawInSetCheck(std::string id, TrajectorySet *s){
    for (auto &si : *s) {
        if(id == si.getID())
            return true;
    }
    return false;
}


Vaw findVaw(std::vector<Vaw> W, std::string drop){
    for (const auto &w : W)
        if(w.id_connect == drop)
            return w;
}

bool deleteCheck(int k, TrajectorySet *V, Graph *G) {
    if (G->countV() == V->size())
        return true;
    Graph *cloneG = new Graph(*G);
    for (auto &vi : *V)
        cloneG->deleteV(vi.getID());
    for (auto &gi : cloneG->DFS(0))
        if (gi.countV() < k) {
            delete cloneG;
            return false;
        }
    delete cloneG;
    return true;
}

std::vector<Vaw> getLinkV(TrajectorySet &V, Graph &G, std::vector<std::string> drop) {
    std::vector<Vaw> LinkVaw;
    for (auto &v : V) {
        std::vector<std::string> linkID = G.linkV(v.getID(), drop);
        for (const auto & id : linkID) {
            if (!same_element(id, V))
                LinkVaw.push_back({v.getID(), id, G.weight(G.find(v.getID()), G.find(id))});
        }
    }
    sort(LinkVaw.begin(), LinkVaw.end(), VawCompare);
    return LinkVaw;
}

TrajectorySet createAnonyV(Graph &PG, int k) {
    std::vector<Vaw> W, MinE;
    TrajectorySet AnonyV;
    MinE = PG.getMinE();

    std::vector<std::string> drop;

    int minEid = 0;
reStart:
    if (minEid >= MinE.size()) {
        //todo do something here
        int abc =1;
    }
    int v1 = PG.find(MinE[minEid].id), v2 = PG.find(MinE[minEid].id_connect);
    TrajectorySet().swap(AnonyV);
    AnonyV.push_back(PG.getV(v1)), AnonyV.push_back(PG.getV(v2));
    while (AnonyV.size() < k) {
        W = getLinkV(AnonyV, PG, drop);
        if (W.empty()) {
            drop.push_back(AnonyV.back().getID());
            AnonyV.pop_back();
            if (AnonyV.size() < 2) {
                minEid++;
                drop.clear();
                goto reStart;
            }
            continue;
        }
        sort(W.begin(), W.end(), VawCompare);
        AnonyV.push_back(PG.getV(PG.find(W.front().id_connect)));
        if (AnonyV.size() == k){ // 删除检测 是否会导致规模小于<k的连通分量产生？
            if (!deleteCheck(k, &AnonyV, &PG)) { //未通过删除检测
                drop.push_back(AnonyV.back().getID());
                AnonyV.pop_back();
            }
        }
        std::vector<Vaw>().swap(W);
    }
    return AnonyV;
}

void isCycle(Graph *PG) {
    int w;
    for (int i = 0; i < PG->countV(); ++i) {
        for (int j = i + 1; j < PG->countV(); ++j) {
            if (PG->weight(i,j) != INF)
                w++;
        }
    }
    if (w >= PG->countV())
        std::cout << "Have a Cycle!" << std::endl;
    return;
}

void deleteCycle(Graph *PG) {
    for (int i = 0; i < PG->countV(); ++i) {
        std::vector<Vaw> connectV;
        for (int j = i + 1; j < PG->countV(); ++j) {
            if (PG->weight(i,j) != INF) {
                connectV.push_back({std::to_string(i),std::to_string(j),PG->weight(i,j)});
            }
        }
        if (connectV.size() <= 1)
            continue;
        else {
            sort(connectV.rbegin(), connectV.rend(), VawCompare);
            for (const Vaw &v : connectV) {
                Graph *cloneG = new Graph(*PG);
                cloneG->insertE(i, std::stoi(v.id_connect), INF);
                if (cloneG->DFS(0).size() > 1) {
                    continue;
                }
                else {
                    delete cloneG;
                    PG->insertE(i ,std::stoi(v.id_connect), INF);
                }
            }
        }
    }
}

double AnonyTrack(TrajectorySet &TEC, int k, double s,
                  double lambda, double alpha, double beta, int &ti) {
    Graph TG(createTG(TEC, s, lambda, alpha, beta));
    std::vector<AnonyArea> AnonyTrackSet;
    std::vector<Graph> PartTG(TG.DFS(0));
    std::vector<TrajectorySet> S;
    std::vector<Vaw> W, MinE, DropW;
    TrajectorySet AnonyC;
    int hideV = 0, dropV = 0;
    double sizeTEC = TEC.size(), TSR = 0;
    bool run[PartTG.size()], tag[PartTG.size()];
    memset(tag, true, sizeof(tag)), memset(run, true , sizeof(run));
    for (int i = 0; i < PartTG.size(); ++i) { //隐匿规模小于k的连通分量
        if (PartTG[i].countV() < k){
            tag[i] = false, run[i] = false;
            hideV += PartTG[i].countV();
        }
    }
    while (!AnonyDoneCheck(run, PartTG.size())) {
        for (int i = 0; i < PartTG.size(); ++i) {
            if (!tag[i])  //跳过规模小于k的连通分量
                continue;
            if (PartTG[i].countV() >= k) {
                //deleteCycle(&PartTG[i]);
                S.push_back(createAnonyV(PartTG[i], k));
                for (auto &ci : S[S.size()-1]) {  //在TG删除V的节点及其关联边
                    TG.deleteV(ci.getID());
                    PartTG[i].deleteV(ci.getID());
                }
            }
            else {
                run[i]= false;
                continue;
            }
        }
    }

    sort(DropW.begin(), DropW.end(), VawCompare);
    for (const auto &wi : DropW) {
        for (auto &si : S) {
            if(si.size()>=2*k-1)
                continue;
            if(VawInSetCheck(wi.id, &si)){
                if (TG.find(wi.id_connect) == -1)
                    break;
                si.push_back(TG.getV(TG.find(wi.id_connect)));
                TG.deleteV(wi.id_connect);
                break;
            }
        }
    }
    for (const auto &s : S)
        if(s.size() < k)
            dropV += s.size();
    // TODO 信息损失
    // TODO InfoLoss+=XXX
    double IL1=0, IL2=0;
    // InfoLoss1

    // InfoLoss2
    IL2 = (sizeTEC-hideV == 0)? 1.0 : (double)(dropV+TG.countV()-hideV)/(sizeTEC-hideV);

    /*
    for (auto anonyArea : S)  // 生成轨迹匿名域集合
        AnonyTrackSet.push_back(AnonyArea(anonyArea.getT()));
    */
    /*
    AnonyTrackSet.push_back(AnonyArea(S[0].getT()));
    std::cout<<AnonyTrackSet[0].getID()[0];
    std::fstream ft;
    ft.open("C:\\Users\\71423\\Desktop\\ppp\\area.txt", std::ios::out);
    //Trajectory ttx(S[0].getT()[2]);
    for (int i = 0; i < AnonyTrackSet[0].getLength(); i++) {
        Area xtt = AnonyTrackSet[0].getArea(i);
        ft << std::fixed << std::setprecision(12) << (xtt.xMax+xtt.xMin)/2 << "\t" << (xtt.yMax+xtt.yMin)/2
           << std::endl;
    }
    ft.close();
    */
    std::cout<<IL1+IL2<<std::endl;

    return IL1+IL2;
}

time_t noSec(time_t t, bool f) {
    tm *date = localtime(&t);
    if (date->tm_sec == 0) return t;
    date->tm_sec = 0;
    if (f) date->tm_min += 1;
    t = mktime(date);
    return t;
}

void eraseFast(TrajectorySet &T, std::string id) {  //快速删除vector
    for (int i = 0; i < T.size(); ++i) {
        if (T[i].getID() == id) {
            std::swap(T[i], T[T.size() - 1]);
            T.pop_back();
            break;
        }
    }
}

bool same_element(std::string id, TrajectorySet &v) {
    for (int i = 0; i < v.size(); ++i) {
        if (id == v[i].getID()) return true;
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