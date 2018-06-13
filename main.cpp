#include <time.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "en_tra.h"
#define DATA_LIST "/Volumes/Macintosh HD 2/Documents/Work/new_data/list.txt"
#define DATA_PATH "/Volumes/Macintosh HD 2/Documents/Work/new_data/new_"
#define DATA_OUT "/Volumes/Macintosh HD 2/Documents/Trajectory-Privacy/out/"
#define LIST_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\new_data\\list.txt"
#define PATH_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\new_data\\new_"
using namespace std;

double AREA;

double test(int, double);
TrajectorySet readData();
void writeData(Trajectory &);

int main() {
    for (int i = 4; i <= 10; i++) {
        clock_t begin;
        double used_time;
        begin = clock();
        used_time = test(i, 0.035);
        AREA = 0;
        used_time = (double)(clock() - begin) / CLOCKS_PER_SEC - used_time;
        cout << "TotalTime:" << used_time << "s" << endl;
        break;
    }
}

double test(int k, double s) {
    clock_t readTime = clock();
    int ti = 0, n_TEC = 0;
    double IL = 0, TSR = 0;
    TrajectorySet ALL_T;
    vector<TrajectorySet> ALL_TEC;
    ALL_T = readData();
    readTime = clock() - readTime;
    while (ALL_T.size() != 0) {
        ALL_TEC.push_back(EqualTrack(ALL_T));  //TODO 同步轨迹集暂不改动
    }
    //writeData(ALL_TEC[0][0]);
    n_TEC = ALL_TEC.size();

    /*
    int bb=0,bmax=0,bmin=INF;
    for(auto aa:ALL_TEC){
        bb+=aa.size();
        if(aa.size()>bmax)
            bmax=aa.size();
        if(aa.size()<bmin)
            bmin=aa.size();
    }
    cout<<n_TEC<<" "<<bb<<" "<<bmax<<" "<<bmin<<endl;
    */

    for (auto TEC : ALL_TEC) {
        if (TEC.size() < k) {
            n_TEC--;
            continue;
        }
        /* 等价类规模过小:D1 is dropped since it does not satisfy the
        3-anonymity requirement. V1,V2 and V3 are trajectory k-anonymity
        sets with k=3. */
        TSR += AnonyTrack(TEC, k, s, 1.47, 0.3, 0.7, ti);  // 0.837758
    }
    //cout << "IL=" << IL / (n_TEC * AREA) * 100 << "%,";
    cout << "InfoLoss=" << TSR / n_TEC * 100 << "%" << endl;
    return (double)(readTime / CLOCKS_PER_SEC);
}

TrajectorySet readData() {  //换用FILE提高速度
    int length;
    Coord tempCod;
    string id, path;
    FILE *fData;
    fstream fList;
    vector<Coord> tempCodSet;
    TrajectorySet TrackData;
    fList.open(DATA_LIST, ios::in);
    while (!fList.eof()) {  //读取出租车列表
        fList >> id >> length;
        path = DATA_PATH + id + ".txt";
        fData = fopen(path.c_str(), "r");
        tempCodSet.reserve((unsigned long)length);
        for (int i = 0; i < length; ++i) {  //读取每个出租车的轨迹
            fscanf(fData, "%lf %lf %lf", &tempCod.x, &tempCod.y,
                   &tempCod.t);  //纬度, 经度, 时间(UNIX时间戳)
            tempCodSet.push_back(tempCod);
        }
        reverse(tempCodSet.begin(), tempCodSet.end());
        TrackData.emplace_back(Trajectory(id, tempCodSet));
        vector<Coord>().swap(tempCodSet), id.clear(), path.clear();
        fclose(fData);
    }
    fList.close();
    return TrackData;
}

void writeData(Trajectory &t) {  //格式化输出测试数据,便于可视化
    // TODO temp
    fstream f;
    f.open(string(DATA_OUT) + t.getID() + ".out", ios::out);
    f << "lat,lon" << endl;
    for (int i = 0; i < t.getLength(); ++i) {
        Coord temp = t.getCoord(i);
        f << setprecision(10) << temp.x << "," << temp.y << endl;
    }
}