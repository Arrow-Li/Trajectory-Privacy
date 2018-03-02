#include <time.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include "en_tra.h"
#define DATA_LIST "/Volumes/Macintosh HD 2/Documents/Work/new_data/list.txt"
#define DATA_PATH "/Volumes/Macintosh HD 2/Documents/Work/new_data/new_"
#define DATA_OUT "/Volumes/Macintosh HD 2/Documents/Trajectory-Privacy/out/"
#define DATA_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\test_data\\new_"
#define TEST_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\test_data\\list.txt"
#define TEST_LIST "/Volumes/Macintosh HD 2/Documents/Work/test_data/list.txt"
#define TEST_PATH "/Volumes/Macintosh HD 2/Documents/Work/test_data/new_"
using namespace std;

double AREA;

void test(int, int);
TrajectorySet readData();
void writeData(Trajectory &);

int main() {
    for (int i = 4; i <= 10; i++) {
        clock_t begin;
        double used_time;
        begin = clock();
        test(i, 5);
        AREA = 0;
        used_time = (double)(clock() - begin) / CLOCKS_PER_SEC;
        cout << used_time << "s" << endl;
    }
}

void test(int k, int s) {
    int ti = 0, n_TEC = 0;
    double IL = 0, TSR = 0;
    TrajectorySet ALL_T;
    vector<TrajectorySet> ALL_TEC;
    ALL_T = readData();
    while (ALL_T.size() != 0) {
        ALL_TEC.push_back(EqualTrack(ALL_T));
    }
    writeData(ALL_TEC[0][0]);
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

        clock_t o1=clock();
        TSR += AnonyTrack(IL, TEC, k, s, 0.9, 0.3, 0.7, ti);  // 0.837758
        cout<<(double)(clock()-o1)/CLOCKS_PER_SEC<<endl;
    }
    cout << "IL=" << IL / (n_TEC * AREA) * 100 << "%,";
    cout << "TSR=" << TSR / n_TEC * 100 << "%" << endl;
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
        TrackData.push_back(Trajectory(id, tempCodSet));
        vector<Coord>().swap(tempCodSet), id.clear(), path.clear();
        fclose(fData);
    }
    fList.close();
    return TrackData;
}

void writeData(Trajectory &t) {  //格式化输出测试数据,便于可视化
    // TODO temp
    fstream f;
    f.open(string(DATA_OUT) + t.getId() + ".out", ios::out);
    f << "lat,lon" << endl;
    for (int i = 0; i < t.getLength(); ++i) {
        Coord temp = t.getCoord(i);
        f << setprecision(10) << temp.x << "," << temp.y << endl;
    }
}