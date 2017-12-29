#include <iomanip>
#include <fstream>
#include <time.h>
#include "en_tra.h"
#define DATA_LIST "/Volumes/Macintosh HD 2/Documents/data/list.txt"
#define DATA_PATH "/Volumes/Macintosh HD 2/Documents/Work/test_data/new_"
#define DATA_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\test_data\\new_"
#define TEST_WIN "C:\\Users\\Jeep Li\\Documents\\Cprogram\\Work\\test_data\\list.txt"
#define TEST_LIST "/Volumes/Macintosh HD 2/Documents/Work/test_data/list.txt"

double AREA;
vector<Trajectory> read_data();
void fun_run(int,int);
void write_data(vector<Trajectory>&);

int main(){
    for (int i = 1; i <= 7; i++) {
        int a=clock();
        fun_run(5,i);
        AREA = 0;
        int b=clock()-a;
        cout<<b<<"ms"<<endl;
    }
}

void fun_run(int k,int s){
    int ti=0,n_TEC=0;
    double IL=0,TSR=0;
    vector<Trajectory> ALL_T;
    vector<vector<Trajectory>> ALL_TEC;
    ALL_T=read_data();

    while(ALL_T.size()!=0){
        ALL_TEC.push_back(Equal_tracks(ALL_T,100));
    }
    n_TEC=ALL_TEC.size();

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

    for(auto TEC:ALL_TEC){
        if(TEC.size()<k){
            n_TEC--;
            continue;
        }
        //等价类规模过小 隐匿率100%怎么办
        TSR+=Anony_track(IL,TEC,k,s,0.9,0.3,0.7,ti); //0.837758
    }
    //cout<<"IL="<<IL/(n_TEC*AREA)*100<<"%,";
    //cout<<"TSR="<<TSR/n_TEC*100<<"%"<<endl;
}

vector<Trajectory> read_data(){
    int length,skip;
    bool fb=true;
    double x_max,x_min,y_max,y_min;
    string id,path;
    fstream f,f_c;
    vector<Trajectory> all_t;
    vector<Coord> temp_t;
    Coord temp_pos;
    //f.open(DATA_LIST,ios::in);
    //f.open(TEST_LIST,ios::in);
    f.open(TEST_LIST,ios::in);
    while(!f.eof()){ //读取出租车列表
        f>>id>>length;
        //path=DATA_PATH+id+".txt";
        path=DATA_PATH+id+".txt";
        f_c.open(path,ios::in);
        for (int i = 0; i < length; ++i) { //读取每个出租车的轨迹
            f_c>>temp_pos.x>>temp_pos.y>>skip>>temp_pos.t; //纬度，经度，无用，时间(UNIX时间戳)
            temp_t.push_back(temp_pos);
            if(fb){
                x_max=temp_pos.x;x_min=x_max;
                y_max=temp_pos.y;y_min=y_max;
                fb= false;
            }
            x_max=(temp_pos.x>x_max)?temp_pos.x:x_max;
            x_min=(temp_pos.x<x_min)?temp_pos.x:x_min;
            y_max=(temp_pos.y>y_max)?temp_pos.y:y_max;
            y_min=(temp_pos.y<y_min)?temp_pos.y:y_min;
        }
        reverse(temp_t.begin(),temp_t.end());
        Trajectory t(id,temp_t);
        all_t.push_back(t);
        temp_t.clear();
        f_c.close();
    }
    AREA=(x_max-x_min)*(y_max-y_min);
    f.close();
    return all_t;
}

void write_data(vector<Trajectory> &t){ //格式化输出测试数据,方便导入Excel
    fstream f;
    string path="";
    path=DATA_WIN+path+"one.out";
    f.open(path,ios::out);
    for(auto i:t){
        for(auto point:i.cod){
            f<<i.getid()<<","<<point.x<<","<<point.y<<","<<setprecision(10)<<point.t<<endl;
        }
    }
}