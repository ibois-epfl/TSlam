#include "reslam.h"
#include <fstream>


vector<string> getParts(string s,string delimiter){

vector<string> parts;
size_t pos = 0;
std::string token;
while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    parts.push_back(token);
    s.erase(0, pos + delimiter.length());
}
return parts;
}
string getExecDir(const vector<string> &parts){
    if(parts.size()<4)return "";
    string cd,path;
    stringstream sstr;sstr<<parts[3];
    sstr>> cd>>path;
    return path;
}

string getExtension(string in){
    if(in.size()<3)return "";
    string ext;
    for(int i=in.size()-3;i<in.size();i++)
        ext.push_back(in[i]);
    return ext;
}
string getLogFile(const vector<string> &parts){
    string execCmd=parts[5];
    string subpart;
    stringstream sstr;sstr<<execCmd;
    for(int i=0;i<10;i++){
        sstr>>subpart;
         if(getExtension( subpart)=="log") {
             return  subpart;
        }
    }
    return "";
}
int main(int argc,char **argv){

    if(argc!=2) return -1;

    ifstream file(argv[1]);
    if(!file.is_open()){
        cerr<<"No open"<<endl;return -1;
    }
    string line;
    while(!file.eof()){
        std::getline(file,line);
        if(line.empty())continue;
        auto parts=getParts(line,";");
        for(auto p:parts)cout<<p<<endl;
        string pathToTimeFile=getExecDir(parts)+"/timefile";
        string cmd;cmd="cp "+pathToTimeFile+" "+getLogFile(parts)+"time";
        cout<<cmd<<endl;
      //  char c;cin>>c;
        system(cmd.c_str());

    }


}
