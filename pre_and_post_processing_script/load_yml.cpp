#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
using namespace std;

bool isDigit(char c){
    return c == '-' || c == '.' || (c >= '0' && c <= '9');
}

string getDigit(string s, int &i){
    while(!isDigit(s[i])) i++;
    string digit = "";
    while(isDigit(s[i])){
        digit += s[i];
        i++;
    }
    cout << digit << endl;
    return digit;
}

double getDouble(string s, int &i){
    return stod(getDigit(s, i));
}

int getInt(string s, int &i){
    return stoi(getDigit(s, i));
}

int main(){
    string yamlString;

    // Read from the text file
    ifstream yamlFile("/home/tpp/UCOSlam-IBOIS/build/utils/merged_map.yml");


    // Use a while loop together with the getline() function to read the file line by line
    string tmpStr;
    while (getline(yamlFile, tmpStr)) {
        // Output the text from the file
        yamlString += tmpStr;
    }

    map<int, vector<vector<double> > > markers;

    int index = yamlString.find("corners");
    int markerAmount = 95;
    for(int mi = 0; mi < markerAmount; mi++) {
        vector<vector<double> > markerCorners;
        for(int ci = 0; ci < 4; ci++){
            vector<double> markerCorner(3);
            for(int vi = 0; vi < 3; vi++){
                markerCorner[vi] = getDouble(yamlString, index);
            }
            markerCorners.push_back(markerCorner);
        }
        int id = getInt(yamlString, index);
        markers[id] = markerCorners;
    }
    
    // stringstream yamlStream;
    // yamlStream << yamlString;
    
    // char _c;
    // string _s;
    // yamlStream >> _s >> _s >> _s >> _s >> _s >> _c; // eat header

    // yamlStream >> _s;
    // while(yamlStream >> _s && _s.find("aruco_bc_nmarkers") == std::string::npos){
    //     // while it's not digit, keep eating
    //     while(yamlStream >> _c && _c != '-' && !(_c >= '0' && _c <= '9')){
    //         yamlStream >> _c;
    //     }
    //     for()
    // }

    cout << yamlString << endl;

    // Close the file
    // MyReadFile.close(); 
}