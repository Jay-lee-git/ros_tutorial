#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

int main(){
    vector<int> v = {1, 2, 3, 4};
    
    for_each(v.begin(), v.end(), [&](int& n){
        cout << n << endl;
    });
}