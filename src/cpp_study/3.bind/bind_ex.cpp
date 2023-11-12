#include <functional>
#include <iostream>
using namespace std::placeholders;

void print(const int a, const int b){
    std::cout << a << " " << b << std::endl;
}

int main(){
    // 함수를 선언과 동시에 생성한다
    auto bindPrint = std::bind(print, _1, _2);
    bindPrint(2, 3);
    return 0;
}
