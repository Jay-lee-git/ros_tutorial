#include <iostream>

int main(){
    int x = 3;
    auto increase_val = [x](int a){
        return x + a;
    };

    int new_val = increase_val(10);
    std::cout << new_val << std::endl;

}
