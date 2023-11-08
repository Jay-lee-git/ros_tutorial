#include <iostream>

int main(){
    int x = 10;

    // lamda
    auto increment_x = [x]() mutable -> int {
        return ++ x;
    };

    int new_value = increment_x();
    std::cout << "New value is " << new_value << std::endl;
    std::cout << "Original Value is "<< x << std::endl;

    return 0;
}
