// 개수세기
#include <iostream>

int main(){
    int num;
    std::cin >> num;
    int array[num];
    for (int i = 0; i < num; i ++){
        std::cin >> array[i];
    }
    int target;
    std::cin >> target;
    
    int answer = 0;
    for (int i = 0; i < num; i ++){
        if (array[i] == target){
            answer ++;
        }
    }
    std::cout << answer << std::endl;
}