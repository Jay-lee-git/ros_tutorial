#include <iostream>

int main() {
    // 두 정수를 받아서 그 합을 반환하는 람다 표현식을 정의합니다.
    auto add = [](int a, int b) -> int {
        return a + b;
    };

    // 람다 표현식을 사용합니다.
    int sum = add(10, 5);
    std::cout << "The sum is " << sum << std::endl;

    return 0;
}
