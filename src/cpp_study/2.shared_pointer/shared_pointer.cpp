#include <iostream>
#include <memory>

class Person {
public:
    std::string name;
    explicit Person(const std::string& n) : name(n) {
        std::cout << "Person " << name << " created.\n";
    }
    ~Person(){
        std::cout << "Person " << name << " destroyed.\n";
    }
};

int main(){
    std::shared_ptr<Person> personPtr1 = std::make_shared<Person>("Alice");

    {
        std::shared_ptr<Person> personPtr2 = personPtr1;

        std::cout << "Inside the block: \n";
        std::cout << "personPtr1's name: " << personPtr1->name << "\n";
        std::cout << "personPtr2's name: " << personPtr2->name << "\n";
    }

    std::cout << "Outside the block:\n";
    std::cout << "personPtr1's name: " << personPtr1->name << "\n";

    return 0;
    /*
    이 예제에서 Person 객체는 std::make_shared를 통해 생성되었고, personPtr1이라는 이름의 std::shared_ptr에 의해 관리됩니다. 
    그 후 personPtr2라는 두 번째 std::shared_ptr가 생성되어 personPtr1과 동일한 객체를 공유합니다. 블록 안에서 personPtr2가 범위를 벗어나면 참조 카운트가 감소하지만, 
    personPtr1이 여전히 존재하기 때문에 Person 객체는 메모리에서 해제되지 않습니다. 
    마지막으로 main 함수의 끝에서 personPtr1이 범위를 벗어나면서 Person 객체의 참조 카운트가 0이 되고, 객체가 메모리에서 해제됩니다.
    */
}